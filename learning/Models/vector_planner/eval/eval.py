"""holdout での基本指標（経路・地図・検出）を JSON に出す。"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from Models.vector_planner.data.dataset import SLOTS, X_RANGE, Y_MIN, Y_SPAN, split_datasets
from Models.vector_planner.eval import metrics as M
from Models.vector_planner.model.network import load_network
from Models.vector_planner.training.trainer import parse_data_spec

TENSOR_KEYS = ('path_xy', 'path_mask', 'path_slot_ok', 'map_points', 'map_mask', 'boxes', 'box_cls', 'box_mask', 'has_boxes')
OUTPUT_KEYS = ('path_xy', 'map_logits', 'map_points', 'det_boxes', 'det_scores', 'det_cls')


def collect(model, datasets, device, batch_size=8, workers=0):
    records = []
    model.eval()
    for ds in datasets:
        offset = 0
        for batch in DataLoader(ds, batch_size=batch_size, shuffle=False, num_workers=workers):
            with torch.no_grad():
                image = batch['image'].to(device).float() / 255.0
                outputs = model(image)
            for i in range(image.shape[0]):
                stem = ds.stems[offset + i]
                records.append({
                    'dir': ds.split, 'stem': stem, 'label_root': ds.label_root,
                    'gt': {k: batch[k][i].numpy() for k in TENSOR_KEYS},
                    'pred': {k: outputs[k][i].float().cpu().numpy() for k in OUTPUT_KEYS}})
            offset += image.shape[0]
    return records


def _load_labels(records):
    for r in records:
        path = Path(r['label_root']) / 'labels' / r['dir'] / f"{r['stem']}.json"
        r['label'] = json.loads(path.read_text())


def _median(values):
    return float(np.median(values)) if values else None


def path_metrics(records, slot_index, slot):
    totals = {}
    for r in records:
        entry = r['label']['paths'].get(slot)
        if entry is None or entry['status'] not in ('clear', 'avoid') or not r['gt']['path_slot_ok'][slot_index]:
            continue
        t = totals.setdefault(entry['status'], {'n_frames': 0, 'l1': 0.0, 'lateral': 0.0, 'count': 0})
        gt = torch.from_numpy(r['gt']['path_xy'][slot_index])
        mask = torch.from_numpy(r['gt']['path_mask'][slot_index])
        pred = torch.from_numpy(np.asarray(r['pred']['path_xy'][slot_index], np.float32))
        sums = M.path_error_sums(pred, gt, mask)
        t['l1'] += sums['l1']
        t['lateral'] += sums['lateral']
        t['count'] += int(mask.sum())
        t['n_frames'] += 1
    result = {}
    for status, t in totals.items():
        c = max(t['count'], 1)
        result[status] = {'n_frames': t['n_frames'], 'n_points': t['count'], 'l1': t['l1'] / (2 * c),
                          'lateral': t['lateral'] / c}
    return result


def _map_polylines(points):
    pts = np.asarray(points, float).copy()
    pts[..., 0] *= X_RANGE
    pts[..., 1] = pts[..., 1] * Y_SPAN + Y_MIN
    return list(pts)


def _softmax(x):
    e = np.exp(x - x.max(axis=-1, keepdims=True))
    return e / e.sum(axis=-1, keepdims=True)


def vectormap_metrics(records, conf=0.5, radius=1.0):
    chamfers, hits = [], np.zeros(4, int)
    skipped = {'pred_empty': 0, 'gt_empty': 0}
    for r in records:
        if '__p' in r['stem']:
            continue
        prob = _softmax(r['pred']['map_logits'])
        keep = (prob.argmax(-1) < 3) & (prob.max(-1) >= conf)
        pred = _map_polylines(r['pred']['map_points'][keep])
        gt = _map_polylines(r['gt']['map_points'][r['gt']['map_mask']])
        if not pred and not gt:
            continue
        s = M.vectormap_scores(pred, gt, radius)
        hits += [s['precision_hits'], s['precision_total'], s['recall_hits'], s['recall_total']]
        if s['chamfer'] is None:
            skipped['pred_empty' if not pred else 'gt_empty'] += 1
        else:
            chamfers.append(s['chamfer'])
    return {'n_frames': len(chamfers), 'frames_pred_empty': skipped['pred_empty'],
            'frames_gt_empty': skipped['gt_empty'], 'chamfer_mean': float(np.mean(chamfers)) if chamfers else None,
            'chamfer_median': _median(chamfers), 'radius_m': radius, 'conf': conf,
            'precision': float(hits[0] / hits[1]) if hits[1] else None,
            'recall': float(hits[2] / hits[3]) if hits[3] else None}


def detection_metrics(records):
    gt_boxes, gt_cls, preds = [], [], []
    for r in records:
        if not r['gt']['has_boxes']:
            continue
        mask = r['gt']['box_mask']
        gt_boxes.append([tuple(b) for b in r['gt']['boxes'][mask]])
        gt_cls.append([int(c) for c in r['gt']['box_cls'][mask]])
        p = r['pred']
        preds.append([(*p['det_boxes'][k], float(p['det_scores'][k]), int(p['det_cls'][k]))
                      for k in range(len(p['det_scores']))])
    ap = {str(c): M.average_precision(gt_boxes, gt_cls, preds, c) for c in range(4)}
    valid = [v for v in ap.values() if v is not None]
    return {'n_frames': len(gt_boxes), 'ap50': ap, 'map50': float(np.mean(valid)) if valid else None}


def evaluate(model, specs, split_map, split='holdout', slot='straight', device='cpu', batch_size=8,
             workers=0, input_size=(384, 640), conf=0.5, radius=1.0, max_frames=None):
    datasets = split_datasets(specs, split_map, split, input_size)
    if max_frames is not None:
        for ds in datasets:
            ds.stems = ds.stems[:max_frames]
    records = collect(model, datasets, device, batch_size, workers)
    _load_labels(records)
    return {'split': split, 'slot': slot, 'n_frames': len(records),
            'path': path_metrics(records, SLOTS.index(slot), slot),
            'vectormap': vectormap_metrics(records, conf, radius),
            'detection': detection_metrics(records)}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--data', action='append', required=True, help='IMAGE_ROOT:LABEL_ROOT（複数指定可）')
    p.add_argument('--split-json', required=True)
    p.add_argument('--split', default='holdout')
    p.add_argument('--slot', default='straight', choices=SLOTS)
    p.add_argument('--out', default=None)
    p.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    p.add_argument('--batch-size', type=int, default=8)
    p.add_argument('--workers', type=int, default=2)
    p.add_argument('--max-frames', type=int, default=None, help='データセット単位の先頭 N 枚だけ評価（動作確認用）')
    args = p.parse_args(argv)

    model, ckpt = load_network(args.checkpoint, args.device)
    split_map = json.loads(Path(args.split_json).read_text())
    result = evaluate(model, [parse_data_spec(s) for s in args.data], split_map, args.split, args.slot,
                      args.device, args.batch_size, args.workers, model.input_size,
                      max_frames=args.max_frames)
    result['checkpoint'] = {'path': args.checkpoint, 'epoch': ckpt.get('epoch')}
    text = json.dumps(result, indent=1)
    if args.out:
        Path(args.out).write_text(text)
    print(text)
    return result


if __name__ == '__main__':
    main()

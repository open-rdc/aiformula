import argparse
import glob
import json
import os
import re
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

SOURCE_PREFIX = 'aiformula_real_v4pl__'
SPLITS = ('train', 'val', 'test')
PAD_TOP, SRC_HEIGHT, SRC_WIDTH, OUT_HEIGHT = 12, 360, 640, 384
PANEL_CLASS = {0: 2, 1: 3}
KEEP_DET_CLASSES = (0, 1)
MIN_SCORE = 0.5
GEN_MIN_SCORE = 0.3
OUT_SPLIT = 'train'
MEAN, STD = np.array([.485, .456, .406]), np.array([.229, .224, .225])


def holdout_bags(split_map):
    # split_map から hold-out の bag 名を集める
    return {key.split('/', 1)[1].rsplit('_', 1)[0] for key, value in split_map.items() if value == 'holdout'}


def source_bag(filename):
    stem = os.path.splitext(os.path.basename(filename))[0]
    if not stem.startswith(SOURCE_PREFIX):
        return None
    return re.sub(r'_\d+$', '', stem[len(SOURCE_PREFIX):])


def _norm_box(cls, score, cx, cy, w, h):
    # 640x360 の正規化座標 -> 上 12 px パッド済み 640x384 の正規化座標
    return {'class': int(cls), 'score': float(score), 'cx': float(cx),
            'cy': float((PAD_TOP + cy * SRC_HEIGHT) / OUT_HEIGHT), 'w': float(w), 'h': float(h * SRC_HEIGHT / OUT_HEIGHT)}


def yolo_panel_boxes(text):
    boxes = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 5 or int(parts[0]) not in PANEL_CLASS:
            continue
        cx, cy, w, h = (float(v) for v in parts[1:5])
        boxes.append(_norm_box(PANEL_CLASS[int(parts[0])], 1.0, cx, cy, w, h))
    return boxes


def rfdetr_raw_boxes(dets, labels, min_score=GEN_MIN_SCORE):
    # RF-DETR の生出力（全クラス・score つき・640x384 正規化）
    dets, logits = dets[0], labels[0][:, :4]
    cls = logits.argmax(1)
    score = 1.0 / (1.0 + np.exp(-logits[np.arange(len(cls)), cls]))
    keep = score >= min_score
    return [_norm_box(c, s, *d) for c, s, d in zip(cls[keep], score[keep], dets[keep])]


def pseudo_boxes(raw, min_score=MIN_SCORE):
    return [b for b in raw if b['class'] in KEEP_DET_CLASSES and b['score'] >= min_score]


def det_only_label(boxes):
    return {'map_elements': [], 'paths': {}, 'boxes': boxes, 'has_boxes': True, 'det_only': True}


def pad_frame(image):
    out = np.zeros((OUT_HEIGHT, SRC_WIDTH, 3), np.uint8)
    out[PAD_TOP:PAD_TOP + SRC_HEIGHT] = image
    return out


def make_rfdetr_detector(onnx_path, threads=2):
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    session = ort.InferenceSession(onnx_path, options, providers=['CPUExecutionProvider'])

    def detect(images):
        results = []
        for image in images:
            x = cv2.cvtColor(cv2.resize(image, (640, 640), interpolation=cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB) / 255.0
            x = ((x - MEAN) / STD).astype(np.float32).transpose(2, 0, 1)[None]
            dets, labels = session.run(['dets', 'labels'], {'input': x})
            results.append(rfdetr_raw_boxes(dets, labels))
        return results

    return detect


def _candidates(ds_root, bags, counts, holdout):
    found = []
    for split in SPLITS:
        for path in sorted(glob.glob(os.path.join(ds_root, 'images', split, SOURCE_PREFIX + '*'))):
            name = os.path.basename(path)
            is_holdout = source_bag(name) in bags
            if is_holdout != holdout:
                counts['excluded_holdout' if not holdout else 'excluded_train'] += 1
            else:
                found.append((split, path))
    return found


def build_det_panel(ds_root, split_map, out_img, out_lab, detect, batch_size=2, limit=None, holdout=False):
    # ds_panel（YOLO 形式）から検出専用フレームを作る。コーン・人 = detect の擬似ラベル、パネル = 手作業ラベル。
    # holdout=True は hold-out bag のみ（評価専用）、False は除く（学習用）
    counts = Counter({'excluded_holdout': 0, 'excluded_train': 0, 'frames': 0, 'skipped_existing': 0})
    items = _candidates(ds_root, holdout_bags(split_map), counts, holdout)
    if limit is not None:
        items = items[:limit]
    img_dir, lab_dir = Path(out_img) / 'images' / OUT_SPLIT, Path(out_lab) / 'labels' / OUT_SPLIT
    img_dir.mkdir(parents=True, exist_ok=True)
    lab_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = Path(out_lab) / 'raw' / OUT_SPLIT
    raw_dir.mkdir(parents=True, exist_ok=True)
    todo = []
    for split, path in items:
        stem = os.path.splitext(os.path.basename(path))[0]
        if (lab_dir / f'{stem}.json').exists():
            counts['skipped_existing'] += 1
            continue
        todo.append((split, path, stem))
    for start in range(0, len(todo), batch_size):
        chunk = todo[start:start + batch_size]
        images = [cv2.imread(path, cv2.IMREAD_COLOR) for _, path, _ in chunk]
        detections = detect(images)
        for (split, path, stem), image, det in zip(chunk, images, detections):
            label_file = Path(ds_root) / 'labels' / split / f'{stem}.txt'
            panels = yolo_panel_boxes(label_file.read_text()) if label_file.exists() else []
            boxes = pseudo_boxes(det) + panels
            (raw_dir / f'{stem}.json').write_text(json.dumps(det))
            cv2.imwrite(str(img_dir / f'{stem}.png'), pad_frame(image))
            (lab_dir / f'{stem}.json.tmp').write_text(json.dumps(det_only_label(boxes)))
            os.replace(lab_dir / f'{stem}.json.tmp', lab_dir / f'{stem}.json')
            counts['frames'] += 1
    return dict(counts)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--ds-panel', required=True, help='YOLO 形式の ds_panel（images/・labels/）')
    p.add_argument('--split-json', required=True)
    p.add_argument('--onnx', required=True)
    p.add_argument('--out-img', required=True, help='画像ルート（images/train/*.png を作る）')
    p.add_argument('--out-lab', required=True, help='ラベルルート（labels/train/*.json を作る）')
    p.add_argument('--threads', type=int, default=2)
    p.add_argument('--batch-size', type=int, default=2)
    p.add_argument('--limit', type=int, default=None)
    p.add_argument('--holdout', action='store_true', help='hold-out bag の画像だけを出力（評価専用）')
    args = p.parse_args(argv)
    split_map = json.loads(Path(args.split_json).read_text())
    detect = make_rfdetr_detector(args.onnx, args.threads)
    counts = build_det_panel(args.ds_panel, split_map, args.out_img, args.out_lab, detect, args.batch_size, args.limit, args.holdout)
    print(json.dumps(counts))
    return counts


if __name__ == '__main__':
    main()

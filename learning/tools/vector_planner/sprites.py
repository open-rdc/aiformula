import argparse
import glob
import json
import os
import zlib
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from Models.common.projection import load_camera
from Models.common.vectormap import load_vector_map
from Models.vector_planner.data.frame_label import (
    DEFAULT_PARAMS, TRACE_HORIZON_M, build_frame_label, load_frame_meta, make_config, parse_lengths)
from Models.vector_planner.data.map_labels import densify, load_line_map, smooth_reference, to_base, trace_slots
from Models.vector_planner.data.paste_aug import (
    IMAGE_HEIGHT, IMAGE_WIDTH, load_sprite_bank, paste_sprite, sample_pylons)

PAD_TOP = 12
BANK_MIN_SCORE = 0.8
BANK_MIN_HEIGHT_PX = 40
EDGE_MARGIN_PX = 2
FG_RATIO_RANGE = (0.25, 0.9)
PERSON_MIN_HEIGHT_PX = 60
PERSON_FG_RATIO_RANGE = (0.2, 0.95)
MIN_ORANGE_RATIO = 0.3


def _is_cut(x0, y0, x1, y1, height, width, pad_top):
    return (x0 < EDGE_MARGIN_PX or x1 > width - EDGE_MARGIN_PX
            or y0 < pad_top + EDGE_MARGIN_PX or y1 > height - pad_top - EDGE_MARGIN_PX)


def _grabcut_foreground(image, box, pad_top, min_height):
    # box の矩形を GrabCut で分け、矩形内の最大連結成分を返す。(crop, fg, crop 内の box 座標) か None
    height, width = image.shape[:2]
    x0, x1 = int(round((box['cx'] - box['w'] / 2) * width)), int(round((box['cx'] + box['w'] / 2) * width))
    y0, y1 = int(round((box['cy'] - box['h'] / 2) * height)), int(round((box['cy'] + box['h'] / 2) * height))
    if y1 - y0 < min_height or x1 - x0 < 4 or _is_cut(x0, y0, x1, y1, height, width, pad_top):
        return None

    margin = max(4, (y1 - y0) // 10)
    cx0, cy0 = max(0, x0 - margin), max(pad_top, y0 - margin)
    cx1, cy1 = min(width, x1 + margin), min(height - pad_top, y1 + margin)
    crop = np.ascontiguousarray(image[cy0:cy1, cx0:cx1])
    rect = (x0 - cx0, y0 - cy0, x1 - x0, y1 - y0)
    mask = np.zeros(crop.shape[:2], np.uint8)
    try:
        cv2.grabCut(crop, mask, rect, np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64), 5,
                    cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return None

    fg = ((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)).astype(np.uint8)
    fg[:rect[1]] = 0
    fg[rect[1] + rect[3]:] = 0
    fg[:, :rect[0]] = 0
    fg[:, rect[0] + rect[2]:] = 0
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(fg)
    if count < 2:
        return None
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return crop, (labels == largest).astype(np.uint8), (x0 - cx0, y0 - cy0, x1 - cx0, y1 - cy0)


def _tight_rgba(crop, fg):
    ys, xs = np.nonzero(fg)
    tight = (slice(ys.min(), ys.max() + 1), slice(xs.min(), xs.max() + 1))
    return np.ascontiguousarray(np.dstack([crop, fg * 255])[tight].astype(np.uint8))


def extract_sprite(image, box, pad_top=PAD_TOP, min_score=BANK_MIN_SCORE, min_height=BANK_MIN_HEIGHT_PX):
    if box['class'] != 0 or box['score'] < min_score:
        return None
    found = _grabcut_foreground(image, box, pad_top, min_height)
    if found is None:
        return None
    crop, fg, (rx0, ry0, rx1, ry1) = found
    ratio = fg.sum() / float((rx1 - rx0) * (ry1 - ry0))
    if not FG_RATIO_RANGE[0] <= ratio <= FG_RATIO_RANGE[1]:
        return None
    ys, xs = np.nonzero(fg)
    if ys.max() - ys.min() + 1 < 0.6 * (ry1 - ry0):
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[fg > 0]
    orange = (hsv[:, 0] >= 5) & (hsv[:, 0] <= 25) & (hsv[:, 1] >= 100) & (hsv[:, 2] >= 80)
    if orange.mean() < MIN_ORANGE_RATIO:
        return None
    return _tight_rgba(crop, fg)


def extract_person_sprite(image, box, pad_top=PAD_TOP, min_score=BANK_MIN_SCORE, min_height=PERSON_MIN_HEIGHT_PX):
    # 人（cls1）用。色の条件はなく、足元が下端に来ない抽出は捨てる
    if box['class'] != 1 or box['score'] < min_score:
        return None
    found = _grabcut_foreground(image, box, pad_top, min_height)
    if found is None:
        return None
    crop, fg, (rx0, ry0, rx1, ry1) = found
    ratio = fg.sum() / float((rx1 - rx0) * (ry1 - ry0))
    if not PERSON_FG_RATIO_RANGE[0] <= ratio <= PERSON_FG_RATIO_RANGE[1]:
        return None
    ys, xs = np.nonzero(fg)
    if ys.max() - ys.min() + 1 < 0.8 * (ry1 - ry0) or ys.max() < ry1 - 1 - 0.1 * (ry1 - ry0):
        return None
    return _tight_rgba(crop, fg)


def _iter_box_files(pairs):
    for image_root, boxes_root in pairs:
        for path in sorted(glob.glob(os.path.join(boxes_root, 'boxes', '*', '*.json'))):
            split, stem = Path(path).parent.name, Path(path).stem
            yield os.path.join(image_root, 'images', split, f'{stem}.png'), path


def _save_bank(sprites, out_npz, max_sprites):
    if len(sprites) > max_sprites:
        sprites = [sprites[i] for i in np.linspace(0, len(sprites) - 1, max_sprites).astype(int)]
    np.savez_compressed(out_npz, count=len(sprites), **{f'sprite_{i}': s for i, s in enumerate(sprites)})
    return len(sprites)


def build_sprite_bank(pairs, out_npz, max_sprites=300, pad_top=PAD_TOP, min_score=BANK_MIN_SCORE,
                      min_height=BANK_MIN_HEIGHT_PX):
    sprites = []
    for image_path, boxes_path in _iter_box_files(pairs):
        boxes = [b for b in json.loads(Path(boxes_path).read_text()) if b['class'] == 0 and b['score'] >= min_score]
        if not boxes or not os.path.exists(image_path):
            continue
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        for box in boxes:
            sprite = extract_sprite(image, box, pad_top, min_score, min_height)
            if sprite is not None:
                sprites.append(sprite)
    return _save_bank(sprites, out_npz, max_sprites)


def build_person_bank(pairs, out_npz, allow=None, max_sprites=300, pad_top=PAD_TOP, min_score=BANK_MIN_SCORE,
                      min_height=PERSON_MIN_HEIGHT_PX):
    # pairs: (image_root, label_root)。labels の boxes（擬似ラベル）から cls1 を切り出す。allow: 使う 'split/stem' の集合（holdout 除外用）
    sprites = []
    for image_root, label_root in pairs:
        for path in sorted(glob.glob(os.path.join(label_root, 'labels', '*', '*.json'))):
            split, stem = Path(path).parent.name, Path(path).stem
            if allow is not None and f'{split}/{stem}' not in allow:
                continue
            boxes = [b for b in json.loads(Path(path).read_text()).get('boxes', [])
                     if b['class'] == 1 and b['score'] >= min_score]
            image_path = os.path.join(image_root, 'images', split, f'{stem}.png')
            if not boxes or not os.path.exists(image_path):
                continue
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            for box in boxes:
                sprite = extract_person_sprite(image, box, pad_top, min_score, min_height)
                if sprite is not None:
                    sprites.append(sprite)
    return _save_bank(sprites, out_npz, max_sprites)


def variant_rng(stem, index, seed=0):
    return np.random.default_rng([zlib.crc32(stem.encode()), int(index), int(seed)])


def augment_dataset(image_root, label_root, osm_path, bank_path, out_dir, variants=2, params_path=DEFAULT_PARAMS,
                    pylon_height=0.7, pylon_width=0.25, limit=None, seed=0, teacher_lengths=None):
    cfg = make_config(params_path, teacher_lengths)
    camera = load_camera(params_path, 'nHD', IMAGE_HEIGHT, IMAGE_WIDTH)
    bank = load_sprite_bank(bank_path)
    line_map, vector_map = load_line_map(osm_path), load_vector_map(osm_path)
    root = Path(image_root)
    poses, gnss, obstacles = load_frame_meta(root)

    stats = Counter()
    sources = sorted(glob.glob(os.path.join(label_root, 'labels', '*', '*.json')))
    for path in sources:
        split, stem = Path(path).parent.name, Path(path).stem
        label = json.loads(Path(path).read_text())
        straight = label['paths'].get('straight')
        if straight is None or straight['status'] != 'clear':
            continue
        key = f'{split}/{stem}'
        pose = poses.get(key)
        image_path = root / 'images' / split / f'{stem}.png'
        if pose is None or not image_path.exists():
            stats['no_pose_or_image'] += 1
            continue
        if limit is not None and stats['straight_clear_sources'] >= limit:
            break
        stats['straight_clear_sources'] += 1

        position = np.array([pose['x'], pose['y']], float)
        trace = trace_slots(vector_map, position, pose['yaw'], horizon=TRACE_HORIZON_M).get('straight', (None,))[0]
        if trace is None:
            stats['no_trace'] += 1
            continue
        reference = smooth_reference(densify(trace))
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        existing = [tuple(o) for o in obstacles.get(gnss.get(key, {}).get('bag'), [])]

        for index in range(variants):
            rng = variant_rng(stem, index, seed)
            current, pasted, boxes = image, [], []
            for s, d, x, y in sorted(sample_pylons(reference, rng), key=lambda p: -p[0]):
                sprite = bank[int(rng.integers(len(bank)))]
                base = to_base(np.array([[x, y]]), position, pose['yaw'])[0]
                result = paste_sprite(current, sprite, base, camera, pylon_height * rng.uniform(0.95, 1.05), rng)
                if result is None:
                    continue
                current = result[0]
                boxes.append(result[1])
                pasted.append((x, y, base))
            if not pasted:
                stats['variant_no_visible_pylon'] += 1
                continue

            new_label = build_frame_label(line_map, vector_map, position, pose['yaw'],
                                          existing + [(x, y, pylon_width) for x, y, _ in pasted], cfg)
            new_label['boxes'] = list(label.get('boxes', [])) + boxes
            new_label['has_boxes'] = True
            new_label['pasted_base'] = [[float(b[0]), float(b[1]), pylon_width] for _, _, b in pasted]
            new_label['source'] = key

            out_stem = f'{stem}__p{index}'
            image_out = Path(out_dir) / 'images' / split / f'{out_stem}.png'
            label_out = Path(out_dir) / 'labels' / split / f'{out_stem}.json'
            image_out.parent.mkdir(parents=True, exist_ok=True)
            label_out.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(image_out), current)
            label_out.write_text(json.dumps(new_label))

            new_straight = new_label['paths'].get('straight')
            stats[f"status_{new_straight['status'] if new_straight else 'none'}"] += 1
            if new_straight and new_straight['status'] == 'avoid':
                stats[f"side_{new_straight['side']}"] += 1
            stats['variants'] += 1
    return stats


def main(argv=None):
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)

    bank = sub.add_parser('bank')
    bank.add_argument('--cls', type=int, choices=[0, 1], default=0, help='0=コーン（--boxes-root）、1=人（--label-root の labels の boxes）')
    bank.add_argument('--image-root', action='append', required=True)
    bank.add_argument('--boxes-root', action='append', default=[])
    bank.add_argument('--label-root', action='append', default=[], help='--cls 1: <root>/labels/<split>/<stem>.json（boxes 入り）')
    bank.add_argument('--split-json', default=None, help='--cls 1: {"split/stem": "train"|"holdout"|"drop"}')
    bank.add_argument('--split-name', default='train', help='--cls 1: --split-json でこの値のフレームだけ使う')
    bank.add_argument('--out', required=True)
    bank.add_argument('--max-sprites', type=int, default=300)
    bank.add_argument('--min-score', type=float, default=BANK_MIN_SCORE)
    bank.add_argument('--min-height', type=int, default=None, help='元 bbox の最小高さ px（縮小のみにするため既定 コーン 40 / 人 60）')

    aug = sub.add_parser('augment')
    aug.add_argument('--image-root', required=True)
    aug.add_argument('--label-root', required=True)
    aug.add_argument('--osm', required=True)
    aug.add_argument('--bank', required=True)
    aug.add_argument('--out', required=True)
    aug.add_argument('--variants', type=int, default=2)
    aug.add_argument('--pylon-height', type=float, required=True)
    aug.add_argument('--pylon-width', type=float, default=0.25)
    aug.add_argument('--params', default=str(DEFAULT_PARAMS))
    aug.add_argument('--limit', type=int, default=None)
    aug.add_argument('--seed', type=int, default=0)
    aug.add_argument('--teacher-lengths', default='6', help='教師 Frenet の target_lengths（例 6 / 6,8）')
    args = parser.parse_args(argv)

    if args.command == 'bank' and args.cls == 1:
        if len(args.image_root) != len(args.label_root):
            parser.error('--cls 1 は --image-root と --label-root を同数')
        allow = None
        if args.split_json:
            allow = {k for k, v in json.loads(Path(args.split_json).read_text()).items() if v == args.split_name}
        count = build_person_bank(list(zip(args.image_root, args.label_root)), args.out, allow, args.max_sprites,
                                  min_score=args.min_score, min_height=args.min_height or PERSON_MIN_HEIGHT_PX)
        print(f'sprites: {count}')
        return count
    if args.command == 'bank':
        if len(args.image_root) != len(args.boxes_root):
            parser.error('--image-root と --boxes-root を同数')
        count = build_sprite_bank(list(zip(args.image_root, args.boxes_root)), args.out, args.max_sprites,
                                  min_score=args.min_score, min_height=args.min_height or BANK_MIN_HEIGHT_PX)
        print(f'sprites: {count}')
        return count
    stats = augment_dataset(args.image_root, args.label_root, args.osm, args.bank, args.out, args.variants,
                            args.params, args.pylon_height, args.pylon_width, args.limit, args.seed,
                            teacher_lengths=parse_lengths(args.teacher_lengths))
    for name, value in sorted(stats.items()):
        print(f'{name}: {value}')
    return stats


if __name__ == '__main__':
    main()

import argparse
import csv
import glob
import json
import os
from collections import Counter
from pathlib import Path

import numpy as np

from Models.common.vectormap import load_vector_map
from Models.vector_planner.data.frame_label import (
    DEFAULT_PARAMS, build_frame_label, load_frame_meta, make_config, parse_lengths)
from Models.vector_planner.data.map_labels import load_line_map

MIN_SCORE = 0.5
DEFAULT_HOLDOUT = (56.0, None, None, -2.0)
DEFAULT_BUFFER_M = 25.0


def _write_label(out_root, split, name, label):
    target = Path(out_root) / 'labels' / split / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(label))


def _gap(value, low, high):
    low = -np.inf if low is None else low
    high = np.inf if high is None else high
    return max(low - value, 0.0, value - high)


def assign_split(x, y, holdout=DEFAULT_HOLDOUT, buffer_m=DEFAULT_BUFFER_M):
    xmin, xmax, ymin, ymax = holdout
    distance = float(np.hypot(_gap(x, xmin, xmax), _gap(y, ymin, ymax)))
    if distance == 0.0:
        return 'holdout'
    return 'train' if distance >= buffer_m else 'drop'


def build_split_map(frames, **kwargs):
    return {key: assign_split(pose['x'], pose['y'], **kwargs) for key, pose in frames.items()}


def aggregate_static_obstacles(observations, cluster_radius=0.7, min_views=5, max_range=15.0):
    kept = [o for o in observations if o[4] <= max_range]
    if not kept:
        return []

    keys = [o[0] for o in kept]
    xy = np.array([[o[1], o[2]] for o in kept], float)
    widths = np.array([o[3] for o in kept], float)
    weights = np.array([1.0 / (o[4] + 1.0) ** 2 for o in kept])

    unassigned = np.ones(len(kept), bool)
    results = []
    for seed in np.argsort(-weights, kind='stable'):
        if not unassigned[seed]:
            continue
        members = unassigned & (np.linalg.norm(xy - xy[seed], axis=1) <= cluster_radius)
        unassigned &= ~members
        idx = np.flatnonzero(members)
        views = len({keys[i] for i in idx})
        if views < min_views:
            continue
        center = np.average(xy[idx], axis=0, weights=weights[idx])
        results.append((float(center[0]), float(center[1]), float(np.median(widths[idx])), views))
    return results


def _excluded_keys(root):
    manifest = Path(root) / 'manifest.csv'
    if not manifest.exists():
        return set()
    with open(manifest, newline='') as f:
        return {row['key'] for row in csv.DictReader(f) if row.get('excluded_reason')}


def build_dataset(root, osm_path, out_dir, params_path=DEFAULT_PARAMS, teacher_lengths=None):
    cfg = make_config(params_path, teacher_lengths)
    line_map, vector_map = load_line_map(osm_path), load_vector_map(osm_path)
    poses, gnss, obstacles = load_frame_meta(root)
    excluded = _excluded_keys(root)

    counts = Counter()
    for image in sorted(glob.glob(os.path.join(root, 'images', '*', '*.png'))):
        split, stem = Path(image).parent.name, Path(image).stem
        key = f'{split}/{stem}'
        if key in excluded:
            counts['excluded'] += 1
            continue
        pose = poses.get(key)
        if pose is None:
            counts['no_pose'] += 1
            continue
        bag = gnss.get(key, {}).get('bag')
        label = build_frame_label(line_map, vector_map, (pose['x'], pose['y']), pose['yaw'],
                                  [tuple(o) for o in obstacles.get(bag, [])], cfg)
        if not label['paths']:
            counts['no_path'] += 1
        for path in label['paths'].values():
            counts[path['status']] += 1
        _write_label(out_dir, split, f'{stem}.json', label)
        counts['frames'] += 1
    return counts


def attach_boxes(label_root, out_root, boxes_root=None, min_score=MIN_SCORE):
    boxes_root = label_root if boxes_root is None else boxes_root
    counts = Counter()
    for path in sorted(glob.glob(os.path.join(label_root, 'labels', '*', '*.json'))):
        split, name = Path(path).parent.name, Path(path).name
        label = json.loads(Path(path).read_text())
        boxes_path = Path(boxes_root) / 'boxes' / split / name
        if boxes_path.exists():
            detections = json.loads(boxes_path.read_text())
            label['boxes'] = [b for b in detections if b['score'] >= min_score]
            label['has_boxes'] = True
            counts['with_boxes'] += 1
            counts['boxes'] += len(label['boxes'])
        else:
            counts['no_boxes_file'] += 1
        _write_label(out_root, split, name, label)
        counts['frames'] += 1
    return counts


def _print_counts(counts):
    for name, value in sorted(counts.items()):
        print(f'{name}: {value}')
    return counts


def main(argv=None):
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)

    build = sub.add_parser('build', help='ポーズ・地図・障害物から経路/走行帯ラベルを生成')
    build.add_argument('root')
    build.add_argument('osm')
    build.add_argument('out')
    build.add_argument('--params', default=str(DEFAULT_PARAMS))
    build.add_argument('--teacher-lengths', default='6', help='教師 Frenet の target_lengths（例 6 / 6,8）')

    boxes = sub.add_parser('boxes', help='検出 bbox（boxes/<split>/*.json）をラベルへ付与')
    boxes.add_argument('label_root')
    boxes.add_argument('out_root')
    boxes.add_argument('--boxes-root', default=None)
    boxes.add_argument('--min-score', type=float, default=MIN_SCORE)

    split = sub.add_parser('split', help='poses.json から train/holdout/drop の split json を作る')
    split.add_argument('--poses', nargs='+', required=True)
    split.add_argument('--out', required=True)
    split.add_argument('--holdout', type=float, nargs=4, default=None,
                       metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'), help='nan で無限')
    split.add_argument('--buffer', type=float, default=DEFAULT_BUFFER_M)

    args = parser.parse_args(argv)
    if args.command == 'build':
        return _print_counts(build_dataset(args.root, args.osm, args.out, args.params,
                                           parse_lengths(args.teacher_lengths)))
    if args.command == 'boxes':
        return _print_counts(attach_boxes(args.label_root, args.out_root, args.boxes_root, args.min_score))

    holdout = DEFAULT_HOLDOUT if args.holdout is None else tuple(None if np.isnan(v) else v for v in args.holdout)
    split_map = {}
    for path in args.poses:
        split_map.update(build_split_map(json.loads(Path(path).read_text())['frames'], holdout=holdout,
                                         buffer_m=args.buffer))
    Path(args.out).write_text(json.dumps(split_map))
    _print_counts(Counter(split_map.values()))
    return split_map


if __name__ == '__main__':
    main()

import argparse
import json
import os
import sys

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

from Models.data_utils.vision_planner.pose import apply_fit, ecef_to_local, fit_quality, fit_to_map, headings, interpolate_pose, resample_centerlines, speed_mask
from Models.data_utils.vision_planner.projection import letterbox, load_camera, row_anchor_columns, row_anchor_targets
from Models.data_utils.vision_planner.vectormap import load_vector_map, trace_paths

SLOT_NAMES = ('straight', 'left', 'right')
SLOT_COLORS = ((0, 220, 0), (230, 120, 0), (0, 140, 255))

MIN_VISIBLE_ROWS = 10       # これ未満なら経路が画像にほとんど写っていない
EDGE_RATIO = 0.5            # 可視行のうちこの割合以上が画像端なら投影が破綻している
EDGE_MARGIN = 0.01

POSE_TOPIC = '/vectornav/pose'

# encoding -> チャンネル数。並びは触らず先頭 3ch をそのまま通す
IMAGE_CHANNELS = {'bgr8': 3, 'bgra8': 4, 'rgb8': 3, 'rgba8': 4}


def convert_kenta(image):
    if image.shape[:2] != (600, 960):
        raise ValueError(f'convert_kenta: 想定していない入力サイズ {image.shape[:2]}（960x600 を想定）')
    resized = cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA)
    return resized[20:380]


SOURCES = {
    'nHD': {'topic': '/zed/zed_node/rgb/image_rect_color', 'intrinsics': 'nHD', 'convert': None},
    'kenta': {'topic': '/image_raw', 'intrinsics': 'kenta', 'convert': convert_kenta},
}


def detect_source(topics):
    for key, spec in SOURCES.items():
        if spec['topic'] in topics:
            return key
    return None


def densify(polyline, step=0.2):
    polyline = np.asarray(polyline, float)
    length = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(polyline, axis=0), axis=1))]
    if length[-1] < step:
        return polyline
    sample = np.r_[np.arange(0.0, length[-1], step), length[-1]]
    return np.stack([np.interp(sample, length, polyline[:, 0]), np.interp(sample, length, polyline[:, 1])], axis=1)


def path_to_base(polyline, position, heading):
    relative = np.asarray(polyline, float) - np.asarray(position, float)
    cos, sin = np.cos(-heading), np.sin(-heading)
    x = relative[:, 0] * cos - relative[:, 1] * sin
    y = relative[:, 0] * sin + relative[:, 1] * cos
    return np.stack([x, y, np.zeros(len(relative))], axis=1)


def build_label(paths, position, heading, camera, num_rows, height, width):
    label = []
    for slot in SLOT_NAMES:
        if slot not in paths:
            continue
        base = path_to_base(densify(paths[slot]), position, heading)
        pixels, visible = camera.project(base)
        columns, valid = row_anchor_columns(pixels, visible, num_rows, height, width)
        if not valid.any():
            continue
        label.append({'class': slot, 'xp': [float(c) for c in columns], 'h_vector': [int(v) for v in valid]})
    return label


def rejection_reasons(entries):
    if not entries:
        return ['経路なし']

    reasons = []
    visible_counts = [int(np.sum(np.asarray(e['h_vector']) > 0)) for e in entries]
    if max(visible_counts) < MIN_VISIBLE_ROWS:
        reasons.append(f'可視行 {max(visible_counts)} < {MIN_VISIBLE_ROWS}')

    # 分岐が画面外へ抜けるのは正常なので、基準になる straight だけを見る
    for entry in entries:
        if entry['class'] != 'straight':
            continue
        columns = np.asarray(entry['xp'], float)[np.asarray(entry['h_vector']) > 0]
        if columns.size == 0:
            continue
        at_edge = (columns <= EDGE_MARGIN) | (columns >= 1.0 - EDGE_MARGIN)
        if at_edge.mean() >= EDGE_RATIO:
            reasons.append(f'straight の可視行の {at_edge.mean() * 100:.0f}% が画像端')

    return reasons


def val_blocks(num_blocks, val_ratio):
    if val_ratio <= 0:
        return set()

    # val ブロックを等間隔に選ぶ（コース全体をまんべんなく評価するため）
    count = min(max(1, round(num_blocks * val_ratio)), num_blocks)
    step = num_blocks / count
    return {int(step * i + step / 2) for i in range(count)}


def draw_preview(image, label, num_rows, height):
    preview = image.copy()
    targets = row_anchor_targets(num_rows, height)
    for entry in label:
        color = SLOT_COLORS[SLOT_NAMES.index(entry['class'])]
        for row, (column, ok) in enumerate(zip(entry['xp'], entry['h_vector'])):
            if ok:
                center = (int(column * (preview.shape[1] - 1)), int(targets[row]))
                cv2.circle(preview, center, 3, color, -1)
    return preview


def _message_stamp(message, fallback_ns):
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return stamp.sec + stamp.nanosec * 1e-09
    return fallback_ns / 1000000000.0


def read_topic(reader_factory, bag, topic, extract, stride=1):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = reader_factory()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    message_type = get_message(types[topic])
    index = 0
    while reader.has_next():
        _, data, recv_stamp = reader.read_next()
        if index % stride == 0:
            message = deserialize_message(data, message_type)
            yield index, _message_stamp(message, recv_stamp), extract(message)
        index += 1


def bag_topics(bag):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def image_to_array(message):
    channels = IMAGE_CHANNELS.get(message.encoding)
    if channels is None:
        raise ValueError(f"image_to_array: 未対応の encoding '{message.encoding}'（{'/'.join(IMAGE_CHANNELS)} のみ対応）")
    frame = np.frombuffer(message.data, np.uint8).reshape(message.height, message.width, channels)
    return frame[:, :, :3].copy()


def exclude_val_bags(bags, val_bags):
    val_set = {bag.rstrip('/') for bag in val_bags}
    return [bag for bag in bags if bag.rstrip('/') not in val_set]


def process_bag(bag, split, args, vector_map, points, tangents, stats):
    import rosbag2_py

    name = os.path.basename(bag.rstrip('/'))
    topics = bag_topics(bag)
    source = detect_source(topics)
    if source is None or POSE_TOPIC not in topics:
        print(f'  {name}: 画像または姿勢トピックが無いので飛ばす')
        return None

    spec = SOURCES[source]
    params = os.path.join(get_package_share_directory('main_executor'), 'config', 'main_params.yaml')
    camera = load_camera(params, spec['intrinsics'], args.input_height, args.input_width)

    poses = list(read_topic(rosbag2_py.SequentialReader, bag, POSE_TOPIC, lambda m: (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)))
    times = np.array([t for _, t, _ in poses])

    if times.size > 1 and np.any(np.diff(times) < 0):
        raise ValueError(f'{name}: /vectornav/pose の times（header.stamp 由来）が単調増加していません。'
                         'headings()/interpolate_pose() は非減少を前提にしているため、非単調のまま処理を続けると誤った index を無言で参照し誤ったラベルを作ってしまいます')

    ecef = np.array([p for _, _, p in poses])
    xy = ecef_to_local(ecef)

    keep = speed_mask(times, ecef, xy, args.max_speed)
    times, xy = times[keep], xy[keep]

    fitted = apply_fit(xy, fit_to_map(xy, points, tangents))
    residual, agreement = fit_quality(fitted, points, tangents)
    if agreement < args.min_agreement:
        print(f'  {name}: 進行方向一致率 {agreement:.2f} < {args.min_agreement} のため棄却')
        return None

    heading, ok = headings(times, fitted)

    for name_ in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', name_), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', name_), exist_ok=True)

    blocks = val_blocks(args.num_blocks, args.val_ratio) if split is None else set()
    span = max(times[-1] - times[0], 1e-09)

    written = skipped_pose = skipped_path = skipped_broken = 0
    num_rows = args.input_height // 8

    for index, stamp, message in read_topic(rosbag2_py.SequentialReader, bag, spec['topic'], lambda m: m, args.stride):
        pose = interpolate_pose(times, fitted, heading, ok, stamp)
        if pose is None:
            skipped_pose += 1
            continue
        position, angle = pose

        paths = trace_paths(vector_map, position, angle, args.horizon)
        label = build_label(paths, position, angle, camera, num_rows, args.input_height, args.input_width)
        if not label:
            skipped_path += 1
            continue
        if rejection_reasons(label):
            skipped_broken += 1
            continue

        frame = image_to_array(message)
        if spec['convert'] is not None:
            frame = spec['convert'](frame)
        frame = letterbox(frame, args.input_height, args.input_width, camera.pad_top, camera.pad_left)

        frame_split = split
        if frame_split is None:
            block = min(int((stamp - times[0]) / span * args.num_blocks), args.num_blocks - 1)
            frame_split = 'val' if block in blocks else 'train'

        stem = f'{name}_{index:06d}'
        cv2.imwrite(os.path.join(args.out, 'images', frame_split, stem + '.png'), frame)
        with open(os.path.join(args.out, 'labels', frame_split, stem + '.txt'), 'w') as f:
            json.dump(label, f)

        for entry in label:
            stats[entry['class']] += 1
            stats['valid_rows'] += sum(entry['h_vector'])

        written += 1
        if written <= args.preview and frame_split == 'train':
            os.makedirs(os.path.join(args.out, 'preview'), exist_ok=True)
            cv2.imwrite(os.path.join(args.out, 'preview', stem + '.png'), draw_preview(frame, label, num_rows, args.input_height))

    stats['written'] += written
    print(f'  {name}: {written} 枚  残差 {residual:.3f} m  一致率 {agreement:.2f}'
          f'  姿勢欠落 {skipped_pose}  経路なし {skipped_path}  投影破綻 {skipped_broken}')


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--bags', nargs='+', required=True, help='bag ディレクトリ全体。--val-bags と重複するものは val 側に回す')
    parser.add_argument('--val-bags', nargs='*', default=[], help='val に回す bag ディレクトリ')
    parser.add_argument('--out', required=True)
    parser.add_argument('--input-height', type=int, default=384)
    parser.add_argument('--input-width', type=int, default=640)
    parser.add_argument('--stride', type=int, default=2)
    parser.add_argument('--horizon', type=float, default=20.0)
    parser.add_argument('--max-speed', type=float, default=15.0)
    parser.add_argument('--min-agreement', type=float, default=0.8)
    parser.add_argument('--preview', type=int, default=30)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--num-blocks', type=int, default=10)
    args = parser.parse_args(argv)

    if args.stride < 1:
        parser.error('--stride は 1 以上を指定してください')

    osm = os.path.join(get_package_share_directory('vectormap_server'), 'config', 'aiformula_course.osm')
    vector_map = load_vector_map(osm)
    points, tangents = resample_centerlines(vector_map)

    stats = {'written': 0, 'valid_rows': 0, 'straight': 0, 'left': 0, 'right': 0}

    if args.val_bags:
        groups = (('train', exclude_val_bags(args.bags, args.val_bags)), ('val', args.val_bags))
    else:
        groups = ((None, args.bags),)

    for split, bags in groups:
        for bag in bags:
            try:
                process_bag(bag, split, args, vector_map, points, tangents, stats)
            except ValueError as error:
                name = os.path.basename(bag.rstrip('/'))
                print(f'  {name}: 例外により棄却 ({error})')

    print(f'\n合計 {stats["written"]} 枚')
    for slot in SLOT_NAMES:
        ratio = 100.0 * stats[slot] / max(stats['written'], 1)
        print(f'  {slot:9s} 出現 {stats[slot]:6d} 枚 ({ratio:5.1f}%)')

    total_slots = sum(stats[s] for s in SLOT_NAMES)
    print(f'  有効行 平均 {stats["valid_rows"] / max(total_slots, 1):.1f} / {args.input_height // 8}')

    return 0 if stats['written'] else 1


if __name__ == '__main__':
    sys.exit(main())

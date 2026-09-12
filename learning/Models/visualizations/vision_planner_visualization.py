import argparse
import os
import sys
from collections import defaultdict

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import torch

from Models.model_components.vision_planner.vision_planner_head import decode_positions
from Models.model_components.vision_planner.vision_planner_network import VisionPlannerNetwork
from Models.data_parsing.rosbag.converter import POSE_TOPIC, SOURCES, bag_topics, build_label, detect_source, image_to_array, read_topic
from Models.data_utils.vision_planner.pose import apply_fit, ecef_to_local, fit_quality, fit_to_map, headings, interpolate_pose, resample_centerlines, speed_mask
from Models.data_utils.vision_planner.projection import load_camera, letterbox, row_anchor_targets
from Models.data_utils.vision_planner.vectormap import load_vector_map, trace_paths

SLOT_NAMES = ('straight', 'left', 'right')
SLOT_COLORS = ((0, 220, 0), (230, 120, 0), (0, 140, 255))
NUM_SLOTS = 3

MIN_STILL_GAP = 30


def prepare_input(frame_bgr):
    chw = np.ascontiguousarray(frame_bgr.transpose(2, 0, 1))
    return torch.from_numpy(chw).float() / 255.0


def infer(model, frame_bgr, device):
    tensor = prepare_input(frame_bgr).unsqueeze(0).to(device)
    exist, valid, dist = model(tensor)
    return exist[0].detach().cpu(), valid[0].detach().cpu(), dist[0].detach().cpu()


def to_model_frame(frame_bgr, convert, camera, height, width):
    if convert is not None:
        frame_bgr = convert(frame_bgr)
    return letterbox(frame_bgr, height, width, camera.pad_top)


def analyze_frame(exist_logits, valid_logits, dist_logits, exist_threshold, valid_threshold, num_rows, height, width):
    exist_prob = torch.sigmoid(exist_logits)
    valid_prob = torch.sigmoid(valid_logits)
    positions = decode_positions(dist_logits[None])[0]
    anchors = row_anchor_targets(num_rows, height)

    points = []
    for slot in range(exist_prob.shape[0]):
        if float(exist_prob[slot]) <= exist_threshold:
            continue
        for row in range(num_rows):
            if float(valid_prob[slot, row]) <= valid_threshold:
                continue
            points.append({'slot': slot, 'row': row,
                           'x': int(float(positions[slot, row]) * (width - 1)),
                           'y': int(anchors[row])})

    return points, exist_prob, valid_prob, positions


def select_gt_points(label, num_rows, height, width):
    anchors = row_anchor_targets(num_rows, height)
    points = []
    for entry in label:
        slot = SLOT_NAMES.index(entry['class'])
        for row, (column, ok) in enumerate(zip(entry['xp'], entry['h_vector'])):
            if ok:
                points.append({'slot': slot, 'row': row,
                               'x': int(column * (width - 1)),
                               'y': int(anchors[row])})
    return points


def lateral_errors(label, positions):
    errors = []
    for entry in label:
        slot = SLOT_NAMES.index(entry['class'])
        for row, (column, ok) in enumerate(zip(entry['xp'], entry['h_vector'])):
            if ok:
                errors.append(abs(float(positions[slot, row]) - column))
    return errors


MARKER_STYLES = (
    {'radius': 6, 'thickness': 2},
    {'radius': 4, 'thickness': -1},
    {'radius': 2, 'thickness': -1},
)


def draw_marker(image, x, y, slot, scale=1.0):
    style = MARKER_STYLES[slot]
    radius = max(int(round(style['radius'] * scale)), 1)
    thickness = style['thickness']
    if thickness > 0:
        thickness = max(int(round(thickness * scale)), 1)
    cv2.circle(image, (x, y), radius, SLOT_COLORS[slot], thickness)
    return image


def draw_predictions(image, points, connect=True):
    grouped = defaultdict(list)
    for point in points:
        grouped[point['slot']].append(point)

    for slot in sorted(grouped, key=lambda s: MARKER_STYLES[s]['radius'], reverse=True):
        entries = sorted(grouped[slot], key=lambda p: p['row'])
        color = SLOT_COLORS[slot]
        if connect:
            faint = tuple(int(c * 0.4) for c in color)
            for a, b in zip(entries, entries[1:]):
                if b['row'] - a['row'] == 1:
                    cv2.line(image, (a['x'], a['y']), (b['x'], b['y']), faint, 1)
        for point in entries:
            draw_marker(image, point['x'], point['y'], slot)

    return image


def draw_gt(image, points, length=4):
    for point in points:
        color = SLOT_COLORS[point['slot']]
        cv2.line(image, (point['x'] - length, point['y']), (point['x'] + length, point['y']), color, 2)
    return image


def gt_status_text(has_map, gt_error_px):
    if not has_map:
        return None
    if gt_error_px is None:
        return 'GT なし'
    return f'GT誤差 {gt_error_px:.1f}px'


def draw_hud(image, exist_prob, exist_threshold, frame_index, bag_name, has_map=False, gt_error_px=None):
    y = 18
    for slot in range(len(exist_prob)):
        color = SLOT_COLORS[slot]
        prob = float(exist_prob[slot])
        marker = '>' if prob > exist_threshold else ' '
        draw_marker(image, 14, y - 4, slot, scale=0.7)
        text = f'{marker}{SLOT_NAMES[slot]} {prob:.2f}'
        cv2.putText(image, text, (26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        y += 16

    cv2.putText(image, f'frame {frame_index}', (max(image.shape[1] - 150, 0), 18),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, bag_name, (8, image.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    status = gt_status_text(has_map, gt_error_px)
    if status is not None:
        cv2.putText(image, status, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    return image


def build_title_frames(width, height, fps, lines, seconds=1.5):
    if not lines:
        return []

    frame = np.zeros((height, width, 3), np.uint8)
    y = max(height // 2 - len(lines) * 13, 20)
    for line in lines:
        cv2.putText(frame, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        y += 26

    count = max(int(round(fps * seconds)), 1)
    return [frame.copy() for _ in range(count)]


def select_stills(records, count, min_gap=MIN_STILL_GAP):
    if count <= 0 or not records:
        return []

    n = len(records)
    selected = []

    def far_enough(candidate):
        return all(abs(records[candidate]['index'] - records[other]['index']) >= min_gap for other in selected)

    branch_order = sorted(range(n), key=lambda i: records[i]['branch_score'], reverse=True)
    for index in branch_order:
        if len(selected) >= count:
            break
        if records[index]['branch_score'] <= 0.0:
            break
        if far_enough(index):
            selected.append(index)

    if len(selected) < count:
        remaining = [i for i in range(n) if i not in selected]
        pools = [sorted(remaining, key=lambda i: records[i]['straight_valid'], reverse=True),
                sorted(remaining, key=lambda i: records[i]['straight_valid'])]
        cursors = [0, 0]
        pool_turn = 0
        exhausted = [False, False]
        while len(selected) < count and not all(exhausted):
            pool, cursor = pools[pool_turn], cursors[pool_turn]
            advanced = False
            while cursor < len(pool):
                candidate = pool[cursor]
                cursor += 1
                if candidate in selected:
                    continue
                if far_enough(candidate):
                    selected.append(candidate)
                    advanced = True
                    break
            cursors[pool_turn] = cursor
            if cursor >= len(pool):
                exhausted[pool_turn] = True
            if not advanced and len(selected) >= count:
                return sorted(selected)
            pool_turn = 1 - pool_turn

    return sorted(selected)


def _build_gt_context(args, reader_factory):
    osm = os.path.join(get_package_share_directory('vectormap_server'), 'config', 'aiformula_course.osm')
    vector_map = load_vector_map(osm)
    points, tangents = resample_centerlines(vector_map)

    poses = list(read_topic(reader_factory, args.bag, POSE_TOPIC,
                            lambda m: (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)))
    times = np.array([t for _, t, _ in poses])
    ecef = np.array([p for _, _, p in poses])
    xy = ecef_to_local(ecef)

    keep = speed_mask(times, ecef, xy, args.max_speed)
    times, xy = times[keep], xy[keep]

    fitted = apply_fit(xy, fit_to_map(xy, points, tangents))
    residual, agreement = fit_quality(fitted, points, tangents)
    heading, ok = headings(times, fitted)

    print(f'  GT: 残差 {residual:.3f} m  進行方向一致率 {agreement:.2f}')

    return {'vector_map': vector_map, 'times': times, 'xy': fitted, 'heading': heading, 'ok': ok}


def _new_stats(num_slots):
    return {
        'frames': 0,
        'exist_count': [0] * num_slots,
        'straight_valid_sum': 0,
        'gt_errors': [],
    }


def _update_stats(stats, exist_prob, exist_threshold, straight_valid, gt_errors):
    stats['frames'] += 1
    for slot in range(len(stats['exist_count'])):
        if float(exist_prob[slot]) > exist_threshold:
            stats['exist_count'][slot] += 1
    stats['straight_valid_sum'] += straight_valid
    stats['gt_errors'].extend(gt_errors)


def _print_summary(bag_name, stats, num_rows, width, has_map):
    print(f'{bag_name}: 処理フレーム数 {stats["frames"]}')
    for slot in range(len(stats['exist_count'])):
        rate = stats['exist_count'][slot] / max(stats['frames'], 1)
        name = SLOT_NAMES[slot] if slot < len(SLOT_NAMES) else str(slot)
        print(f'  exist[{name}] 検出率 {rate:.3f}')

    avg_valid = stats['straight_valid_sum'] / max(stats['frames'], 1)
    print(f'  straight 平均有効行数 {avg_valid:.2f} / {num_rows}')

    if has_map:
        errors = stats['gt_errors']
        if errors:
            mean = float(np.mean(errors))
            print(f'  GT横位置誤差 平均 {mean:.4f} (正規化) = {mean * (width - 1):.2f}px  件数 {len(errors)}')
        else:
            print('  GT横位置誤差: GT が作れたフレームがありませんでした')


def _write_video(out_path, images, fps, width, height):
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    for image in images:
        writer.write(image)
    writer.release()

    size = os.path.getsize(out_path) if os.path.exists(out_path) else 0
    if size == 0:
        raise RuntimeError(f'{out_path} が 0 バイトです。cv2.VideoWriter が mp4v コーデックを開けなかった可能性があります')

    capture = cv2.VideoCapture(out_path)
    actual_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    capture.release()

    print(f'動画書き出し: {out_path}  {size} バイト  期待フレーム数 {len(images)}  読み戻しフレーム数 {actual_frames}')
    if actual_frames != len(images):
        print('  警告: 読み戻しフレーム数が一致しません（コンテナのメタデータが不正確な場合があります）')

    return size, actual_frames


def _write_stills(stills_dir, bag_name, records, count):
    os.makedirs(stills_dir, exist_ok=True)
    chosen = select_stills(records, count)
    for index in chosen:
        record = records[index]
        exist_prob = record['exist_prob']
        name = f'{bag_name}_f{record["index"]:06d}_s{exist_prob[0]:.2f}_l{exist_prob[1]:.2f}_r{exist_prob[2]:.2f}.png'
        cv2.imwrite(os.path.join(stills_dir, name), record['image'])
    print(f'静止画書き出し: {len(chosen)} 枚 ({stills_dir})')
    return chosen


def run_bag(args, model=None, reader_factory=None):
    device = torch.device(args.device)
    if model is None:
        model = VisionPlannerNetwork().load_model(args.version, args.weights)
    if hasattr(model, 'to'):
        model = model.to(device)
    if hasattr(model, 'eval'):
        model.eval()

    bag_name = os.path.basename(args.bag.rstrip('/'))
    topics = bag_topics(args.bag)
    source = detect_source(topics)
    if source is None:
        raise ValueError(f'{bag_name}: 画像トピックが見つかりません')
    spec = SOURCES[source]
    params = os.path.join(get_package_share_directory('main_executor'), 'config', 'main_params.yaml')
    camera = load_camera(params, spec['intrinsics'], args.input_height, args.input_width)
    num_rows = args.input_height // 8

    if reader_factory is None:
        import rosbag2_py
        reader_factory = rosbag2_py.SequentialReader

    gt_context = _build_gt_context(args, reader_factory) if args.with_gt else None

    records = []
    stats = _new_stats(NUM_SLOTS)
    frame_count = 0

    for index, stamp, message in read_topic(reader_factory, args.bag, spec['topic'], lambda m: m, args.stride):
        if index < args.start_frame:
            continue
        if args.max_frames is not None and frame_count >= args.max_frames:
            break

        frame = image_to_array(message)
        frame = to_model_frame(frame, spec['convert'], camera, args.input_height, args.input_width)

        with torch.no_grad():
            exist, valid, dist = infer(model, frame, device)

        points, exist_prob, _, positions = analyze_frame(
            exist, valid, dist, args.exist_threshold, args.valid_threshold,
            num_rows, args.input_height, args.input_width)

        gt_points, gt_errors, gt_error_px = [], [], None
        if gt_context is not None:
            pose = interpolate_pose(gt_context['times'], gt_context['xy'], gt_context['heading'], gt_context['ok'], stamp)
            if pose is not None:
                position, angle = pose
                paths = trace_paths(gt_context['vector_map'], position, angle, args.horizon)
                label = build_label(paths, position, angle, camera, num_rows, args.input_height, args.input_width)
                gt_points = select_gt_points(label, num_rows, args.input_height, args.input_width)
                gt_errors = lateral_errors(label, positions)
                if gt_errors:
                    gt_error_px = float(np.mean(gt_errors)) * (args.input_width - 1)

        drawn = frame.copy()
        draw_predictions(drawn, points)
        draw_gt(drawn, gt_points)
        draw_hud(drawn, exist_prob, args.exist_threshold, index, bag_name, args.with_gt, gt_error_px)

        straight_valid = sum(1 for p in points if p['slot'] == SLOT_NAMES.index('straight'))
        branch_score = max(float(exist_prob[SLOT_NAMES.index('left')]), float(exist_prob[SLOT_NAMES.index('right')]))

        records.append({'index': index, 'image': drawn,
                        'exist_prob': [float(v) for v in exist_prob],
                        'straight_valid': straight_valid, 'branch_score': branch_score})

        _update_stats(stats, exist_prob, args.exist_threshold, straight_valid, gt_errors)

        frame_count += 1

    intro_lines = []
    if args.title or args.subtitle:
        intro_lines.append(f'bag: {bag_name}')
        intro_lines.append(f'weights: {args.weights}')
        if args.title:
            intro_lines.append(args.title)
        if args.subtitle:
            intro_lines.append(args.subtitle)
    intro_frames = build_title_frames(args.input_width, args.input_height, args.fps, intro_lines)

    video_frames = intro_frames + [record['image'] for record in records]
    _write_video(args.out_video, video_frames, args.fps, args.input_width, args.input_height)

    if args.stills_dir and args.stills > 0:
        _write_stills(args.stills_dir, bag_name, records, args.stills)

    _print_summary(bag_name, stats, num_rows, args.input_width, args.with_gt)

    return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--out-video', required=True)
    parser.add_argument('--version', default='n')
    parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--stride', type=int, default=2)
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--max-frames', type=int, default=None)
    parser.add_argument('--exist-threshold', type=float, default=0.5)
    parser.add_argument('--valid-threshold', type=float, default=0.5)
    parser.add_argument('--stills-dir', default=None)
    parser.add_argument('--stills', type=int, default=0)
    parser.add_argument('--with-gt', action='store_true')
    parser.add_argument('--start-frame', type=int, default=0)
    parser.add_argument('--input-height', type=int, default=384)
    parser.add_argument('--input-width', type=int, default=640)
    parser.add_argument('--horizon', type=float, default=20.0)
    parser.add_argument('--max-speed', type=float, default=15.0)
    parser.add_argument('--title', default=None)
    parser.add_argument('--subtitle', default=None)
    args = parser.parse_args(argv)

    return run_bag(args)


if __name__ == '__main__':
    sys.exit(main())

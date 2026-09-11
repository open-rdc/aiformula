"""複数の重みを同一 bag に同時に当て、グリッド動画にする。

各パネルに1つの重みの推論結果を描く。同じフレームを横並びで比べられるので、
混合比ごとの挙動の差が直接見える。
"""
import argparse
import os

import cv2
import numpy as np
import rosbag2_py
import torch

from Models.data_parsing.rosbag.converter import SOURCES, bag_topics, detect_source, image_to_array, read_topic
from Models.data_utils.vision_planner.projection import letterbox, load_camera, row_anchor_targets
from Models.model_components.vision_planner.vision_planner_head import decode_positions
from Models.model_components.vision_planner.vision_planner_network import VisionPlannerNetwork

SLOT_NAMES = ('straight', 'left', 'right')
SLOT_COLORS = ((0, 220, 0), (230, 120, 0), (0, 140, 255))
NUM_SLOTS = 3


def load_models(specs, device):
    models = []
    for label, weights in specs:
        checkpoint = torch.load(weights, map_location='cpu', weights_only=True)
        model = VisionPlannerNetwork().load_model(checkpoint['version'], weights).to(device).eval()
        models.append((label, model, checkpoint.get('epoch')))
    return models


def predict(model, frame, device):
    chw = np.ascontiguousarray(frame.transpose(2, 0, 1))
    tensor = torch.from_numpy(chw).float().div(255.0).unsqueeze(0).to(device)
    exist, valid, raw = model(tensor)
    return (torch.sigmoid(exist[0]).cpu(), torch.sigmoid(valid[0]).cpu(),
            decode_positions(raw)[0].cpu())


def draw_panel(frame, exist, valid, positions, label, anchors, width,
               exist_threshold, valid_threshold, scale):
    canvas = frame.copy()
    for slot in range(NUM_SLOTS):
        if float(exist[slot]) <= exist_threshold:
            continue
        for row in range(len(anchors)):
            if float(valid[slot, row]) <= valid_threshold:
                continue
            x = int(float(positions[slot, row]) * (width - 1))
            y = int(anchors[row])
            cv2.circle(canvas, (x, y), 3, SLOT_COLORS[slot], -1)

    canvas = cv2.resize(canvas, scale)
    bar = np.full((26, scale[0], 3), 40, np.uint8)
    cv2.putText(bar, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    tags = '  '.join(f'{SLOT_NAMES[s][0].upper()}{float(exist[s]):.2f}' for s in range(NUM_SLOTS))
    cv2.putText(bar, tags, (scale[0] - 190, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                (180, 220, 180), 1, cv2.LINE_AA)
    return np.vstack([bar, canvas])


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--weights', nargs='+', required=True, help='ラベル=パス の形式')
    parser.add_argument('--out-video', required=True)
    parser.add_argument('--params', default='/home/kyo/formula_ws/src/main_executor/config/main_params.yaml')
    parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--max-frames', type=int, default=None)
    parser.add_argument('--cols', type=int, default=3)
    parser.add_argument('--panel-width', type=int, default=480)
    parser.add_argument('--exist-threshold', type=float, default=0.5)
    parser.add_argument('--valid-threshold', type=float, default=0.5)
    parser.add_argument('--input-height', type=int, default=384)
    parser.add_argument('--input-width', type=int, default=640)
    args = parser.parse_args(argv)

    specs = [tuple(w.split('=', 1)) for w in args.weights]
    models = load_models(specs, args.device)
    print(f'{len(models)} モデル読み込み: ' + ', '.join(f'{l}(ep{e})' for l, _, e in models))

    topics = bag_topics(args.bag)
    source = detect_source(topics)
    spec = SOURCES[source]
    camera = load_camera(args.params, spec['intrinsics'], args.input_height, args.input_width)
    anchors = row_anchor_targets(args.input_height // 8, args.input_height)
    panel_scale = (args.panel_width, int(args.panel_width * args.input_height / args.input_width))

    writer = None
    count = 0
    with torch.no_grad():
        for index, stamp, message in read_topic(rosbag2_py.SequentialReader, args.bag, spec['topic'], lambda m: m, args.stride):
            frame = image_to_array(message)
            if spec['convert'] is not None:
                frame = spec['convert'](frame)
            frame = letterbox(frame, args.input_height, args.input_width, camera.pad_top)

            panels = [draw_panel(frame, *predict(model, frame, args.device), label, anchors,
                                 args.input_width, args.exist_threshold, args.valid_threshold,
                                 panel_scale)
                      for label, model, _ in models]
            while len(panels) % args.cols:
                panels.append(np.zeros_like(panels[0]))
            rows = [np.hstack(panels[i:i + args.cols]) for i in range(0, len(panels), args.cols)]
            grid = np.vstack(rows)

            if writer is None:
                os.makedirs(os.path.dirname(args.out_video) or '.', exist_ok=True)
                writer = cv2.VideoWriter(args.out_video, cv2.VideoWriter_fourcc(*'mp4v'),
                                         args.fps, (grid.shape[1], grid.shape[0]))
            writer.write(grid)
            count += 1
            if count % 500 == 0:
                print(f'  {count} フレーム')
            if args.max_frames and count >= args.max_frames:
                break
    if writer is not None:
        writer.release()

    size = os.path.getsize(args.out_video)
    print(f'{args.out_video}  {size:,} バイト  {count} フレーム  {grid.shape[1]}x{grid.shape[0]}')


if __name__ == '__main__':
    main()

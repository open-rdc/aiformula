import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch

from Models.vector_planner.data import geo_aug
from Models.vector_planner.data.dataset import SLOTS, split_datasets
from Models.vector_planner.model.network import load_network
from Models.vector_planner.training.trainer import parse_data_spec

MAP_X_RANGE, MAP_Y_MIN, MAP_Y_SPAN = 25.0, -8.0, 16.0
SLOT_COLORS = {'straight': (0, 200, 0), 'left': (255, 140, 0), 'right': (0, 0, 255)}
EXIST_THRESHOLD, MAP_CONF, BOX_CONF = 0.5, 0.5, 0.3
WHITE, BOX_COLOR = (255, 255, 255), (0, 255, 0)
VIEW_WIDTH = 640


def _map_lines(pred):
    prob = np.exp(pred['map_logits'] - pred['map_logits'].max(-1, keepdims=True))
    prob /= prob.sum(-1, keepdims=True)
    keep = (prob.argmax(-1) < 3) & (prob.max(-1) >= MAP_CONF)
    lines = pred['map_points'][keep].copy()
    lines[..., 0] *= MAP_X_RANGE
    lines[..., 1] = lines[..., 1] * MAP_Y_SPAN + MAP_Y_MIN
    return lines


def _draw_path(image, camera, xy, color, scale):
    pixels, visible = camera.project(np.concatenate([xy, np.zeros((len(xy), 1))], axis=1))
    pixels = np.round(pixels * scale).astype(int)
    for a in range(len(pixels) - 1):
        if visible[a] and visible[a + 1]:
            cv2.line(image, tuple(pixels[a]), tuple(pixels[a + 1]), color, 3, cv2.LINE_AA)


def _bev_pixels(xy, height):
    ppm = height / MAP_X_RANGE
    width = MAP_Y_SPAN * ppm
    return np.round(np.stack([width / 2 - xy[:, 1] * ppm, height - xy[:, 0] * ppm], 1)).astype(np.int32)


def render_frame(image, pred, camera):
    """左: カメラ（経路・box）、右: 推論 BEV（地図線・走行帯・経路）。BGR を返す。"""
    h, w = image.shape[:2]
    view = cv2.resize(image, (VIEW_WIDTH, int(round(h * VIEW_WIDTH / w))))
    scale = VIEW_WIDTH / geo_aug.REF_WIDTH
    height = view.shape[0]
    bev = np.full((height, int(round(MAP_Y_SPAN * height / MAP_X_RANGE)), 3), 30, np.uint8)
    exist = np.asarray(pred['branch_exist']) >= 0  # sigmoid >= 0.5

    for line in _map_lines(pred):
        cv2.polylines(bev, [_bev_pixels(line, height)], False, WHITE, 2, cv2.LINE_AA)
    for i, slot in enumerate(SLOTS):
        if not exist[i]:
            continue
        color = SLOT_COLORS[slot]
        ring = _bev_pixels(np.concatenate([pred['corridor_left'][i], pred['corridor_right'][i][::-1]]), height)
        overlay = bev.copy()
        cv2.fillPoly(overlay, [ring], color)
        bev = cv2.addWeighted(overlay, 0.35, bev, 0.65, 0)
        cv2.polylines(bev, [_bev_pixels(pred['path_xy'][i], height)], False, color, 2, cv2.LINE_AA)
        _draw_path(view, camera, pred['path_xy'][i], color, scale)
    for cx, cy, bw, bh in np.asarray(pred['det_boxes'])[np.asarray(pred['det_scores']) >= BOX_CONF]:
        cv2.rectangle(view, (int((cx - bw / 2) * VIEW_WIDTH), int((cy - bh / 2) * height)),
                      (int((cx + bw / 2) * VIEW_WIDTH), int((cy + bh / 2) * height)), BOX_COLOR, 2)
    return np.ascontiguousarray(np.concatenate([view, bev], axis=1))


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--data', action='append', required=True, help='IMAGE_ROOT:LABEL_ROOT（ラベルは画像の列挙にのみ使う）')
    p.add_argument('--split-json', required=True)
    p.add_argument('--split', default='holdout')
    p.add_argument('--out', required=True)
    p.add_argument('--max-frames', type=int, default=None)
    p.add_argument('--fps', type=float, default=5.0)
    args = p.parse_args(argv)

    camera = geo_aug.load_aug_camera()
    torch.set_num_threads(2)
    model, ckpt = load_network(args.checkpoint, 'cpu')
    saved = ckpt.get('args') or {}
    size = (saved.get('input_height', 384), saved.get('input_width', 640))
    split_map = json.loads(Path(args.split_json).read_text())
    datasets = split_datasets([parse_data_spec(s) for s in args.data], split_map, args.split, size)

    writer, count = None, 0
    for ds in datasets:
        for index, stem in enumerate(ds.stems):
            if '__p' in stem:
                continue
            if args.max_frames is not None and count >= args.max_frames:
                break
            image = ds[index]['image']
            with torch.no_grad():
                out = model(image[None].float() / 255.0)
            frame = render_frame(image.permute(1, 2, 0).numpy().copy(), {k: v[0].float().numpy() for k, v in out.items()}, camera)
            if writer is None:
                writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, frame.shape[1::-1])
            writer.write(frame)
            count += 1
    if writer is None:
        raise ValueError('描画できるフレームがありません')
    writer.release()
    return count


if __name__ == '__main__':
    main()

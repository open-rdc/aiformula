#!/usr/bin/env python3
"""学習データの白線マスク (mask_images/) を YOLOPv2 で作る。

create_data_from_bag.py の出力 (images/) に、元の学習データ (e2e_train_20260928_rgb_mask) と
同じ形式のマスク (640x360、白線を赤 (0, 0, 255) で塗った BGR、それ以外は黒) を書く。
YOLOPv2Processor.process_image(img, (640, 360)) をそのまま使い、元のデータの 16_44_53 と画素単位で一致する。

    python3 scripts/create_lane_masks.py --model weights/yolopv2.pt --dataset <dir> [<dir> ...] [--device cuda]
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import torch

from util.yolop_processor import YOLOPv2Processor

MASK_SIZE = (640, 360)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', type=Path, required=True)
    ap.add_argument('--dataset', type=Path, nargs='+', required=True)
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    ap.add_argument('--overwrite', action='store_true')
    args = ap.parse_args()

    processor = YOLOPv2Processor(args.model, torch.device(args.device))
    for dataset in args.dataset:
        out_dir = dataset / 'mask_images'
        out_dir.mkdir(exist_ok=True)
        images = sorted((dataset / 'images').glob('*.png'))
        written = 0
        for image_path in images:
            out = out_dir / image_path.name
            if out.exists() and not args.overwrite:
                continue
            lane = processor.process_image(cv2.imread(str(image_path)), MASK_SIZE)
            mask = np.zeros((MASK_SIZE[1], MASK_SIZE[0], 3), np.uint8)
            mask[lane > 0] = (0, 0, 255)
            cv2.imwrite(str(out), mask)
            written += 1
        print(f'{dataset.name}: {written} 枚 (images {len(images)})', flush=True)


if __name__ == '__main__':
    main()

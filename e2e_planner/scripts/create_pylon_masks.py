#!/usr/bin/env python3
"""学習データの各フレームにパイロンマスク (pylon_masks/<stem>.png, 640x360 mono8) を作る。

    python3 create_pylon_masks.py --onnx ../weights/yolox_s_aiformula.onnx --dataset <root>

root は train_curve_weight.py の --dataset と同じもの（images/ を持つディレクトリか、その親）。
"""

import argparse
from pathlib import Path

import cv2
from tqdm import tqdm

from train_curve_weight import find_datasets
from util.pylon_mask import YoloxOnnx, draw_pylon_mask


def create_masks(onnx_path: str, root: Path, overwrite: bool = False) -> int:
    detector = YoloxOnnx(onnx_path)
    written = 0
    for dataset in find_datasets(Path(root)):
        out_dir = dataset / 'pylon_masks'
        out_dir.mkdir(exist_ok=True)
        for image_file in tqdm(sorted((dataset / 'images').glob('*.png')), desc=dataset.name):
            out_file = out_dir / image_file.name
            if out_file.exists() and not overwrite:
                continue
            cv2.imwrite(str(out_file), draw_pylon_mask(detector.detect(cv2.imread(str(image_file)))))
            written += 1
    return written


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--onnx', required=True)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    print(f'{create_masks(args.onnx, Path(args.dataset), args.overwrite)} 枚書いた')


if __name__ == '__main__':
    main()

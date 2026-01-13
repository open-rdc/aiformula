#!/usr/bin/env python3

import sys
import cv2
import torch
import numpy as np
from pathlib import Path
from tqdm import tqdm

sys.path.append(str(Path(__file__).parent.parent / 'e2e_planner' / 'scripts'))
from util.yolop_processor import YOLOPv2Processor


def process_dataset(dataset_path: Path, yolop_weight_path: Path) -> None:
    images_dir = dataset_path / 'images'
    mask_images_dir = dataset_path / 'mask_images'

    if not images_dir.exists():
        print(f'Error: images directory not found: {images_dir}')
        sys.exit(1)

    mask_images_dir.mkdir(exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'Using device: {device}')

    yolop_processor = YOLOPv2Processor(yolop_weight_path, device)

    image_files = sorted(list(images_dir.glob('*.png')))
    print(f'Found {len(image_files)} images')

    for image_file in tqdm(image_files, desc='Processing images'):
        image = cv2.imread(str(image_file))

        if image is None:
            print(f'Warning: Failed to load {image_file}')
            continue

        original_size = (image.shape[1], image.shape[0])
        mask = yolop_processor.process_image(image, original_size)

        color_mask = cv2.cvtColor(np.zeros((mask.shape[0], mask.shape[1]), dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        color_mask[mask == 1] = [0, 0, 255]

        mask_output_path = mask_images_dir / image_file.name
        cv2.imwrite(str(mask_output_path), color_mask)

    print(f'Mask images saved to: {mask_images_dir}')


def main() -> None:
    if len(sys.argv) != 2:
        print('Usage: python3 e2e_add_yolopv2_data.py <dataset_path>')
        sys.exit(1)

    dataset_path = Path(sys.argv[1])
    if not dataset_path.exists():
        print(f'Error: Dataset path does not exist: {dataset_path}')
        sys.exit(1)

    yolop_weight_path = Path(__file__).parent.parent / 'e2e_planner' / 'weights' / 'yolopv2.pt'
    if not yolop_weight_path.exists():
        print(f'Error: YOLOPv2 weight file not found: {yolop_weight_path}')
        sys.exit(1)

    process_dataset(dataset_path, yolop_weight_path)
    print('Done!')


if __name__ == '__main__':
    main()

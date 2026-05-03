#!/usr/bin/env python3
"""
収集したRGB画像にYOLOPを適用して二値マスクを生成する。

使い方:
  python3 binarize_dataset.py --session ~/ros2_ws/dataset/session_YYYYMMDD_HHMMSS

出力:
  session/masks/XXXXXX.jpg  (64x48 二値マスク、0 or 255)
  session/data.csv に mask カラムを追加（data_binarized.csv として保存）
  data.csv がない sim/create_data 形式では session/mask_images/XXXXX.png を生成
"""

import argparse
import sys
import cv2
import numpy as np
import pandas as pd
import torch
from pathlib import Path
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).parent))
from util.yolop_processor import YOLOPv2Processor


def resolve_weights_path(weights_arg):
    if weights_arg:
        return Path(weights_arg)

    candidates = [
        Path.home() / 'ros2_ws/install/e2e_planner/share/e2e_planner/weights/yolopv2.pt',
        Path(__file__).parent.parent / 'weights/yolopv2.pt',
    ]
    return next((p for p in candidates if p.exists()), None)


def create_processor(weights_path):
    if weights_path is None:
        print('エラー: yolopv2.pt が見つかりません。--weights で指定してください')
        sys.exit(1)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'デバイス: {device}')
    print('YOLOPを読み込み中...')
    return YOLOPv2Processor(weights_path, device)


def warmup_processor(proc, mask_size):
    dummy = np.zeros((300, 480, 3), dtype=np.uint8)
    proc.process_image(dummy, tuple(mask_size))
    print('ウォームアップ完了')


def binarize_image(proc, img, mask_size):
    mask = proc.process_image(img, tuple(mask_size))  # 0 or 1, int32
    return (mask * 255).astype(np.uint8)


def process_csv_dataset(session_dir, proc, mask_size, overwrite):
    mask_dir = session_dir / 'masks'
    mask_dir.mkdir(exist_ok=True)

    csv_path = session_dir / 'data.csv'
    df = pd.read_csv(csv_path)
    mask_files = []

    print(f'{len(df)} フレームを処理中...')
    for _, row in tqdm(df.iterrows(), total=len(df)):
        mask_filename = Path(row['image']).stem + '.jpg'
        mask_path = mask_dir / mask_filename
        mask_relpath = f'masks/{mask_filename}'
        if mask_path.exists() and not overwrite:
            mask_files.append(mask_relpath)
            continue

        img_path = session_dir / row['image']
        img = cv2.imread(str(img_path))
        if img is None:
            mask_files.append('')
            continue

        mask_img = binarize_image(proc, img, mask_size)

        cv2.imwrite(str(mask_path), mask_img)
        mask_files.append(mask_relpath)

    df['mask'] = mask_files
    out_csv = session_dir / 'data_binarized.csv'
    df.to_csv(out_csv, index=False)
    print(f'完了。{out_csv} に保存しました')


def process_split_dataset(session_dir, proc, mask_size, overwrite):
    images_dir = session_dir / 'images'
    mask_images_dir = session_dir / 'mask_images'
    mask_images_dir.mkdir(exist_ok=True)

    image_files = sorted(
        p for p in images_dir.iterdir()
        if p.suffix.lower() in {'.jpg', '.jpeg', '.png'}
    )
    if not image_files:
        print(f'エラー: {images_dir} に画像が見つかりません')
        sys.exit(1)

    print(f'{len(image_files)} フレームを処理中...')
    for img_path in tqdm(image_files):
        mask_path = mask_images_dir / f'{img_path.stem}.png'
        if mask_path.exists() and not overwrite:
            continue

        img = cv2.imread(str(img_path))
        if img is None:
            continue

        mask_img = binarize_image(proc, img, mask_size)
        cv2.imwrite(str(mask_path), mask_img)

    print(f'完了。{mask_images_dir} に保存しました')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True, help='セッションディレクトリのパス')
    parser.add_argument('--weights', default=None, help='yolopv2.pt のパス（省略時は自動検索）')
    parser.add_argument('--mask-size', nargs=2, type=int, default=[64, 48],
                        help='マスクサイズ [width height] (デフォルト: 64 48)')
    parser.add_argument('--overwrite', action='store_true',
                        help='既存のマスクも再生成する')
    args = parser.parse_args()

    session_dir = Path(args.session)
    csv_path = session_dir / 'data.csv'
    images_dir = session_dir / 'images'
    if not csv_path.exists() and not images_dir.is_dir():
        print(f'エラー: {csv_path} が見つからず、{images_dir} も存在しません')
        sys.exit(1)

    # YOLOPウェイト自動検索
    weights_path = resolve_weights_path(args.weights)
    proc = create_processor(weights_path)
    warmup_processor(proc, args.mask_size)

    if csv_path.exists():
        process_csv_dataset(session_dir, proc, args.mask_size, args.overwrite)
    else:
        process_split_dataset(session_dir, proc, args.mask_size, args.overwrite)


if __name__ == '__main__':
    main()

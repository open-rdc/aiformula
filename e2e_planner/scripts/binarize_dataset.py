#!/usr/bin/env python3
"""
収集したRGB画像にYOLOPを適用して二値マスクを生成する。

使い方:
  python3 binarize_dataset.py --session ~/ros2_ws/dataset/session_YYYYMMDD_HHMMSS

出力:
  session/masks/XXXXXX.jpg  (64x48 二値マスク、0 or 255)
  session/data.csv に mask カラムを追加（data_binarized.csv として保存）
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True, help='セッションディレクトリのパス')
    parser.add_argument('--weights', default=None, help='yolopv2.pt のパス（省略時は自動検索）')
    parser.add_argument('--mask-size', nargs=2, type=int, default=[64, 48],
                        help='マスクサイズ [width height] (デフォルト: 64 48)')
    args = parser.parse_args()

    session_dir = Path(args.session)
    mask_dir = session_dir / 'masks'
    mask_dir.mkdir(exist_ok=True)

    csv_path = session_dir / 'data.csv'
    if not csv_path.exists():
        print(f'エラー: {csv_path} が見つかりません')
        sys.exit(1)

    # YOLOPウェイト自動検索
    if args.weights:
        weights_path = Path(args.weights)
    else:
        candidates = [
            Path.home() / 'ros2_ws/install/e2e_planner/share/e2e_planner/weights/yolopv2.pt',
            Path(__file__).parent.parent / 'weights/yolopv2.pt',
        ]
        weights_path = next((p for p in candidates if p.exists()), None)
        if weights_path is None:
            print('エラー: yolopv2.pt が見つかりません。--weights で指定してください')
            sys.exit(1)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'デバイス: {device}')
    print('YOLOPを読み込み中...')
    proc = YOLOPv2Processor(weights_path, device)

    # ウォームアップ
    dummy = np.zeros((300, 480, 3), dtype=np.uint8)
    proc.process_image(dummy, tuple(args.mask_size))
    print('ウォームアップ完了')

    df = pd.read_csv(csv_path)
    mask_files = []

    print(f'{len(df)} フレームを処理中...')
    for _, row in tqdm(df.iterrows(), total=len(df)):
        img_path = session_dir / row['image']
        img = cv2.imread(str(img_path))
        if img is None:
            mask_files.append('')
            continue

        mask = proc.process_image(img, tuple(args.mask_size))  # 0 or 1, int32
        mask_img = (mask * 255).astype(np.uint8)

        mask_filename = Path(row['image']).stem + '.jpg'
        mask_path = mask_dir / mask_filename
        cv2.imwrite(str(mask_path), mask_img)
        mask_files.append(f'masks/{mask_filename}')

    df['mask'] = mask_files
    out_csv = session_dir / 'data_binarized.csv'
    df.to_csv(out_csv, index=False)
    print(f'完了。{out_csv} に保存しました')


if __name__ == '__main__':
    main()

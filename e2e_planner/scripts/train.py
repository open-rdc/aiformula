#!/usr/bin/env python3

import sys
import time
import yaml
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, Subset, random_split
from torch.utils.tensorboard import SummaryWriter
import cv2
import csv
from pathlib import Path
import numpy as np
from typing import Tuple
from tqdm import tqdm
from network import Network
from util.preprocess import (
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    extract_red_mask,
    preprocess_mask,
    preprocess_rgb,
    normalize_waypoints,
)
from util.augment_train import horizontal_flip, color_jitter, to_grayscale

NUM_WAYPOINTS = 20

# train/val の分割を run 間で固定する。拡張の有無で val が変わると比較できない
SPLIT_SEED = 42


class E2EDataset(Dataset):
    """白線マスク画像と対応するRGB画像のペアを返す"""

    def __init__(self, dataset_path: Path, augment_config: dict = None):
        # augment_config が None のときは拡張なし（検証用）
        self.augment_config = augment_config
        self.dataset_path = dataset_path
        self.mask_images_dir = dataset_path / 'mask_images'
        self.rgb_images_dir = dataset_path / 'images'
        self.path_dir = dataset_path / 'path'

        if not self.rgb_images_dir.exists():
            raise FileNotFoundError(f'RGB images directory not found: {self.rgb_images_dir}')

        self.mask_files = [
            mask_file for mask_file in sorted(self.mask_images_dir.glob('*.png'))
            if (self.rgb_images_dir / mask_file.name).exists()
            and (self.path_dir / f'{mask_file.stem}.csv').exists()
        ]

    def __len__(self) -> int:
        return len(self.mask_files)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        mask_file = self.mask_files[idx]
        rgb_file = self.rgb_images_dir / mask_file.name
        csv_file = self.path_dir / f'{mask_file.stem}.csv'

        mask_bgr = cv2.imread(str(mask_file), cv2.IMREAD_COLOR)
        rgb_bgr = cv2.imread(str(rgb_file), cv2.IMREAD_COLOR)
        if mask_bgr is None or rgb_bgr is None:
            raise RuntimeError(f'Failed to load image pair: {mask_file.name}')

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            waypoints = [[float(row['x']), float(row['y'])] for row in reader]

        mask_binary = extract_red_mask(mask_bgr)
        waypoints_m = np.array(waypoints, dtype=np.float32)

        if self.augment_config is not None:
            # numpy のグローバル RNG は DataLoader の worker 間で再シードされないため、
            # 全 worker が同じ乱数列を出してしまう。torch の RNG は worker ごとに
            # 正しくずらされるので、そこから種を引いて per-sample の Generator を作る
            rng = np.random.default_rng(int(torch.randint(0, 2**31 - 1, (1,)).item()))
            cfg = self.augment_config

            if rng.random() < cfg['flip_prob']:
                rgb_bgr, mask_binary, waypoints_m = horizontal_flip(
                    rgb_bgr, mask_binary, waypoints_m)

            if rng.random() < cfg['color_prob']:
                rgb_bgr = color_jitter(
                    rgb_bgr, rng,
                    brightness=cfg['brightness'],
                    contrast=cfg['contrast'],
                    saturation=cfg['saturation'],
                    hue=cfg['hue'],
                )

            # 色そのものを落とす。色を手がかりにできない状態でも走れるようにする
            if rng.random() < cfg['gray_prob']:
                rgb_bgr = to_grayscale(rgb_bgr)

        mask_tensor = torch.from_numpy(preprocess_mask(mask_binary))
        rgb_tensor = torch.from_numpy(preprocess_rgb(rgb_bgr))
        waypoints_tensor = torch.from_numpy(normalize_waypoints(waypoints_m))

        return mask_tensor, rgb_tensor, waypoints_tensor

class Config:
    def __init__(self, config_path: Path, package_root: Path, run_name: str):
        self.package_root = package_root
        self.run_name = run_name

        with open(config_path, 'r') as f:
            config_dict = yaml.safe_load(f)

        self.epochs = config_dict['epochs']
        self.batch_size = config_dict['batch_size']
        self.learning_rate = config_dict['learning_rate']
        self.num_workers = config_dict['num_workers']
        # 重みも run 名で分ける。run のたびに e2e_model.pt が上書きされて
        # 過去の学習結果が失われる事故を防ぐ
        base = Path(config_dict['weight_file'])
        self.weight_file = f'{base.stem}_{run_name}{base.suffix}'

        self.embed_dim = config_dict.get('embed_dim', 128)
        self.num_heads = config_dict.get('num_heads', 4)
        self.num_layers = config_dict.get('num_layers', 4)
        self.dropout = config_dict.get('dropout', 0.1)
        self.grad_clip_norm = config_dict.get('grad_clip_norm', 1.0)

        # データ拡張（学習側のみ。検証には適用しない）
        self.augment_config = {
            # 左右反転は既定で無効。コースは反時計回りで左折が右折の 4.86 倍あり、
            # 反転すると分岐で曲がる向きが競合する
            'flip_prob': config_dict.get('augment_flip_prob', 0.0),
            'color_prob': config_dict.get('augment_color_prob', 0.8),
            'brightness': config_dict.get('augment_brightness', 0.3),
            'contrast': config_dict.get('augment_contrast', 0.3),
            'saturation': config_dict.get('augment_saturation', 0.3),
            'hue': config_dict.get('augment_hue', 0.03),
            'gray_prob': config_dict.get('augment_gray_prob', 0.3),
        }
        self.augment_enabled = config_dict.get('augment', True)

        # train/val の分け方。'temporal' は時系列で後半を val にする（既定）。
        # 'random' は従来どおりランダム分割だが、隣接フレームのリークがある
        self.split_mode = config_dict.get('split_mode', 'temporal')
        self.split_gap = config_dict.get('split_gap', 25)

        self.weights_dir = package_root / 'weights'
        self.weights_dir.mkdir(exist_ok=True)

        # run ごとにサブディレクトリを切る。runs/ 直下に直接書くと
        # TensorBoard が全 run を "." という 1 つの run に合流させてしまう
        self.logs_dir = package_root / 'runs' / run_name

        self.device = torch.device('cuda')

class Trainer:
    def __init__(self, dataset_path: Path, config: Config):
        self.config = config
        self.device = config.device

        # random_split の Subset は元の Dataset を共有するので、そのままだと
        # 検証にも拡張がかかる。拡張あり/なしの 2 インスタンスを作り、
        # 同じ index で Subset を組み直す
        aug_cfg = config.augment_config if config.augment_enabled else None
        train_source = E2EDataset(dataset_path, augment_config=aug_cfg)
        val_source = E2EDataset(dataset_path, augment_config=None)

        dataset = val_source
        n = len(dataset)
        train_size = int(0.8 * n)

        if config.split_mode == 'temporal':
            # ファイル名は収集順の連番なので index 順 = 時系列順。
            # random_split だと 0.2s しか離れていないほぼ同一フレームが train と val に
            # 分かれて入り、val loss が汎化性能を過大評価する（リーク）。
            # 前半 80% を train、後半を val にして時間で切り離す。
            # さらに境界に gap を空ける: waypoint は 2.5s 先まで見るので、
            # 境界付近のフレームは train と未来の軌跡を共有してしまう
            gap = config.split_gap
            train_indices = list(range(train_size))
            val_indices = list(range(min(train_size + gap, n), n))
        else:
            split_generator = torch.Generator().manual_seed(SPLIT_SEED)
            tr_idx, va_idx = random_split(
                range(n), [train_size, n - train_size], generator=split_generator)
            train_indices, val_indices = list(tr_idx), list(va_idx)

        train_dataset = Subset(train_source, train_indices)
        val_dataset = Subset(val_source, val_indices)

        self.train_loader = DataLoader(
            train_dataset,
            batch_size=config.batch_size,
            shuffle=True,
            num_workers=config.num_workers
        )
        self.val_loader = DataLoader(
            val_dataset,
            batch_size=config.batch_size,
            shuffle=False,
            num_workers=config.num_workers
        )

        self.model = Network(
            num_waypoints=NUM_WAYPOINTS,
            image_height=IMAGE_HEIGHT,
            image_width=IMAGE_WIDTH,
            embed_dim=config.embed_dim,
            num_heads=config.num_heads,
            num_layers=config.num_layers,
            dropout=config.dropout,
        ).to(self.device)
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=config.learning_rate)
        self.mseloss = nn.MSELoss()
        self.writer = SummaryWriter(log_dir=str(config.logs_dir))

        self.best_val_loss = float('inf')

        print(f'Using device: {self.device}')
        print(f'Dataset: {len(dataset)} mask/RGB pairs')
        print(f'Train size: {len(train_dataset)}, Val size: {len(val_dataset)}')
        print(f'Augment (train only): {aug_cfg if aug_cfg else "disabled"}')
        print(f'Split: {config.split_mode}' +
              (f' (gap={config.split_gap} samples)' if config.split_mode == 'temporal' else ''))

    def validate(self) -> float:
        self.model.eval()
        total_loss = 0.0

        with torch.no_grad():
            pbar = tqdm(self.val_loader, desc='Validation')
            for masks, rgbs, waypoints in pbar:
                masks = masks.to(self.device)
                rgbs = rgbs.to(self.device)
                waypoints = waypoints.to(self.device)

                outputs = self.model(masks, rgbs)

                loss = self.mseloss(outputs, waypoints)
                total_loss += loss.item()
                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

        return total_loss / len(self.val_loader)

    def save_checkpoint(self, val_loss: float) -> None:
        if val_loss < self.best_val_loss:
            self.best_val_loss = val_loss
            weight_path = self.config.weights_dir / self.config.weight_file
            was_training = self.model.training
            self.model.eval()
            scripted_model = torch.jit.script(self.model)
            scripted_model.save(str(weight_path))
            self.model.train(was_training)
            print(f'Best model saved: {weight_path} (val_loss: {val_loss:.6f})')

    def train(self, epochs: int) -> None:
        for epoch in range(1, epochs + 1):
            self.model.train()
            total_train_loss = 0.0
            total_grad_norm = 0.0

            pbar = tqdm(self.train_loader, desc=f'Epoch {epoch} [Train]')
            for masks, rgbs, waypoints in pbar:
                masks = masks.to(self.device)
                rgbs = rgbs.to(self.device)
                waypoints = waypoints.to(self.device)

                self.optimizer.zero_grad()
                outputs = self.model(masks, rgbs)

                loss = self.mseloss(outputs, waypoints)
                loss.backward()

                # 勾配クリッピング。外れ値サンプルで学習が発散するのを防ぐ
                # （clip 無しでは epoch 60 付近で train 0.005 -> 0.050 に飛んだ）
                grad_norm = torch.nn.utils.clip_grad_norm_(
                    self.model.parameters(), self.config.grad_clip_norm
                )
                total_grad_norm += grad_norm.item()

                self.optimizer.step()

                total_train_loss += loss.item()
                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

            train_loss = total_train_loss / len(self.train_loader)
            val_loss = self.validate()

            self.writer.add_scalar('Loss/train', train_loss, epoch)
            self.writer.add_scalar('Loss/val', val_loss, epoch)
            self.writer.add_scalar('GradNorm/train', total_grad_norm / len(self.train_loader), epoch)

            print(f'Epoch [{epoch}/{epochs}], Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}')

            self.save_checkpoint(val_loss)

        self.writer.close()

def main() -> None:
    if len(sys.argv) not in (2, 3):
        print('Usage: python3 train.py <dataset_path> [run_name]')
        sys.exit(1)

    dataset_path = Path(sys.argv[1])
    if not dataset_path.exists():
        print(f'Dataset path does not exist: {dataset_path}')
        sys.exit(1)

    run_name = sys.argv[2] if len(sys.argv) == 3 else time.strftime('%Y%m%d_%H%M%S')

    script_dir = Path(__file__).parent
    package_root = script_dir.parent
    config_path = package_root / 'config' / 'train.yaml'

    config = Config(config_path, package_root, run_name)
    trainer = Trainer(dataset_path, config)

    print(f'Run name: {run_name}')
    print(f'TensorBoard logs: {config.logs_dir}')
    print(f'Weight file: {config.weights_dir / config.weight_file}')
    print(f'Starting training for {config.epochs} epochs')
    trainer.train(config.epochs)

    print('Training complete.')

if __name__ == '__main__':
    main()

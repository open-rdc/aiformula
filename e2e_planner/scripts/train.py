#!/usr/bin/env python3

import sys
import time
import yaml
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
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

NUM_WAYPOINTS = 10


class E2EDataset(Dataset):
    """白線マスク画像と対応するRGB画像のペアを返す"""

    def __init__(self, dataset_path: Path):
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

        mask_tensor = torch.from_numpy(preprocess_mask(extract_red_mask(mask_bgr)))
        rgb_tensor = torch.from_numpy(preprocess_rgb(rgb_bgr))
        waypoints_tensor = torch.from_numpy(normalize_waypoints(np.array(waypoints, dtype=np.float32)))

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

        dataset = E2EDataset(dataset_path)
        train_size = int(0.8 * len(dataset))
        val_size = len(dataset) - train_size
        train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

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

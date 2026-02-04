#!/usr/bin/env python3

import sys
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
from schedulefree import RAdamScheduleFree
from network import Network

IMAGE_WIDTH = 64
IMAGE_HEIGHT = 48
NUM_WAYPOINTS = 10


class E2EDataset(Dataset):
    def __init__(self, dataset_path: Path):
        self.dataset_path = dataset_path
        self.mask_images_dir = dataset_path / 'mask_images'
        self.path_dir = dataset_path / 'path'
        self.mask_files = sorted(list(self.mask_images_dir.glob('*.png')))

    def __len__(self) -> int:
        return len(self.mask_files)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        mask_file = self.mask_files[idx]
        csv_file = self.path_dir / f'{mask_file.stem}.csv'

        mask_bgr = cv2.imread(str(mask_file), cv2.IMREAD_COLOR)

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            waypoints = [[float(row['x']), float(row['y'])] for row in reader]

        mask_binary = ((mask_bgr[:, :, 2] > 200) & (mask_bgr[:, :, 0] < 50) & (mask_bgr[:, :, 1] < 50)).astype(np.uint8)

        cropped_mask = mask_binary[:, 40:440]
        resized_mask = cv2.resize(cropped_mask, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_NEAREST)

        mask_normalized = resized_mask.astype(np.float32)
        mask_tensor = torch.from_numpy(mask_normalized).unsqueeze(0)

        waypoints_tensor = torch.tensor(waypoints, dtype=torch.float32).flatten()
        waypoints_tensor[0::2] = waypoints_tensor[0::2] / 5.0 - 1.0
        waypoints_tensor[1::2] = (waypoints_tensor[1::2] + 3.0) / 3.0 - 1.0

        return mask_tensor, waypoints_tensor

class Config:
    def __init__(self, config_path: Path, package_root: Path):
        self.package_root = package_root

        with open(config_path, 'r') as f:
            config_dict = yaml.safe_load(f)

        self.epochs = config_dict['epochs']
        self.batch_size = config_dict['batch_size']
        self.learning_rate = config_dict['learning_rate']
        self.num_workers = config_dict['num_workers']
        self.weight_file = config_dict['weight_file']

        self.weights_dir = package_root / 'weights'
        self.weights_dir.mkdir(exist_ok=True)

        self.logs_dir = package_root / 'runs'

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

        self.model = Network(num_waypoints=NUM_WAYPOINTS).to(self.device)
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=config.learning_rate)
        self.mseloss = nn.MSELoss()
        self.writer = SummaryWriter(log_dir=str(config.logs_dir))

        self.best_val_loss = float('inf')

        print(f'Using device: {self.device}')
        print(f'Train size: {len(train_dataset)}, Val size: {len(val_dataset)}')

    def validate(self) -> float:
        self.model.eval()
        total_loss = 0.0

        with torch.no_grad():
            pbar = tqdm(self.val_loader, desc='Validation')
            for images, waypoints in pbar:
                images = images.to(self.device)
                waypoints = waypoints.to(self.device)

                outputs = self.model(images)

                loss = self.mseloss(outputs, waypoints)
                total_loss += loss.item()
                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

        return total_loss / len(self.val_loader)

    def save_checkpoint(self, val_loss: float) -> None:
        if val_loss < self.best_val_loss:
            self.best_val_loss = val_loss
            weight_path = self.config.weights_dir / self.config.weight_file
            scripted_model = torch.jit.script(self.model)
            scripted_model.save(str(weight_path))
            print(f'Best model saved: {weight_path} (val_loss: {val_loss:.6f})')

    def train(self, epochs: int) -> None:
        for epoch in range(1, epochs + 1):
            self.model.train()
            total_train_loss = 0.0

            pbar = tqdm(self.train_loader, desc=f'Epoch {epoch} [Train]')
            for images, waypoints in pbar:
                images = images.to(self.device)
                waypoints = waypoints.to(self.device)

                self.optimizer.zero_grad()
                outputs = self.model(images)

                loss = self.mseloss(outputs, waypoints)
                loss.backward()
                self.optimizer.step()

                total_train_loss += loss.item()
                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

            train_loss = total_train_loss / len(self.train_loader)
            val_loss = self.validate()

            self.writer.add_scalar('Loss/train', train_loss, epoch)
            self.writer.add_scalar('Loss/val', val_loss, epoch)

            print(f'Epoch [{epoch}/{epochs}], Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}')

            self.save_checkpoint(val_loss)

        self.writer.close()

def main() -> None:
    if len(sys.argv) != 2:
        print('Usage: python3 train.py <dataset_path>')
        sys.exit(1)

    dataset_path = Path(sys.argv[1])
    if not dataset_path.exists():
        print(f'Dataset path does not exist: {dataset_path}')
        sys.exit(1)

    script_dir = Path(__file__).parent
    package_root = script_dir.parent
    config_path = package_root / 'config' / 'train.yaml'

    config = Config(config_path, package_root)
    trainer = Trainer(dataset_path, config)

    print(f'Starting training for {config.epochs} epochs')
    trainer.train(config.epochs)

    print('Training complete.')

if __name__ == '__main__':
    main()

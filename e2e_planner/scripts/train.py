#!/usr/bin/env python3

import sys
import yaml
from datetime import datetime
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
import cv2
import csv
import numpy as np
from typing import List, Tuple
from tqdm import tqdm
from network import Network
from util.preprocessing import lane_mask_to_tensor_array

NUM_WAYPOINTS = 6
NUM_BRANCHES = 4
DEFAULT_COMMAND = 1
WAYPOINT_X_MIN = -0.5
WAYPOINT_X_MAX = 12.5
WAYPOINT_Y_MIN = -8.5
WAYPOINT_Y_MAX = 8.5
METADATA_CURVE_SCORE_INDEX = 7
METADATA_CURVE_BIN_INDEX = 8
METADATA_SAMPLE_WEIGHT_INDEX = 9


def normalize_axis(values: torch.Tensor, min_value: float, max_value: float) -> torch.Tensor:
    return (values - min_value) / (max_value - min_value) * 2.0 - 1.0


def denormalize_axis(values: torch.Tensor, min_value: float, max_value: float) -> torch.Tensor:
    return (values + 1.0) * 0.5 * (max_value - min_value) + min_value


def denormalize_waypoints(waypoints: torch.Tensor) -> torch.Tensor:
    denormalized = waypoints.clone()
    denormalized[..., 0::2] = denormalize_axis(waypoints[..., 0::2], WAYPOINT_X_MIN, WAYPOINT_X_MAX)
    denormalized[..., 1::2] = denormalize_axis(waypoints[..., 1::2], WAYPOINT_Y_MIN, WAYPOINT_Y_MAX)
    return denormalized


class E2EDataset(Dataset):
    def __init__(
        self,
        dataset_path: Path,
        curve_weight_enabled: bool = False,
        curve_threshold_m: float = 0.5,
        curve_side_bins: int = 3,
        curve_step_weight_factor: float = 4.0,
    ):
        self.dataset_path = dataset_path
        self.mask_images_dir = dataset_path / 'mask_images'
        self.path_dir = dataset_path / 'path'
        self.command_dir = dataset_path / 'commands'
        self.mask_files = sorted(list(self.mask_images_dir.glob('*.png')))
        self.curve_weight_enabled = curve_weight_enabled
        self.curve_threshold_m = curve_threshold_m
        self.curve_side_bins = max(1, curve_side_bins)
        self.curve_step_weight_factor = max(1.0, curve_step_weight_factor)
        self.curve_scores, self.curve_bins, self.sample_weights = self._build_curve_bin_weights()

    def __len__(self) -> int:
        return len(self.mask_files)

    def _read_waypoints(self, csv_file: Path) -> List[List[float]]:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            waypoints = [[float(row['x']), float(row['y'])] for row in reader]
        if len(waypoints) < NUM_WAYPOINTS:
            raise ValueError(f'Expected at least {NUM_WAYPOINTS} waypoints in {csv_file}, got {len(waypoints)}')
        return waypoints[:NUM_WAYPOINTS]

    def _curve_score(self, waypoints: List[List[float]]) -> float:
        raw_waypoints = np.array(waypoints, dtype=np.float32)
        max_abs_y_index = int(np.argmax(np.abs(raw_waypoints[:, 1])))
        return float(raw_waypoints[max_abs_y_index, 1])

    def _build_curve_bin_weights(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        curve_scores = np.zeros(len(self.mask_files), dtype=np.float32)
        curve_bins = np.zeros(len(self.mask_files), dtype=np.float32)
        weights = np.ones(len(self.mask_files), dtype=np.float32)

        if not self.curve_weight_enabled or len(self.mask_files) == 0:
            return curve_scores, curve_bins, weights

        for idx, mask_file in enumerate(self.mask_files):
            waypoints = self._read_waypoints(self.path_dir / f'{mask_file.stem}.csv')
            curve_scores[idx] = self._curve_score(waypoints)

        curve_magnitudes = np.abs(curve_scores)
        curve_mask = curve_magnitudes >= self.curve_threshold_m
        if not np.any(curve_mask):
            return curve_scores, curve_bins, weights

        curve_values = curve_magnitudes[curve_mask]
        max_curve = float(np.max(curve_values))
        if max_curve <= self.curve_threshold_m:
            signed_bins = np.sign(curve_scores[curve_mask]).astype(np.float32)
            curve_bins[curve_mask] = signed_bins
            weights[curve_mask] = self.curve_step_weight_factor
            return curve_scores, curve_bins, weights

        bin_edges = np.linspace(self.curve_threshold_m, max_curve, self.curve_side_bins + 1)
        bin_levels = np.digitize(curve_values, bin_edges[1:-1], right=True) + 1
        signed_bins = bin_levels.astype(np.float32) * np.sign(curve_scores[curve_mask]).astype(np.float32)

        curve_bins[curve_mask] = signed_bins
        weights[curve_mask] = np.power(self.curve_step_weight_factor, bin_levels).astype(np.float32)
        return curve_scores, curve_bins, weights

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, str, torch.Tensor]:
        mask_file = self.mask_files[idx]
        csv_file = self.path_dir / f'{mask_file.stem}.csv'
        command_file = self.command_dir / f'{mask_file.stem}.csv'

        mask_bgr = cv2.imread(str(mask_file), cv2.IMREAD_COLOR)
        mask_nonzero = int(np.count_nonzero(mask_bgr))

        waypoints = self._read_waypoints(csv_file)

        mask_normalized = lane_mask_to_tensor_array(mask_bgr)
        mask_tensor = torch.from_numpy(mask_normalized).unsqueeze(0)

        waypoints_tensor = torch.tensor(waypoints, dtype=torch.float32).flatten()
        waypoints_tensor[0::2] = normalize_axis(waypoints_tensor[0::2], WAYPOINT_X_MIN, WAYPOINT_X_MAX)
        waypoints_tensor[1::2] = normalize_axis(waypoints_tensor[1::2], WAYPOINT_Y_MIN, WAYPOINT_Y_MAX)

        command = DEFAULT_COMMAND
        if command_file.exists():
            with open(command_file, 'r', newline='') as f:
                command = int(float(next(csv.reader(f))[0]))

        command = min(max(command, 0), NUM_BRANCHES - 1)
        command_tensor = torch.zeros(NUM_BRANCHES, dtype=torch.float32)
        command_tensor[command] = 1.0

        raw_waypoints = np.array(waypoints, dtype=np.float32)
        metadata = torch.tensor(
            [
                float(command),
                float(mask_nonzero),
                float(raw_waypoints[-1, 0]),
                float(raw_waypoints[-1, 1]),
                float(np.max(np.abs(raw_waypoints[:, 1]))),
                float(np.min(raw_waypoints[:, 0])),
                float(np.max(raw_waypoints[:, 0])),
                float(self.curve_scores[idx]),
                float(self.curve_bins[idx]),
                float(self.sample_weights[idx]),
            ],
            dtype=torch.float32,
        )

        return mask_tensor, waypoints_tensor, command_tensor, mask_file.stem, metadata

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
        self.split_seed = int(config_dict.get('split_seed', 0))
        curve_weighting = config_dict.get('curve_bin_weighting', config_dict.get('curve_surprise_weighting', {}))
        self.curve_weight_enabled = bool(curve_weighting.get('enabled', False))
        self.curve_threshold_m = float(curve_weighting.get('threshold_m', 0.5))
        self.curve_side_bins = int(curve_weighting.get('side_bins', 3))
        self.curve_step_weight_factor = float(curve_weighting.get('step_weight_factor', 4.0))

        self.weights_dir = package_root / 'weights'
        self.weights_dir.mkdir(exist_ok=True)

        self.logs_dir = package_root / 'runs'
        self.logs_dir.mkdir(exist_ok=True)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class Trainer:
    def __init__(self, dataset_path: Path, config: Config):
        self.config = config
        self.device = config.device
        self.dataset_path = dataset_path

        dataset = E2EDataset(
            dataset_path,
            curve_weight_enabled=config.curve_weight_enabled,
            curve_threshold_m=config.curve_threshold_m,
            curve_side_bins=config.curve_side_bins,
            curve_step_weight_factor=config.curve_step_weight_factor,
        )
        if len(dataset) == 0:
            raise RuntimeError(
                f'No training masks found in {dataset_path / "mask_images"}. '
                'Run binarize_dataset.py before train.py.'
            )
        train_size = int(0.8 * len(dataset))
        val_size = len(dataset) - train_size
        split_generator = torch.Generator().manual_seed(config.split_seed)
        train_dataset, val_dataset = random_split(dataset, [train_size, val_size], generator=split_generator)

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

        self.model = Network(num_waypoints=NUM_WAYPOINTS, num_branches=NUM_BRANCHES).to(self.device)
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=config.learning_rate)
        self.mseloss = nn.MSELoss()
        run_name = f'{datetime.now().strftime("%Y%m%d_%H%M%S")}_{dataset_path.name}'
        self.run_dir = config.logs_dir / run_name
        self.writer = SummaryWriter(log_dir=str(self.run_dir))

        self.best_val_loss = float('inf')
        self.last_val_details = []
        self.last_val_rmse_m = 0.0

        print(f'Using device: {self.device}')
        print(f'Train size: {len(train_dataset)}, Val size: {len(val_dataset)}')
        if config.curve_weight_enabled:
            curve_count = int(np.count_nonzero(np.abs(dataset.curve_scores) >= config.curve_threshold_m))
            print(
                'Curve bin weighting: '
                f'threshold={config.curve_threshold_m:.3f} m, '
                f'side_bins={config.curve_side_bins}, '
                f'step_weight_factor={config.curve_step_weight_factor:.3f}, '
                f'curve_samples={curve_count}/{len(dataset)}, '
                f'weight_range={float(np.min(dataset.sample_weights)):.3f}-{float(np.max(dataset.sample_weights)):.3f}'
            )
        print(f'TensorBoard log dir: {self.run_dir}')
        print(f'View with: tensorboard --logdir {config.logs_dir}')

    def weighted_mse_loss(self, outputs: torch.Tensor, targets: torch.Tensor, sample_weights: torch.Tensor) -> torch.Tensor:
        sample_losses = torch.mean((outputs - targets) ** 2, dim=1)
        return torch.mean(sample_losses * sample_weights)

    def validate(self) -> float:
        self.model.eval()
        total_loss = 0.0
        total_rmse_m = 0.0
        self.last_val_details = []

        with torch.no_grad():
            pbar = tqdm(self.val_loader, desc='Validation')
            for images, waypoints, commands, sample_names, metadata in pbar:
                images = images.to(self.device)
                waypoints = waypoints.to(self.device)
                commands = commands.to(self.device)
                metadata = metadata.to(self.device)

                outputs = self.model(images, commands)

                loss = self.mseloss(outputs, waypoints)
                total_loss += loss.item()
                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

                sample_losses = torch.mean((outputs - waypoints) ** 2, dim=1)
                output_meters = denormalize_waypoints(outputs)
                target_meters = denormalize_waypoints(waypoints)
                sample_rmse_meters = torch.sqrt(torch.mean((output_meters - target_meters) ** 2, dim=1))
                total_rmse_m += sample_rmse_meters.mean().item()

                for i, sample_name in enumerate(sample_names):
                    self.last_val_details.append({
                        'sample': sample_name,
                        'loss': float(sample_losses[i].item()),
                        'rmse_m': float(sample_rmse_meters[i].item()),
                        'command': int(metadata[i, 0].item()),
                        'mask_nonzero': int(metadata[i, 1].item()),
                        'last_x_3s': float(metadata[i, 2].item()),
                        'last_y_3s': float(metadata[i, 3].item()),
                        'max_abs_y_3s': float(metadata[i, 4].item()),
                        'min_x_3s': float(metadata[i, 5].item()),
                        'max_x_3s': float(metadata[i, 6].item()),
                        'curve_y_m': float(metadata[i, METADATA_CURVE_SCORE_INDEX].item()),
                        'curve_bin': int(metadata[i, METADATA_CURVE_BIN_INDEX].item()),
                        'sample_weight': float(metadata[i, METADATA_SAMPLE_WEIGHT_INDEX].item()),
                    })

        self.last_val_rmse_m = total_rmse_m / len(self.val_loader)
        return total_loss / len(self.val_loader)

    def write_val_details(self, epoch: int) -> None:
        if not self.last_val_details:
            return

        sorted_details = sorted(self.last_val_details, key=lambda row: row['loss'], reverse=True)
        csv_path = self.run_dir / f'val_loss_epoch_{epoch:04d}.csv'
        fieldnames = [
            'rank',
            'sample',
            'loss',
            'rmse_m',
            'command',
            'mask_nonzero',
            'last_x_3s',
            'last_y_3s',
            'max_abs_y_3s',
            'min_x_3s',
            'max_x_3s',
            'curve_y_m',
            'curve_bin',
            'sample_weight',
        ]
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for rank, row in enumerate(sorted_details, start=1):
                writer.writerow({'rank': rank, **row})

        worst = sorted_details[:5]
        worst_summary = ', '.join(f"{row['sample']}:{row['loss']:.4f}" for row in worst)
        print(f'Validation details saved: {csv_path}')
        print(f'Worst val samples: {worst_summary}')

    def save_checkpoint(self, val_loss: float) -> None:
        if val_loss < self.best_val_loss:
            self.best_val_loss = val_loss
            weight_path = self.config.weights_dir / self.config.weight_file
            scripted_model = torch.jit.script(self.model)
            scripted_model.save(str(weight_path))
            print(f'Best model saved: {weight_path} (val_loss: {val_loss:.6f})')

    def train(self, epochs: int) -> None:
        self.writer.add_text('run/dataset_path', str(self.dataset_path), 0)
        self.writer.add_text('run/device', str(self.device), 0)
        self.writer.add_scalar('Dataset/train_size', len(self.train_loader.dataset), 0)
        self.writer.add_scalar('Dataset/val_size', len(self.val_loader.dataset), 0)

        for epoch in range(1, epochs + 1):
            self.model.train()
            total_train_loss = 0.0
            total_train_rmse_m = 0.0
            total_train_weight = 0.0

            pbar = tqdm(self.train_loader, desc=f'Epoch {epoch} [Train]')
            for images, waypoints, commands, _, metadata in pbar:
                images = images.to(self.device)
                waypoints = waypoints.to(self.device)
                commands = commands.to(self.device)
                metadata = metadata.to(self.device)
                sample_weights = metadata[:, METADATA_SAMPLE_WEIGHT_INDEX]

                self.optimizer.zero_grad()
                outputs = self.model(images, commands)

                loss = self.weighted_mse_loss(outputs, waypoints, sample_weights)
                loss.backward()
                self.optimizer.step()

                total_train_loss += loss.item()
                with torch.no_grad():
                    total_train_weight += sample_weights.mean().item()
                    output_meters = denormalize_waypoints(outputs)
                    target_meters = denormalize_waypoints(waypoints)
                    rmse_meters = torch.sqrt(torch.mean((output_meters - target_meters) ** 2, dim=1))
                    total_train_rmse_m += rmse_meters.mean().item()
                pbar.set_postfix({
                    'loss': f'{loss.item():.6f}',
                    'w': f'{sample_weights.mean().item():.3f}',
                })

            train_loss = total_train_loss / len(self.train_loader)
            train_rmse_m = total_train_rmse_m / len(self.train_loader)
            train_weight_mean = total_train_weight / len(self.train_loader)
            val_loss = self.validate()

            self.writer.add_scalar('Loss/train', train_loss, epoch)
            self.writer.add_scalar('Loss/val', val_loss, epoch)
            self.writer.add_scalar('RMSE_m/train', train_rmse_m, epoch)
            self.writer.add_scalar('RMSE_m/val', self.last_val_rmse_m, epoch)
            if self.config.curve_weight_enabled:
                self.writer.add_scalar('CurveWeight/train_mean', train_weight_mean, epoch)
            self.writer.add_scalar('LearningRate', self.optimizer.param_groups[0]['lr'], epoch)
            self.writer.add_scalar('Best/val_loss', min(self.best_val_loss, val_loss), epoch)
            self.writer.flush()

            print(
                f'Epoch [{epoch}/{epochs}], '
                f'Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}, '
                f'Train RMSE: {train_rmse_m:.3f} m, Val RMSE: {self.last_val_rmse_m:.3f} m'
            )
            self.write_val_details(epoch)

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

    script_dir = Path(__file__).resolve().parent
    package_root = script_dir.parent
    config_path = package_root / 'config' / 'train.yaml'

    config = Config(config_path, package_root)
    trainer = Trainer(dataset_path, config)

    print(f'Starting training for {config.epochs} epochs')
    trainer.train(config.epochs)

    print('Training complete.')

if __name__ == '__main__':
    main()

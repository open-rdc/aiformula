from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

IMAGE_HEIGHT = 288
IMAGE_WIDTH = 512


def to_model_input(bgr_image: np.ndarray) -> np.ndarray:
    yuv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YUV)
    yuv = cv2.resize(yuv, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
    yuv = yuv.astype(np.float32) / 127.5 - 1.0
    return np.transpose(yuv, (2, 0, 1))


class PilotNetDataset(Dataset):
    def __init__(self, data_dir: str | Path):
        data_dir = Path(data_dir)
        self.images = np.load(data_dir / 'images.npy', mmap_mode='r')
        self.targets = np.load(data_dir / 'targets.npy')
        if len(self.images) != len(self.targets):
            raise ValueError(
                f'images/targets length mismatch: {len(self.images)} vs {len(self.targets)}')

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int):
        image = to_model_input(self.images[idx])
        target = self.targets[idx].astype(np.float32)
        return torch.from_numpy(image), torch.from_numpy(target)

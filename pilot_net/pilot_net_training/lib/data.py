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
    """flip=True で左右反転サンプルを後半に足す。

    反転をディスクに実体化するとデータ量が倍になりページキャッシュに載らなくなるため、
    読み出し時に生成する。
    """

    def __init__(self, data_dir: str | Path, sample_weights: np.ndarray = None,
                 flip: bool = False):
        data_dir = Path(data_dir)
        self.images = np.load(data_dir / 'images.npy', mmap_mode='r')
        self.targets = np.load(data_dir / 'targets.npy')
        self.flip = flip
        if len(self.images) != len(self.targets):
            raise ValueError(
                f'images/targets length mismatch: {len(self.images)} vs {len(self.targets)}')
        if sample_weights is None:
            self.sample_weights = np.ones(len(self.targets), dtype=np.float32)
        else:
            if len(sample_weights) != len(self.targets):
                raise ValueError(
                    f'targets/weights length mismatch: '
                    f'{len(self.targets)} vs {len(sample_weights)}')
            self.sample_weights = np.asarray(sample_weights, dtype=np.float32)

    def __len__(self) -> int:
        return 2 * len(self.images) if self.flip else len(self.images)

    def __getitem__(self, idx: int):
        stored = len(self.images)
        mirrored = idx >= stored
        idx = idx - stored if mirrored else idx

        image = np.asarray(self.images[idx])
        target = self.targets[idx].astype(np.float32)
        if mirrored:
            image = image[:, ::-1]
            target = -target
        weight = np.float32(self.sample_weights[idx])
        return (torch.from_numpy(to_model_input(image)),
                torch.from_numpy(target), torch.tensor(weight))

import glob
import json
import os

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

SLOT_INDEX = {'straight': 0, 'left': 1, 'right': 2}


cv2.setNumThreads(0)


def label_path(image_path):
    head, tail = os.path.split(image_path)
    marker = os.sep + 'images' + os.sep
    before, sep, after = head.rpartition(marker)
    new_head = before + os.sep + 'labels' + os.sep + after if sep else head

    return os.path.join(new_head, os.path.splitext(tail)[0] + '.txt')


def find_images(root, split):
    return sorted(glob.glob(os.path.join(root, 'images', split, '*.png')))


class PathDataset(Dataset):
    def __init__(self, image_paths, num_slots=3, num_rows=48, input_size=(384, 640)):
        self.image_paths = list(image_paths)
        self.num_slots = num_slots
        self.num_rows = num_rows
        self.input_size = tuple(input_size)

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, index):
        path = self.image_paths[index]
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'画像を読めない: {path}')
        if image.shape[:2] != self.input_size:
            raise ValueError(f'画像サイズが {self.input_size} ではない: {image.shape[:2]} ({path}). レターボックスは bag_to_dataset.py で済ませておくこと')

        # Convert HWC to CHW
        sample = torch.from_numpy(np.ascontiguousarray(image.transpose(2, 0, 1)))

        exist = torch.zeros(self.num_slots)
        valid = torch.zeros(self.num_slots, self.num_rows)
        xp = torch.zeros(self.num_slots, self.num_rows)
        with open(label_path(path)) as f:
            for entry in json.load(f):
                slot = SLOT_INDEX.get(entry['class'])
                if slot is None or slot >= self.num_slots:
                    continue
                exist[slot] = 1.0
                valid[slot] = torch.tensor(entry['h_vector'], dtype=torch.float32)
                xp[slot] = torch.tensor(entry['xp'], dtype=torch.float32)

        return sample, exist, valid, xp

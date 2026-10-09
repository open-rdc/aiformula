import glob
import json
import math
import os
from collections import Counter

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, WeightedRandomSampler

from Models.model_components.vision_planner.vision_planner_head import ROW_START

SLOT_INDEX = {'straight': 0, 'left': 1, 'right': 2}
SEPARATION_THRESHOLD = 2.0 / 639  # 分岐とみなす最小の横差(ラベル規約 u/(width-1))


cv2.setNumThreads(0)


def label_path(image_path):
    head, tail = os.path.split(image_path)
    marker = os.sep + 'images' + os.sep
    before, sep, after = head.rpartition(marker)
    new_head = before + os.sep + 'labels' + os.sep + after if sep else head

    return os.path.join(new_head, os.path.splitext(tail)[0] + '.txt')


def find_images(root, split):
    return sorted(glob.glob(os.path.join(root, 'images', split, '*.png')))


def load_label(image_path, num_slots, num_rows):
    valid = torch.zeros(num_slots, num_rows)
    xp = torch.zeros(num_slots, num_rows)
    present = [False] * num_slots
    with open(label_path(image_path)) as f:
        for entry in json.load(f):
            slot = SLOT_INDEX[entry['class']]
            valid[slot] = torch.tensor(entry['h_vector'], dtype=torch.float32)
            xp[slot] = torch.tensor(entry['xp'], dtype=torch.float32)
            present[slot] = True

    straight = SLOT_INDEX['straight']
    for slot in range(num_slots):
        if not present[slot]:
            valid[slot] = valid[straight]
            xp[slot] = xp[straight]

    return valid[:, ROW_START:], xp[:, ROW_START:]


def branch_sampler(image_paths, num_slots, num_rows):
    keys = []
    for path in image_paths:
        valid, xp = load_label(path, num_slots, num_rows)
        diverged = (xp[1:] - xp[:1]).abs() > SEPARATION_THRESHOLD
        diverged = diverged & (valid[1:] > 0.5)
        keys.append(tuple(diverged.any(dim=1).tolist()))

    counts = Counter(keys)
    weights = [1.0 / math.sqrt(counts[key]) for key in keys]

    return WeightedRandomSampler(weights, len(weights), replacement=True)


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

        # Convert HWC to CHW
        sample = torch.from_numpy(np.ascontiguousarray(image.transpose(2, 0, 1)))
        valid, xp = load_label(path, self.num_slots, self.num_rows)

        return sample, valid, xp

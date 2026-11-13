import glob
import json
import os
import re

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from Models.vector_planner.data import geo_aug

MAP_CLASSES = {'solid': 0, 'dashed': 1, 'road_border': 2}
SLOTS = ('straight', 'left', 'right')
OK_STATUS = ('clear', 'avoid')
NO_BRANCH_STATUS = ('reject', 'fail')
X_RANGE, Y_MIN, Y_SPAN = 25.0, -8.0, 16.0
SIGNAL_STATES = ('none', 'red', 'green', 'off')
SIGNAL_IGNORE = -100
SIGNAL_COLOR_STATES = ('red', 'green')
SPLIT_DIRS = ('train', 'val')
MAX_PYLONS = 8
GEO_FLIP_P, GEO_YAW_P, GEO_YAW_MAX_DEG = 0.5, 0.5, 6.0


class E2EDataset(Dataset):
    def __init__(self, image_root, label_root, split, input_size=(384, 640), max_map=30, max_boxes=20, stems=None,
                 geo_aug=False, camera_params=None, online_paste=None, pose_shift=None, hue_swap=None, hue_swap_prob=0.0):
        self.image_root, self.label_root, self.split = image_root, label_root, split
        self.input_size, self.max_map, self.max_boxes = tuple(input_size), max_map, max_boxes
        self.geo_aug = geo_aug
        self.online_paste = online_paste
        self.pose_shift = pose_shift
        self.hue_swap, self.hue_swap_prob = hue_swap, float(hue_swap_prob)
        self._camera = None
        self._camera_params = camera_params
        if stems is None:
            labels = sorted(glob.glob(os.path.join(label_root, 'labels', split, '*.json')))
            stems = [os.path.splitext(os.path.basename(p))[0] for p in labels]
        self.stems = list(stems)

    def __len__(self):
        return len(self.stems)

    def camera(self):
        if self._camera is None:
            self._camera = geo_aug.load_aug_camera(self._camera_params or geo_aug.DEFAULT_PARAMS)
        return self._camera

    def read_label(self, index):
        with open(os.path.join(self.label_root, 'labels', self.split, f'{self.stems[index]}.json')) as f:
            return json.load(f)

    def __getitem__(self, index):
        flip, yaw = False, 0.0
        if self.geo_aug:
            flip = bool(torch.rand(()) < GEO_FLIP_P)
            if self.pose_shift is None and torch.rand(()) < GEO_YAW_P:
                yaw = float((torch.rand(()) * 2.0 - 1.0) * GEO_YAW_MAX_DEG)
        shift = None if self.pose_shift is None else self.pose_shift.sample()
        return self.build_item(index, flip, yaw, shift)

    def build_item(self, index, flip=False, yaw_deg=0.0, pose_shift=None):
        """pose_shift=(dy[m], dyaw[rad])。順序は 姿勢シフト -> online paste -> resize -> flip。"""
        stem = self.stems[index]
        label = self.read_label(index)
        det_only = bool(label.get('det_only', False))
        signal_only = bool(label.get('signal_only', False))
        geo_free = det_only or signal_only
        if det_only:
            yaw_deg = 0.0
        if geo_free:
            pose_shift = None
        image = cv2.imread(os.path.join(self.image_root, 'images', self.split, f'{stem}.png'), cv2.IMREAD_COLOR)
        signal = label.get('signal')
        pose_override = None
        shifted = False
        if self.pose_shift is not None and pose_shift is not None:
            image, label, pose_override = self.pose_shift.apply(self.image_root, self.split, stem, label, image,
                                                                *pose_shift)
            shifted = pose_override is not None
        if self.online_paste is not None and not geo_free:
            image, label = self.online_paste.apply(self.image_root, self.split, stem, label, image,
                                                   pose_override=pose_override)
        if signal is not None and label.get('signal') is None:
            label = {**label, 'signal': signal}
        y_masked = shifted and signal is not None
        if y_masked:
            label = {**label, 'signal': self._shift_signal(signal, *pose_shift)}
        image, label = self._maybe_hue_swap(image, label)
        height, width = self.input_size
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
        if flip or yaw_deg != 0.0:
            camera = self.camera()
            h = geo_aug.scale_homography(geo_aug.combined_homography(camera, flip, yaw_deg), self.input_size)
            image = geo_aug.transform_image(image, h)
            label = geo_aug.transform_label(label, flip, yaw_deg, camera, self.input_size)

        item = {'image': torch.from_numpy(np.ascontiguousarray(image.transpose(2, 0, 1)))}
        item.update(self._map(label.get('map_elements', [])))
        item.update(self._paths(label.get('paths', {})))
        item.update(self._corridor(label.get('paths', {})))
        item.update(self._pylons(label.get('pasted_base', [])))
        item.update(self._boxes(label.get('boxes', []), bool(label.get('has_boxes', False))))
        item.update(self._signal(label.get('signal'), y_masked))
        item['has_geo'] = torch.tensor(not geo_free)
        return item

    @staticmethod
    def _shift_signal(signal, dy, dyaw):
        # y は地面ホモグラフィで視差が再現されないので、シフト後は損失から外す（signal_xy_mask）
        out = dict(signal)
        if signal.get('xy') is not None:
            c, s = np.cos(dyaw), np.sin(dyaw)
            rel_x, rel_y = signal['xy'][0], signal['xy'][1] - dy
            out['xy'] = [float(c * rel_x + s * rel_y), float(-s * rel_x + c * rel_y)]
        return out

    def _maybe_hue_swap(self, image, label):
        signal = label.get('signal')
        if (self.hue_swap is None or self.hue_swap_prob <= 0.0 or signal is None
                or signal.get('state') not in SIGNAL_COLOR_STATES or float(torch.rand(())) >= self.hue_swap_prob):
            return image, label
        rng = np.random.default_rng(int(torch.randint(0, 2 ** 31 - 1, ())))
        return self.hue_swap(image, label, rng)

    @staticmethod
    def _signal(signal, y_masked=False):
        cls, has = SIGNAL_IGNORE, False
        xy = torch.zeros(2)
        mask = torch.zeros(2, dtype=torch.bool)
        if signal is not None:
            has = bool(signal.get('has_signal', False))
            state = signal.get('state')
            if state in SIGNAL_STATES:
                cls = SIGNAL_STATES.index(state)
            if has and state != 'none' and signal.get('xy') is not None:
                xy = torch.tensor(signal['xy'], dtype=torch.float32)
                mask = torch.tensor([True, not y_masked])
        return {'signal_cls': torch.tensor(cls), 'signal_xy': xy, 'signal_xy_mask': mask, 'has_signal': torch.tensor(has)}

    def _map(self, elements):
        points = torch.zeros(self.max_map, 20, 2)
        classes = torch.zeros(self.max_map, dtype=torch.long)
        mask = torch.zeros(self.max_map, dtype=torch.bool)
        for i, element in enumerate(elements[:self.max_map]):
            p = np.asarray(element['points'], np.float32)
            points[i, :, 0] = torch.from_numpy(p[:, 0] / X_RANGE)
            points[i, :, 1] = torch.from_numpy((p[:, 1] - Y_MIN) / Y_SPAN)
            classes[i] = MAP_CLASSES[element['class']]
            mask[i] = True
        return {'map_points': points, 'map_cls': classes, 'map_mask': mask}

    def _paths(self, paths):
        xy = torch.zeros(len(SLOTS), 40, 2)
        valid = torch.zeros(len(SLOTS), 40, dtype=torch.bool)
        mask = torch.zeros(len(SLOTS), 40, dtype=torch.bool)
        slot_ok = torch.zeros(len(SLOTS), dtype=torch.bool)
        avoid = torch.zeros(len(SLOTS), dtype=torch.bool)
        exist = torch.zeros(len(SLOTS), dtype=torch.bool)
        for i, slot in enumerate(SLOTS):
            entry = paths.get(slot)
            if entry is None:
                continue
            exist[i] = entry['status'] not in NO_BRANCH_STATUS
            avoid[i] = entry['status'] == 'avoid'
            xy[i] = torch.tensor(entry['xy'], dtype=torch.float32)
            valid[i] = torch.tensor(entry['valid'], dtype=torch.bool)
            if entry['status'] in OK_STATUS:
                slot_ok[i] = True
                mask[i] = valid[i]
        return {'path_xy': xy, 'path_valid': valid, 'path_mask': mask, 'path_slot_ok': slot_ok, 'path_avoid': avoid,
                'branch_exist': exist}

    def _corridor(self, paths):
        left = torch.zeros(len(SLOTS), 40, 2)
        right = torch.zeros(len(SLOTS), 40, 2)
        mask = torch.zeros(len(SLOTS), 40, dtype=torch.bool)
        observable = torch.zeros(len(SLOTS), 40, dtype=torch.bool)
        for i, slot in enumerate(SLOTS):
            entry = paths.get(slot)
            corridor = None if entry is None else entry.get('corridor')
            if corridor is None or entry['status'] not in OK_STATUS:
                continue
            left[i] = torch.tensor(corridor['left'], dtype=torch.float32)
            right[i] = torch.tensor(corridor['right'], dtype=torch.float32)
            mask[i] = torch.tensor(corridor['valid'], dtype=torch.bool)
            observable[i] = torch.tensor(corridor['observable'], dtype=torch.bool)
        return {'corridor_left': left, 'corridor_right': right, 'corridor_mask': mask, 'corridor_obs': observable}

    def _pylons(self, pasted):
        out = torch.zeros(MAX_PYLONS, 3)
        mask = torch.zeros(MAX_PYLONS, dtype=torch.bool)
        for i, p in enumerate(sorted(pasted, key=lambda p: p[0])[:MAX_PYLONS]):
            out[i] = torch.tensor([p[0], p[1], p[2]], dtype=torch.float32)
            mask[i] = True
        return {'pylons': out, 'pylon_mask': mask}

    def _boxes(self, boxes, has_boxes):
        out = torch.zeros(self.max_boxes, 4)
        classes = torch.zeros(self.max_boxes, dtype=torch.long)
        mask = torch.zeros(self.max_boxes, dtype=torch.bool)
        if has_boxes:
            for i, box in enumerate(boxes[:self.max_boxes]):
                out[i] = torch.tensor([box['cx'], box['cy'], box['w'], box['h']])
                classes[i] = int(box['class'])
                mask[i] = True
        return {'boxes': out, 'box_cls': classes, 'box_mask': mask, 'has_boxes': torch.tensor(has_boxes)}


def base_stem(stem):
    return stem.rsplit('__p', 1)[0] if '__p' in stem else stem


def parse_stem(stem):
    match = re.match(r'^(.*)_(\d+)$', stem)
    return (match.group(1), int(match.group(2))) if match else None


def select_stems(split_map, directory, stems, wanted):
    return [s for s in stems if split_map.get(f'{directory}/{base_stem(s)}') == wanted]


def split_datasets(specs, split_map, wanted, input_size=(384, 640), **dataset_kwargs):
    stem_filter = dataset_kwargs.pop('stem_filter', None)
    datasets = []
    for image_root, label_root in specs:
        for directory in SPLIT_DIRS:
            all_stems = E2EDataset(image_root, label_root, directory, input_size, **dataset_kwargs).stems
            stems = select_stems(split_map, directory, all_stems, wanted)
            if stem_filter is not None:
                stems = [s for s in stems if stem_filter(s)]
            if stems:
                datasets.append(E2EDataset(image_root, label_root, directory, input_size, stems=stems,
                                           **dataset_kwargs))
    return datasets

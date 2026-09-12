import copy
import math
import os
import random
import re

import cv2
import numpy as np
import torch

SLOT_COLORS = ((0, 220, 0), (230, 120, 0), (0, 140, 255))



def setup_seed(seed=0):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def set_params(model, decay):
    norm_types = tuple(v for k, v in torch.nn.__dict__.items() if 'Norm' in k)
    no_decay, decayed = [], []
    for module in model.modules():
        for name, parameter in module.named_parameters(recurse=False):
            if not parameter.requires_grad:
                continue
            if name == 'bias' or isinstance(module, norm_types):
                no_decay.append(parameter)
            else:
                decayed.append(parameter)
    return [{'params': no_decay, 'weight_decay': 0.0},
            {'params': decayed, 'weight_decay': decay}]


class LinearLR:
    def __init__(self, params, epochs, num_steps):
        warmup = int(max(params['warmup_epochs'] * num_steps, 100))
        decay = max(int(epochs * num_steps - warmup), 1)
        self.total = np.concatenate((np.linspace(params['min_lr'], params['max_lr'], warmup, endpoint=False),
                                      np.linspace(params['max_lr'], params['min_lr'], decay)))

    def step(self, step, optimizer):
        value = float(self.total[min(step, len(self.total) - 1)])
        for group in optimizer.param_groups:
            group['lr'] = value


class EMA:
    def __init__(self, model, decay=0.9999, tau=2000):
        self.ema = copy.deepcopy(model).eval()
        self.updates = 0
        self.decay = lambda x: decay * (1 - math.exp(-x / tau))
        for parameter in self.ema.parameters():
            parameter.requires_grad_(False)

    def update(self, model):
        source = model.module if hasattr(model, 'module') else model
        with torch.no_grad():
            self.updates += 1
            factor = self.decay(self.updates)
            state = source.state_dict()
            for key, value in self.ema.state_dict().items():
                if value.dtype.is_floating_point:
                    value *= factor
                    value += (1.0 - factor) * state[key].detach()


class AverageMeter:
    def __init__(self):
        self.sum = 0.0
        self.count = 0
        self.avg = 0.0

    def update(self, value, n=1):
        if math.isnan(float(value)):
            return
        self.sum += value * n
        self.count += n
        self.avg = self.sum / max(self.count, 1)


def next_run_dir(runs_dir):
    used = []
    if os.path.isdir(runs_dir):
        for name in os.listdir(runs_dir):
            match = re.fullmatch(r'run(\d+)', name)
            if match and os.path.isdir(os.path.join(runs_dir, name)):
                used.append(int(match.group(1)))
    return os.path.join(runs_dir, f'run{max(used) + 1 if used else 1}')


def visualize(sample, outputs, targets, num_rows):
    from Models.model_components.vision_planner.vision_planner_head import decode_positions
    from Models.data_utils.vision_planner.projection import row_anchor_targets

    exist, valid, dist = outputs
    target_valid, target_xp = targets[1], targets[2]

    image = np.ascontiguousarray(sample.detach().cpu().numpy().transpose(1, 2, 0))
    height, width = image.shape[:2]

    anchors = row_anchor_targets(num_rows, height)
    predicted = decode_positions(dist.detach()[None])[0]

    for slot in range(dist.shape[0]):
        color = SLOT_COLORS[slot % len(SLOT_COLORS)]
        for row in range(num_rows):
            center_y = int(anchors[row])
            if float(target_valid[slot, row]) > 0.5:
                x = int(float(target_xp[slot, row]) * (width - 1))
                cv2.line(image, (x - 4, center_y), (x + 4, center_y), color, 2)
            if torch.sigmoid(valid[slot, row]) > 0.5 and torch.sigmoid(exist[slot]) > 0.5:
                x = int(float(predicted[slot, row]) * (width - 1))
                cv2.circle(image, (x, center_y), 2, color, -1)
    return np.ascontiguousarray(image[:, :, ::-1])  # TensorBoard は RGB を期待する

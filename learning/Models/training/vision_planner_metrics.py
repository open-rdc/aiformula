import numpy as np
import torch

from Models.model_components.vision_planner.vision_planner_head import decode_positions

SLOT_NAMES = ('straight', 'left', 'right')


def _f1(predicted, actual):
    true_positive = float((predicted & actual).sum())
    if true_positive == 0.0:
        return 0.0
    precision = true_positive / float(predicted.sum())
    recall = true_positive / float(actual.sum())
    return 2.0 * precision * recall / (precision + recall)


class PathMetrics:
    def __init__(self, num_slots=3, threshold=0.02, input_width=640, min_slot_rows=200):
        self.num_slots = num_slots
        self.threshold = threshold
        self.input_width = input_width

        self.min_slot_rows = min_slot_rows
        self.errors = [[] for _ in range(num_slots)]
        self.valid_pred = []
        self.valid_true = []
        self.exist_pred = []
        self.exist_true = []

    def update(self, outputs, targets):
        exist, valid, dist = outputs
        target_exist, target_valid, target_xp = targets
        position = decode_positions(dist.detach())

        for slot in range(self.num_slots):
            mask = target_valid[:, slot] > 0.5
            if mask.any():
                error = (position[:, slot] - target_xp[:, slot]).abs()[mask]
                self.errors[slot].extend(error.detach().cpu().numpy().tolist())

        self.valid_pred.append((torch.sigmoid(valid) > 0.5).detach().cpu().numpy())
        self.valid_true.append((target_valid > 0.5).detach().cpu().numpy())
        self.exist_pred.append((torch.sigmoid(exist) > 0.5).detach().cpu().numpy())
        self.exist_true.append((target_exist > 0.5).detach().cpu().numpy())

    def _all_errors(self):
        return np.array([e for slot in self.errors for e in slot], dtype=float)

    def _qualifying_slots(self):
        return [(slot, float(np.array(self.errors[slot], dtype=float).mean()))
                for slot in range(self.num_slots)
                if len(self.errors[slot]) >= self.min_slot_rows]

    def score(self):
        qualifying = self._qualifying_slots()
        if qualifying:
            return float(np.mean([mean for _, mean in qualifying]))
        errors = self._all_errors()
        return float(errors.mean()) if errors.size else 0.0

    def compute(self):
        errors = self._all_errors()

        valid_pred = np.concatenate(self.valid_pred)
        valid_true = np.concatenate(self.valid_true)
        exist_pred = np.concatenate(self.exist_pred)
        exist_true = np.concatenate(self.exist_true)

        macro = self.score()
        qualifying_slots = {slot for slot, _ in self._qualifying_slots()}

        result = {
            'lateral_mean': float(errors.mean()) if errors.size else 0.0,
            'lateral_median': float(np.median(errors)) if errors.size else 0.0,
            'lateral_macro': macro,
            'lateral_macro/slots_used': len(qualifying_slots),
            'lateral_px': float(errors.mean() * (self.input_width - 1)) if errors.size else 0.0,
            'lateral_macro_px': float(macro * (self.input_width - 1)),
            'within_threshold': float((errors < self.threshold).mean()) if errors.size else 0.0,
            'valid_f1': _f1(valid_pred, valid_true),
            'exist_f1': _f1(exist_pred, exist_true),
        }

        for slot in range(self.num_slots):
            values = np.array(self.errors[slot], dtype=float)
            name = SLOT_NAMES[slot] if slot < len(SLOT_NAMES) else str(slot)
            result[f'lateral_mean/{name}'] = float(values.mean()) if values.size else 0.0
            result[f'lateral_median/{name}'] = float(np.median(values)) if values.size else 0.0
            result[f'valid_f1/{name}'] = _f1(valid_pred[:, slot], valid_true[:, slot])
            result[f'exist_f1/{name}'] = _f1(exist_pred[:, slot], exist_true[:, slot])
            result[f'lateral_macro/excluded/{name}'] = 0.0 if slot in qualifying_slots else 1.0

        return result

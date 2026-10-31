import numpy as np

SLOT_NAMES = ('straight', 'left', 'right')
BRANCH_SLOTS = (1, 2)
SEP_THRESHOLD_PX = 2.0  # 教師が分岐しているとみなす最小の横差(px)


class PathMetrics:
    def __init__(self, num_slots=3, threshold=0.02, input_width=640):
        self.num_slots = num_slots
        self.threshold = threshold
        self.input_width = input_width
        self.sep_threshold = SEP_THRESHOLD_PX / (input_width - 1)

        self.errors = [[] for _ in range(num_slots)]
        self.gt_sep = {slot: [] for slot in BRANCH_SLOTS}
        self.pred_sep = {slot: [] for slot in BRANCH_SLOTS}
        self.valid_tp = [0] * num_slots
        self.valid_fp = [0] * num_slots
        self.valid_fn = [0] * num_slots
        self.row_jumps = [[] for _ in range(num_slots)]

    def update(self, outputs, targets):
        valid, position = outputs
        valid = valid.detach()
        position = position.detach()
        target_valid, target_xp = targets

        pred_valid = valid.sigmoid() > 0.5
        gt_valid = target_valid > 0.5
        for slot in range(self.num_slots):
            self.valid_tp[slot] += int((pred_valid[:, slot] & gt_valid[:, slot]).sum())
            self.valid_fp[slot] += int((pred_valid[:, slot] & ~gt_valid[:, slot]).sum())
            self.valid_fn[slot] += int((~pred_valid[:, slot] & gt_valid[:, slot]).sum())

        for slot in range(self.num_slots):
            mask = target_valid[:, slot] > 0.5
            if mask.any():
                error = (position[:, slot] - target_xp[:, slot]).abs()[mask]
                self.errors[slot].extend(error.detach().cpu().numpy().tolist())

        scale = self.input_width - 1
        row_diff = (position[:, :, 1:] - position[:, :, :-1]).abs() * scale
        row_pair_valid = pred_valid[:, :, 1:] & pred_valid[:, :, :-1]
        for slot in range(self.num_slots):
            values = row_diff[:, slot][row_pair_valid[:, slot]]
            if values.numel():
                self.row_jumps[slot].extend(values.detach().cpu().numpy().tolist())

        straight_valid = target_valid[:, 0] > 0.5
        gt_diff = (target_xp[:, 1:] - target_xp[:, :1]).abs()
        pred_diff = (position[:, 1:] - position[:, :1]).abs()
        for offset, slot in enumerate(BRANCH_SLOTS):
            overlap = straight_valid & (target_valid[:, slot] > 0.5)
            for frame in range(overlap.shape[0]):
                rows = overlap[frame]
                if not rows.any():
                    continue
                self.gt_sep[slot].append(float(gt_diff[frame, offset][rows].max()))
                self.pred_sep[slot].append(float(pred_diff[frame, offset][rows].max()))

    def _all_errors(self):
        return np.array([e for slot in self.errors for e in slot], dtype=float)

    def score(self):
        errors = self._all_errors()
        lateral = float(errors.mean()) * (self.input_width - 1) if errors.size else 0.0

        separation = 0.0
        for slot in BRANCH_SLOTS:
            gt_median, pred_median, _, false_positive = self._separation_metrics(slot)
            separation += abs(pred_median - gt_median) + false_positive
        separation /= len(BRANCH_SLOTS)

        return lateral + separation

    def _separation_metrics(self, slot):
        gt = np.array(self.gt_sep[slot], dtype=float)
        pred = np.array(self.pred_sep[slot], dtype=float)
        keep = gt > self.sep_threshold
        scale = self.input_width - 1
        false_positive = float(np.median(pred[~keep] * scale)) if (~keep).any() else 0.0
        if not keep.any():
            return 0.0, 0.0, 0.0, false_positive

        gt_kept = gt[keep] * scale
        pred_kept = pred[keep] * scale
        # 予測分離量が定数に潰れると corrcoef が nan を返す。崩壊時こそ見たい指標なので 0 に倒す
        correlation = 0.0
        if keep.sum() >= 2 and gt_kept.std() > 0.0 and pred_kept.std() > 0.0:
            correlation = float(np.corrcoef(gt_kept, pred_kept)[0, 1])
        return float(np.median(gt_kept)), float(np.median(pred_kept)), correlation, false_positive

    def _valid_f1(self, tp, fp, fn):
        denominator = tp + 0.5 * (fp + fn)
        return float(tp / denominator) if denominator > 0 else 0.0

    def compute(self):
        errors = self._all_errors()

        result = {
            'lateral_mean': float(errors.mean()) if errors.size else 0.0,
            'lateral_median': float(np.median(errors)) if errors.size else 0.0,
            'lateral_px': float(errors.mean() * (self.input_width - 1)) if errors.size else 0.0,
            'within_threshold': float((errors < self.threshold).mean()) if errors.size else 0.0,
        }

        for slot in range(self.num_slots):
            values = np.array(self.errors[slot], dtype=float)
            name = SLOT_NAMES[slot]
            result[f'lateral_mean/{name}'] = float(values.mean()) if values.size else 0.0
            result[f'lateral_median/{name}'] = float(np.median(values)) if values.size else 0.0

        for slot in range(self.num_slots):
            name = SLOT_NAMES[slot]
            result[f'valid_f1/{name}'] = self._valid_f1(self.valid_tp[slot], self.valid_fp[slot], self.valid_fn[slot])
        result['valid_f1'] = self._valid_f1(sum(self.valid_tp), sum(self.valid_fp), sum(self.valid_fn))

        for slot in BRANCH_SLOTS:
            name = SLOT_NAMES[slot]
            gt_median, pred_median, correlation, false_positive = self._separation_metrics(slot)
            result[f'separation_gt/{name}'] = gt_median
            result[f'separation/{name}'] = pred_median
            result[f'separation_r/{name}'] = correlation
            result[f'separation_fp/{name}'] = false_positive

        all_jumps = []
        for slot in range(self.num_slots):
            values = np.array(self.row_jumps[slot], dtype=float)
            name = SLOT_NAMES[slot]
            result[f'row_jump_mean/{name}'] = float(values.mean()) if values.size else 0.0
            result[f'row_jump_p95/{name}'] = float(np.percentile(values, 95)) if values.size else 0.0
            all_jumps.extend(self.row_jumps[slot])
        all_jumps = np.array(all_jumps, dtype=float)
        result['row_jump_mean'] = float(all_jumps.mean()) if all_jumps.size else 0.0
        result['row_jump_p95'] = float(np.percentile(all_jumps, 95)) if all_jumps.size else 0.0

        return result

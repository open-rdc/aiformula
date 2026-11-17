from dataclasses import dataclass
from typing import Optional

import numpy as np

LEFT = -1.0
RIGHT = +1.0


@dataclass
class LineObservation:
    valid: bool
    x_ref: float = float('nan')
    slope: float = float('nan')
    var_x: float = float('inf')
    var_slope: float = float('inf')
    inlier_ratio: float = 0.0
    points: Optional[np.ndarray] = None

    def to_list(self):
        return [float(self.valid), self.x_ref, self.slope, self.var_x, self.var_slope, self.inlier_ratio]


@dataclass
class LinePrior:
    x_ref: float
    slope: float
    sigma_x: float


@dataclass
class ObserverConfig:
    y_start: int = 240
    y_end: int = 340
    y_ref: int = 330
    max_run_width: int = 45
    min_run_width: int = 2
    gate_sigma: float = 3.0
    gate_margin: float = 15.0
    ransac_iters: int = 60
    ransac_thresh: float = 6.0
    min_inlier_ratio: float = 0.3
    max_abs_slope: float = 4.0
    min_init_slope: float = 0.3
    max_slope_jump: float = 0.8
    var_x_floor: float = 4.0


def extract_runs(white_row: np.ndarray):
    padded = np.concatenate(([False], white_row, [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return edges.reshape(-1, 2)


def count_rows(points: np.ndarray) -> int:
    return len(np.unique(points[:, 0]))


class LaneObserver:
    def __init__(self, config: Optional[ObserverConfig] = None, seed: int = 0):
        self.cfg = config if config is not None else ObserverConfig()
        self.rng = np.random.default_rng(seed)

    def observe(self, mask: np.ndarray,
                prior_left: Optional[LinePrior] = None,
                prior_right: Optional[LinePrior] = None):
        height, width = mask.shape[:2]
        ys = max(0, min(self.cfg.y_start, height))
        ye = max(0, min(self.cfg.y_end, height))
        if ys >= ye:
            return LineObservation(False), LineObservation(False)

        pts = self._run_centers(mask, ys, ye)
        n_rows = ye - ys
        left = self._observe_side(pts, prior_left, LEFT, width / 2.0, n_rows)
        right = self._observe_side(pts, prior_right, RIGHT, width / 2.0, n_rows)
        return left, right

    def _run_centers(self, mask, ys, ye):
        cfg = self.cfg
        width = mask.shape[1]
        points = []
        for y in range(ys, ye):
            for x0, x1 in extract_runs(mask[y] > 0):
                if not cfg.min_run_width <= x1 - x0 <= cfg.max_run_width:
                    continue
                if x0 == 0 or x1 == width:
                    continue
                points.append((y, 0.5 * (x0 + x1 - 1)))
        return np.array(points, dtype=np.float64).reshape(-1, 2)

    def _observe_side(self, pts, prior, side, center, n_rows):
        cfg = self.cfg
        if len(pts) == 0:
            return LineObservation(False)
        if prior is not None and not np.isfinite(prior.x_ref):
            prior = None

        if prior is not None:
            pred = prior.x_ref + prior.slope * (pts[:, 0] - cfg.y_ref)
            gate = cfg.gate_sigma * prior.sigma_x + cfg.gate_margin
            p = pts[np.abs(pts[:, 1] - pred) <= gate]
        else:
            p = pts[pts[:, 1] < center] if side == LEFT else pts[pts[:, 1] >= center]

        min_rows = max(3, int(np.ceil(cfg.min_inlier_ratio * n_rows)))
        if count_rows(p) < min_rows:
            return LineObservation(False)

        inliers = self._ransac(p, lambda b: self._slope_ok(b, prior, side))
        if inliers is None:
            return LineObservation(False)
        q = p[inliers]
        inlier_rows = count_rows(q)
        if inlier_rows < min_rows:
            return LineObservation(False)

        fit = self._least_squares(q)
        if fit is None:
            return LineObservation(False)
        x_ref, slope, var_x, var_slope = fit
        if not self._slope_ok(slope, prior, side):
            return LineObservation(False)
        return LineObservation(True, x_ref, slope, var_x, var_slope, inlier_rows / n_rows, q)

    def _slope_ok(self, slope, prior, side):
        cfg = self.cfg
        if abs(slope) > cfg.max_abs_slope:
            return False
        if prior is not None:
            return abs(slope - prior.slope) <= cfg.max_slope_jump
        return slope * side >= cfg.min_init_slope

    def _ransac(self, p, slope_ok):
        cfg = self.cfg
        best, best_count = None, 0
        for _ in range(cfg.ransac_iters):
            i, j = self.rng.choice(len(p), 2, replace=False)
            dy = p[j, 0] - p[i, 0]
            if dy == 0:
                continue
            slope = (p[j, 1] - p[i, 1]) / dy
            if not slope_ok(slope):
                continue
            resid = np.abs(p[:, 1] - (p[i, 1] + slope * (p[:, 0] - p[i, 0])))
            inliers = resid <= cfg.ransac_thresh
            count = count_rows(p[inliers])
            if count > best_count:
                best, best_count = inliers, count
        return best

    def _least_squares(self, q):
        cfg = self.cfg
        u = q[:, 0] - cfg.y_ref
        x = q[:, 1]
        n = len(q)
        u_mean = u.mean()
        sxx = np.sum((u - u_mean) ** 2)
        if n < 3 or sxx <= 0:
            return None
        b = np.sum((u - u_mean) * (x - x.mean())) / sxx
        a = x.mean() - b * u_mean
        resid = x - (a + b * u)
        s2 = max(np.sum(resid ** 2) / (n - 2), cfg.var_x_floor)
        var_a = s2 * (1.0 / n + u_mean ** 2 / sxx)
        var_b = s2 / sxx
        return float(a), float(b), float(max(var_a, cfg.var_x_floor)), float(var_b)

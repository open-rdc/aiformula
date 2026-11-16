import math
from dataclasses import dataclass, field

import numpy as np

EPS = 1.0e-6
STEERING_SAFETY_FACTOR = 0.8
COLLISION_LONGITUDINAL_MIN = 0.5
MAX_HEADING_ERROR = 0.25 * math.pi
SIDE_TIE_M = 0.05
OBSTACLE_REAR_M = 0.5
HOLD_RANGE_M = 4.0
HOLD_SETTLE_M = 3.0
HOLD_RELEASE_M = 0.5
HOLD_GRAZE_MARGIN_M = 0.05


@dataclass
class FrenetConfig:
    horizon: float = 20.0
    interval: float = 0.2
    detection_forward: float = 20.0
    hard_margin: float = 0.35
    soft_margin: float = 1.0
    max_shift: float = 2.4
    cluster_gap: float = 0.8
    lateral_step: float = 0.15
    collision_margin: float = 0.15
    target_lengths: list = field(default_factory=lambda: [15.0])
    weight_curvature: float = 2000.0
    weight_lateral: float = 150.0
    tread: float = 0.6
    wheelbase: float = 0.8
    steering_max_deg: float = 15.0
    stop_standoff: float = 1.0
    boundary_relax_length: float = 5.0
    max_reference_heading_error_deg: float = 30.0
    curvature_baseline: float = 1.0
    road_margin: float = None


@dataclass
class FrenetResult:
    points_xy: np.ndarray
    status: str
    side: str = None
    target_d: float = 0.0


class QuinticPolynomial:
    def __init__(self, x0, x0v, x0a, xT, xTv, xTa, T):
        self.c0, self.c1, self.c2 = x0, x0v, 0.5 * x0a
        T = max(T, 1e-9)
        T2, T3, T4, T5 = T ** 2, T ** 3, T ** 4, T ** 5
        dp = xT - x0 - x0v * T - 0.5 * x0a * T2
        dv = xTv - x0v - x0a * T
        da = xTa - x0a
        self.c3 = (10 * dp - 4 * dv * T + 0.5 * da * T2) / T3
        self.c4 = (-15 * dp + 7 * dv * T - da * T2) / T4
        self.c5 = (6 * dp - 3 * dv * T + 0.5 * da * T2) / T5

    def position(self, t):
        return self.c0 + t * (self.c1 + t * (self.c2 + t * (self.c3 + t * (self.c4 + t * self.c5))))


class _Path:
    def __init__(self, xy):
        self.xy = xy
        seg = np.diff(xy, axis=0)
        length = np.linalg.norm(seg, axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(length)])
        self.seg_yaw = np.arctan2(seg[:, 1], seg[:, 0])
        self.max_s = float(self.s[-1])

    def point_at(self, s):
        s = np.clip(np.asarray(s, float), 0.0, self.max_s)
        i = np.clip(np.searchsorted(self.s, s, side='right') - 1, 0, len(self.s) - 2)
        length = self.s[i + 1] - self.s[i]
        ratio = np.where(length > EPS, (s - self.s[i]) / np.where(length > EPS, length, 1.0), 0.0)
        p = self.xy[i] + ratio[..., None] * (self.xy[i + 1] - self.xy[i])
        return p[..., 0], p[..., 1], self.seg_yaw[i]

    def project(self, point):
        a, b = self.xy[:-1], self.xy[1:]
        v = b - a
        length_sq = np.sum(v * v, axis=1)
        t = np.clip(np.sum((point - a) * v, axis=1) / np.maximum(length_sq, EPS), 0.0, 1.0)
        foot = a + t[:, None] * v
        diff = point - foot
        i = int(np.argmin(np.sum(diff * diff, axis=1)))
        yaw = self.seg_yaw[i]
        s = self.s[i] + t[i] * (self.s[i + 1] - self.s[i])
        d = -math.sin(yaw) * diff[i, 0] + math.cos(yaw) * diff[i, 1]
        self.last_overshoot = float(np.linalg.norm(diff[i])) - abs(d)
        return float(s), float(d), float(yaw)

    def curvature_at(self, s, h):
        yaw_behind = self.point_at(s - h)[2]
        yaw_ahead = self.point_at(s + h)[2]
        return math.remainder(float(yaw_ahead - yaw_behind), 2 * math.pi) / (2 * h)


@dataclass
class _Obstacle:
    s: float
    d: float
    half_width: float
    half_length: float


def _find_obstacles(path, current_s, obstacles, cfg, rear=0.0):
    lateral_limit = cfg.tread * 0.5 + cfg.hard_margin + cfg.soft_margin
    found = []
    for x, y, width in obstacles:
        s, d, _ = path.project(np.array([x, y], float))
        half_width = 0.5 * max(0.0, float(width))
        if path.last_overshoot > 1e-6:
            continue
        if s - current_s < -rear or s - current_s > cfg.detection_forward:
            continue
        if abs(d) > lateral_limit + half_width:
            continue
        found.append(_Obstacle(s, d, half_width, 0.0))
    found.sort(key=lambda o: o.s)

    clusters = []
    for o in found:
        left, right = o.d + o.half_width, o.d - o.half_width
        m = clusters[-1] if clusters else None
        adjacent = (
            m is not None
            and o.s - (m.s + m.half_length) <= cfg.cluster_gap + EPS
            and right - (m.d + m.half_width) <= cfg.cluster_gap + EPS
            and (m.d - m.half_width) - left <= cfg.cluster_gap + EPS
        )
        if not adjacent:
            clusters.append(o)
            continue
        s_front = min(m.s - m.half_length, o.s)
        s_back = max(m.s + m.half_length, o.s)
        d_left = max(m.d + m.half_width, left)
        d_right = min(m.d - m.half_width, right)
        clusters[-1] = _Obstacle(0.5 * (s_front + s_back), 0.5 * (d_left + d_right),
                                 0.5 * (d_left - d_right), 0.5 * (s_back - s_front))
    return clusters


def _limit_at(path_s, limit, s):
    if limit is None:
        return None
    s = np.asarray(s, float)
    i = np.clip(np.searchsorted(path_s, s, side='right') - 1, 0, len(path_s) - 2)
    length = np.maximum(path_s[i + 1] - path_s[i], EPS)
    t = np.clip((s - path_s[i]) / length, 0.0, 1.0)
    return limit[i] + t * (limit[i + 1] - limit[i])


def _choose_side(first, path, left_limit, right_limit, cfg):
    left_edge, right_edge = first.d + first.half_width, first.d - first.half_width
    left = _limit_at(path.s, left_limit, first.s)
    right = _limit_at(path.s, right_limit, first.s)
    left = cfg.max_shift if left is None or np.isnan(left) else float(left)
    right = -cfg.max_shift if right is None or np.isnan(right) else float(right)
    left_room, right_room = left - left_edge, right_edge - right
    if abs(left_room - right_room) < SIDE_TIE_M:
        return 'left'
    return 'left' if left_room > right_room else 'right'


def _s_grid(initial_s, target_s, interval):
    count = int(math.floor((target_s - initial_s + EPS) / interval))
    return initial_s + interval * np.arange(1, count + 1)


def _lateral_profile(initial, target, interval, end_s=None):
    delta = target[0] - initial[0]
    if delta < interval or interval <= EPS:
        return None
    poly = QuinticPolynomial(initial[1], initial[2], initial[3], target[1], 0.0, 0.0, delta)
    s_pts = _s_grid(initial[0], target[0], interval)
    offsets = poly.position(s_pts - initial[0])
    if len(s_pts) < 2:
        return None
    if end_s is not None:
        extra = int(math.floor((end_s + EPS - s_pts[-1]) / interval))
        if extra > 0:
            tail = s_pts[-1] + interval * np.arange(1, extra + 1)
            s_pts = np.concatenate([s_pts, tail])
            offsets = np.concatenate([offsets, np.full(extra, offsets[-1])])
    return s_pts, offsets


def _to_cartesian(path, s_pts, offsets):
    x, y, yaw = path.point_at(s_pts)
    px, py = x - np.sin(yaw) * offsets, y + np.cos(yaw) * offsets
    points = np.stack([px, py], axis=1)
    heading = np.array(yaw, float)
    d = np.diff(points, axis=0)
    moved = np.hypot(d[:, 0], d[:, 1]) > EPS
    heading[:-1] = np.where(moved, np.arctan2(d[:, 1], d[:, 0]), heading[:-1])
    heading[-1] = heading[-2]
    return points, heading


def _curvatures(points, heading, steps=1):
    steps = max(1, int(steps))
    if len(points) < steps + 2:
        return np.zeros(0)
    seg = np.hypot(*(points[steps:-1] - points[:-1 - steps]).T)
    dyaw = np.remainder(heading[steps:-1] - heading[:-1 - steps] + math.pi, 2 * math.pi) - math.pi
    return np.where(seg < EPS, 0.0, dyaw / np.maximum(seg, EPS))


def _held_profile(initial, hold_d, release_s, return_length, interval, end_s):
    # 現在の横位置 hold_d を release_s まで保ち、その後 0 へ五次曲線で戻す
    start_s = initial[0]
    settle_s = min(start_s + HOLD_SETTLE_M, end_s)
    if settle_s - start_s < interval:
        return None
    s_pts = _s_grid(start_s, end_s, interval)
    if len(s_pts) < 2:
        return None
    settle = QuinticPolynomial(initial[1], initial[2], initial[3], hold_d, 0.0, 0.0, settle_s - start_s)
    offsets = np.where(s_pts <= settle_s + EPS, settle.position(np.minimum(s_pts, settle_s) - start_s), hold_d)
    return_s = max(settle_s, release_s)
    back = QuinticPolynomial(hold_d, 0.0, 0.0, 0.0, 0.0, 0.0, max(return_length, interval))
    u = s_pts - return_s
    offsets = np.where(u > 0.0, back.position(np.clip(u, 0.0, max(return_length, interval))), offsets)
    return s_pts, offsets, return_s


def _behind_reference_start(reference, obstacles, rear):
    # 基準線の始点より手前（rear 以内）にコーンがあるか
    direction = reference[1] - reference[0]
    direction = direction / np.linalg.norm(direction)
    for x, y, _ in obstacles:
        along = float(np.dot(np.array([x, y], float) - reference[0], direction))
        if -rear <= along < 0.0:
            return True
    return False


def plan(reference_xy, ego_xy, ego_yaw, obstacles, config, left_limit=None, right_limit=None):
    result, used_rear = _plan(reference_xy, ego_xy, ego_yaw, obstacles, config, left_limit, right_limit, OBSTACLE_REAR_M)
    if used_rear and result.status in ('stop', 'fail'):
        result, _ = _plan(reference_xy, ego_xy, ego_yaw, obstacles, config, left_limit, right_limit, 0.0)
    return result


def _plan(reference_xy, ego_xy, ego_yaw, obstacles, config, left_limit, right_limit, rear):
    cfg = config
    reference = np.asarray(reference_xy, float)
    empty = np.zeros((0, 2))
    keep = np.concatenate([[True], np.linalg.norm(np.diff(reference, axis=0), axis=1) > EPS]) if len(reference) else np.zeros(0, bool)
    reference = reference[keep]
    if len(reference) < 2:
        return FrenetResult(empty, 'fail', None, 0.0), False
    left_limit = None if left_limit is None else np.asarray(left_limit, float)[keep]
    right_limit = None if right_limit is None else np.asarray(right_limit, float)[keep]

    if rear > 0.0 and _behind_reference_start(reference, obstacles, rear):
        direction = reference[1] - reference[0]
        reference = np.vstack([reference[0] - direction / np.linalg.norm(direction) * rear, reference])
        left_limit = None if left_limit is None else np.concatenate([left_limit[:1], left_limit])
        right_limit = None if right_limit is None else np.concatenate([right_limit[:1], right_limit])

    path = _Path(reference)
    start_s, start_d, path_yaw = path.project(np.asarray(ego_xy, float))
    raw_heading_error = math.remainder(ego_yaw - path_yaw, 2 * math.pi)
    if abs(raw_heading_error) > math.radians(cfg.max_reference_heading_error_deg):
        return FrenetResult(empty, 'reject', None, 0.0), False
    heading_error = float(np.clip(raw_heading_error, -MAX_HEADING_ERROR, MAX_HEADING_ERROR))
    kappa_ref = path.curvature_at(start_s, cfg.interval)
    initial = (start_s, start_d, (1.0 - kappa_ref * start_d) * math.tan(heading_error), 0.0)

    end_s = min(start_s + cfg.horizon, path.max_s)
    if end_s <= start_s + EPS:
        return FrenetResult(empty, 'fail', None, 0.0), False

    found = _find_obstacles(path, start_s, obstacles, cfg, rear)
    used_rear = any(o.s < start_s for o in found)
    ahead = [o for o in found if o.s >= start_s]

    target_s_list = []
    if ahead:
        target_s = ahead[0].s - ahead[0].half_length
        if start_s + cfg.interval * 2.0 < target_s <= end_s:
            target_s_list.append(target_s)
    for length in cfg.target_lengths:
        target_s_list.append(min(end_s, start_s + length))
        if start_s + length >= end_s:
            break
    if not target_s_list:
        target_s_list.append(end_s)

    side = None
    side_targets = {}
    if ahead:
        side = _choose_side(ahead[0], path, left_limit, right_limit, cfg)
        step = cfg.lateral_step if cfg.lateral_step > EPS else cfg.max_shift
        count = max(1, int(math.ceil(cfg.max_shift / step - EPS)))
        for name, sign in (('left', 1.0), ('right', -1.0)):
            side_targets[name] = [sign * min(k * step, cfg.max_shift) for k in range(1, count + 1)]

    kappa_full = math.tan(math.radians(cfg.steering_max_deg)) / cfg.wheelbase
    kappa_tiers = (STEERING_SAFETY_FACTOR * kappa_full, kappa_full)
    survival = cfg.tread * 0.5 + cfg.collision_margin
    curvature_steps = max(1, int(round(cfg.curvature_baseline / max(cfg.interval, EPS))))
    longitudinal = max(COLLISION_LONGITUDINAL_MIN, cfg.interval * 2.0)

    road_survival = survival if cfg.road_margin is None else cfg.tread * 0.5 + cfg.road_margin

    def relaxed_bounds(limit, s_pts, sign):
        bound = _limit_at(path.s, limit, s_pts)
        if bound is None:
            return None
        bound = bound - sign * road_survival
        start_bound = float(_limit_at(path.s, limit, start_s)) - sign * road_survival
        excess = sign * (start_d - start_bound)
        if not excess > 0.0:
            return bound
        decay = np.clip(1.0 - (s_pts - start_s) / max(cfg.boundary_relax_length, EPS), 0.0, 1.0)
        return bound + sign * excess * decay

    def check_s(target_s):
        if not found:
            return end_s
        return max(target_s, max(o.s + o.half_length for o in found) + longitudinal)

    def evaluate(s_pts, offsets, target_d, kappa_s, min_clearance, held):
        upper = relaxed_bounds(left_limit, s_pts, 1.0)
        lower = relaxed_bounds(right_limit, s_pts, -1.0)
        if upper is not None and np.any(offsets > upper):
            return None
        if lower is not None and np.any(offsets < lower):
            return None

        clearance = np.inf
        for o in found:
            near = np.abs(s_pts - o.s) <= longitudinal + o.half_length
            if near.any():
                clearance = min(clearance, float(np.min(np.abs(offsets[near] - o.d) - o.half_width)))
        if clearance <= min_clearance:
            return None

        points, heading = _to_cartesian(path, s_pts, offsets)
        kappa = _curvatures(points, heading, curvature_steps)
        feasible_kappa = kappa[s_pts[:len(kappa)] <= check_s(kappa_s)]
        cost = (cfg.weight_curvature * (np.mean(np.abs(kappa)) if len(kappa) else 0.0)
                + cfg.weight_lateral * abs(offsets[-1]))
        return (target_d, points, clearance, float(np.max(np.abs(feasible_kappa))) if len(feasible_kappa) else 0.0, cost, held)

    def generate(target_d_list):
        candidates = []
        for target_s in target_s_list:
            for target_d in target_d_list:
                profile = _lateral_profile(initial, (target_s, target_d), cfg.interval, end_s)
                if profile is None:
                    continue
                candidate = evaluate(*profile, target_d, target_s, survival, False)
                if candidate is not None:
                    candidates.append(candidate)
        return candidates

    def generate_held():
        # ずれた位置を近くのコーンを過ぎるまで保ち、その後に中心へ戻す候補
        if not found:
            return []
        band = survival + cfg.hard_margin
        blocking = [o for o in found
                    if o.s - o.half_length - start_s <= HOLD_RANGE_M and abs(o.d) - o.half_width <= band]
        if not blocking:
            return []
        release_s = max(o.s + o.half_length for o in blocking) + HOLD_RELEASE_M
        profile = _held_profile(initial, initial[1], release_s, return_length, cfg.interval, end_s)
        if profile is None:
            return []
        s_pts, offsets, return_s = profile
        graze = cfg.tread * 0.5 + HOLD_GRAZE_MARGIN_M
        candidate = evaluate(s_pts, offsets, float(initial[1]), return_s, graze, True)
        return [] if candidate is None else [candidate]

    def result_of(c, used_side):
        if c[5]:
            nearest = min(found, key=lambda o: abs(o.s - start_s))
            return FrenetResult(c[1], 'avoid', 'left' if c[0] >= nearest.d else 'right', float(c[0]))
        if c[0] == 0.0:
            return FrenetResult(c[1], 'clear', None, 0.0)
        return FrenetResult(c[1], 'avoid', used_side, float(c[0]))

    def select(candidates, used_side, kappa_max):
        for margin in (survival + cfg.hard_margin, survival):
            best = None
            for c in candidates:
                if c[3] <= kappa_max and c[2] > margin and (best is None or (not c[5], c[4]) < (not best[5], best[4])):
                    best = c
            if best is not None:
                return result_of(best, used_side)
        return None

    return_length = cfg.target_lengths[0] if cfg.target_lengths else cfg.horizon
    held = generate_held()
    preferred = held + generate([0.0] + side_targets.get(side, []))
    opposite_side = None if not ahead else ('right' if side == 'left' else 'left')
    opposite = generate(side_targets[opposite_side]) if ahead else []
    for kappa_max in kappa_tiers:
        result = select(preferred, side, kappa_max)
        if result is not None:
            return result, used_rear
        if ahead:
            result = select(opposite, opposite_side, kappa_max)
            if result is not None:
                return result, used_rear
    for kappa_max in kappa_tiers:
        for c in held:
            if c[3] <= kappa_max:
                return result_of(c, side), used_rear
    for c in preferred:
        if c[0] == 0.0 and not c[5]:
            return FrenetResult(c[1], 'clear', None, 0.0), used_rear

    if ahead:
        stop_end = max(start_s + cfg.interval, ahead[0].s - cfg.stop_standoff)
        profile = _lateral_profile(initial, (stop_end, initial[1]), cfg.interval)
        if profile is not None:
            points, _ = _to_cartesian(path, *profile)
            return FrenetResult(points, 'stop', None, float(initial[1])), used_rear
    return FrenetResult(empty, 'fail', None, 0.0), used_rear

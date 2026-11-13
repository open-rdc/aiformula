import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

_EARTH_RADIUS_MIN = 6300000.0
_EARTH_RADIUS_MAX = 6400000.0


def ecef_to_local(ecef):
    ecef = np.asarray(ecef, float)
    origin = ecef[0]
    origin_radius = float(np.linalg.norm(origin))
    if not (_EARTH_RADIUS_MIN <= origin_radius <= _EARTH_RADIUS_MAX):
        raise ValueError(f'ecef[0] のノルムが地球半径から外れています ({origin_radius:.1f} m)。先頭サンプルが破損している可能性があります')

    up = origin / origin_radius
    east = np.cross([0.0, 0.0, 1.0], up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)

    delta = ecef - origin
    return np.stack([delta @ east, delta @ north], axis=1)


def speed_mask(times, ecef, xy, max_speed=15.0):
    times = np.asarray(times, float)
    xy = np.asarray(xy, float)

    radius = np.linalg.norm(np.asarray(ecef, float), axis=1)
    mask = (radius >= _EARTH_RADIUS_MIN) & (radius <= _EARTH_RADIUS_MAX)

    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    speed = step / np.maximum(np.diff(times), 1e-06)
    mask[1:] &= speed <= max_speed

    return mask


def resample_centerlines(vector_map, step=0.2):
    points, tangents = [], []
    for polyline in vector_map.centerlines.values():
        length = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(polyline, axis=0), axis=1))]
        if length[-1] < step:
            continue

        sample = np.arange(0.0, length[-1], step)
        dense = np.stack([np.interp(sample, length, polyline[:, 0]), np.interp(sample, length, polyline[:, 1])], axis=1)

        tangent = np.gradient(dense, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-09)

        points.append(dense)
        tangents.append(tangent)

    return np.vstack(points), np.vstack(tangents)


def _rotation(theta):
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def fit_to_map(xy, points, tangents, direction_gain=2.0, stride=5):
    tree = cKDTree(points)
    sample = np.asarray(xy, float)[::stride]

    def cost(parameters):
        dx, dy, theta = parameters
        moved = sample @ _rotation(theta).T + [dx, dy]
        distance, index = tree.query(moved)
        value = float(np.mean(np.minimum(distance, 5.0) ** 2))

        tangent = np.gradient(moved, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-09)
        agreement = np.sum(tangent * tangents[index], axis=1)

        return value + direction_gain * float(np.mean(np.maximum(0.0, -agreement)))

    best, best_cost = None, np.inf
    for start in np.linspace(-np.pi, np.pi, 24, endpoint=False):
        shift = points.mean(axis=0) - (sample @ _rotation(start).T).mean(axis=0)
        result = minimize(cost, [shift[0], shift[1], start], method='Nelder-Mead',
                          options={'maxiter': 3000, 'xatol': 0.0001, 'fatol': 1e-06})
        if result.fun < best_cost:
            best, best_cost = result.x, result.fun

    if best is None:
        raise ValueError('fit_to_map: すべての初期回転で最適化が発散し、有効な解が得られませんでした')

    return float(best[0]), float(best[1]), float(best[2])


def apply_fit(xy, fit):
    dx, dy, theta = fit
    return np.asarray(xy, float) @ _rotation(theta).T + [dx, dy]


def fit_quality(fitted, points, tangents):
    fitted = np.asarray(fitted, float)
    distance, index = cKDTree(points).query(fitted)

    tangent = np.gradient(fitted, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-09)
    agreement = np.sum(tangent * tangents[index], axis=1)

    return float(np.median(distance)), float((agreement > 0.0).mean())


def headings(times, xy, window=0.5, min_travel=0.1):
    times = np.asarray(times, float)
    xy = np.asarray(xy, float)
    half = window / 2.0

    heading = np.zeros(len(xy))
    ok = np.zeros(len(xy), bool)

    lo = np.searchsorted(times, times - half, side='left')
    hi = np.searchsorted(times, times + half, side='right') - 1

    last = 0.0
    established = False
    for index in range(len(xy)):
        delta = xy[min(hi[index], len(xy) - 1)] - xy[max(lo[index], 0)]
        if float(np.hypot(*delta)) >= min_travel:
            last = float(np.arctan2(delta[1], delta[0]))
            established = True
        heading[index] = last
        ok[index] = established

    return heading, ok


def interpolate_pose(times, xy, heading, ok, t, max_gap=0.25):  # 0.2 だと publish 周期ちょうどを丸めで落とす
    times = np.asarray(times, float)
    if t < times[0] or t > times[-1]:
        return None

    index = int(np.clip(np.searchsorted(times, t), 1, len(times) - 1))
    span = times[index] - times[index - 1]
    if span > max_gap:
        return None

    ratio = 0.0 if span <= 0 else (t - times[index - 1]) / span
    position = xy[index - 1] + ratio * (xy[index] - xy[index - 1])

    if ok[index - 1] and ok[index]:
        base = heading[index - 1]
        difference = (heading[index] - base + np.pi) % (2 * np.pi) - np.pi
        angle = base + ratio * difference
    elif ok[index - 1]:
        angle = heading[index - 1]
    elif ok[index]:
        angle = heading[index]
    else:
        return None

    return position, float(angle)

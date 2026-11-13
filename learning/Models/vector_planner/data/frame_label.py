import json
from pathlib import Path

import numpy as np
import yaml

from Models.vector_planner.data.frenet_label import FrenetConfig, plan
from Models.vector_planner.data.map_labels import (
    DEFAULT_PARAMS, SMOOTH_WINDOW, densify, lateral_limits, local_map_elements, sample_path_base, smooth_reference,
    to_base, trace_slots)

OBSERVABLE_SUBTYPES = ('solid', 'road_border')
MAX_BOUNDARY_DISTANCE_M = 8.0
NUM_POINTS, SPACING = 40, 0.5
SANITY_RANGE_X_M = 10.0
MIN_CORRIDOR_WIDTH_M = 1.2
MAX_CENTER_STEP_M = 1.0
PLAN_HORIZON_M = 21.0
TRACE_HORIZON_M = 25.0
MIN_VALID_LIMIT_FRACTION = 0.5


def to_world(points, position, yaw):
    cos, sin = np.cos(yaw), np.sin(yaw)
    points = np.asarray(points, float)
    return np.stack([points[:, 0] * cos - points[:, 1] * sin, points[:, 0] * sin + points[:, 1] * cos],
                    axis=1) + np.asarray(position, float)


def _nearest_on_polyline(polyline, queries):
    dense = densify(polyline, 0.1)
    d = np.linalg.norm(dense[None] - queries[:, None], axis=2)
    index = d.argmin(axis=1)
    return dense[index], d[np.arange(len(queries)), index]


def _centerline_distance(line_map_centerlines, lanelet_id, queries):
    return _nearest_on_polyline(line_map_centerlines[lanelet_id], queries)[1]


def corridor_is_sane(c):
    # 幅が狭すぎる／隣接点間で中心が横に飛ぶ走行帯を弾く
    left, right = np.asarray(c['left']), np.asarray(c['right'])
    rows = np.asarray(c['valid'], bool) & (0.0 <= left[:, 0]) & (left[:, 0] <= SANITY_RANGE_X_M)
    if not rows.any():
        return True
    if (left[rows, 1] - right[rows, 1]).min() < MIN_CORRIDOR_WIDTH_M:
        return False
    center = 0.5 * (left[rows, 1] + right[rows, 1])
    return len(center) < 2 or float(np.abs(np.diff(center)).max()) <= MAX_CENTER_STEP_M


def corridor_for_ids(line_map, vector_map, ids, reference, position, yaw):
    empty = {'left': np.zeros((NUM_POINTS, 2)), 'right': np.zeros((NUM_POINTS, 2)),
             'valid': np.zeros(NUM_POINTS, bool), 'observable': np.zeros(NUM_POINTS, bool)}
    base, valid = sample_path_base(np.vstack([position, reference]), position, yaw, NUM_POINTS, SPACING)
    if not valid.any():
        return empty
    world = to_world(base, position, yaw)

    dist = np.stack([_centerline_distance(vector_map.centerlines, i, world) for i in ids], axis=1)
    owner = dist.argmin(axis=1)
    out = {k: v.copy() for k, v in empty.items()}
    for k, lanelet_id in enumerate(ids):
        rows = np.flatnonzero(valid & (owner == k))
        lanelet = line_map.lanelets.get(lanelet_id)
        if len(rows) == 0 or lanelet is None:
            continue
        sides = {}
        for name, way_id in (('left', lanelet.left_id), ('right', lanelet.right_id)):
            line = line_map.lines.get(way_id)
            if line is None or len(line.points) < 2:
                break
            foot, distance = _nearest_on_polyline(line.points, world[rows])
            sides[name] = (foot, distance, line.subtype in OBSERVABLE_SUBTYPES)
        if len(sides) < 2:
            continue
        ok = (sides['left'][1] <= MAX_BOUNDARY_DISTANCE_M) & (sides['right'][1] <= MAX_BOUNDARY_DISTANCE_M)
        rows = rows[ok]
        for name in ('left', 'right'):
            out[name][rows] = to_base(sides[name][0][ok], position, yaw)
        out['valid'][rows] = True
        out['observable'][rows] = sides['left'][2] and sides['right'][2]
    return out


def _ray_hits(points, origins, normals):
    # origins から normals 方向（符号付き）に points の折れ線へ伸ばした交点までの符号付き距離。交差なしは nan
    a, b = points[:-1], points[1:]
    seg = b - a
    rel = a[None] - origins[:, None]
    denom = normals[:, None, 0] * seg[None, :, 1] - normals[:, None, 1] * seg[None, :, 0]
    ok = np.abs(denom) > 1e-12
    safe = np.where(ok, denom, 1.0)
    t = (rel[..., 0] * seg[None, :, 1] - rel[..., 1] * seg[None, :, 0]) / safe
    u = (rel[..., 0] * normals[:, None, 1] - rel[..., 1] * normals[:, None, 0]) / safe
    hit = ok & (u >= -1e-9) & (u <= 1 + 1e-9) & (np.abs(t) <= MAX_BOUNDARY_DISTANCE_M)
    return np.where(hit, t, np.nan)


def corridor_limits(line_map, vector_map, ids, reference):
    # 参照線に沿った左右の横限界を走行帯と同じ lanelet 境界から作る。符号は lateral_limits と同じ（左 +、右 -）、取れない点は nan
    reference = np.asarray(reference, float)
    n = len(reference)
    left, right = np.full(n, np.nan), np.full(n, np.nan)
    if n < 2 or not len(ids):
        return left, right
    tangent = np.gradient(reference, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)

    dist = np.stack([_centerline_distance(vector_map.centerlines, i, reference) for i in ids], axis=1)
    owner = dist.argmin(axis=1)
    for k, lanelet_id in enumerate(ids):
        rows = np.flatnonzero(owner == k)
        lanelet = line_map.lanelets.get(lanelet_id)
        if len(rows) == 0 or lanelet is None:
            continue
        for out, way_id, sign in ((left, lanelet.left_id, 1.0), (right, lanelet.right_id, -1.0)):
            line = line_map.lines.get(way_id)
            if line is None or len(line.points) < 2:
                continue
            t = _ray_hits(np.asarray(line.points, float), reference[rows], normal[rows]) * sign
            t = np.where(t > 0, t, np.nan)
            with np.errstate(all='ignore'):
                best = np.nanmin(np.where(np.isnan(t), np.inf, t), axis=1)
            out[rows] = np.where(np.isinf(best), np.nan, sign * best)
    return left, right


def frame_corridors(line_map, vector_map, position, yaw, horizon=TRACE_HORIZON_M, smooth_window=SMOOTH_WINDOW):
    position = np.asarray(position, float)
    result = {}
    for slot, (path, sequence) in trace_slots(vector_map, position, yaw, horizon=horizon).items():
        c = corridor_for_ids(line_map, vector_map, sequence,
                              smooth_reference(densify(path, SPACING), smooth_window), position, yaw)
        if not corridor_is_sane(c):
            continue
        result[slot] = {'left': c['left'].tolist(), 'right': c['right'].tolist(),
                        'valid': c['valid'].tolist(), 'observable': c['observable'].tolist()}
    return result


def load_vehicle_params(yaml_path):
    with open(yaml_path) as f:
        common = yaml.safe_load(f)['/**']['ros__parameters']
    return {'tread': float(common['tread']), 'wheelbase': float(common['wheelbase']),
            'steering_max_deg': float(common['steering_max']['pos'])}


def parse_lengths(text):
    return [float(v) for v in text.split(',')] if text else None


def make_config(yaml_path=DEFAULT_PARAMS, teacher_lengths=None):
    lengths = [5.0, 10.0, 15.0] if not teacher_lengths else [float(v) for v in teacher_lengths]
    return FrenetConfig(target_lengths=lengths, horizon=PLAN_HORIZON_M, **load_vehicle_params(yaml_path))


def _limits(line_map, vector_map, reference, ids, position, yaw, limits_source):
    if limits_source == 'corridor' and ids is not None:
        left, right = corridor_limits(line_map, vector_map, ids, reference)
        valid = np.isfinite(left) & np.isfinite(right)
        if valid.mean() >= MIN_VALID_LIMIT_FRACTION and corridor_is_sane(
                corridor_for_ids(line_map, vector_map, ids, reference, position, yaw)):
            return left, right
    return lateral_limits(line_map, reference)


def build_frame_label(line_map, vector_map, position, yaw, obstacles, cfg, corridors=True, slots=None, cache=None,
                      map_elements=None, traces=None, smooth_window=SMOOTH_WINDOW, limits_source='corridor'):
    # cache: slot -> (reference, left, right)。障害物だけ変えた再計画で走行可能幅の計算を使い回す。traces: (points, ids) か points。
    # limits_source 'corridor' は走行帯と同じ lanelet 境界、不健全なら白線ベース（lateral_limits）にフォールバック
    position = np.asarray(position, float)
    label = {
        'map_elements': local_map_elements(line_map, position, yaw) if map_elements is None else map_elements,
        'paths': {},
        'boxes': [],
        'has_boxes': False,
    }
    traced = trace_slots(vector_map, position, yaw, horizon=TRACE_HORIZON_M) if traces is None else traces
    for slot, entry in traced.items():
        polyline, ids = entry if isinstance(entry, tuple) else (entry, None)
        if slots is not None and slot not in slots:
            continue
        if cache is not None and slot in cache:
            reference, left, right = cache[slot]
        else:
            reference = smooth_reference(densify(polyline), smooth_window)
            left, right = _limits(line_map, vector_map, reference, ids, position, yaw, limits_source)
            if cache is not None:
                cache[slot] = (reference, left, right)
        result = plan(reference, position, yaw, obstacles, cfg, left, right)
        if len(result.points_xy):
            xy, valid = sample_path_base(np.vstack([position, result.points_xy]), position, yaw)
        else:
            xy, valid = np.zeros((40, 2)), np.zeros(40, bool)
        label['paths'][slot] = {'xy': xy.tolist(), 'valid': valid.tolist(), 'status': result.status, 'side': result.side}
    if corridors:
        for slot, corridor in frame_corridors(line_map, vector_map, position, yaw, TRACE_HORIZON_M, smooth_window).items():
            if slot in label['paths']:
                label['paths'][slot]['corridor'] = corridor
    return label


def load_json(path, default):
    return json.loads(Path(path).read_text()) if Path(path).exists() else default


def load_frame_meta(root):
    root = Path(root)
    return (load_json(root / 'poses.json', {}).get('frames', {}), load_json(root / 'gnss.json', {}).get('frames', {}),
            load_json(root / 'obstacles.json', {}))

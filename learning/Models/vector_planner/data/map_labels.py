import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from Models.common.vectormap import (
    SLOT_NAMES, _drop_consecutive_duplicates, _nearest_point_on_polyline, _project_onto_polyline, _walk, _walk_ids,
    locate_lanelet, trace_lanelet_ids, trace_paths)

DEFAULT_PARAMS = Path(__file__).resolve().parents[4] / 'main_executor' / 'config' / 'main_params.yaml'
CLASS_BY_SUBTYPE = {'solid': 'solid', 'dashed': 'dashed', 'road_border': 'road_border'}
MIN_LENGTH_M = 1.0
VERTICAL_DX_M = 0.05


@dataclass
class Line:
    id: int
    points: np.ndarray
    marking_type: str
    subtype: str
    is_observable: bool


@dataclass
class Lanelet:
    id: int
    subtype: str
    left_id: int = None
    right_id: int = None
    centerline_id: int = None


@dataclass
class LineMap:
    lines: dict = field(default_factory=dict)
    lanelets: dict = field(default_factory=dict)


def load_line_map(osm_path):
    root = ET.parse(osm_path).getroot()
    nodes = {n.get('id'): (float(n.get('x')), float(n.get('y'))) for n in root.findall('node')}
    line_map = LineMap()

    for way in root.findall('way'):
        tags = {t.get('k'): t.get('v') for t in way.findall('tag')}
        points = np.array([nodes[nd.get('ref')] for nd in way.findall('nd')], float).reshape(-1, 2)
        line_map.lines[int(way.get('id'))] = Line(
            int(way.get('id')), points, tags.get('marking_type'), tags.get('subtype'),
            tags.get('is_observable') == 'true')

    for relation in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in relation.findall('tag')}
        if tags.get('type') != 'lanelet':
            continue
        members = {m.get('role'): int(m.get('ref')) for m in relation.findall('member')}
        line_map.lanelets[int(relation.get('id'))] = Lanelet(
            int(relation.get('id')), tags.get('subtype'),
            members.get('left'), members.get('right'), members.get('centerline'))

    return line_map


def to_base(points, position, yaw):
    relative = np.asarray(points, float) - np.asarray(position, float)
    cos, sin = np.cos(-yaw), np.sin(-yaw)
    return np.stack([relative[:, 0] * cos - relative[:, 1] * sin,
                     relative[:, 0] * sin + relative[:, 1] * cos], axis=1)


def _clip_segment(a, b, x_range, y_range):
    t0, t1 = 0.0, 1.0
    delta = b - a
    for p, q in ((-delta[0], a[0] - x_range[0]), (delta[0], x_range[1] - a[0]),
                 (-delta[1], a[1] - y_range[0]), (delta[1], y_range[1] - a[1])):
        if abs(p) < 1e-12:
            if q < 0:
                return None
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return None
    return t0, t1


def _clip_polyline(points, x_range, y_range):
    pieces, current = [], []

    def flush():
        nonlocal current
        if len(current) >= 2:
            pieces.append(np.array(current))
        current = []

    for a, b in zip(points[:-1], points[1:]):
        clipped = _clip_segment(a, b, x_range, y_range)
        if clipped is None:
            flush()
            continue
        t0, t1 = clipped
        start, end = a + t0 * (b - a), a + t1 * (b - a)
        if current and np.linalg.norm(current[-1] - start) > 1e-9:
            flush()
        if not current:
            current.append(start)
        current.append(end)
        if t1 < 1.0:
            flush()
    flush()
    return pieces


def _resample(points, num_points):
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    targets = np.linspace(0.0, arc[-1], num_points)
    return np.stack([np.interp(targets, arc, points[:, 0]), np.interp(targets, arc, points[:, 1])], axis=1)


def _normalize_direction(points):
    dx = points[-1, 0] - points[0, 0]
    reverse = points[0, 1] > points[-1, 1] if abs(dx) < VERTICAL_DX_M else dx < 0
    return points[::-1] if reverse else points


def local_map_elements(line_map, position, yaw, x_range=(0.0, 25.0), y_range=(-8.0, 8.0), num_points=20):
    elements = []
    for line in line_map.lines.values():
        cls = CLASS_BY_SUBTYPE.get(line.subtype)
        if cls is None or not line.is_observable or len(line.points) < 2:
            continue
        base = to_base(line.points, position, yaw)
        for piece in _clip_polyline(base, x_range, y_range):
            if np.sum(np.linalg.norm(np.diff(piece, axis=0), axis=1)) < MIN_LENGTH_M:
                continue
            points = _normalize_direction(_resample(piece, num_points))
            elements.append({'class': cls, 'points': points.tolist()})
    return elements


REFERENCE_STEP_M = 0.5


def densify(polyline, step=REFERENCE_STEP_M):
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(polyline, axis=0), axis=1))])
    samples = np.unique(np.concatenate([np.arange(0.0, arc[-1], step), [arc[-1]]]))
    return np.stack([np.interp(samples, arc, polyline[:, 0]), np.interp(samples, arc, polyline[:, 1])], axis=1)


def _dedupe(points):
    points = np.asarray(points, float)
    keep = np.concatenate([[True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9])
    return points[keep]


def _arc_position_of_projection(points, arc, position):
    a, b = points[:-1], points[1:]
    seg = b - a
    length_sq = np.sum(seg ** 2, axis=1)
    t = np.clip(np.sum((position - a) * seg, axis=1) / length_sq, 0.0, 1.0)
    foot = a + t[:, None] * seg
    i = int(np.argmin(np.linalg.norm(foot - position, axis=1)))
    return arc[i] + t[i] * np.sqrt(length_sq[i])


def sample_path_base(points_xy, position, yaw, num_points=40, spacing=0.5):
    xy = np.zeros((num_points, 2))
    valid = np.zeros(num_points, bool)
    points = _dedupe(points_xy)
    if len(points) < 2:
        return xy, valid

    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    start = _arc_position_of_projection(points, arc, np.asarray(position, float))
    targets = start + spacing * np.arange(1, num_points + 1)
    valid = targets <= arc[-1] + 1e-9
    sampled = np.stack([np.interp(targets, arc, points[:, 0]), np.interp(targets, arc, points[:, 1])], axis=1)
    xy[valid] = to_base(sampled[valid], position, yaw)
    return xy, valid


def lateral_limits(line_map, reference_xy, search=5.0):
    reference = np.asarray(reference_xy, float)
    n = len(reference)
    left = np.full(n, np.inf)
    right = np.full(n, np.inf)
    if n < 2:
        return np.full(n, np.nan), np.full(n, np.nan)

    tangent = np.gradient(reference, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)

    for line in line_map.lines.values():
        if line.subtype not in ('road_border', 'solid') or len(line.points) < 2:
            continue
        a, b = line.points[:-1], line.points[1:]
        seg = b - a
        rel = a[None, :, :] - reference[:, None, :]
        denom = normal[:, None, 0] * seg[None, :, 1] - normal[:, None, 1] * seg[None, :, 0]
        ok = np.abs(denom) > 1e-12
        safe = np.where(ok, denom, 1.0)
        t = (rel[..., 0] * seg[None, :, 1] - rel[..., 1] * seg[None, :, 0]) / safe
        u = (rel[..., 0] * normal[:, None, 1] - rel[..., 1] * normal[:, None, 0]) / safe
        hit = ok & (u >= -1e-9) & (u <= 1 + 1e-9) & (np.abs(t) <= search)
        left = np.minimum(left, np.where(hit & (t > 0), t, np.inf).min(axis=1))
        right = np.minimum(right, np.where(hit & (t <= 0), -t, np.inf).min(axis=1))

    left = np.where(np.isinf(left), np.nan, left)
    right = np.where(np.isinf(right), np.nan, -right)
    return left, right


SMOOTH_WINDOW = 7


def smooth_reference(points, window=SMOOTH_WINDOW):
    # 参照経路の移動平均（端点固定・点数不変）。lanelet 接続部の折れ点による曲率スパイクを除く
    points = np.asarray(points, float)
    half = max(int(window) // 2, 0)
    n = len(points)
    if half == 0 or n < 3:
        return points.copy()
    csum = np.vstack([np.zeros((1, points.shape[1])), np.cumsum(points, axis=0)])
    out = np.empty_like(points)
    for i in range(n):
        k = min(half, i, n - 1 - i)
        out[i] = (csum[i + k + 1] - csum[i - k]) / (2 * k + 1)
    out[0], out[-1] = points[0], points[-1]
    return out


MAX_SIBLING_ANGLE_RAD = np.deg2rad(60.0)


def _branch_parent(vector_map, lanelet_id):
    for parent, outs in vector_map.connections.items():
        if len(outs) >= 2 and any(to_id == lanelet_id for to_id, _ in outs):
            return parent
    return None


def _trunk(vector_map, lanelet_id, index, position, horizon):
    start, next_index = _project_onto_polyline(vector_map.centerlines[lanelet_id], index, position)
    points, remaining, terminal, visited = _walk(vector_map, lanelet_id, start, next_index, horizon)
    ids = _walk_ids(vector_map, lanelet_id, next_index, horizon, start)[0]
    return points, ids, remaining, terminal, visited


def _with_branches(vector_map, lanelet_id, index, position, horizon):
    # trace_paths / trace_lanelet_ids と同じ分岐処理を、指定 lanelet・セグメントから開始して行う
    points, ids, remaining, terminal, visited = _trunk(vector_map, lanelet_id, index, position, horizon)
    successors = [s for s in vector_map.connections.get(terminal, []) if s[0] not in visited]
    if remaining <= 0.0 or len(successors) < 2:
        return None, (points, ids)

    paths = {}
    for to_id, turn in successors:
        if turn not in SLOT_NAMES:
            continue
        polyline = vector_map.centerlines[to_id]
        branch_points = _walk(vector_map, to_id, polyline[0], 1, remaining)[0]
        branch_ids = _walk_ids(vector_map, to_id, 1, remaining, polyline[0])[0]
        merged = _drop_consecutive_duplicates(np.vstack([points, branch_points]))
        if turn not in paths or len(merged) > len(paths[turn][0]):
            paths[turn] = (merged, ids + branch_ids)

    return (paths or None), (points, ids)


def trace_slots(vector_map, position, heading, horizon=20.0, max_distance=5.0, sibling_distance=1.5):
    # slot -> (points_world, lanelet_ids)。交差点内は兄弟 lanelet も turn_direction で枠に割り当て、枠の入れ替わりを防ぐ
    position = np.asarray(position, float)
    located = locate_lanelet(vector_map, position, heading, max_distance)
    if located is None:
        return {}

    lanelet_id, index = located
    parent = _branch_parent(vector_map, lanelet_id)
    if parent is None:
        paths = trace_paths(vector_map, position, heading, horizon, max_distance)
        ids = trace_lanelet_ids(vector_map, position, heading, horizon, max_distance)
        return {slot: (points, ids[slot]) for slot, points in paths.items()}

    direction = np.array([np.cos(heading), np.sin(heading)])
    result = {}

    def offer(slot, points, ids):
        if slot not in result or len(points) > len(result[slot][0]):
            result[slot] = (points, ids)

    for to_id, turn in vector_map.connections[parent]:
        if to_id == lanelet_id:
            branches, (points, ids) = _with_branches(vector_map, lanelet_id, index, position, horizon)
            if branches is not None:
                for slot, (p, i) in branches.items():
                    offer(slot, p, i)
            else:
                offer(turn if turn in SLOT_NAMES else 'straight', points, ids)
            continue

        if turn not in SLOT_NAMES:
            continue
        nearest = _nearest_point_on_polyline(vector_map.centerlines[to_id], position)
        if nearest is None:
            continue
        distance, _, sibling_index, tangent = nearest
        if distance > sibling_distance or float(np.dot(tangent, direction)) < np.cos(MAX_SIBLING_ANGLE_RAD):
            continue
        points, ids = _trunk(vector_map, to_id, sibling_index, position, horizon)[:2]
        offer(turn, points, ids)

    return result

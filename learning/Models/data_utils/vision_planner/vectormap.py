import xml.etree.ElementTree as ET

import numpy as np

SLOT_NAMES = ('straight', 'left', 'right')



class VectorMap:
    def __init__(self, centerlines, connections):
        self.centerlines = centerlines
        self.connections = connections


def load_vector_map(osm_path):
    root = ET.parse(osm_path).getroot()
    nodes = {n.get('id'): (float(n.get('x')), float(n.get('y'))) for n in root.findall('node')}
    ways = {w.get('id'): np.array([nodes[nd.get('ref')] for nd in w.findall('nd')], float) for w in root.findall('way')}

    centerlines, connections = {}, {}
    for relation in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in relation.findall('tag')}
        members = {m.get('role'): m.get('ref') for m in relation.findall('member')}

        if tags.get('type') == 'lanelet' and 'centerline' in members:
            centerlines[int(relation.get('id'))] = ways[members['centerline']]
            continue

        if tags.get('type') == 'lane_connection':
            connections.setdefault(int(members['from']), []).append((int(members['to']), tags.get('turn_direction', 'straight')))

    return VectorMap(centerlines, connections)


def _nearest_point_on_polyline(polyline, position):
    best = None
    for i in range(len(polyline) - 1):
        a, b = polyline[i], polyline[i + 1]
        segment = b - a
        length = float(np.linalg.norm(segment))
        if length < 1e-9:
            continue

        tangent = segment / length
        t = min(max(float(np.dot(position - a, segment) / (length * length)), 0.0), 1.0)
        point = a + t * segment
        distance = float(np.linalg.norm(point - position))
        if best is None or distance < best[0]:
            best = (distance, point, i, tangent)

    return best


def locate_lanelet(vector_map, position, heading, max_distance=5.0):
    direction = np.array([np.cos(heading), np.sin(heading)])
    best, best_distance = None, max_distance
    for lanelet_id, polyline in vector_map.centerlines.items():
        nearest = _nearest_point_on_polyline(polyline, position)
        if nearest is None:
            continue

        distance, _, index, tangent = nearest
        if float(np.dot(tangent, direction)) <= 0.0:
            continue

        if distance < best_distance:
            best, best_distance = (lanelet_id, index), distance

    return best


def _project_onto_polyline(polyline, index, position):
    a, b = polyline[index], polyline[index + 1]
    segment = b - a
    length_sq = float(np.dot(segment, segment))
    t = 0.0 if length_sq < 1e-12 else float(np.dot(position - a, segment) / length_sq)
    t = min(max(t, 0.0), 1.0)
    point = a + t * segment
    return point, index + 1


def _drop_consecutive_duplicates(points):
    kept = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - kept[-1]) > 1e-9:
            kept.append(point)

    return np.array(kept)


def _walk(vector_map, lanelet_id, start_point, next_index, remaining):
    points = [start_point]
    visited = {lanelet_id}
    current, index = lanelet_id, next_index

    while True:
        polyline = vector_map.centerlines[current]
        while index < len(polyline):
            remaining -= float(np.linalg.norm(polyline[index] - points[-1]))
            points.append(polyline[index])
            index += 1
            if remaining <= 0.0:
                return _drop_consecutive_duplicates(np.array(points)), 0.0, current, visited

        successors = [s for s in vector_map.connections.get(current, []) if s[0] not in visited]
        if len(successors) != 1:
            return _drop_consecutive_duplicates(np.array(points)), remaining, current, visited

        current = successors[0][0]
        visited.add(current)
        index = 0


def trace_paths(vector_map, position, heading, horizon=20.0, max_distance=5.0):
    located = locate_lanelet(vector_map, position, heading, max_distance)
    if located is None:
        return {}

    lanelet_id, index = located
    start_point, next_index = _project_onto_polyline(vector_map.centerlines[lanelet_id], index, position)

    trunk, remaining, terminal, visited = _walk(vector_map, lanelet_id, start_point, next_index, horizon)
    successors = [s for s in vector_map.connections.get(terminal, []) if s[0] not in visited]

    if remaining <= 0.0 or len(successors) < 2:
        return {'straight': trunk}

    paths = {}
    for to_id, turn in successors:
        if turn not in SLOT_NAMES:
            continue

        branch_polyline = vector_map.centerlines[to_id]
        branch, _, _, _ = _walk(vector_map, to_id, branch_polyline[0], 1, remaining)
        merged = _drop_consecutive_duplicates(np.vstack([trunk, branch]))

        if turn not in paths or len(merged) > len(paths[turn]):
            paths[turn] = merged

    return paths or {'straight': trunk}

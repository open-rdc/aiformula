import copy

import cv2
import numpy as np

from Models.common.projection import load_camera
from Models.vector_planner.data.map_labels import DEFAULT_PARAMS, MIN_LENGTH_M, _clip_polyline, _normalize_direction, _resample

REF_HEIGHT, REF_WIDTH = 384, 640
MAP_X_RANGE, MAP_Y_RANGE = (0.0, 25.0), (-8.0, 8.0)
MIN_BOX_VISIBLE = 0.3
MIN_BOX_SIDE_PX = 2.0
SWAP = {'left': 'right', 'right': 'left'}
SLOT_SWAP = {'straight': 'straight', 'left': 'right', 'right': 'left'}


def load_aug_camera(params_path=DEFAULT_PARAMS):
    return load_camera(params_path, 'nHD', REF_HEIGHT, REF_WIDTH)


def _intrinsics(camera):
    return np.array([[camera.fx, 0.0, camera.cx], [0.0, camera.fy, camera.cy], [0.0, 0.0, 1.0]])


def yaw_homography(camera, yaw_deg):
    # カメラを中心回りに yaw_deg（反時計回り +）回したときの 参照解像度 画素 -> 画素
    psi = np.radians(yaw_deg)
    c, s = np.cos(-psi), np.sin(-psi)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    k = _intrinsics(camera)
    r = camera.rotation
    return k @ r.T @ rz @ r @ np.linalg.inv(k)


def flip_homography(camera):
    return np.array([[-1.0, 0.0, 2.0 * camera.cx], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


def scale_homography(h, size):
    height, width = size
    s = np.diag([width / REF_WIDTH, height / REF_HEIGHT, 1.0])
    return s @ h @ np.linalg.inv(s)


def transform_image(image, h):
    height, width = image.shape[:2]
    return cv2.warpPerspective(image, h, (width, height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                               borderValue=(0, 0, 0))


def rotate_points(points, center_xy, yaw_deg):
    # 画像の yaw 回転に対応する、base_link 点のカメラ位置回りの回転（シーンが -yaw 回る）
    psi = np.radians(yaw_deg)
    c, s = np.cos(-psi), np.sin(-psi)
    rel = np.asarray(points, float) - np.asarray(center_xy, float)
    return np.stack([c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]], axis=1) + np.asarray(center_xy, float)


def _points(points, flip, center, yaw_deg):
    out = rotate_points(np.asarray(points, float).reshape(-1, 2), center, yaw_deg)
    if flip:
        out[:, 1] = -out[:, 1]
    return out


def _map_elements(elements, flip, center, yaw_deg):
    result = []
    for element in elements:
        moved = _points(element['points'], flip, center, yaw_deg)
        for piece in _clip_polyline(moved, MAP_X_RANGE, MAP_Y_RANGE):
            if np.sum(np.linalg.norm(np.diff(piece, axis=0), axis=1)) < MIN_LENGTH_M:
                continue
            n = len(element['points'])
            result.append({**element, 'points': _normalize_direction(_resample(piece, n)).tolist()})
    return result


def _path_entry(entry, flip, center, yaw_deg):
    out = dict(entry)
    valid = np.asarray(entry['valid'], bool)

    def move(points):
        moved = np.zeros((len(valid), 2))
        if valid.any():
            moved[valid] = _points(np.asarray(points, float)[valid], flip, center, yaw_deg)
        return moved.tolist()

    out['xy'] = move(entry['xy'])
    if flip and entry.get('side') in SWAP:
        out['side'] = SWAP[entry['side']]
    corridor = entry.get('corridor')
    if corridor is not None:
        cvalid = np.asarray(corridor['valid'], bool)

        def move_c(points):
            moved = np.zeros((len(cvalid), 2))
            if cvalid.any():
                moved[cvalid] = _points(np.asarray(points, float)[cvalid], flip, center, yaw_deg)
            return moved.tolist()

        left, right = move_c(corridor['left']), move_c(corridor['right'])
        if flip:
            left, right = right, left
        out['corridor'] = {**corridor, 'left': left, 'right': right}
    return out


def _boxes(boxes, h):
    out = []
    for box in boxes:
        x0, x1 = (box['cx'] - box['w'] / 2) * REF_WIDTH, (box['cx'] + box['w'] / 2) * REF_WIDTH
        y0, y1 = (box['cy'] - box['h'] / 2) * REF_HEIGHT, (box['cy'] + box['h'] / 2) * REF_HEIGHT
        corners = np.array([[x0, y0, 1.0], [x1, y0, 1.0], [x1, y1, 1.0], [x0, y1, 1.0]])
        mapped = (h @ corners.T).T
        mapped = mapped[:, :2] / mapped[:, 2:]
        nx0, ny0 = mapped.min(axis=0)
        nx1, ny1 = mapped.max(axis=0)
        area = max((nx1 - nx0) * (ny1 - ny0), 1e-9)
        cx0, cx1, cy0, cy1 = max(nx0, 0.0), min(nx1, REF_WIDTH), max(ny0, 0.0), min(ny1, REF_HEIGHT)
        w, hh = cx1 - cx0, cy1 - cy0
        if w < MIN_BOX_SIDE_PX or hh < MIN_BOX_SIDE_PX or (w * hh) / area < MIN_BOX_VISIBLE:
            continue
        out.append({**box, 'cx': (cx0 + cx1) / 2 / REF_WIDTH, 'cy': (cy0 + cy1) / 2 / REF_HEIGHT,
                    'w': w / REF_WIDTH, 'h': hh / REF_HEIGHT})
    return out


def _signal(signal, flip, center, yaw_deg):
    out = dict(signal)
    if signal.get('xy') is not None:
        out['xy'] = _points([signal['xy']], flip, center, yaw_deg)[0].tolist()
    return out


def combined_homography(camera, flip, yaw_deg):
    h = yaw_homography(camera, yaw_deg)
    return flip_homography(camera) @ h if flip else h


def transform_label(label, flip, yaw_deg, camera, image_size=None):
    center = camera.translation[:2]
    out = copy.deepcopy(label)
    out['map_elements'] = _map_elements(label.get('map_elements', []), flip, center, yaw_deg)
    paths = {}
    for slot, entry in label.get('paths', {}).items():
        paths[SLOT_SWAP[slot] if flip else slot] = _path_entry(entry, flip, center, yaw_deg)
    out['paths'] = paths
    if label.get('boxes'):
        out['boxes'] = _boxes(label['boxes'], combined_homography(camera, flip, yaw_deg))
    if label.get('signal') is not None:
        out['signal'] = _signal(label['signal'], flip, center, yaw_deg)
    if 'pasted_base' in label:
        out['pasted_base'] = [[*_points([[p[0], p[1]]], flip, center, yaw_deg)[0].tolist(), *p[2:]]
                              for p in label['pasted_base']]
    return out


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ground_matrices(camera):
    k, r, t = _intrinsics(camera), camera.rotation, camera.translation
    m = r @ np.linalg.inv(k)
    # 仮想画素 -> 仮想 base の地面 (X, Y, W) 斉次座標
    to_ground = np.stack([-t[2] * m[0] + t[0] * m[2], -t[2] * m[1] + t[1] * m[2], m[2]])
    to_pixel = k @ r.T @ np.array([[1.0, 0.0, -t[0]], [0.0, 1.0, -t[1]], [0.0, 0.0, -t[2]]])
    return to_ground, to_pixel


def ground_shift_homography(camera, dx, dy, dyaw):
    # 記録 base で車両を (dx, dy) 平行移動・dyaw 回転した仮想カメラの 画素 -> 記録画素（地面 z=0 仮定）
    to_ground, to_pixel = _ground_matrices(camera)
    c, s = np.cos(dyaw), np.sin(dyaw)
    move = np.array([[c, -s, dx], [s, c, dy], [0.0, 0.0, 1.0]])
    return to_pixel @ move @ to_ground


def rotation_homography(camera, dyaw):
    # 無限遠平面（回転のみ）の 仮想画素 -> 記録画素
    k, r = _intrinsics(camera), camera.rotation
    return k @ r.T @ _rz(dyaw) @ r @ np.linalg.inv(k)


def synthesize_view(image, camera, dx, dy, dyaw, ground_range=60.0, fill='replicate'):
    height, width = image.shape[:2]
    to_ground, _ = _ground_matrices(camera)
    sx, sy = width / REF_WIDTH, height / REF_HEIGHT
    uu, vv = np.meshgrid((np.arange(width) + 0.0) / sx, (np.arange(height) + 0.0) / sy)
    pix = np.stack([uu, vv, np.ones_like(uu)], axis=-1).reshape(-1, 3)
    g = pix @ to_ground.T
    x_base = g[:, 0] / np.where(g[:, 2] == 0, 1, g[:, 2])
    on_ground = (g[:, 2] < -1e-9) & (x_base <= ground_range) & (x_base > -ground_range)
    hg = ground_shift_homography(camera, dx, dy, dyaw)
    hr = rotation_homography(camera, dyaw)
    mapped = np.where(on_ground[:, None], pix @ hg.T, pix @ hr.T)
    src = (mapped[:, :2] / np.where(np.abs(mapped[:, 2:]) < 1e-12, 1e-12, mapped[:, 2:]))
    src = src * [sx, sy]
    map_x = src[:, 0].reshape(height, width).astype(np.float32)
    map_y = src[:, 1].reshape(height, width).astype(np.float32)
    top = int(round(camera.pad_top * sy))
    if fill == 'replicate':
        map_x = np.clip(map_x, 0, width - 1)
        map_y = np.clip(map_y, top, height - 1 - top)
        border = cv2.BORDER_REPLICATE
    elif fill == 'black':
        border = cv2.BORDER_CONSTANT
    else:
        raise ValueError(fill)
    out = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=border, borderValue=(0, 0, 0))
    if top > 0:
        out[:top] = 0
        out[height - top:] = 0
    return out


def virtual_world_pose(pose, ey, epsi):
    x, y, yaw = pose
    return (x - np.sin(yaw) * ey, y + np.cos(yaw) * ey, yaw + epsi)

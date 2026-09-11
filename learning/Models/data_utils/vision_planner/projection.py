import cv2
import numpy as np
import yaml


INTRINSICS = {
    'nHD': {
        'fx': 254.391622,
        'fy': 254.391622,
        'cx': 330.013020833,
        'cy': 181.149637858,
        'width': 640,
        'height': 360,
    },
    'kenta': {
        'fx': 251.58282470703125,
        'fy': 251.58282470703125,
        'cx': 330.0039469401042,
        'cy': 181.1462605794271,
        'width': 640,
        'height': 360,
    },
}


def rotation_from_rpy(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])

    rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)],
    ])
    ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)],
    ])
    rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],
    ])

    return rz @ ry @ rx


class CameraModel:
    def __init__(self, fx, fy, cx, cy, rotation, translation, pad_top, pad_left=0):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.rotation = np.asarray(rotation, float)
        self.translation = np.asarray(translation, float)
        self.pad_top = pad_top
        self.pad_left = pad_left

    def project(self, base_points):
        points = np.atleast_2d(np.asarray(base_points, float))
        camera = (points - self.translation) @ self.rotation
        depth = camera[:, 2]
        visible = depth > 0.05
        safe = np.where(visible, depth, 1.0)
        u = self.fx * camera[:, 0] / safe + self.cx
        v = self.fy * camera[:, 1] / safe + self.cy
        return np.stack([u, v], axis=1), visible


def load_camera(params_path, intrinsics_key, input_height, input_width):
    with open(params_path) as f:
        params = yaml.safe_load(f)

    camera = params['/**']['ros__parameters']['camera']
    spec = INTRINSICS[intrinsics_key]
    pad_top = (input_height - spec['height']) // 2
    pad_left = (input_width - spec['width']) // 2
    position = np.array([camera['position']['x'], camera['position']['y'], camera['position']['z']])
    rotation = rotation_from_rpy(camera['orientation']['roll'], camera['orientation']['pitch'], camera['orientation']['yaw'])

    return CameraModel(spec['fx'], spec['fy'], spec['cx'] + pad_left, spec['cy'] + pad_top,
                       rotation, position, pad_top, pad_left)


def letterbox(image, height, width, pad_top=None, pad_left=None):
    src_height, src_width = image.shape[:2]

    computed_pad_top = (height - src_height) // 2
    computed_pad_left = (width - src_width) // 2

    if pad_top is not None and computed_pad_top != pad_top:
        raise ValueError(f'letterbox: 実フレーム {src_height}x{src_width} から計算した pad_top ({computed_pad_top}) が load_camera の想定 ({pad_top}) と一致しません。INTRINSICS の height/width と実フレームサイズがずれています')

    if pad_left is not None and computed_pad_left != pad_left:
        raise ValueError(f'letterbox: 実フレーム {src_height}x{src_width} から計算した pad_left ({computed_pad_left}) が load_camera の想定 ({pad_left}) と一致しません。INTRINSICS の height/width と実フレームサイズがずれています')

    if (src_height, src_width) == (height, width):
        return image

    pad_bottom = height - src_height - computed_pad_top
    pad_right = width - src_width - computed_pad_left
    if computed_pad_top < 0 or pad_bottom < 0 or computed_pad_left < 0 or pad_right < 0:
        raise ValueError(f'letterbox: フレーム {src_height}x{src_width} が出力サイズ {height}x{width} に収まりません')

    return cv2.copyMakeBorder(image, computed_pad_top, pad_bottom, computed_pad_left, pad_right, cv2.BORDER_CONSTANT, value=(0, 0, 0))


def row_anchor_targets(num_rows, height):
    row_height = height // num_rows
    return np.arange(num_rows) * row_height + (row_height - 1) / 2.0


def row_anchor_columns(pixels, visible, num_rows, height, width):
    targets = row_anchor_targets(num_rows, height)
    columns = np.zeros(num_rows)
    valid = np.zeros(num_rows, bool)

    order = np.arange(len(pixels))
    for a, b in zip(order[:-1], order[1:]):
        if not (visible[a] and visible[b]):
            continue

        v0, v1 = pixels[a, 1], pixels[b, 1]
        lo, hi = (v0, v1) if v0 <= v1 else (v1, v0)

        hit = (targets >= lo) & (targets <= hi)
        if not hit.any():
            continue

        span = v1 - v0
        for index in np.nonzero(hit)[0]:
            ratio = 0.0 if abs(span) < 1e-09 else (targets[index] - v0) / span
            u = pixels[a, 0] + ratio * (pixels[b, 0] - pixels[a, 0])

            if u < 0.0 or u > width - 1:
                continue

            if valid[index]:
                continue

            columns[index] = u / (width - 1)
            valid[index] = True

    return columns, valid

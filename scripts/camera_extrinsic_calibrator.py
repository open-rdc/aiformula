#!/usr/bin/env python3
"""カメラ外部パラメータ推定ツール（最小二乗法）

zed_wrapper が配信する画像トピックと CameraInfo トピックを購読し、
画像上で指定した画素とその位置の実世界座標との対応を集めて、
最小二乗法でカメラ外部パラメータ（base_link -> カメラ光学座標系）を推定する。

表示方式は 2 種類あり、起動時に自動判定する:
  gui : OpenCV のウィンドウに画像を出し、クリックで画素を選ぶ。
        実世界座標は端末から入力する（JetPack 5.x の Jetson で動作する構成）。
  cli : ウィンドウを一切開かず、端末だけで "u v x y z" を入力する。
        画像は PNG に保存されるので、別のビューアで画素座標を読み取る。

座標系の定義:
  base_link      : x 前方 / y 左方 / z 上方 [m]  (ROS REP-103)
  camera optical : x 右方 / y 下方 / z 光軸前方 [m]  (REP-103 光学座標系)
  画像           : u 右方 / v 下方 [px]、原点は画像左上

推定する外部パラメータ (rotation, translation) は
  X_camera_optical = rotation @ X_base_link + translation
を満たす。localization/main_params.yaml の camera_to_base は
逆向き（光学座標系 -> base_link）なので、rotation の転置と
カメラ光学中心の base_link 座標に変換して出力する。

使い方:
  python3 camera_extrinsic_calibrator.py --points 8
  python3 camera_extrinsic_calibrator.py --display cli   # 端末入力のみ
"""

import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

IMAGE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"

EXPECTED_IMAGE_WIDTH = 640
EXPECTED_IMAGE_HEIGHT = 360

MIN_CORRESPONDENCES = 6
COLLINEAR_TOLERANCE = 1.0e-3
PLANAR_TOLERANCE = 1.0e-2

WINDOW_NAME = "camera extrinsic calibrator"
REGISTERED_COLOR = (0, 255, 0)
PENDING_COLOR = (0, 0, 255)
SCREEN_MARGIN = 0.85
MIN_DISPLAY_SCALE = 0.5
MAX_DISPLAY_SCALE = 3.0
DEFAULT_SNAPSHOT_PATH = "/tmp/camera_extrinsic_frame.png"


@dataclass(frozen=True)
class ExtrinsicResult:
    """外部パラメータ推定結果"""

    rotation: np.ndarray  # 3x3 base_link -> camera optical
    translation: np.ndarray  # 3 base_link -> camera optical [m]
    camera_position_base: np.ndarray  # 3 カメラ光学中心の base_link 座標 [m]
    rpy: Tuple[float, float, float]  # camera_to_base の roll/pitch/yaw [rad]
    reprojection_errors: np.ndarray  # 各対応点の再投影誤差 [px]
    rms_reprojection_error: float  # 再投影誤差の RMS [px]
    planar: bool  # 実世界座標が同一平面上だったか


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """roll/pitch/yaw から回転行列を作る（Rz(yaw) @ Ry(pitch) @ Rx(roll)）"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def rpy_from_rotation(rotation: np.ndarray) -> Tuple[float, float, float]:
    """回転行列から roll/pitch/yaw を取り出す（rotation_from_rpy の逆変換）"""
    rotation = np.asarray(rotation, dtype=float)
    pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return roll, pitch, yaw


def project_to_pixels(
    rotation: np.ndarray,
    translation: np.ndarray,
    camera_matrix: np.ndarray,
    world_points: np.ndarray,
) -> np.ndarray:
    """base_link 座標の点群を画素座標へ投影する"""
    world_points = np.asarray(world_points, dtype=float).reshape(-1, 3)
    camera_points = world_points @ np.asarray(rotation, dtype=float).T + np.asarray(
        translation, dtype=float
    )
    depth = camera_points[:, 2]
    safe_depth = np.where(np.abs(depth) < 1.0e-9, 1.0e-9, depth)
    normalized = camera_points[:, :2] / safe_depth[:, None]
    camera_matrix = np.asarray(camera_matrix, dtype=float)
    pixels_u = camera_matrix[0, 0] * normalized[:, 0] + camera_matrix[0, 2]
    pixels_v = camera_matrix[1, 1] * normalized[:, 1] + camera_matrix[1, 2]
    return np.column_stack([pixels_u, pixels_v])


def canvas_to_image_pixel(canvas_x: float, canvas_y: float, scale: float) -> Tuple[float, float]:
    """表示ウィンドウ上の座標を画像の画素座標へ変換する"""
    return canvas_x / scale, canvas_y / scale


def _hartley_normalize(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """数値安定化のため重心を原点・RMS 距離を sqrt(次元) に揃える"""
    dimension = points.shape[1]
    centroid = points.mean(axis=0)
    centered = points - centroid
    rms = float(np.sqrt((centered**2).sum(axis=1).mean()))
    scale = np.sqrt(dimension) / rms if rms > 1.0e-12 else 1.0
    transform = np.eye(dimension + 1)
    transform[:dimension, :dimension] *= scale
    transform[:dimension, dimension] = -scale * centroid
    return centered * scale, transform


def _normalized_image_points(image_points: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    """画素座標を内部パラメータで正規化画像座標へ変換する"""
    homogeneous = np.column_stack([image_points, np.ones(len(image_points))])
    normalized = homogeneous @ np.linalg.inv(camera_matrix).T
    return normalized[:, :2] / normalized[:, 2:3]


def _solve_projection_dlt(normalized: np.ndarray, world_points: np.ndarray) -> np.ndarray:
    """非平面配置の対応点から 3x4 射影行列を最小二乗で解く"""
    points, transform = _hartley_normalize(world_points)
    rows = []
    for (x, y), point in zip(normalized, points):
        homogeneous = np.append(point, 1.0)
        zeros = np.zeros(4)
        rows.append(np.concatenate([-homogeneous, zeros, x * homogeneous]))
        rows.append(np.concatenate([zeros, -homogeneous, y * homogeneous]))
    _, _, vt = np.linalg.svd(np.array(rows))
    return vt[-1].reshape(3, 4) @ transform


def _solve_homography(normalized: np.ndarray, plane_points: np.ndarray) -> np.ndarray:
    """平面上の対応点からホモグラフィを最小二乗で解く"""
    points, transform = _hartley_normalize(plane_points)
    rows = []
    for (x, y), point in zip(normalized, points):
        homogeneous = np.append(point, 1.0)
        zeros = np.zeros(3)
        rows.append(np.concatenate([-homogeneous, zeros, x * homogeneous]))
        rows.append(np.concatenate([zeros, -homogeneous, y * homogeneous]))
    _, _, vt = np.linalg.svd(np.array(rows))
    return vt[-1].reshape(3, 3) @ transform


def _pose_from_projection(projection: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """射影行列を回転（直交化）と並進に分解する"""
    matrix = projection[:, :3]
    vector = projection[:, 3]
    if np.linalg.det(matrix) < 0.0:
        matrix, vector = -matrix, -vector
    u, singular, vt = np.linalg.svd(matrix)
    return u @ vt, vector / float(singular.mean())


def _pose_from_plane(
    normalized: np.ndarray, world_points: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """同一平面上の対応点から姿勢を復元する"""
    centroid = world_points.mean(axis=0)
    _, _, vt = np.linalg.svd(world_points - centroid)
    axis_u, axis_v = vt[0], vt[1]
    axis_w = np.cross(axis_u, axis_v)
    basis = np.column_stack([axis_u, axis_v, axis_w])
    plane_points = (world_points - centroid) @ basis[:, :2]

    homography = _solve_homography(normalized, plane_points)
    scale = 2.0 / (np.linalg.norm(homography[:, 0]) + np.linalg.norm(homography[:, 1]))
    if homography[2, 2] * scale < 0.0:
        scale = -scale
    column_u = scale * homography[:, 0]
    column_v = scale * homography[:, 1]
    u, _, vt_pose = np.linalg.svd(
        np.column_stack([column_u, column_v, np.cross(column_u, column_v)])
    )
    rotation = (u @ vt_pose) @ basis.T
    translation = scale * homography[:, 2] - rotation @ centroid
    return rotation, translation


def _refine_pose(
    rotation: np.ndarray,
    translation: np.ndarray,
    image_points: np.ndarray,
    world_points: np.ndarray,
    camera_matrix: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """再投影誤差を目的関数として非線形最小二乗で微調整する"""
    try:
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
    except ImportError:
        return rotation, translation

    def residual(parameters: np.ndarray) -> np.ndarray:
        candidate = Rotation.from_rotvec(parameters[:3]).as_matrix()
        projected = project_to_pixels(candidate, parameters[3:], camera_matrix, world_points)
        return (projected - image_points).ravel()

    initial = np.concatenate([Rotation.from_matrix(rotation).as_rotvec(), translation])
    solution = least_squares(residual, initial, method="lm")
    return Rotation.from_rotvec(solution.x[:3]).as_matrix(), solution.x[3:]


def estimate_extrinsic(
    image_points: np.ndarray,
    world_points: np.ndarray,
    camera_matrix: np.ndarray,
) -> ExtrinsicResult:
    """画素座標と base_link 実世界座標の対応から外部パラメータを推定する

    Args:
        image_points: 画素座標 (N, 2) [px]、u 右方 / v 下方
        world_points: base_link 実世界座標 (N, 3) [m]、x 前方 / y 左方 / z 上方
        camera_matrix: 内部パラメータ行列 K (3, 3)
    """
    image_points = np.asarray(image_points, dtype=float).reshape(-1, 2)
    world_points = np.asarray(world_points, dtype=float).reshape(-1, 3)
    camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)

    if len(image_points) != len(world_points):
        raise ValueError("画素座標と実世界座標の個数が一致していません")
    if len(world_points) < MIN_CORRESPONDENCES:
        raise ValueError(f"対応点が不足しています（{MIN_CORRESPONDENCES}点以上必要）")

    singular = np.linalg.svd(world_points - world_points.mean(axis=0), compute_uv=False)
    if singular[0] < 1.0e-9:
        raise ValueError("実世界座標がすべて同一点です")
    if singular[1] / singular[0] < COLLINEAR_TOLERANCE:
        raise ValueError("実世界座標が一直線上に並んでいます（面的に散らばった点を選んでください）")
    planar = bool(singular[2] / singular[0] < PLANAR_TOLERANCE)

    normalized = _normalized_image_points(image_points, camera_matrix)
    if planar:
        rotation, translation = _pose_from_plane(normalized, world_points)
    else:
        rotation, translation = _pose_from_projection(
            _solve_projection_dlt(normalized, world_points)
        )

    rotation, translation = _refine_pose(
        rotation, translation, image_points, world_points, camera_matrix
    )

    camera_points = world_points @ rotation.T + translation
    if np.any(camera_points[:, 2] <= 0.0):
        raise ValueError("カメラ後方に投影される点があります（対応付けを確認してください）")

    errors = np.linalg.norm(
        project_to_pixels(rotation, translation, camera_matrix, world_points) - image_points,
        axis=1,
    )
    rotation_optical_to_base = rotation.T
    return ExtrinsicResult(
        rotation=rotation,
        translation=translation,
        camera_position_base=-rotation_optical_to_base @ translation,
        rpy=rpy_from_rotation(rotation_optical_to_base),
        reprojection_errors=errors,
        rms_reprojection_error=float(np.sqrt((errors**2).mean())),
        planar=planar,
    )


def format_result(result: ExtrinsicResult) -> str:
    """推定結果を日本語のレポート文字列に整形する"""
    position = result.camera_position_base
    roll, pitch, yaw = result.rpy
    lines = [
        "==================== 推定結果 ====================",
        f"対応点の配置          : {'同一平面上' if result.planar else '非平面'}",
        f"再投影誤差 RMS        : {result.rms_reprojection_error:.3f} px",
        f"再投影誤差 最大        : {result.reprojection_errors.max():.3f} px",
        "",
        "--- 外部パラメータ [R|t] (base_link -> カメラ光学座標系) ---",
        "  X_camera = R @ X_base + t",
    ]
    for row, translation in zip(result.rotation, result.translation):
        lines.append(
            f"  [{row[0]: .6f} {row[1]: .6f} {row[2]: .6f} | {translation: .6f}]"
        )
    lines += [
        "",
        "--- カメラ位置姿勢 (base_link 基準: x 前方 / y 左方 / z 上方) ---",
        f"  位置  x = {position[0]: .4f} m, y = {position[1]: .4f} m, z = {position[2]: .4f} m",
        f"  姿勢  roll = {roll: .6f} rad ({np.rad2deg(roll): .3f} deg)",
        f"        pitch= {pitch: .6f} rad ({np.rad2deg(pitch): .3f} deg)",
        f"        yaw  = {yaw: .6f} rad ({np.rad2deg(yaw): .3f} deg)",
        f"  ※ 光軸の俯角は {-np.rad2deg(roll) - 90.0: .3f} deg（正で下向き）",
        "",
        "--- main_params.yaml へ貼り付ける形式 ---",
        "    camera_to_base:",
        f"      x : {position[0]:.6f}",
        f"      y : {position[1]:.6f}",
        f"      z : {position[2]:.6f}",
        f"      roll : {roll:.11f}",
        f"      pitch : {pitch:.11f}",
        f"      yaw : {yaw:.11f}",
        "==================================================",
    ]
    return "\n".join(lines)


def parse_floats(text: str, count: int) -> Tuple[float, ...]:
    """空白またはカンマ区切りの数値列を count 個読み取る"""
    tokens = text.replace(",", " ").split()
    if len(tokens) != count:
        raise ValueError(f"数値を {count} 個、空白区切りで入力してください（入力: {len(tokens)} 個）")
    try:
        return tuple(float(token) for token in tokens)
    except ValueError as error:
        raise ValueError(f"数値として読み取れない入力があります: {text.strip()}") from error


def parse_world_input(text: str) -> Tuple[float, float, float]:
    """実世界座標の入力 "x y z" [m] を読み取る"""
    x, y, z = parse_floats(text, 3)
    return x, y, z


def parse_correspondence_input(
    text: str,
) -> Tuple[Tuple[float, float], Tuple[float, float, float]]:
    """端末入力 "u v x y z" を画素座標と実世界座標に分解する"""
    u, v, x, y, z = parse_floats(text, 5)
    return (u, v), (x, y, z)


def validate_pixel(
    pixel: Sequence[float], image_size: Optional[Tuple[int, int]]
) -> None:
    """画素座標が画像の範囲内かを確認する"""
    if image_size is None:
        return
    width, height = image_size
    if not (0.0 <= pixel[0] < width and 0.0 <= pixel[1] < height):
        raise ValueError(
            f"画素 (u={pixel[0]:.1f}, v={pixel[1]:.1f}) が画像の範囲外です"
            f"（0 <= u < {width}, 0 <= v < {height}）"
        )


class CalibrationSession:
    """対応点の蓄積と推定を担う（表示方式に依存しない）"""

    def __init__(self, target_points: int):
        self._target_points = target_points
        self._image_points: List[Tuple[float, float]] = []
        self._world_points: List[Tuple[float, float, float]] = []

    @property
    def count(self) -> int:
        return len(self._image_points)

    @property
    def target_points(self) -> int:
        return self._target_points

    @property
    def image_points(self) -> List[Tuple[float, float]]:
        return list(self._image_points)

    @property
    def world_points(self) -> List[Tuple[float, float, float]]:
        return list(self._world_points)

    @property
    def ready(self) -> bool:
        return self.count >= MIN_CORRESPONDENCES

    @property
    def complete(self) -> bool:
        return self.count >= self._target_points

    def add(
        self, pixel: Sequence[float], world: Sequence[float]
    ) -> int:
        self._image_points.append((float(pixel[0]), float(pixel[1])))
        self._world_points.append((float(world[0]), float(world[1]), float(world[2])))
        return self.count

    def undo(self) -> bool:
        if not self._image_points:
            return False
        self._image_points.pop()
        self._world_points.pop()
        return True

    def describe(self) -> List[str]:
        """登録済みの対応点を 1 行ずつの文字列にする"""
        lines = []
        for index, (pixel, world) in enumerate(zip(self._image_points, self._world_points), 1):
            lines.append(
                f"{index:2d}: px(u={pixel[0]:6.1f}, v={pixel[1]:6.1f})"
                f" -> base({world[0]:6.2f}, {world[1]:6.2f}, {world[2]:5.2f})"
            )
        return lines

    def estimate(self, camera_matrix: np.ndarray) -> ExtrinsicResult:
        return estimate_extrinsic(
            np.array(self._image_points), np.array(self._world_points), camera_matrix
        )


def _draw_marker(
    canvas: np.ndarray,
    pixel: Sequence[float],
    scale: float,
    color: Tuple[int, int, int],
    label: Optional[str],
) -> None:
    """十字とリングのマーカーを描く（cv2 は日本語を描けないので番号のみ）"""
    x = int(round(pixel[0] * scale))
    y = int(round(pixel[1] * scale))
    cv2.line(canvas, (x - 10, y), (x + 10, y), color, 1)
    cv2.line(canvas, (x, y - 10), (x, y + 10), color, 1)
    cv2.circle(canvas, (x, y), 5, color, 1)
    if label is not None:
        cv2.putText(canvas, label, (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def render_frame(
    image: np.ndarray,
    scale: float,
    image_points: Sequence[Sequence[float]],
    pending_pixel: Optional[Sequence[float]],
) -> np.ndarray:
    """表示倍率をかけた画像に対応点マーカーを重ねた描画用画像を作る"""
    height, width = image.shape[:2]
    canvas = cv2.resize(
        image,
        (max(int(round(width * scale)), 1), max(int(round(height * scale)), 1)),
        interpolation=cv2.INTER_NEAREST,
    )
    for index, pixel in enumerate(image_points, start=1):
        _draw_marker(canvas, pixel, scale, REGISTERED_COLOR, str(index))
    if pending_pixel is not None:
        _draw_marker(canvas, pending_pixel, scale, PENDING_COLOR, None)
    return canvas


def draw_status_bar(canvas: np.ndarray, text: str) -> None:
    """ウィンドウ下部に操作案内を描く（cv2 は ASCII のみ描画可能）"""
    height, width = canvas.shape[:2]
    cv2.rectangle(canvas, (0, height - 22), (width, height), (0, 0, 0), cv2.FILLED)
    cv2.putText(
        canvas, text, (6, height - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1
    )


def detect_screen_size() -> Optional[Tuple[int, int]]:
    """xrandr から画面解像度を取得する（取得できなければ None）"""
    try:
        completed = subprocess.run(
            ["xrandr", "--current"], capture_output=True, text=True, timeout=5.0
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    for line in completed.stdout.splitlines():
        if " connected" not in line:
            continue
        match = re.search(r"(\d+)x(\d+)\+\d+\+\d+", line)
        if match:
            return int(match.group(1)), int(match.group(2))
    return None


def fit_scale(
    image_size: Tuple[int, int], screen_size: Optional[Tuple[int, int]]
) -> float:
    """画像が画面に収まる表示倍率を 0.1 刻みで求める"""
    width, height = image_size
    if screen_size is None or width <= 0 or height <= 0:
        return 1.0
    screen_width, screen_height = screen_size
    raw = min(screen_width * SCREEN_MARGIN / width, screen_height * SCREEN_MARGIN / height)
    scale = math.floor(raw * 10.0) / 10.0
    return float(min(max(scale, MIN_DISPLAY_SCALE), MAX_DISPLAY_SCALE))


GUI_PROBE_CODE = (
    "import cv2; cv2.namedWindow('probe'); cv2.waitKey(1); cv2.destroyAllWindows()"
)


def probe_opencv_gui(timeout: float = 30.0) -> bool:
    """OpenCV のウィンドウを開けるかを子プロセスで検査する

    表示先が無い状態で cv2 のウィンドウを開くと Qt バックエンドが
    プロセスごと abort し try/except では捕まえられないため、
    必ず別プロセスで試す。
    """
    try:
        completed = subprocess.run(
            [sys.executable, "-c", GUI_PROBE_CODE], capture_output=True, timeout=timeout
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return completed.returncode == 0


def select_display_mode(
    requested: str, has_display: bool, probe: Callable[[], bool]
) -> str:
    """表示方式（gui / cli）を決める"""
    if requested in ("gui", "cli"):
        return requested
    if not has_display:
        return "cli"
    return "gui" if probe() else "cli"


def has_display_environment() -> bool:
    """X11 / Wayland の表示先が設定されているか"""
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


class CameraTopicSubscriber:
    """zed_wrapper の画像と内部パラメータをトピックから取得する ROS 2 ノード"""

    def __init__(self, node, image_topic: str, camera_info_topic: str):
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image

        self._node = node
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._image = None
        self._camera_matrix = None
        self._info_size = None

        node.create_subscription(Image, image_topic, self._on_image, 10)
        node.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)
        node.get_logger().info(f"画像トピックを購読します: {image_topic}")
        node.get_logger().info(f"内部パラメータトピックを購読します: {camera_info_topic}")

    def _on_image(self, msg) -> None:
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._lock:
            self._image = image

    def _on_camera_info(self, msg) -> None:
        with self._lock:
            if self._camera_matrix is None:
                self._node.get_logger().info(
                    f"内部パラメータを取得しました: fx={msg.k[0]:.3f} fy={msg.k[4]:.3f} "
                    f"cx={msg.k[2]:.3f} cy={msg.k[5]:.3f} ({msg.width}x{msg.height})"
                )
                if any(abs(value) > 1.0e-9 for value in msg.d):
                    self._node.get_logger().warn(
                        "歪み係数 D が 0 ではありません。rectify 済み画像を前提に D は無視します"
                    )
            self._camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
            self._info_size = (int(msg.width), int(msg.height))

    def latest_image(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._image is None else self._image.copy()

    def camera_matrix(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._camera_matrix is None else self._camera_matrix.copy()

    def info_size(self) -> Optional[Tuple[int, int]]:
        with self._lock:
            return self._info_size


def wait_for_source(source: CameraTopicSubscriber, node, timeout: float) -> bool:
    """画像と CameraInfo を受信するまで待つ"""
    node.get_logger().info("画像と CameraInfo の受信を待っています...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if source.latest_image() is not None and source.camera_matrix() is not None:
            image = source.latest_image()
            height, width = image.shape[:2]
            node.get_logger().info(f"画像を受信しました: {width}x{height}")
            if (width, height) != (EXPECTED_IMAGE_WIDTH, EXPECTED_IMAGE_HEIGHT):
                node.get_logger().warn(
                    f"画像サイズが想定と異なります（想定 "
                    f"{EXPECTED_IMAGE_WIDTH}x{EXPECTED_IMAGE_HEIGHT}）"
                )
            return True
        time.sleep(0.2)
    return False


def run_estimation(session: CalibrationSession, source: CameraTopicSubscriber, node):
    """対応点から外部パラメータを推定し、結果を表示する"""
    camera_matrix = source.camera_matrix()
    if camera_matrix is None:
        print("内部パラメータ（CameraInfo）をまだ受信していません")
        return None
    image = source.latest_image()
    info_size = source.info_size()
    if image is not None and info_size is not None:
        image_size = (image.shape[1], image.shape[0])
        if info_size != image_size:
            print(
                f"CameraInfo の解像度 {info_size[0]}x{info_size[1]} と画像 "
                f"{image_size[0]}x{image_size[1]} が一致しません（K が画像に対応しません）"
            )
            return None
    try:
        result = session.estimate(camera_matrix)
    except ValueError as error:
        print(f"推定に失敗しました: {error}")
        return None

    for index, error in enumerate(result.reprojection_errors, start=1):
        if error > 3.0:
            print(f"警告: {index}点目の再投影誤差が大きいです: {error:.2f} px")
    print(format_result(result))
    node.get_logger().info(
        f"推定完了 RMS={result.rms_reprojection_error:.2f} px "
        f"位置=({result.camera_position_base[0]:.3f}, "
        f"{result.camera_position_base[1]:.3f}, {result.camera_position_base[2]:.3f}) m"
    )
    return result


GUI_HELP = """
--- 操作方法 (OpenCV ウィンドウ) ---
  画像をクリック : 画素を選択（画像が固定される）
  この端末       : 選択した画素の base_link 座標を "x y z" で入力（Enter で登録 / 空 Enter で取消）
  キー u         : 直前の登録を取り消す
  キー r         : 画像の固定を解除する
  キー e         : 現在の対応点で推定する
  キー s         : 表示中の画像を PNG 保存する
  キー q / ESC   : 終了する
  ※ ウィンドウが大きすぎる場合は --scale 1.0 のように指定してください
"""


class GuiCalibrator:
    """OpenCV のウィンドウで画素を選び、実世界座標は端末から入力する"""

    def __init__(
        self,
        node,
        source: CameraTopicSubscriber,
        session: CalibrationSession,
        scale: float,
        snapshot_path: str,
    ):
        self._node = node
        self._source = source
        self._session = session
        self._scale = scale
        self._snapshot_path = snapshot_path
        self._lock = threading.Lock()
        self._pending_pixel: Optional[Tuple[float, float]] = None
        self._frozen_image: Optional[np.ndarray] = None
        self._pixel_ready = threading.Event()
        self._prompting = False
        self._closed = False
        self._result: Optional[ExtrinsicResult] = None

    def run(self) -> None:
        print(GUI_HELP)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)
        prompt_thread = threading.Thread(target=self._prompt_loop, daemon=True)
        prompt_thread.start()
        try:
            while not self._closed:
                self._render_once()
                self._handle_key(cv2.waitKey(30) & 0xFF)
        except KeyboardInterrupt:
            pass
        finally:
            self._closed = True
            self._pixel_ready.set()
            cv2.destroyAllWindows()
            cv2.waitKey(1)

    def _current_image(self) -> Optional[np.ndarray]:
        with self._lock:
            frozen = self._frozen_image
        return frozen if frozen is not None else self._source.latest_image()

    def _render_once(self) -> None:
        image = self._current_image()
        if image is None:
            return
        with self._lock:
            pending = self._pending_pixel
        canvas = render_frame(image, self._scale, self._session.image_points, pending)
        if pending is not None:
            status = f"enter x y z in the terminal  (u={pending[0]:.0f}, v={pending[1]:.0f})"
        else:
            status = "click a point   u:undo  r:unfreeze  e:estimate  s:save  q:quit"
        draw_status_bar(
            canvas, f"[{self._session.count}/{self._session.target_points}] {status}"
        )
        cv2.imshow(WINDOW_NAME, canvas)

    def _on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self._prompting:
            print("端末で座標を入力中です（空 Enter で取り消せます）")
            return
        image = self._current_image()
        if image is None:
            print("画像をまだ受信していません")
            return
        pixel = canvas_to_image_pixel(float(x), float(y), self._scale)
        try:
            validate_pixel(pixel, (image.shape[1], image.shape[0]))
        except ValueError as error:
            print(f"入力エラー: {error}")
            return
        with self._lock:
            if self._frozen_image is None:
                self._frozen_image = image
            self._pending_pixel = pixel
        self._pixel_ready.set()

    def _handle_key(self, key: int) -> None:
        if key in (ord("q"), 27):
            self._closed = True
        elif key == ord("u"):
            self._undo()
        elif key == ord("r"):
            self._clear_pending()
            print("画像の固定を解除しました")
        elif key == ord("e"):
            self._result = run_estimation(self._session, self._source, self._node)
        elif key == ord("s"):
            save_snapshot(self._current_image(), self._snapshot_path)

    def _undo(self) -> None:
        if self._prompting:
            print("端末で座標を入力中です（空 Enter で取り消せます）")
            return
        with self._lock:
            pending = self._pending_pixel
        if pending is not None:
            self._clear_pending()
            print("選択中の画素を取り消しました")
        elif self._session.undo():
            print(f"直前の対応点を取り消しました（残り {self._session.count} 点）")
        else:
            print("取り消せる対応点がありません")

    def _clear_pending(self) -> None:
        with self._lock:
            self._pending_pixel = None
            self._frozen_image = None

    def _prompt_loop(self) -> None:
        while not self._closed:
            if not self._pixel_ready.wait(0.2):
                continue
            self._pixel_ready.clear()
            with self._lock:
                pixel = self._pending_pixel
            if pixel is None or self._closed:
                continue
            self._prompting = True
            try:
                text = input(
                    f"[{self._session.count + 1}点目] 画素 (u={pixel[0]:.1f}, v={pixel[1]:.1f}) "
                    "の base_link 座標 x y z [m] > "
                )
            except EOFError:
                self._closed = True
                return
            finally:
                self._prompting = False

            if not text.strip():
                self._clear_pending()
                print("選択を取り消しました")
                continue
            try:
                world = parse_world_input(text)
            except ValueError as error:
                print(f"入力エラー: {error}")
                self._pixel_ready.set()
                continue

            index = self._session.add(pixel, world)
            self._clear_pending()
            print(
                f"{index}点目を登録しました: 画素(u={pixel[0]:.1f}, v={pixel[1]:.1f}) = "
                f"base_link({world[0]:.3f}, {world[1]:.3f}, {world[2]:.3f}) [m]"
            )
            if self._session.complete and self._result is None:
                print(f"指定回数（{self._session.target_points}点）に到達しました")
                self._result = run_estimation(self._session, self._source, self._node)


CLI_HELP = """
--- 操作方法 (端末入力モード) ---
  u v x y z : 画素座標 [px] と base_link 座標 [m] を空白区切りで入力して登録
  save      : 現在の画像を PNG 保存する（画素座標はこの画像上の値）
  list      : 登録済みの対応点を一覧表示する
  undo      : 直前の登録を取り消す
  estimate  : 現在の対応点で推定する
  quit      : 終了する
"""


class ConsoleCalibrator:
    """ウィンドウを開かず端末だけで対応点を入力する"""

    def __init__(
        self,
        node,
        source: CameraTopicSubscriber,
        session: CalibrationSession,
        snapshot_path: str,
    ):
        self._node = node
        self._source = source
        self._session = session
        self._snapshot_path = snapshot_path
        self._result: Optional[ExtrinsicResult] = None

    def run(self) -> None:
        print(CLI_HELP)
        save_snapshot(self._source.latest_image(), self._snapshot_path)
        while True:
            try:
                text = input(
                    f"[{self._session.count + 1}点目] u v x y z / save / list / undo / "
                    "estimate / quit > "
                )
            except EOFError:
                print()
                return
            command = text.strip().lower()
            if not command:
                continue
            if command in ("q", "quit", "exit"):
                return
            if command in ("s", "save"):
                save_snapshot(self._source.latest_image(), self._snapshot_path)
                continue
            if command in ("l", "list"):
                self._print_list()
                continue
            if command in ("u", "undo"):
                if self._session.undo():
                    print(f"直前の対応点を取り消しました（残り {self._session.count} 点）")
                else:
                    print("取り消せる対応点がありません")
                continue
            if command in ("e", "estimate"):
                self._result = run_estimation(self._session, self._source, self._node)
                continue

            try:
                pixel, world = parse_correspondence_input(text)
                validate_pixel(pixel, self._source.info_size())
            except ValueError as error:
                print(f"入力エラー: {error}")
                continue
            index = self._session.add(pixel, world)
            print(
                f"{index}点目を登録しました: 画素(u={pixel[0]:.1f}, v={pixel[1]:.1f}) = "
                f"base_link({world[0]:.3f}, {world[1]:.3f}, {world[2]:.3f}) [m]"
            )
            if self._session.complete and self._result is None:
                print(f"指定回数（{self._session.target_points}点）に到達しました")
                self._result = run_estimation(self._session, self._source, self._node)

    def _print_list(self) -> None:
        lines = self._session.describe()
        if not lines:
            print("登録済みの対応点はありません")
            return
        print("\n".join(lines))


def save_snapshot(image: Optional[np.ndarray], path: str) -> bool:
    """現在の画像を PNG に保存する（画素座標の読み取り用）"""
    if image is None:
        print("画像をまだ受信していないため保存できません")
        return False
    if not cv2.imwrite(path, image):
        print(f"画像を保存できませんでした: {path}")
        return False
    height, width = image.shape[:2]
    print(f"画像を保存しました: {path} ({width}x{height}, 画素座標は原寸のまま)")
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="カメラ外部パラメータ推定ツール")
    parser.add_argument("--image-topic", default=IMAGE_TOPIC, help="画像トピック名")
    parser.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC, help="CameraInfo トピック名")
    parser.add_argument(
        "--points", type=int, default=8, help=f"対応点の指定回数（{MIN_CORRESPONDENCES} 以上）"
    )
    parser.add_argument(
        "--display",
        choices=("auto", "gui", "cli"),
        default="auto",
        help="表示方式（auto: GUI が使えるか検査して自動選択）",
    )
    parser.add_argument(
        "--scale", type=float, default=None, help="GUI の画像表示倍率（既定は画面に合わせて自動）"
    )
    parser.add_argument(
        "--snapshot", default=DEFAULT_SNAPSHOT_PATH, help="画像を保存する PNG のパス"
    )
    parser.add_argument("--wait", type=float, default=30.0, help="トピック受信を待つ秒数")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.points < MIN_CORRESPONDENCES:
        raise SystemExit(f"--points は {MIN_CORRESPONDENCES} 以上を指定してください")

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("camera_extrinsic_calibrator")
    node.get_logger().info(
        "実世界座標は base_link 座標系（x 前方 / y 左方 / z 上方、単位 m）で入力してください"
    )
    source = CameraTopicSubscriber(node, args.image_topic, args.camera_info_topic)

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    try:
        if not wait_for_source(source, node, args.wait):
            node.get_logger().error(
                "画像 / CameraInfo を受信できませんでした（トピック名と zed_wrapper の起動を確認してください）"
            )
            return

        mode = select_display_mode(args.display, has_display_environment(), probe_opencv_gui)
        session = CalibrationSession(args.points)
        if mode == "gui":
            image = source.latest_image()
            image_size = (image.shape[1], image.shape[0])
            scale = args.scale if args.scale else fit_scale(image_size, detect_screen_size())
            node.get_logger().info(
                f"GUI モードで起動します（表示倍率 {scale:.1f}、"
                f"ウィンドウ {int(image_size[0] * scale)}x{int(image_size[1] * scale)}）"
            )
            GuiCalibrator(node, source, session, scale, args.snapshot).run()
        else:
            node.get_logger().warn(
                "GUI を開けないため端末入力モードで起動します（--display gui で強制できます）"
            )
            ConsoleCalibrator(node, source, session, args.snapshot).run()
    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

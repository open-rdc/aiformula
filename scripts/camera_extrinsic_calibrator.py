#!/usr/bin/env python3
"""カメラ外部パラメータ推定ツール（最小二乗法）

zed_wrapper が配信する画像トピックと CameraInfo トピックを購読し、
GUI 上でクリックした画素とその位置の実世界座標との対応を集めて、
最小二乗法でカメラ外部パラメータ（base_link -> カメラ光学座標系）を推定する。

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
"""

import argparse
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

IMAGE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"

EXPECTED_IMAGE_WIDTH = 640
EXPECTED_IMAGE_HEIGHT = 360

MIN_CORRESPONDENCES = 6
COLLINEAR_TOLERANCE = 1.0e-3
PLANAR_TOLERANCE = 1.0e-2


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
    """GUI キャンバス上の座標を画像の画素座標へ変換する"""
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


class CalibratorGui:
    """画素クリックと実世界座標入力を受け付ける GUI"""

    def __init__(self, node, source: CameraTopicSubscriber, target_points: int, scale: float):
        import tkinter as tk

        self._tk = tk
        self._node = node
        self._source = source
        self._target_points = target_points
        self._scale = scale
        self._image_points: List[Tuple[float, float]] = []
        self._world_points: List[Tuple[float, float, float]] = []
        self._pending_pixel: Optional[Tuple[float, float]] = None
        self._frozen_image: Optional[np.ndarray] = None
        self._photo = None
        self._result: Optional[ExtrinsicResult] = None

        self._root = tk.Tk()
        self._root.title("カメラ外部パラメータ キャリブレーション")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        canvas_width = int(EXPECTED_IMAGE_WIDTH * scale)
        canvas_height = int(EXPECTED_IMAGE_HEIGHT * scale)
        self._canvas = tk.Canvas(self._root, width=canvas_width, height=canvas_height, bg="black")
        self._canvas.grid(row=0, column=0, padx=6, pady=6)
        self._canvas.bind("<Button-1>", self._on_click)

        panel = tk.Frame(self._root)
        panel.grid(row=0, column=1, sticky="n", padx=6, pady=6)
        self._build_panel(panel)
        self._update_frame()

    def _build_panel(self, panel) -> None:
        tk = self._tk
        tk.Label(panel, text="実世界座標の入力", font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        tk.Label(
            panel,
            justify="left",
            text=(
                "座標系: base_link [m]\n"
                "  x : 前方（車両の進行方向）\n"
                "  y : 左方\n"
                "  z : 上方（地面は z = 0）\n"
                "画像上の点をクリックすると画像が\n固定されます。"
            ),
        ).pack(anchor="w", pady=(0, 6))

        self._entries = {}
        for key, label in (("x", "x [m] 前方"), ("y", "y [m] 左方"), ("z", "z [m] 上方")):
            row = tk.Frame(panel)
            row.pack(anchor="w", pady=1)
            tk.Label(row, text=label, width=11, anchor="w").pack(side="left")
            entry = tk.Entry(row, width=12)
            entry.pack(side="left")
            entry.bind("<Return>", lambda _event: self._on_register())
            self._entries[key] = entry

        button_row = tk.Frame(panel)
        button_row.pack(anchor="w", pady=6)
        tk.Button(button_row, text="登録", width=8, command=self._on_register).pack(side="left")
        tk.Button(button_row, text="取消", width=8, command=self._on_undo).pack(side="left")
        tk.Button(button_row, text="解除", width=8, command=self._on_unfreeze).pack(side="left")

        self._status = tk.Label(panel, justify="left", fg="blue", text="")
        self._status.pack(anchor="w")

        self._listbox = tk.Listbox(panel, width=52, height=12, font=("TkFixedFont", 9))
        self._listbox.pack(anchor="w", pady=6)

        self._estimate_button = tk.Button(
            panel, text="外部パラメータを推定", command=self._on_estimate, state="disabled"
        )
        self._estimate_button.pack(anchor="w")
        self._refresh_status()

    def _refresh_status(self) -> None:
        count = len(self._image_points)
        if self._pending_pixel is not None:
            head = (
                f"画素 (u={self._pending_pixel[0]:.1f}, v={self._pending_pixel[1]:.1f}) を選択中\n"
                "base_link 座標を入力して「登録」"
            )
        else:
            head = "画像上の目標点をクリックしてください"
        self._status.config(text=f"{head}\n登録済み: {count} / {self._target_points} 点")
        state = "normal" if count >= MIN_CORRESPONDENCES else "disabled"
        self._estimate_button.config(state=state)

    def _on_click(self, event) -> None:
        image = self._source.latest_image() if self._frozen_image is None else self._frozen_image
        if image is None:
            self._node.get_logger().warn("画像をまだ受信していません")
            return
        if self._frozen_image is None:
            self._frozen_image = image
            self._node.get_logger().info("画像を固定しました（「解除」で再開）")

        pixel = canvas_to_image_pixel(float(event.x), float(event.y), self._scale)
        height, width = image.shape[:2]
        if not (0.0 <= pixel[0] < width and 0.0 <= pixel[1] < height):
            self._node.get_logger().warn("画像の範囲外がクリックされました")
            return
        self._pending_pixel = pixel
        self._node.get_logger().info(
            f"画素を選択しました: u={pixel[0]:.1f}, v={pixel[1]:.1f} "
            "（u 右方 / v 下方、原点は画像左上）"
        )
        self._entries["x"].focus_set()
        self._redraw()
        self._refresh_status()

    def _on_register(self) -> None:
        if self._pending_pixel is None:
            self._node.get_logger().warn("先に画像上の点をクリックしてください")
            return
        try:
            world = tuple(float(self._entries[key].get()) for key in ("x", "y", "z"))
        except ValueError:
            self._node.get_logger().error("実世界座標は数値で入力してください")
            return

        self._image_points.append(self._pending_pixel)
        self._world_points.append(world)
        index = len(self._image_points)
        self._listbox.insert(
            "end",
            f"{index:2d}: px(u={self._pending_pixel[0]:6.1f}, v={self._pending_pixel[1]:6.1f})"
            f" -> base({world[0]:6.2f}, {world[1]:6.2f}, {world[2]:5.2f})",
        )
        self._node.get_logger().info(
            f"{index}点目を登録しました: 画素(u={self._pending_pixel[0]:.1f}, "
            f"v={self._pending_pixel[1]:.1f}) = base_link({world[0]:.3f}, "
            f"{world[1]:.3f}, {world[2]:.3f}) [m]"
        )
        self._pending_pixel = None
        for entry in self._entries.values():
            entry.delete(0, "end")
        self._on_unfreeze()
        self._refresh_status()

        if index >= self._target_points and self._result is None:
            self._node.get_logger().info(f"指定回数（{self._target_points}点）に到達しました")
            self._on_estimate()

    def _on_undo(self) -> None:
        if self._pending_pixel is not None:
            self._pending_pixel = None
            self._node.get_logger().info("選択中の画素を取り消しました")
        elif self._image_points:
            self._image_points.pop()
            self._world_points.pop()
            self._listbox.delete("end")
            self._node.get_logger().info("直前の対応点を取り消しました")
        self._redraw()
        self._refresh_status()

    def _on_unfreeze(self) -> None:
        self._frozen_image = None

    def _on_estimate(self) -> None:
        try:
            camera_matrix = self._source.camera_matrix()
            if camera_matrix is None:
                raise ValueError("内部パラメータ（CameraInfo）をまだ受信していません")
            image = self._source.latest_image()
            info_size = self._source.info_size()
            if image is not None and info_size is not None:
                image_size = (image.shape[1], image.shape[0])
                if info_size != image_size:
                    raise ValueError(
                        f"CameraInfo の解像度 {info_size[0]}x{info_size[1]} と画像 "
                        f"{image_size[0]}x{image_size[1]} が一致しません（K が画像に対応しません）"
                    )
            self._result = estimate_extrinsic(
                np.array(self._image_points), np.array(self._world_points), camera_matrix
            )
        except ValueError as error:
            self._node.get_logger().error(f"推定に失敗しました: {error}")
            return

        for index, error in enumerate(self._result.reprojection_errors, start=1):
            if error > 3.0:
                self._node.get_logger().warn(
                    f"{index}点目の再投影誤差が大きいです: {error:.2f} px"
                )
        self._node.get_logger().info("\n" + format_result(self._result))
        self._status.config(
            fg="dark green",
            text=(
                f"推定完了 RMS={self._result.rms_reprojection_error:.2f} px\n"
                f"位置 ({self._result.camera_position_base[0]:.3f}, "
                f"{self._result.camera_position_base[1]:.3f}, "
                f"{self._result.camera_position_base[2]:.3f}) m\n"
                "詳細は端末のログを参照"
            ),
        )

    def _draw_marker(self, pixel, index: Optional[int], color: str) -> None:
        x = pixel[0] * self._scale
        y = pixel[1] * self._scale
        self._canvas.create_line(x - 10, y, x + 10, y, fill=color, width=2)
        self._canvas.create_line(x, y - 10, x, y + 10, fill=color, width=2)
        self._canvas.create_oval(x - 5, y - 5, x + 5, y + 5, outline=color, width=2)
        if index is not None:
            self._canvas.create_text(
                x + 12, y - 10, text=str(index), fill=color, anchor="w",
                font=("TkDefaultFont", 11, "bold"),
            )

    def _redraw(self) -> None:
        self._canvas.delete("all")
        if self._photo is not None:
            self._canvas.create_image(0, 0, anchor="nw", image=self._photo)
        for index, pixel in enumerate(self._image_points, start=1):
            self._draw_marker(pixel, index, "lime green")
        if self._pending_pixel is not None:
            self._draw_marker(self._pending_pixel, None, "red")

    def _update_frame(self) -> None:
        image = self._frozen_image if self._frozen_image is not None else self._source.latest_image()
        if image is not None:
            import cv2
            from PIL import Image, ImageTk

            height, width = image.shape[:2]
            if (width, height) != (EXPECTED_IMAGE_WIDTH, EXPECTED_IMAGE_HEIGHT):
                self._node.get_logger().warn(
                    f"画像サイズが想定と異なります: {width}x{height} "
                    f"(想定 {EXPECTED_IMAGE_WIDTH}x{EXPECTED_IMAGE_HEIGHT})",
                    once=True,
                )
            resized = cv2.resize(
                image,
                (int(width * self._scale), int(height * self._scale)),
                interpolation=cv2.INTER_NEAREST,
            )
            self._photo = ImageTk.PhotoImage(
                Image.fromarray(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB))
            )
            self._redraw()
        self._root.after(50, self._update_frame)

    def _on_close(self) -> None:
        self._root.quit()
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="カメラ外部パラメータ推定ツール")
    parser.add_argument("--image-topic", default=IMAGE_TOPIC, help="画像トピック名")
    parser.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC, help="CameraInfo トピック名")
    parser.add_argument(
        "--points", type=int, default=8, help=f"対応点の指定回数（{MIN_CORRESPONDENCES} 以上）"
    )
    parser.add_argument("--scale", type=float, default=2.0, help="GUI の画像表示倍率")
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
        CalibratorGui(node, source, args.points, args.scale).run()
    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

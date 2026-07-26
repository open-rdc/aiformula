#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""推論された舵角から走行予測軌跡を作り、カメラ画像へ重畳して配信する。

/cmd_vel の steering_angle を自転車モデルで曲率へ直し、ロボット座標系の原点から
等間隔に点を並べて 10 m 先までの円弧を作る。それをカメラ行列で画像へ投影すると、
推論した舵角がコース形状に対して妥当かを目視で確認できる。

入力: /zed/zed_node/rgb/image_rect_color, /cmd_vel, /zed/zed_node/rgb/camera_info
出力: /e2e/pilot_net/debug_image

描かれるのは「指令された舵角を運動学モデルに通した理想軌跡」であって実際の走行軌跡ではない。
0630 の実走行 bag で /cmd_vel と /vectornav/imu を突き合わせると、設計ホイールベース 0.8 m の
予測より実際のヨーレートはおよそ 1/4 しかなく、実効ホイールベースは 2 m/s で約 1.4 m、
3 m/s で約 2.1 m、4 m/s で約 2.8 m だった。指令値と実挙動のずれを含めずに見たいときは
wheelbase パラメータへ運用速度の実効値を与えること。
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from steered_drive_msg.msg import SteeredDrive

# 直進とみなす曲率。1/この値 が半径 1e6 m に相当し、画面上は完全な直線になる。
STRAIGHT_CURVATURE = 1e-6


@dataclass
class CameraModel:
    """内部パラメータと、base_link から見た取り付け姿勢。"""

    fx: float
    fy: float
    cx: float
    cy: float
    position: tuple = (0.055, 0.0, 0.69)
    rpy: tuple = (0.0, 0.0, 0.0)
    _rotation: np.ndarray = field(init=False)

    def __post_init__(self):
        roll, pitch, yaw = self.rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy_, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy_, -sy, 0.0], [sy, cy_, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        self._rotation = rz @ ry @ rx

    def to_camera_frame(self, point_base: np.ndarray) -> np.ndarray:
        """base_link 上の点をカメラ座標（x 前方 / y 左 / z 上）へ移す。"""
        return self._rotation.T @ (point_base - np.asarray(self.position, dtype=float))


def curvature_from_steering(steering_angle: float, wheelbase: float) -> float:
    """自転車モデルの曲率 κ = tan(δ) / L。左操舵で正。"""
    return math.tan(steering_angle) / wheelbase


def arc_trajectory(curvature: float, length: float, num_points: int) -> list:
    """曲率一定の円弧を、原点から弧長 length まで等間隔に num_points 点で刻む。"""
    points = []
    for i in range(num_points):
        s = length * i / (num_points - 1) if num_points > 1 else 0.0
        if abs(curvature) < STRAIGHT_CURVATURE:
            points.append((s, 0.0))
            continue
        radius = 1.0 / curvature
        theta = s * curvature
        points.append((radius * math.sin(theta), radius * (1.0 - math.cos(theta))))
    return points


def project_points(camera: CameraModel, points_xy: list) -> list:
    """ロボット座標系の (x, y) 列を画素座標へ投影する。カメラ後方の点は捨てる。"""
    pixels = []
    for x, y in points_xy:
        cam = camera.to_camera_frame(np.array([x, y, 0.0], dtype=float))
        depth = cam[0]
        if depth <= 1e-3:
            continue
        u = camera.fx * (-cam[1] / depth) + camera.cx
        v = camera.fy * (-cam[2] / depth) + camera.cy
        pixels.append((u, v))
    return pixels


def draw_overlay(image: np.ndarray, pixels: list, steering_angle: float,
                 velocity: float, curvature: float, stale: bool) -> np.ndarray:
    """軌跡と数値を画像へ描き込む。"""
    canvas = image.copy()
    integer_pixels = [(int(round(u)), int(round(v))) for u, v in pixels]

    for near, far in zip(integer_pixels, integer_pixels[1:]):
        cv2.line(canvas, near, far, (0, 200, 255), 2, cv2.LINE_AA)
    for i, pixel in enumerate(integer_pixels):
        ratio = i / max(len(integer_pixels) - 1, 1)
        colour = (int(255 * (1.0 - ratio)), 255, int(255 * ratio))
        cv2.circle(canvas, pixel, 4, colour, -1, cv2.LINE_AA)

    radius = math.inf if abs(curvature) < STRAIGHT_CURVATURE else 1.0 / curvature
    lines = [
        f'steer {math.degrees(steering_angle):+6.2f} deg',
        f'vel   {velocity:5.2f} m/s',
        'radius   inf' if math.isinf(radius) else f'radius {radius:+7.2f} m',
    ]
    if stale:
        lines.append('cmd STALE')
    for i, text in enumerate(lines):
        origin = (10, 22 + 20 * i)
        cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 3, cv2.LINE_AA)
        colour = (0, 0, 255) if text == 'cmd STALE' else (255, 255, 255)
        cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    colour, 1, cv2.LINE_AA)
    return canvas


class PilotNetDebugVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('pilot_net_debug_visualizer')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('debug_topic', '/e2e/pilot_net/debug_image')
        self.declare_parameter('wheelbase', 0.8)
        self.declare_parameter('path_length', 10.0)
        self.declare_parameter('num_points', 41)
        self.declare_parameter('cmd_timeout', 0.5)
        # ZED X を 640x360 で配信したときの概算値。camera_info を受け取れば上書きする。
        self.declare_parameter('fx', 224.1)
        self.declare_parameter('fy', 224.1)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 180.0)
        # base_link から見たカメラ取り付け（model.urdf の base_to_chassis + camera_joint）
        self.declare_parameter('camera_position', [0.055, 0.0, 0.69])
        self.declare_parameter('camera_rpy', [-0.0034939840, 0.0436329672, -0.0036430636])

        self.wheelbase = self.get_parameter('wheelbase').value
        self.path_length = self.get_parameter('path_length').value
        self.num_points = self.get_parameter('num_points').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.camera = CameraModel(
            fx=self.get_parameter('fx').value,
            fy=self.get_parameter('fy').value,
            cx=self.get_parameter('cx').value,
            cy=self.get_parameter('cy').value,
            position=tuple(self.get_parameter('camera_position').value),
            rpy=tuple(self.get_parameter('camera_rpy').value))
        self.camera_info_received = False

        self.bridge = CvBridge()
        self.latest_cmd = None
        self.latest_cmd_time = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.image_callback, sensor_qos)
        self.create_subscription(SteeredDrive, self.get_parameter('cmd_topic').value,
                                 self.cmd_callback, 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.camera_info_callback, 10)
        self.debug_publisher = self.create_publisher(
            Image, self.get_parameter('debug_topic').value, 10)

        self.get_logger().info(
            f'舵角から{self.path_length:.1f} m先までの予測軌跡を画像へ重畳し，'
            f'{self.get_parameter("debug_topic").value} へ出版します．')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_info_received:
            return
        self.camera = CameraModel(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5],
                                  position=self.camera.position, rpy=self.camera.rpy)
        self.camera_info_received = True
        self.get_logger().info(
            f'camera_info を受信しました． fx={msg.k[0]:.2f} fy={msg.k[4]:.2f} '
            f'cx={msg.k[2]:.2f} cy={msg.k[5]:.2f}')

    def cmd_callback(self, msg: SteeredDrive) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def image_callback(self, msg: Image) -> None:
        if self.latest_cmd is None:
            return

        elapsed = (self.get_clock().now() - self.latest_cmd_time).nanoseconds * 1e-9
        curvature = curvature_from_steering(self.latest_cmd.steering_angle, self.wheelbase)
        trajectory = arc_trajectory(curvature, self.path_length, self.num_points)
        pixels = project_points(self.camera, trajectory)

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        overlay = draw_overlay(image, pixels, self.latest_cmd.steering_angle,
                               self.latest_cmd.velocity, curvature,
                               stale=elapsed > self.cmd_timeout)

        debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_publisher.publish(debug_msg)


def main() -> None:
    rclpy.init()
    node = PilotNetDebugVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
トポロジカルマップを使ったシミュレーター用ナビゲーションノード。

概要:
  build_topo_map.py で生成した topo_map.json を読み込み、
  キーフレーム列に沿ってピュアパシュートで走行しながら
  ジャンクション分類器の推論結果も同時に配信する。

パラメータ:
  topo_map          topo_map.json のパス (必須)
  model_name        分類器モデルファイル名 (省略時は分類器スキップ)
  speed             走行速度 [m/s]              (default: 1.0)
  lookahead_dist    ピュアパシュート先読み距離 [m]  (default: 1.5)
  wheel_base        ホイールベース [m]            (default: 0.8)
  max_steer         最大舵角 [rad]               (default: 0.5)
  advance_radius    KF通過判定距離 [m]            (default: 0.3)
  odom_topic        オドメトリトピック              (default: /odom)
  image_topic       カメラトピック                 (default: /image_raw)
  use_yolop         YOLOP二値化を使うか            (default: false)
  loop_route        完走後に先頭に戻るか            (default: false)
  interval_ms       制御周期 [ms]                 (default: 100)

配信:
  /cmd_vel                      SteeredDrive  車両制御コマンド
  /topo_nav/command             Int32         マップ由来のナビコマンド
  /topo_nav/predicted_command   Int32         分類器予測コマンド
  /topo_nav/debug_image         Image         デバッグ画像 (マスク+コマンド表示)
"""

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from cv_bridge import CvBridge
def euler_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

from steered_drive_msg.msg import SteeredDrive

LABEL_NAMES: Dict[int, str] = {0: 'straight', 1: 'right', 2: 'left'}
IMAGE_W = 64
IMAGE_H = 48


class TopoNavNodeSim(Node):
    def __init__(self) -> None:
        super().__init__('topo_nav_node_sim')

        # ── パラメータ宣言 ──────────────────────────────────────
        self.declare_parameter('topo_map', '')
        self.declare_parameter('model_name', '')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('lookahead_dist', 1.5)
        self.declare_parameter('wheel_base', 0.8)
        self.declare_parameter('max_steer', 0.5)
        self.declare_parameter('advance_radius', 0.3)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('use_yolop', False)
        self.declare_parameter('loop_route', False)
        self.declare_parameter('interval_ms', 100)

        topo_map_path = self.get_parameter('topo_map').value
        model_name    = self.get_parameter('model_name').value
        self._speed         = float(self.get_parameter('speed').value)
        self._lookahead     = float(self.get_parameter('lookahead_dist').value)
        self._wheel_base    = float(self.get_parameter('wheel_base').value)
        self._max_steer     = float(self.get_parameter('max_steer').value)
        self._advance_radius = float(self.get_parameter('advance_radius').value)
        self._use_yolop     = bool(self.get_parameter('use_yolop').value)
        self._loop_route    = bool(self.get_parameter('loop_route').value)
        interval_ms         = int(self.get_parameter('interval_ms').value)

        odom_topic  = self.get_parameter('odom_topic').value
        image_topic = self.get_parameter('image_topic').value

        # ── トポロジカルマップ読み込み ──────────────────────────
        if not topo_map_path:
            self.get_logger().fatal('topo_map パラメータが必須です')
            raise RuntimeError('topo_map parameter is required')

        self._keyframes: List[Dict] = self._load_topo_map(Path(topo_map_path))
        self._kf_idx: int = 0
        self._route_done: bool = False

        # ── 状態 ───────────────────────────────────────────────
        self.bridge = CvBridge()
        self._current_pose: Optional[tuple] = None   # (x, y, yaw)
        self._latest_image: Optional[np.ndarray] = None

        # ── 推論モデル ─────────────────────────────────────────
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self._classifier = None
        self._yolop_processor = None

        if model_name:
            self._load_classifier(model_name)

        # ── サブスクリプション ──────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_profile_system_default)
        self.sub_image = self.create_subscription(
            Image, image_topic, self._image_cb, qos_profile_sensor_data)

        # ── パブリッシャー ──────────────────────────────────────
        self.pub_cmd      = self.create_publisher(SteeredDrive, '/cmd_vel', 10)
        self.pub_map_cmd  = self.create_publisher(Int32, '/topo_nav/command', 10)
        self.pub_pred_cmd = self.create_publisher(Int32, '/topo_nav/predicted_command', 10)
        self.pub_debug    = self.create_publisher(
            Image, '/topo_nav/debug_image', qos_profile_system_default)

        # ── 制御タイマー ────────────────────────────────────────
        self.timer = self.create_timer(interval_ms / 1000.0, self._control_cb)

        self.get_logger().info(
            f'TopoNavNodeSim 起動: {len(self._keyframes)} KF, '
            f'速度={self._speed} m/s, 先読み={self._lookahead} m'
        )

    # ────────────────────────────────────────────────────────────
    # 初期化ヘルパー
    # ────────────────────────────────────────────────────────────

    def _load_topo_map(self, path: Path) -> List[Dict]:
        if not path.exists():
            raise FileNotFoundError(f'topo_map.json が見つかりません: {path}')
        with open(path) as f:
            data = json.load(f)
        kfs = data['keyframes']
        meta = data.get('metadata', {})
        self.get_logger().info(
            f'トポロジカルマップ読み込み: {len(kfs)} KF, '
            f'総距離={meta.get("total_distance_m", "?")} m, '
            f'分岐点={meta.get("num_decision_points", "?")} 箇所'
        )
        return kfs

    def _load_classifier(self, model_name: str) -> None:
        try:
            from ament_index_python.packages import get_package_share_directory
            import os
            pkg_dir = get_package_share_directory('e2e_planner')
            weight_path = os.path.join(pkg_dir, 'weights', model_name)

            if not os.path.exists(weight_path):
                self.get_logger().warn(f'分類器モデルなし: {weight_path}  (スキップ)')
                return

            self._classifier = torch.jit.load(weight_path, map_location=self.device)
            self._classifier.eval()
            self.get_logger().info(f'分類器読み込み完了: {weight_path}')

            if self._use_yolop:
                yolop_path = Path(pkg_dir) / 'weights' / 'yolopv2.pt'
                if yolop_path.exists():
                    from util.yolop_processor import YOLOPv2Processor
                    self._yolop_processor = YOLOPv2Processor(yolop_path, self.device)
                    # ウォームアップ (JITコンパイル)
                    dummy = np.zeros((300, 480, 3), dtype=np.uint8)
                    self._yolop_processor.process_image(dummy, (IMAGE_W, IMAGE_H))
                    self.get_logger().info('YOLOP初期化完了')
                else:
                    self.get_logger().warn('yolopv2.pt が見つかりません → シム用色抽出を使用')
                    self._use_yolop = False
        except Exception as e:
            self.get_logger().warn(f'分類器の読み込みに失敗: {e}')

    # ────────────────────────────────────────────────────────────
    # コールバック
    # ────────────────────────────────────────────────────────────

    def _nearest_kf(self, x: float, y: float) -> int:
        best, best_dist = 0, float('inf')
        for i, kf in enumerate(self._keyframes):
            d = math.hypot(kf['x'] - x, kf['y'] - y)
            if d < best_dist:
                best_dist, best = d, i
        return best

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if self._current_pose is None:
            self._kf_idx = self._nearest_kf(x, y)
            nearest_kf = self._keyframes[self._kf_idx]
            dist = math.hypot(nearest_kf['x'] - x, nearest_kf['y'] - y)
            self.get_logger().info(
                f'初期KF設定: {self._kf_idx} (距離={dist:.2f} m, '
                f'kf_pos=({nearest_kf["x"]:.2f},{nearest_kf["y"]:.2f}), '
                f'robot_pos=({x:.2f},{y:.2f}))'
            )
        self._current_pose = (x, y, yaw)

    def _image_cb(self, msg: Image) -> None:
        self._latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ────────────────────────────────────────────────────────────
    # 二値化
    # ────────────────────────────────────────────────────────────

    def _binarize(self, image: np.ndarray) -> np.ndarray:
        """BGR画像 → 64×48 二値マスク (0 or 1)。"""
        if self._use_yolop and self._yolop_processor is not None:
            return self._yolop_processor.process_image(image, (IMAGE_W, IMAGE_H))

        # シミュレーター用: 赤ピクセル (R>200, G<50, B<50) を検出
        mask = (
            (image[:, :, 2] > 200) &
            (image[:, :, 1] < 50) &
            (image[:, :, 0] < 50)
        ).astype(np.uint8)
        return cv2.resize(mask, (IMAGE_W, IMAGE_H), interpolation=cv2.INTER_NEAREST)

    # ────────────────────────────────────────────────────────────
    # 分類器推論
    # ────────────────────────────────────────────────────────────

    def _predict_command(self, image: np.ndarray) -> Optional[int]:
        if self._classifier is None:
            return None
        try:
            mask = self._binarize(image)
            tensor = (
                torch.from_numpy(mask.astype(np.float32))
                .unsqueeze(0).unsqueeze(0)
                .to(self.device)
            )
            with torch.no_grad():
                logits = self._classifier(tensor)
            return int(logits.argmax(dim=1).item())
        except Exception as e:
            self.get_logger().warn(f'分類器推論エラー: {e}', throttle_duration_sec=5.0)
            return None

    # ────────────────────────────────────────────────────────────
    # ルート追従ロジック
    # ────────────────────────────────────────────────────────────

    def _update_kf_idx(self, x: float, y: float) -> None:
        """現在位置に基づいてキーフレームインデックスを進める。"""
        while self._kf_idx < len(self._keyframes) - 1:
            kf = self._keyframes[self._kf_idx]
            dist = math.hypot(kf['x'] - x, kf['y'] - y)
            if dist < self._advance_radius:
                self._kf_idx += 1
            else:
                break

        if self._kf_idx >= len(self._keyframes) - 1:
            if self._loop_route:
                self._kf_idx = 0
                self.get_logger().info('ルート完走 → 先頭に戻ります')
            elif not self._route_done:
                self._route_done = True
                self.get_logger().info('ルート完走！ナビゲーション停止。')

    def _find_lookahead_kf(self, x: float, y: float) -> Optional[Dict]:
        """現在位置から先読み距離にあるキーフレームを返す。"""
        best: Optional[Dict] = None
        for i in range(self._kf_idx, min(self._kf_idx + 60, len(self._keyframes))):
            kf = self._keyframes[i]
            dist = math.hypot(kf['x'] - x, kf['y'] - y)
            if dist >= self._lookahead:
                return kf
            best = kf
        return best  # 先読み距離に届かなければ最遠キーフレームを使用

    def _compute_steering(self, x: float, y: float, yaw: float, goal: Dict) -> float:
        """ピュアパシュート操舵角を計算する。"""
        dx = goal['x'] - x
        dy = goal['y'] - y

        # ロボット座標系に変換
        x_local =  dx * math.cos(yaw) + dy * math.sin(yaw)
        y_local = -dx * math.sin(yaw) + dy * math.cos(yaw)

        dist = math.hypot(x_local, y_local)
        if dist < 1e-6:
            return 0.0

        alpha = math.atan2(y_local, x_local)
        steer = math.atan2(
            2.0 * self._wheel_base * math.sin(alpha),
            max(dist, self._lookahead)
        )
        return float(np.clip(steer, -self._max_steer, self._max_steer))

    # ────────────────────────────────────────────────────────────
    # デバッグ画像
    # ────────────────────────────────────────────────────────────

    def _publish_debug(
        self,
        image: np.ndarray,
        nav_cmd: int,
        pred_cmd: Optional[int],
    ) -> None:
        try:
            mask = self._binarize(image)
            debug = cv2.resize(image, (IMAGE_W * 2, IMAGE_H * 2))
            mask_u8 = (mask * 255).astype(np.uint8)
            mask_bgr = cv2.cvtColor(
                cv2.resize(mask_u8, (IMAGE_W * 2, IMAGE_H * 2), interpolation=cv2.INTER_NEAREST),
                cv2.COLOR_GRAY2BGR,
            )
            # 検出ピクセルを赤でオーバーレイ
            overlay = np.zeros_like(debug)
            overlay[mask_bgr[:, :, 0] > 127] = [0, 0, 200]
            debug = cv2.addWeighted(debug, 0.65, overlay, 0.35, 0)

            # コマンド文字
            map_str  = f'Map: {LABEL_NAMES.get(nav_cmd, "?")}'
            pred_str = f'Pred: {LABEL_NAMES.get(pred_cmd, "-")}' if pred_cmd is not None else 'Pred: -'
            kf_str   = f'KF {self._kf_idx}/{len(self._keyframes)}'

            cv2.putText(debug, map_str,  (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            cv2.putText(debug, pred_str, (3, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)
            cv2.putText(debug, kf_str,   (3, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1)

            msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            self.pub_debug.publish(msg)
        except Exception:
            pass

    # ────────────────────────────────────────────────────────────
    # 制御メインループ
    # ────────────────────────────────────────────────────────────

    def _control_cb(self) -> None:
        if self._current_pose is None:
            return

        x, y, yaw = self._current_pose

        # キーフレーム進捗更新
        self._update_kf_idx(x, y)

        # マップ由来のナビコマンド取得・配信
        kf = self._keyframes[self._kf_idx]
        nav_cmd: int = int(kf.get('nav_command', 0))
        self.pub_map_cmd.publish(Int32(data=nav_cmd))

        # 分類器推論
        pred_cmd: Optional[int] = None
        if self._latest_image is not None:
            pred_cmd = self._predict_command(self._latest_image)
            if pred_cmd is not None:
                self.pub_pred_cmd.publish(Int32(data=pred_cmd))

        # デバッグ画像配信
        if self._latest_image is not None:
            self._publish_debug(self._latest_image, nav_cmd, pred_cmd)

        # ── 車両制御コマンド ──
        drive = SteeredDrive()
        if self._route_done:
            drive.velocity = 0.0
            drive.steering_angle = 0.0
            self.pub_cmd.publish(drive)
            return

        goal_kf = self._find_lookahead_kf(x, y)
        if goal_kf is None:
            return

        steer = self._compute_steering(x, y, yaw, goal_kf)
        drive.velocity = self._speed
        drive.steering_angle = steer
        self.pub_cmd.publish(drive)

        # 進捗ログ (30KFごと)
        if self._kf_idx % 30 == 0:
            pred_label = LABEL_NAMES.get(pred_cmd, '-') if pred_cmd is not None else '-'
            self.get_logger().info(
                f'KF {self._kf_idx:4d}/{len(self._keyframes)}'
                f'  map={LABEL_NAMES[nav_cmd]:8s}'
                f'  pred={pred_label:8s}'
                f'  steer={steer:+.3f} rad'
                f'  pos=({x:.2f},{y:.2f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopoNavNodeSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

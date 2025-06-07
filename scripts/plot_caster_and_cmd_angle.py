#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from socketcan_interface_msg.msg import SocketcanIF
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import numpy as np


class CanCmdPlotNode(Node):
    def __init__(self):
        super().__init__('can_cmd_plot_node')

        # 最新の受信データを保持する変数
        self.latest_candata0 = None
        self.latest_angular_z = None
        self.latest_linear_x = None

        # プロット用の履歴データ
        self.x_data = []
        self.y_candata0 = []
        self.y_angular_z = []

        # CANメッセージのサブスクライバ
        self.can_sub = self.create_subscription(
            SocketcanIF,
            '/can_rx_11',
            self.can_callback,
            10
        )

        # cmd_velメッセージのサブスクライバ
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('CanCmdPlotNode 起動完了')

    # CANデータ受信時の処理
    def can_callback(self, msg: SocketcanIF):
        if len(msg.candata) > 1:
            val = int(msg.candata[0])
            sign_flag = int(msg.candata[1])
            signed_val = val if sign_flag == 0 else -val if sign_flag == 255 else val

            # ノイズ除外（絶対値が200を超えるデータ）
            if abs(signed_val) > 200:
                self.get_logger().warn(f'candata[0] == ±{abs(signed_val)} → ノイズ除外')
                return

            # CAN値を角度に変換（-30〜+30度）
            angle_deg = signed_val * (30.0 / 255.0)
            self.latest_candata0 = angle_deg

    # cmd_vel受信時の処理
    def cmd_vel_callback(self, msg: Twist):
        self.latest_angular_z = float(msg.angular.z)
        self.latest_linear_x = float(msg.linear.x)

    # データ記録関数
    def record_data(self, sample_idx: int):
        self.x_data.append(sample_idx)
        self.y_candata0.append(
            self.y_candata0[-1] if self.latest_candata0 is None and self.y_candata0 else self.latest_candata0
        )
        self.y_angular_z.append(
            self.y_angular_z[-1] if self.latest_angular_z is None and self.y_angular_z else self.latest_angular_z
        )


def main(args=None):
    rclpy.init(args=args)
    node = CanCmdPlotNode()

    # マルチスレッドスピンの設定
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    MAX_HISTORY = 200  # 最大履歴長
    sample_idx = 0

    # === グラフ構成（2段） ===
    plt.ion()
    fig, axes = plt.subplots(nrows=2, figsize=(10, 6), sharex=True)
    fig.suptitle('Real-time Caster Angel & CmdVel Plot', fontsize=14)

    ax1, ax2 = axes
    ax1.set_ylabel('Caster Angle (deg)')
    ax2.set_ylabel('CmdVel Angle (rad/s)')
    ax2.set_xlabel('Sample Index')

    line_candata0, = ax1.plot([], [], label='candata[0]', linewidth=1)
    line_angular_z, = ax2.plot([], [], label='angular.z', linewidth=1)

    for ax in axes:
        ax.grid(True)
        ax.axhline(0, color='gray', linestyle='--', linewidth=0.8)

    # リストを最大履歴長に保つ
    def trim(lst):
        if len(lst) > MAX_HISTORY:
            del lst[0:len(lst) - MAX_HISTORY]

    # None値を除外
    def clean_data(x, y):
        cleaned = [(xi, yi) for xi, yi in zip(x, y) if yi is not None]
        if len(cleaned) < 4:
            return np.array([]), np.array([])
        x_new, y_new = zip(*cleaned)
        return np.array(x_new), np.array(y_new)

    # 曲線を平滑化
    def smooth_curve(x, y):
        if len(x) < 4:
            return x, y
        sorted_indices = np.argsort(x)
        x_sorted = x[sorted_indices]
        y_sorted = y[sorted_indices]
        x_smooth = np.linspace(x_sorted.min(), x_sorted.max(), 300)
        y_smooth = make_interp_spline(x_sorted, y_sorted, k=3)(x_smooth)
        return x_smooth, y_smooth

    try:
        while rclpy.ok():
            node.record_data(sample_idx)

            trim(node.x_data)
            trim(node.y_candata0)
            trim(node.y_angular_z)

            x_c, y_candata0 = clean_data(node.x_data, node.y_candata0)
            x_a, y_angular_z = clean_data(node.x_data, node.y_angular_z)

            x_s_c, y_s_candata0 = smooth_curve(x_c, y_candata0)
            x_s_a, y_s_angular_z = smooth_curve(x_a, y_angular_z)

            line_candata0.set_data(x_s_c, y_s_candata0)
            line_angular_z.set_data(x_s_a, y_s_angular_z)

            # X軸のスクロール表示
            if node.x_data:
                max_x = node.x_data[-1]
                min_x = max_x - MAX_HISTORY
                for ax in axes:
                    ax.set_xlim(min_x, max_x)
                    ax.set_xticks(np.arange(min_x, max_x + 1, 50))

            # Y軸スケーリング（0中心）
            for ax, data in zip(axes, [y_s_candata0, y_s_angular_z]):
                if len(data) > 0:
                    max_abs = max(abs(np.min(data)), abs(np.max(data)))
                    margin = 0.1 * max_abs if max_abs != 0 else 1.0
                    ax.set_ylim(-max_abs - margin, max_abs + margin)

            # linear.x をタイトルに表示
            linear_x_val = node.latest_linear_x if node.latest_linear_x is not None else 0.0
            fig.suptitle(f'Real-time Caster Angle & CmdVel Plot  |  linear.x = {linear_x_val:.1f} (m/s)', fontsize=14)

            fig.canvas.draw()
            fig.canvas.flush_events()

            sample_idx += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    node.get_logger().info('シャットダウン中...')
    executor.shutdown()
    rclpy.shutdown()
    plt.close(fig)


if __name__ == '__main__':
    main()

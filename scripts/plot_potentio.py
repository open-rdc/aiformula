#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
from socketcan_interface_msg.msg import SocketcanIF

import matplotlib.pyplot as plt


# =========================================
# CANデータの先頭2バイトを符号付きshort型に変換
# =========================================
def bytes_to_short(byte_list):
    return struct.unpack('<h', bytearray(byte_list[:2]))[0]


# =========================================
# ROSノード：CANからPotentioを抽出しプロット
# =========================================
class PlotPotentioNode(Node):
    def __init__(self):
        super().__init__('plot_potentio')
        qos = rclpy.qos.QoSProfile(depth=10)

        # CANデータを受信し、/potentio に変換・発行
        self.create_subscription(SocketcanIF, '/can_rx_011', self._can_callback, qos)
        self.potentio_pub = self.create_publisher(Int64, '/potentio', qos)

        # /potentio と /cmd_vel を購読してプロット用に記録
        self.create_subscription(Int64, '/potentio', self._potentio_callback, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmdvel_callback, qos)

        # 最新値と履歴バッファ
        self.latest_potentio = 0
        self.latest_cmdvel = 0.0
        self.x_data = []
        self.y_potentio = []
        self.y_cmdvel = []

    def _can_callback(self, msg):
        # candata の先頭2バイトを short に変換して /potentio にパブリッシュ
        value = bytes_to_short(msg.candata[:2])
        self.potentio_pub.publish(Int64(data=value))
        self.get_logger().debug(f"Published /potentio: {value}")

    def _potentio_callback(self, msg):
        self.latest_potentio = msg.data

    def _cmdvel_callback(self, msg):
        self.latest_cmdvel = msg.angular.z


# =========================================
# メイン関数：ノード起動＆matplotlibリアルタイムプロット
# =========================================
def main(args=None):
    rclpy.init(args=args)
    node = PlotPotentioNode()

    # マルチスレッドスピンの実行
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # プロット設定
    MAX_HISTORY = 200
    sample_idx = 0
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Potentio & CmdVel Plot')

    ax1.set_ylabel('Potentio')
    ax2.set_ylabel('CmdVel.angular.z')
    ax2.set_xlabel('Sample Index')

    line_potentio, = ax1.plot([], [], label='/potentio')
    line_cmdvel, = ax2.plot([], [], label='/cmd_vel')

    for ax in (ax1, ax2):
        ax.grid(True)
        ax.axhline(0, color='gray', linestyle='--', linewidth=0.8)
        ax.legend()

    def trim_history(lst):
        if len(lst) > MAX_HISTORY:
            del lst[0:len(lst) - MAX_HISTORY]

    try:
        while plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            sample_idx += 1

            pot = node.latest_potentio
            cmd = node.latest_cmdvel

            node.x_data.append(sample_idx)
            node.y_potentio.append(pot)
            node.y_cmdvel.append(cmd)

            trim_history(node.x_data)
            trim_history(node.y_potentio)
            trim_history(node.y_cmdvel)

            line_potentio.set_data(node.x_data, node.y_potentio)
            line_cmdvel.set_data(node.x_data, node.y_cmdvel)

            # Y軸を0中心でスケーリング
            max_pot = max(abs(v) for v in node.y_potentio) or 1
            max_cmd = max(abs(v) for v in node.y_cmdvel) or 1
            ax1.set_ylim(-max_pot * 1.1, max_pot * 1.1)
            ax2.set_ylim(-max_cmd * 1.1, max_cmd * 1.1)

            # X軸スライド表示
            ax1.set_xlim(max(0, sample_idx - MAX_HISTORY), sample_idx)
            ax2.set_xlim(max(0, sample_idx - MAX_HISTORY), sample_idx)

            plt.pause(0.05)

    except KeyboardInterrupt:
        print("プロットを中断しました。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

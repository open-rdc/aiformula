#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from collections import deque
import math
import sys

class KinematicModelEvaluator(Node):
    def __init__(self):
        super().__init__('kinematic_model_evaluator')

        # パラメータ設定
        # wheelbase (L): 前輪と後輪の距離。モデルの旋回半径計算に使用
        self.declare_parameter('wheelbase', 0.8)
        self.L = self.get_parameter('wheelbase').value
        
        # steer_gain: 操舵角(caster_data)に対する補正ゲイン
        self.declare_parameter('steer_gain', 1)
        self.steer_gain = self.get_parameter('steer_gain').value
        
        self.lock = threading.Lock()

        # State [x, y, theta]
        # State [x, y, theta]
        # 状態変数の初期化 [x, y, theta]
        self.est_state = [0.0, 0.0, 0.0]  # モデル推定値 (操舵角ベース: "計算上の動き")
        self.odom_state = [0.0, 0.0, 0.0] # 実測オドメトリ (IMUジャイロベース: "実際の動き")
        
        # Inputs
        # 入力データ
        self.latest_sensor_v = 0.0      # 車速 (m/s)
        self.latest_sensor_delta = 0.0  # 操舵角 (rad) - 緑色の線
        self.latest_sensor_omega = 0.0  # 角速度 (rad/s) - 紫色の線 (IMU実測値)

        self.last_update_time = None

        # Data History
        # Data History
        self.time_hist = deque(maxlen=50000)
        self.est_x_hist = deque(maxlen=50000)
        self.odom_x_hist = deque(maxlen=50000)
        self.delta_hist = deque(maxlen=50000)
        self.omega_hist = deque(maxlen=50000)
        
        # Subscribers
        # Subscribers
        self.create_subscription(TwistWithCovarianceStamped, '/vectornav/velocity_body', self.sensor_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/caster_data', self.sensor_caster_callback, 10)
        
        # Publisher
        self.pub_est_pose = self.create_publisher(PoseStamped, '/kinematic_estimation/pose', 10)

        # Timer for integration loop (100Hz)
        self.create_timer(0.01, self.integration_loop)

        self.start_time = time.time()
        self.get_logger().info(f"Evaluator Started. Integrating Command (Model) vs Sensor (Truth). L={self.L}m, SteerGain={self.steer_gain}")

    def sensor_caster_callback(self, msg):
        if msg.data:
            self.latest_sensor_delta = msg.data[0]

    def sensor_vel_callback(self, msg):
        self.latest_sensor_v = msg.twist.twist.linear.x
        # IMU angular velocity is typicaly CCW+ in ROS.
        # If user system is Right+ (CW+), this might need inversion?
        # But for comparison, if both models use consistent internal frame (ENU), 
        # we assume msg is standard ROS (CCW+) or we consistently apply sign.
        # ROSの標準的な角速度は反時計回り(CCW)が正ですが、
        # 操舵角(Steering)の定義に合わせて符号を反転させます
        self.latest_sensor_omega = -msg.twist.twist.angular.z

    def integration_loop(self):
        current_time = time.time()
        if self.last_update_time is None:
            self.last_update_time = current_time
            return

        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if dt > 0.1: # 時間差が大きすぎる場合は飛躍を防ぐためにスキップ
            return

        # --- モデル推定 (Sensor Driven Model) ---
        # 操舵角(delta)と車速(v)から、「本来これだけ曲がるはず」という軌道を計算
        # 赤色の線
        # x_dot = v * cos(theta)
        # theta_dot = v * tan(delta) / L  <-- 運動学モデル(Bicycle Model)
        sx, sy, stheta = self.est_state
        sv = self.latest_sensor_v
        sdelta = self.latest_sensor_delta * self.steer_gain
        
        sx += sv * math.cos(stheta) * dt
        sy += sv * math.sin(stheta) * dt
        stheta += (sv * math.tan(sdelta) / self.L) * dt
        
        self.est_state = [sx, sy, stheta]

        # --- 実測オドメトリ (Truth / IMU Odometry) ---
        # IMUの角速度(omega)と車速(v)から、「実際に車両がどう動いたか」を計算
        # 青色の点線
        # x_dot = v * cos(theta)
        # theta_dot = omega (IMUの直接計測値)
        ox, oy, otheta = self.odom_state
        # sv is same (velocity_body)
        somega = self.latest_sensor_omega

        ox += sv * math.cos(otheta) * dt
        oy += sv * math.sin(otheta) * dt
        otheta += somega * dt # Assuming CCW+ standard ROS
        
        self.odom_state = [ox, oy, otheta]

        # 推定された姿勢をパブリッシュ (モデル予測値)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map" # グローバル座標系(map)と仮定
        pose_msg.pose.position.x = sx
        pose_msg.pose.position.y = sy
        
        # ヨー角をクォータニオンに変換
        cy_q = math.cos(stheta * 0.5)
        sy_q = math.sin(stheta * 0.5)
        pose_msg.pose.orientation.w = cy_q
        pose_msg.pose.orientation.z = sy_q
        
        self.pub_est_pose.publish(pose_msg)

        # Store
        with self.lock:
            self.time_hist.append(current_time - self.start_time)
            self.est_x_hist.append(sx)
            self.odom_x_hist.append(ox)
            self.delta_hist.append(sdelta)
            self.omega_hist.append(somega)

def plot_thread(node):
    plt.ion()
    fig, ax = plt.subplots(2, 1, figsize=(10, 10))
    
    # Plot 1: Position X
    # グラフ1: 位置の比較 (軌跡)
    # 赤実線: モデル予測 (操舵角から計算)
    # 青点線: 実測オドメトリ (IMUから計算)
    ax[0].set_title("Model Evaluation: Model (Steering) vs IMU Odom (Gyro)")
    ax[0].set_xlabel("Time [s]")
    ax[0].set_ylabel("Position X [m]")
    ax[0].grid(True)
    
    line_est, = ax[0].plot([], [], 'r-', label='Model X (from Steering)', linewidth=2)
    line_odom, = ax[0].plot([], [], 'b--', label='Odom X (from Gyro)', linewidth=2, alpha=0.7)
    ax[0].legend()

    # グラフ2: 入力値の比較
    # 緑線: 操舵角 (入力)
    # 紫線: 角速度 (結果/実測値)
    ax[1].set_title("Input Comparison: Steering Angle vs Angular Velocity")
    ax[1].set_xlabel("Time [s]")
    ax[1].set_ylabel("Amplitude [rad or rad/s]")
    ax[1].grid(True)
    
    line_delta, = ax[1].plot([], [], 'g-', label='Steering Delta (rad)', linewidth=1, alpha=0.8)
    line_omega, = ax[1].plot([], [], 'm-', label='IMU Omega (rad/s)', linewidth=1, alpha=0.8)
    ax[1].legend()

    while rclpy.ok():
        # ロックを取得してデータのスナップショットを作成
        with node.lock:
            # ロック内で再度データ長を確認
            if len(node.time_hist) <= 1:
                time.sleep(0.1)
                continue
                
            t_data = list(node.time_hist)
            est_x = list(node.est_x_hist)
            odom_x = list(node.odom_x_hist)
            delta_data = list(node.delta_hist)
            omega_data = list(node.omega_hist)
        
        # 万が一の競合に備えて、全てのリストの長さを最小のものに合わせる
        min_len = min(len(t_data), len(est_x), len(odom_x), len(delta_data), len(omega_data))
        t_data = t_data[:min_len]
        est_x = est_x[:min_len]
        odom_x = odom_x[:min_len]
        delta_data = delta_data[:min_len]
        omega_data = omega_data[:min_len]
            
        # Update plot
        try:
            line_est.set_data(t_data, est_x)
            line_odom.set_data(t_data, odom_x)
            
            line_delta.set_data(t_data, delta_data)
            line_omega.set_data(t_data, omega_data)
            
            ax[0].relim()
            ax[0].autoscale_view()
            ax[1].relim()
            ax[1].autoscale_view()
            
            fig.canvas.draw()
            fig.canvas.flush_events()
        except ValueError as e:
            # エラー発生時のフェイルセーフ
            print(f"Plot error: {e}")
        
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicModelEvaluator()
    
    # Run ROS spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run Plotting in Main Thread
    try:
        plot_thread(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # spin_thread does not need explicit join as it is daemon, but cleaner if we could.

if __name__ == '__main__':
    main()

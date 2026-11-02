import gymnasium as gym
from gymnasium import spaces
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

class AiformulaEnv(gym.Env[np.ndarray, np.ndarray]):
    """
    観測：カメラ画像 (1チャンネル, 84x84)
    報酬：車速とコース中央からのズレに基づく報酬関数
    終了条件：コースアウト or タイムアウト
    """
    metadata = {"render_modes": [], "render_fps": 50}

    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('gym_gazebo_bridge_node')
        self.cv_bridge = CvBridge()

        # アクション空間の定義 (ステアリング, アクセル)
        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        # 観測空間の定義 (AIへの入力はカメラ画像のみ: 1チャンネル, 84x84)
        self.observation_space = spaces.Box(
            low=0.0, 
            high=255.0, 
            shape=(1, 84, 84), 
            dtype=np.float32
        )

        # ROS 2 通信設定
        self.cmd_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # カメラ画像の購読
        self.latest_image = np.zeros((1, 84, 84), dtype=np.float32)
        self.image_subscriber = self.node.create_subscription(
            Image, 
            '/camera/image_raw', 
            self._image_callback, 
            10
        )

        # オドメトリ情報の購読（報酬計算用の「内部データ」として使用）
        self.current_velocity = 0.0
        self.current_lateral_error = 0.0  # 必要に応じてオドメトリや真値から算出
        self.odom_subscriber = self.node.create_subscription(
            Odometry, 
            '/odom', 
            self._odom_callback, 
            10
        )

        self.current_step = 0
        self.max_episode_steps = 1000

    def _image_callback(self, msg: Image):
        # ROSの画像をOpenCV形式に変換して84x84のグレースケールにリサイズ
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        resized = cv2.resize(cv_image, (84, 84))
        # shape = (1, 84, 84) に整形
        self.latest_image = np.expand_dims(resized, axis=0).astype(np.float32)

    def _odom_callback(self, msg: Odometry):
        # 報酬計算のために現在の速度や位置を内部保持
        self.current_velocity = msg.twist.twist.linear.x
        
        # Y座標をコース中央からのズレに見立てる場合（環境に合わせて調整）
        self.current_lateral_error = msg.pose.pose.position.y

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        
        # シミュレーションを少し進めて最新のセンサー値・画像を取り込む
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        info = {}
        return self.latest_image, info

    def step(self, action: np.ndarray):
        # アクションをGazeboへ送信
        twist = Twist()
        twist.angular.z = float(action[0])  # ステアリング
        twist.linear.x = float(action[1])   # アクセル
        self.cmd_publisher.publish(twist)

        # シミュレーションのステップを進める
        rclpy.spin_once(self.node, timeout_sec=0.02)
        self.current_step += 1

        # AIへの観測（Next State）は「最新のカメラ画像」のみを返す
        next_state = self.latest_image

        # 終了判定（コースアウトやタイムアウト）
        # 内部で保持しているオドメトリのズレや車速から判定
        out_of_lane = abs(self.current_lateral_error) > 2.0
        terminated = bool(out_of_lane)
        truncated = bool(self.current_step >= self.max_episode_steps)

        # 報酬の計算（内部のオドメトリ情報や速度をフル活用！）
        if terminated:
            reward = -100.0  # コースアウトの大きなペナルティ
        else:
            # 速度が出ていて、かつコース中央（ズレが0）にいるほど高い報酬
            reward = float(self.current_velocity - abs(self.current_lateral_error) * 2.0)

        info = {
            "velocity": self.current_velocity,
            "lateral_error": self.current_lateral_error
        }

        return next_state, reward, terminated, truncated, info

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        super().close()
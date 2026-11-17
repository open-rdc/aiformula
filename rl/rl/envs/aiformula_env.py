import math
import subprocess
import threading
import time

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, Int8
from steered_drive_msg.msg import SteeredDrive
from cv_bridge import CvBridge
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class EnvConfig:
    sim: bool = True
    topomap_path: Optional[str] = None
    max_velocity: float = 3.0
    max_steering_deg: float = 15.0
    world_name: str = 'car_world'
    model_name: str = 'ai_car1'
    start_pose: Tuple[float, float, float, float] = (0.5, 0.3, 0.0, -1.5)
    settle_sec: float = 1.0
    auto_reset: bool = False
    max_episode_sec: float = 300.0
    control_period_sec: float = 0.05
    slope_s0: float = 3.0
    lane_penalty_k: float = 3.0
    departure_penalty: float = -200.0
    intervention_penalty: float = -400.0

class AiformulaEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata = {"render_modes": [], "render_fps": 50}

    def __init__(self, config: Optional[EnvConfig] = None):
        super().__init__()

        self.config = config if config is not None else EnvConfig()
        self.max_velocity = self.config.max_velocity
        self.max_steering_rad = np.deg2rad(self.config.max_steering_deg)

        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('gym_gazebo_bridge_node')
        self.cv_bridge = CvBridge()

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=0.0, 
            high=255.0, 
            shape=(1, 84, 84), 
            dtype=np.float32
        )

        self.cmd_publisher = self.node.create_publisher(SteeredDrive, '/cmd_vel', 10)

        self.latest_image = np.zeros((1, 84, 84), dtype=np.float32)
        self.image_subscriber = self.node.create_subscription(
            Image, 
            '/zed/zed_node/rgb/image_rect_color',
            self._image_callback,
            qos_profile_sensor_data
        )

        self.current_velocity = 0.0

        self.lane_closeness = (0.0, 0.0)
        self.lane_state_subscriber = self.node.create_subscription(
            Float32MultiArray,
            '/lane_state',
            self._lane_state_callback,
            qos_profile_sensor_data
        )

        self.vectornav_velocity_subscriber = self.node.create_subscription(
            TwistWithCovarianceStamped, 
            '/vectornav/velocity_body', 
            self._vectornav_velocity_callback,
            qos_profile_sensor_data
        )

        self.departure = 0
        self.departure_subscriber = self.node.create_subscription(
            Int8,
            '/lane_departure',
            self._departure_callback,
            10
        )

        self.autonomous = False
        self.autonomous_starts = 0
        self.autonomous_subscriber = self.node.create_subscription(
            Bool, '/autonomous', self._autonomous_callback, 10)

        self.current_step = 0
        self.episode_start = time.monotonic()

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.next_step_time = time.monotonic()

    def _departure_callback(self, msg: Int8):
        if msg.data in (-1, 1):
            self.departure = int(msg.data)

    def _autonomous_callback(self, msg: Bool):
        if msg.data and not self.autonomous:
            self.autonomous_starts += 1
        self.autonomous = bool(msg.data)

    def _wait_for_autonomous(self, require_new_start: bool):
        starts = self.autonomous_starts
        def ready():
            return self.autonomous and (not require_new_start or self.autonomous_starts > starts)
        if not ready():
            if require_new_start and self.autonomous:
                msg = 'Share で手動に切り替えて車をコースに戻し、もう一度 Share で自動にすると次のエピソードを始めます'
            else:
                msg = '車をコースに置いて Share で自動にすると次のエピソードを始めます'
            self.node.get_logger().warn(msg)
        while rclpy.ok() and not ready():
            time.sleep(0.05)

    def _publish_stop(self):
        if self.autonomous:
            self.cmd_publisher.publish(SteeredDrive())

    def _teleport_to_start(self):
        x, y, z, yaw = self.config.start_pose
        req = (f'name: "{self.config.model_name}", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        cmd = ['ign', 'service', '-s', f'/world/{self.config.world_name}/set_pose',
               '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
               '--timeout', '2000', '--req', req]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            ok = result.returncode == 0 and 'data: true' in result.stdout
        except (OSError, subprocess.TimeoutExpired) as e:
            ok, result = False, e
        if not ok:
            self.node.get_logger().error(f'車の初期位置への移動に失敗しました: {result}')

    def _spin_for(self, duration: float):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._publish_stop()
            time.sleep(0.05)

    def _image_callback(self, msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        resized = cv2.resize(cv_image, (84, 84))
        self.latest_image = np.expand_dims(resized, axis=0).astype(np.float32)

    def _lane_state_callback(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 10:
            return
        self.lane_closeness = (self._closeness(d[0], d[2]), self._closeness(d[5], d[7]))

    def _closeness(self, valid: float, slope: float) -> float:
        if valid < 0.5 or not math.isfinite(slope):
            return 0.0
        return float(np.clip(1.0 - abs(slope) / self.config.slope_s0, 0.0, 1.0))

    def _vectornav_velocity_callback(self, msg: TwistWithCovarianceStamped):
        self.current_velocity = msg.twist.twist.linear.x

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0

        self._publish_stop()
        if self.config.sim and self.config.auto_reset:
            self._teleport_to_start()
            self._wait_for_autonomous(require_new_start=False)
        else:
            self._wait_for_autonomous(require_new_start=True)
        self._spin_for(self.config.settle_sec)
        self.departure = 0
        self.episode_start = time.monotonic()
        self.next_step_time = self.episode_start

        info = {}
        return self.latest_image, info

    def step(self, action: np.ndarray):
        intervened = not self.autonomous
        if not intervened:
            cmd = SteeredDrive()
            cmd.steering_angle = float(action[0]) * self.max_steering_rad
            cmd.velocity = (float(action[1]) + 1.0) / 2.0 * self.max_velocity
            self.cmd_publisher.publish(cmd)

        self.next_step_time = max(self.next_step_time + self.config.control_period_sec, time.monotonic())
        time.sleep(max(0.0, self.next_step_time - time.monotonic()))
        self.current_step += 1

        next_state = self.latest_image

        terminated = bool(self.departure != 0 or intervened)
        elapsed = time.monotonic() - self.episode_start
        truncated = bool(not terminated and elapsed >= self.config.max_episode_sec)

        closeness = max(self.lane_closeness)
        if intervened:
            reward = self.config.intervention_penalty
        elif terminated:
            reward = self.config.departure_penalty
        else:
            reward = float(self.current_velocity - self.config.lane_penalty_k * closeness)

        info = {
            "velocity": self.current_velocity,
            "closeness_left": self.lane_closeness[0],
            "closeness_right": self.lane_closeness[1],
            "departure": self.departure,
            "intervened": intervened,
            "elapsed": elapsed,
        }

        return next_state, reward, terminated, truncated, info

    def close(self):
        self._publish_stop()
        self.executor.shutdown()
        self.spin_thread.join(timeout=1.0)
        self.node.destroy_node()
        rclpy.shutdown()
        super().close()
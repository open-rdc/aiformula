#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import csv
from pathlib import Path
from collections import deque
from typing import Optional, List, Tuple, Deque
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
import pymap3d as pm
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.125
NUM_WAYPOINTS = 20

# ZED の camera_fps と揃える。grab() を撮影レートで回して古いフレームを掴まないようにする
TIMER_PERIOD = 1.0 / 30.0

# pose 履歴に残しておく最低限のマージン[s]。補間に必要な「target_time より前の1点」を確保する
POSE_HISTORY_MARGIN = 0.5


class PoseSample:
    """補間に使う最小限の姿勢。ECEF 位置とヨー角だけ持つ"""

    def __init__(self, timestamp: float, position: np.ndarray, yaw: float):
        self.timestamp: float = timestamp
        self.position: np.ndarray = position
        self.yaw: float = yaw


class Sample:
    def __init__(self, image: np.ndarray, timestamp: float):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        # 画像取得時刻ちょうどの姿勢は後から補間で解決する
        self.reference: Optional[PoseSample] = None
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]


class DataCollectionNode(Node):
    def __init__(self) -> None:
        super().__init__('data_collection_node')

        self.declare_parameter('sdk_flag', True)
        self.sdk_flag_ = self.get_parameter('sdk_flag').value

        self.bridge: CvBridge = CvBridge()
        self.latest_image: Optional[Image] = None

        self.samples: List[Sample] = []
        self.pose_history: Deque[PoseSample] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]]]] = []
        self.last_sample_time: Optional[float] = None
        self.warned_missing_stamp: bool = False

        self.is_paused: bool = True
        self.prev_button_state: int = 0

        self.zed_camera: Optional[sl.Camera] = None
        self.zed_image: Optional[sl.Mat] = None
        self.zed_runtime_params: Optional[sl.RuntimeParameters] = None

        if self.sdk_flag_:
            if not ZED_SDK_AVAILABLE:
                self.get_logger().error('ZED SDK not available. Install pyzed package.')
                raise RuntimeError('ZED SDK not available')
            self._initialize_zed_camera()
        else:
            self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile_sensor_data)

        # VectorNav pose subscription (used in both SDK and ROS modes)
        # 間引かずコールバックのたびに履歴へ積む（タイマーで拾うと 10Hz に落ちてラベル誤差になる）
        self.create_subscription(PoseWithCovarianceStamped, '/vectornav/pose', self.pose_callback, qos_profile_sensor_data)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.get_logger().info('⚪Create data started')

    def _initialize_zed_camera(self) -> None:
        self.zed_camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        init_params.coordinate_units = sl.UNIT.METER

        err = self.zed_camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open ZED camera: {err}')
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        self.zed_image = sl.Mat()
        self.zed_runtime_params = sl.RuntimeParameters()
        self.get_logger().info('ZED camera initialized (image only)')

    def _capture_data_from_zed(self) -> Tuple[Optional[np.ndarray], Optional[float]]:
        """画像と、その画像が実際に撮影された時刻(epoch秒)を返す"""
        if self.zed_camera.grab(self.zed_runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None, None

        self.zed_camera.retrieve_image(self.zed_image, sl.VIEW.LEFT)
        image = self.zed_image.get_data()
        height, width = image.shape[:2]
        resized_image = cv2.resize(image, (width // 2, height // 2))

        # 受信時刻ではなく撮影時刻を使う。5m/s では 100ms のズレが 0.5m のラベル誤差になる
        image_timestamp = self.zed_camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() * 1e-9

        return resized_image, image_timestamp

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def _stamp_to_seconds(self, stamp) -> Optional[float]:
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if seconds <= 0.0:
            return None
        return seconds

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        if timestamp is None:
            # ドライバが stamp を埋めていない場合のみ受信時刻で代用する
            timestamp = time.time()
            if not self.warned_missing_stamp:
                self.get_logger().warn('/vectornav/pose has empty header.stamp; falling back to arrival time')
                self.warned_missing_stamp = True

        # 同時刻/逆順のメッセージは補間を壊すので捨てる
        if self.pose_history and timestamp <= self.pose_history[-1].timestamp:
            return

        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.pose_history.append(PoseSample(timestamp, position, yaw))

    def joy_callback(self, msg: Joy) -> None:
        if len(msg.buttons) > 2:
            current_button_state = msg.buttons[2]
            if current_button_state == 1 and self.prev_button_state == 0:
                self.is_paused = not self.is_paused
                if self.is_paused:
                    self.get_logger().info('⏸️ Data collection paused')
                else:
                    self.pose_history.clear()
                    self.samples.clear()
                    self.last_sample_time = None
                    self.get_logger().info('▶️ Data collection resumed')
            self.prev_button_state = current_button_state

    def timer_callback(self) -> None:
        if self.sdk_flag_:
            # 一時停止中もカメラバッファは消費し続けないと古いフレームが溜まる
            image, image_timestamp = self._capture_data_from_zed()
        else:
            image, image_timestamp = None, None
            if self.latest_image is not None:
                image_timestamp = self._stamp_to_seconds(self.latest_image.header.stamp)
                if image_timestamp is not None:
                    image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgra8')

        if self.is_paused:
            return

        if image is not None and image_timestamp is not None:
            if self.last_sample_time is None or image_timestamp - self.last_sample_time >= SAMPLE_INTERVAL:
                self.samples.append(Sample(image, image_timestamp))
                self.last_sample_time = image_timestamp

        for sample in self.samples:
            if sample.reference is None:
                sample.reference = self.interpolate_pose(sample.timestamp)
            if sample.reference is not None:
                self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((sample.image, sample.waypoints))
            self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]

        self.cleanup_pose_history()

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            pose = self.interpolate_pose(target_time)
            if pose is None:
                break
            x, y = self.transform_to_robot_frame(sample.reference, pose)
            sample.waypoints.append((x, y))

    def interpolate_pose(self, target_time: float) -> Optional[PoseSample]:
        """target_time を挟む2点から線形補間した姿勢を返す。
        最近傍で済ませると pose レートの半周期分（常に未来寄り）のバイアスが乗るため補間する"""
        if len(self.pose_history) < 2:
            return None
        if target_time < self.pose_history[0].timestamp or target_time > self.pose_history[-1].timestamp:
            return None

        for i in range(len(self.pose_history) - 1):
            before = self.pose_history[i]
            after = self.pose_history[i + 1]
            if after.timestamp < target_time:
                continue

            span = after.timestamp - before.timestamp
            ratio = 0.0 if span <= 0.0 else (target_time - before.timestamp) / span

            position = before.position + (after.position - before.position) * ratio
            # ヨーは ±π をまたぐので最短回転側で補間する
            delta_yaw = (after.yaw - before.yaw + np.pi) % (2.0 * np.pi) - np.pi
            yaw = before.yaw + delta_yaw * ratio

            return PoseSample(target_time, position, yaw)

        return None

    def cleanup_pose_history(self) -> None:
        if not self.pose_history:
            return

        if self.samples:
            # 未解決の reference と、次に必要な target_time のうち最も古い時刻まで残す
            required_times = []
            for sample in self.samples:
                if sample.reference is None:
                    required_times.append(sample.timestamp)
                if len(sample.waypoints) < NUM_WAYPOINTS:
                    required_times.append(sample.target_times[len(sample.waypoints)])
            oldest_required = min(required_times) if required_times else self.pose_history[-1].timestamp
        else:
            oldest_required = self.pose_history[-1].timestamp

        # 補間には oldest_required より前の1点が要るので、2番目が古い間だけ捨てる
        while len(self.pose_history) > 2 and self.pose_history[1].timestamp < oldest_required - POSE_HISTORY_MARGIN:
            self.pose_history.popleft()

    def transform_to_robot_frame(self, reference: PoseSample, current: PoseSample) -> Tuple[float, float]:
        lat0, lon0, alt0 = pm.ecef2geodetic(reference.position[0], reference.position[1], reference.position[2])
        yaw0 = reference.yaw

        e, n, u = pm.ecef2enu(current.position[0], current.position[1], current.position[2], lat0, lon0, alt0)

        x_robot = -e * np.sin(yaw0) + n * np.cos(yaw0)
        y_robot = -e * np.cos(yaw0) - n * np.sin(yaw0)

        return x_robot, y_robot

    def save_data(self) -> None:
        if len(self.collected_data) == 0:
            self.get_logger().info('🔴No data to save')
            return

        package_root = Path(__file__).parent.parent
        data_base_dir = package_root / 'data'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        dataset_dir = data_base_dir / f'{timestamp}_dataset'
        images_dir = dataset_dir / 'images'
        path_dir = dataset_dir / 'path'

        images_dir.mkdir(parents=True, exist_ok=True)
        path_dir.mkdir(parents=True, exist_ok=True)

        for idx, (image, waypoints) in enumerate(self.collected_data, start=1):
            image_path = images_dir / f'{idx:05d}.png'
            waypoints_path = path_dir / f'{idx:05d}.csv'

            cv2.imwrite(str(image_path), image)

            with open(str(waypoints_path), 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['x', 'y'])
                for x, y in waypoints:
                    csv_writer.writerow([x, y])

        self.get_logger().info(f'🔵Saved {len(self.collected_data)} samples to {dataset_dir}')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataCollectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

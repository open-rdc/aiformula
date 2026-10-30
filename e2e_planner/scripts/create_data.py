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
from util.waypoints import (
    PoseSample,
    build_forward_arc,
    interpolate_pose,
    pose_at_distance,
    transform_to_robot_frame,
)
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

SAMPLE_INTERVAL = 0.2
NUM_WAYPOINTS = 20

# waypoint を「時間」で切るか「距離」で切るか。
#
# 'time' は WAYPOINT_INTERVAL 秒ごと = 収集時の速度で経路長が決まる。3.0 m/s で集めると
# 2.5s 先は 7.5 m にしかならず、5.5 m/s で走らせると pure_pursuit の lookahead が
# 経路末端に張り付いて linear_scale = distance/lookahead で速度が頭打ちになる。
#
# 'distance' は WAYPOINT_DISTANCE m ごと = 純粋な幾何経路になり収集速度から独立する。
# 低速で集めたバグからでも目標速度に必要な長さの経路が作れる。
# ただし走行ラインそのものは収集時の速度のものなので、限界域の挙動は別途データが要る。
WAYPOINT_MODE = 'distance'

WAYPOINT_INTERVAL = 0.125   # 'time' のとき使う[s]
WAYPOINT_DISTANCE = 0.5     # 'distance' のとき使う[m]。0.5 x 20点 = 10 m 先まで

# 'distance' で、目標距離に達しないまま放置されたサンプルを捨てるまでの時間[s]。
# 停車すると弧長が伸びず永久に完成しないため、pose_history が無限に伸びるのを防ぐ
MAX_SAMPLE_WAIT = 30.0

# ZED の camera_fps と揃える。grab() を撮影レートで回して古いフレームを掴まないようにする
TIMER_PERIOD = 1.0 / 30.0

# pose 履歴に残しておく最低限のマージン[s]。補間に必要な「target_time より前の1点」を確保する
POSE_HISTORY_MARGIN = 0.5


class Sample:
    def __init__(self, image: np.ndarray, timestamp: float):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        # 画像取得時刻ちょうどの姿勢は後から補間で解決する
        self.reference: Optional[PoseSample] = None
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]
        self.target_distances: List[float] = [WAYPOINT_DISTANCE * (i + 1) for i in range(NUM_WAYPOINTS)]


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

        horizon = (f'{WAYPOINT_DISTANCE * NUM_WAYPOINTS:.1f} m ({WAYPOINT_DISTANCE} m x {NUM_WAYPOINTS}点)'
                   if WAYPOINT_MODE == 'distance'
                   else f'{WAYPOINT_INTERVAL * NUM_WAYPOINTS:.2f} s ({WAYPOINT_INTERVAL} s x {NUM_WAYPOINTS}点)')
        self.get_logger().info(f'⚪Create data started / waypoint mode={WAYPOINT_MODE}, horizon={horizon}')

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
                sample.reference = interpolate_pose(self.pose_history, sample.timestamp)
            if sample.reference is not None:
                self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((sample.image, sample.waypoints))
            self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]
        self.drop_stale_samples()

        self.cleanup_pose_history()

    def drop_stale_samples(self) -> None:
        """距離モードで、停車などにより目標距離に届かないサンプルを捨てる"""
        if WAYPOINT_MODE != 'distance' or not self.samples or not self.pose_history:
            return

        latest = self.pose_history[-1].timestamp
        kept = [s for s in self.samples if latest - s.timestamp <= MAX_SAMPLE_WAIT]
        dropped = len(self.samples) - len(kept)
        if dropped:
            self.get_logger().warn(
                f'{dropped} 件のサンプルを破棄しました'
                f'（{MAX_SAMPLE_WAIT:.0f}s 以内に {WAYPOINT_DISTANCE * NUM_WAYPOINTS:.1f}m 進まなかった）')
        self.samples = kept

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        if WAYPOINT_MODE == 'distance':
            arc = build_forward_arc(self.pose_history, sample.reference)
            for i in range(len(sample.waypoints), NUM_WAYPOINTS):
                pose = pose_at_distance(arc, sample.target_distances[i])
                if pose is None:
                    break
                x, y = transform_to_robot_frame(sample.reference, pose)
                sample.waypoints.append((x, y))
            return

        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            pose = interpolate_pose(self.pose_history, target_time)
            if pose is None:
                break
            x, y = transform_to_robot_frame(sample.reference, pose)
            sample.waypoints.append((x, y))

    def cleanup_pose_history(self) -> None:
        if not self.pose_history:
            return

        if self.samples:
            if WAYPOINT_MODE == 'distance':
                # 目標距離に達する時刻は事前に分からないので、未完成サンプルの
                # 基準時刻そのものまで残す（そこから前方の弧長を積み直すため）
                oldest_required = min(sample.timestamp for sample in self.samples)
            else:
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

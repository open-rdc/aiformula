#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy, PointCloud2
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import csv
import copy
from pathlib import Path
from collections import deque
from typing import Optional, List, Tuple, Deque
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs_py.point_cloud2 as pc2
import pymap3d as pm
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 10
DEFAULT_COMMAND = 1
COMMAND_LABELS = {
    0: 'roadside',
    1: 'straight',
    2: 'left',
    3: 'right',
}

class Sample:
    def __init__(self, image: np.ndarray, timestamp: float, reference_pose: PoseWithCovarianceStamped, command: int, point_cloud: Optional[np.ndarray] = None):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        self.reference_pose: PoseWithCovarianceStamped = reference_pose
        self.command: int = command
        self.point_cloud: Optional[np.ndarray] = point_cloud
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]

class DataCollectionNode(Node):
    def __init__(self) -> None:
        super().__init__('data_collection_node')

        self.declare_parameter('sdk_flag', True)
        self.declare_parameter('toggle_button_index', 0)
        self.declare_parameter('left_button_index', 4)
        self.declare_parameter('right_button_index', 5)
        self.sdk_flag_ = self.get_parameter('sdk_flag').value
        self.toggle_button_index = int(self.get_parameter('toggle_button_index').value)
        self.left_button_index = int(self.get_parameter('left_button_index').value)
        self.right_button_index = int(self.get_parameter('right_button_index').value)

        self.bridge: CvBridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.latest_pose: Optional[PoseWithCovarianceStamped] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_command: int = DEFAULT_COMMAND

        self.samples: List[Sample] = []
        self.pose_history: Deque[Tuple[float, PoseWithCovarianceStamped]] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]], int, Optional[np.ndarray]]] = []
        self.last_sample_time: Optional[float] = None

        self.is_paused: bool = True
        self.prev_toggle_button_state: int = 0

        self.zed_camera: Optional[sl.Camera] = None
        self.zed_image: Optional[sl.Mat] = None
        self.zed_point_cloud: Optional[sl.Mat] = None
        self.zed_pose: Optional[sl.Pose] = None
        self.zed_runtime_params: Optional[sl.RuntimeParameters] = None

        if self.sdk_flag_:
            if not ZED_SDK_AVAILABLE:
                self.get_logger().error('ZED SDK not available. Install pyzed package.')
                raise RuntimeError('ZED SDK not available')
            self._initialize_zed_camera()
        else:
            self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/zed/zed_node/pointcloud', self.pointcloud_callback, qos_profile_sensor_data)

        # VectorNav pose subscription (used in both SDK and ROS modes)
        self.create_subscription(PoseWithCovarianceStamped, '/vectornav/pose', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/command', self.command_callback, 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Create data ready: A toggles collection, L1 labels left, R1 labels right'
        )

    def _initialize_zed_camera(self) -> None:
        self.zed_camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER

        err = self.zed_camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open ZED camera: {err}')
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        tracking_params = sl.PositionalTrackingParameters()
        err = self.zed_camera.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to enable positional tracking: {err}')
            raise RuntimeError(f'Failed to enable positional tracking: {err}')

        self.zed_image = sl.Mat()
        self.zed_point_cloud = sl.Mat()
        self.zed_pose = sl.Pose()
        self.zed_runtime_params = sl.RuntimeParameters()
        self.get_logger().info('ZED camera initialized with tracking and depth sensing')

    def _capture_data_from_zed(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self.zed_camera.grab(self.zed_runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None, None

        self.zed_camera.retrieve_image(self.zed_image, sl.VIEW.LEFT)
        image = self.zed_image.get_data()
        height, width = image.shape[:2]
        resized_image = cv2.resize(image, (width // 2, height // 2))

        self.zed_camera.retrieve_measure(self.zed_point_cloud, sl.MEASURE.XYZRGBA)
        point_cloud = self.zed_point_cloud.get_data()

        return resized_image, point_cloud

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg

    def command_callback(self, msg: UInt8) -> None:
        self.latest_command = int(msg.data)

    def _convert_pointcloud2_to_array(self, pointcloud_msg: PointCloud2) -> np.ndarray:
        points_list = [[point[0], point[1], point[2], point[3]]
                       for point in pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))]
        return np.array(points_list, dtype=np.float32)

    def _button_pressed(self, msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

    def _set_command_from_joy(self, msg: Joy) -> None:
        previous_command = self.latest_command

        if self._button_pressed(msg, self.left_button_index) and not self._button_pressed(msg, self.right_button_index):
            self.latest_command = 2
        elif self._button_pressed(msg, self.right_button_index) and not self._button_pressed(msg, self.left_button_index):
            self.latest_command = 3
        else:
            self.latest_command = DEFAULT_COMMAND

        if self.latest_command != previous_command:
            self.get_logger().info(f'Command label: {COMMAND_LABELS[self.latest_command]} ({self.latest_command})')

    def joy_callback(self, msg: Joy) -> None:
        self._set_command_from_joy(msg)

        current_toggle_button_state = 1 if self._button_pressed(msg, self.toggle_button_index) else 0
        if current_toggle_button_state == 1 and self.prev_toggle_button_state == 0:
            self.is_paused = not self.is_paused
            if self.is_paused:
                self.get_logger().info('Data collection stopped')
            else:
                self.pose_history.clear()
                self.samples.clear()
                self.last_sample_time = None
                self.get_logger().info('Data collection started')
        self.prev_toggle_button_state = current_toggle_button_state

    def timer_callback(self) -> None:
        if self.is_paused:
            return

        current_time = time.time()

        if self.latest_pose is not None:
            self.pose_history.append((current_time, copy.deepcopy(self.latest_pose)))

        if self.sdk_flag_:
            image, point_cloud = self._capture_data_from_zed()
            if image is None or self.latest_pose is None:
                return

            if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                sample = Sample(image, current_time, copy.deepcopy(self.latest_pose), self.latest_command, point_cloud)
                self.samples.append(sample)
                self.last_sample_time = current_time
        else:
            if self.latest_image is not None and self.latest_pose is not None:
                if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                    cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgra8')
                    point_cloud = self._convert_pointcloud2_to_array(self.latest_pointcloud) if self.latest_pointcloud is not None else None
                    sample = Sample(cv_image, current_time, copy.deepcopy(self.latest_pose), self.latest_command, point_cloud)
                    self.samples.append(sample)
                    self.last_sample_time = current_time

        for sample in self.samples:
            self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((sample.image, sample.waypoints, sample.command, sample.point_cloud))
            self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]

        self.cleanup_pose_history()

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            pose = self.find_closest_pose(target_time)
            if pose is not None:
                x, y = self.transform_to_robot_frame(sample.reference_pose, pose)
                sample.waypoints.append((x, y))
            else:
                break

    def find_closest_pose(self, target_time: float) -> Optional[PoseWithCovarianceStamped]:
        for t, pose in self.pose_history:
            if t >= target_time:
                return pose
        return None

    def cleanup_pose_history(self) -> None:
        if not self.samples or not self.pose_history:
            return
        incomplete_samples = [s for s in self.samples if len(s.waypoints) < NUM_WAYPOINTS]
        if not incomplete_samples:
            return
        min_target_time = min(sample.target_times[len(sample.waypoints)] for sample in incomplete_samples)
        while self.pose_history and self.pose_history[0][0] < min_target_time:
            self.pose_history.popleft()

    def transform_to_robot_frame(self, reference_pose: PoseWithCovarianceStamped, current_pose: PoseWithCovarianceStamped) -> Tuple[float, float]:
        x0_ecef = reference_pose.pose.pose.position.x
        y0_ecef = reference_pose.pose.pose.position.y
        z0_ecef = reference_pose.pose.pose.position.z
        lat0, lon0, alt0 = pm.ecef2geodetic(x0_ecef, y0_ecef, z0_ecef)

        q0 = reference_pose.pose.pose.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])

        xi_ecef = current_pose.pose.pose.position.x
        yi_ecef = current_pose.pose.pose.position.y
        zi_ecef = current_pose.pose.pose.position.z
        e, n, u = pm.ecef2enu(xi_ecef, yi_ecef, zi_ecef, lat0, lon0, alt0)

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
        commands_dir = dataset_dir / 'commands'
        pointclouds_dir = dataset_dir / 'pointclouds'

        images_dir.mkdir(parents=True, exist_ok=True)
        path_dir.mkdir(parents=True, exist_ok=True)
        commands_dir.mkdir(parents=True, exist_ok=True)
        pointclouds_dir.mkdir(parents=True, exist_ok=True)

        for idx, (image, waypoints, command, point_cloud) in enumerate(self.collected_data, start=1):
            image_path = images_dir / f'{idx:05d}.png'
            waypoints_path = path_dir / f'{idx:05d}.csv'
            command_path = commands_dir / f'{idx:05d}.csv'
            pointcloud_path = pointclouds_dir / f'{idx:05d}.npy'

            cv2.imwrite(str(image_path), image)

            with open(str(waypoints_path), 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['x', 'y'])
                for x, y in waypoints:
                    csv_writer.writerow([x, y])

            with open(str(command_path), 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([command])

            if point_cloud is not None:
                np.save(str(pointcloud_path), point_cloud)

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

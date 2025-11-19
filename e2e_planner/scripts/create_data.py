#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy, PointCloud2
from nav_msgs.msg import Odometry
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
import sensor_msgs_py.point_cloud2 as pc2
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 10

class Sample:
    def __init__(self, image: np.ndarray, timestamp: float, reference_odom: Odometry, point_cloud: Optional[np.ndarray] = None):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        self.reference_odom: Odometry = reference_odom
        self.point_cloud: Optional[np.ndarray] = point_cloud
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]

class DataCollectionNode(Node):
    def __init__(self) -> None:
        super().__init__('data_collection_node')

        self.declare_parameter('sdk_flag', True)
        self.sdk_flag_ = self.get_parameter('sdk_flag').value

        self.bridge: CvBridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_pointcloud: Optional[PointCloud2] = None

        self.samples: List[Sample] = []
        self.odom_history: Deque[Tuple[float, Odometry]] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]], Optional[np.ndarray]]] = []
        self.last_sample_time: Optional[float] = None

        self.is_paused: bool = False
        self.prev_button_state: int = 0

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
            self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/zed/zed_node/pointcloud', self.pointcloud_callback, qos_profile_sensor_data)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('⚪Create data started')

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

    def _capture_data_from_zed(self) -> Tuple[Optional[np.ndarray], Optional[Odometry], Optional[np.ndarray]]:
        if self.zed_camera.grab(self.zed_runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None, None, None

        self.zed_camera.retrieve_image(self.zed_image, sl.VIEW.LEFT)
        image = self.zed_image.get_data()
        height, width = image.shape[:2]
        resized_image = cv2.resize(image, (width // 2, height // 2))

        self.zed_camera.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
        odom = self._convert_pose_to_odometry(self.zed_pose)

        self.zed_camera.retrieve_measure(self.zed_point_cloud, sl.MEASURE.XYZRGBA)
        point_cloud = self.zed_point_cloud.get_data()

        return resized_image, odom, point_cloud

    def _convert_pose_to_odometry(self, pose: sl.Pose) -> Odometry:
        odom = Odometry()
        translation = pose.get_translation()
        orientation = pose.get_orientation()

        odom.pose.pose.position.x = translation.get()[0]
        odom.pose.pose.position.y = translation.get()[1]
        odom.pose.pose.position.z = translation.get()[2]

        odom.pose.pose.orientation.x = orientation.get()[0]
        odom.pose.pose.orientation.y = orientation.get()[1]
        odom.pose.pose.orientation.z = orientation.get()[2]
        odom.pose.pose.orientation.w = orientation.get()[3]

        return odom

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg

    def _convert_pointcloud2_to_array(self, pointcloud_msg: PointCloud2) -> np.ndarray:
        points_list = [[point[0], point[1], point[2], point[3]]
                       for point in pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))]
        return np.array(points_list, dtype=np.float32)

    def joy_callback(self, msg: Joy) -> None:
        if len(msg.buttons) > 2:
            current_button_state = msg.buttons[2]
            if current_button_state == 1 and self.prev_button_state == 0:
                self.is_paused = not self.is_paused
                if self.is_paused:
                    self.get_logger().info('⏸️ Data collection paused')
                else:
                    self.odom_history.clear()
                    self.samples.clear()
                    self.last_sample_time = None
                    self.get_logger().info('▶️ Data collection resumed')
            self.prev_button_state = current_button_state

    def timer_callback(self) -> None:
        if self.is_paused:
            return

        current_time = time.time()

        if self.sdk_flag_:
            image, odom, point_cloud = self._capture_data_from_zed()
            if image is None or odom is None:
                return

            self.odom_history.append((current_time, odom))

            if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                sample = Sample(image, current_time, odom, point_cloud)
                self.samples.append(sample)
                self.last_sample_time = current_time
        else:
            if self.latest_odom is not None:
                self.odom_history.append((current_time, self.latest_odom))

            if self.latest_image is not None and self.latest_odom is not None:
                if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                    cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgra8')
                    point_cloud = self._convert_pointcloud2_to_array(self.latest_pointcloud) if self.latest_pointcloud is not None else None
                    sample = Sample(cv_image, current_time, self.latest_odom, point_cloud)
                    self.samples.append(sample)
                    self.last_sample_time = current_time

        for sample in self.samples:
            self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((sample.image, sample.waypoints, sample.point_cloud))
            self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]

        self.cleanup_odom_history()

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            odom = self.find_closest_odom(target_time)
            if odom is not None:
                x, y = self.transform_to_robot_frame(sample.reference_odom, odom)
                sample.waypoints.append((x, y))
            else:
                break

    def find_closest_odom(self, target_time: float) -> Optional[Odometry]:
        for t, odom in self.odom_history:
            if t >= target_time:
                return odom
        return None

    def cleanup_odom_history(self) -> None:
        if not self.samples or not self.odom_history:
            return
        incomplete_samples = [s for s in self.samples if len(s.waypoints) < NUM_WAYPOINTS]
        if not incomplete_samples:
            return
        min_target_time = min(sample.target_times[len(sample.waypoints)] for sample in incomplete_samples)
        while self.odom_history and self.odom_history[0][0] < min_target_time:
            self.odom_history.popleft()

    def transform_to_robot_frame(self, reference_odom: Odometry, odom: Odometry) -> Tuple[float, float]:
        x0 = reference_odom.pose.pose.position.x
        y0 = reference_odom.pose.pose.position.y
        q0 = reference_odom.pose.pose.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])

        xi = odom.pose.pose.position.x
        yi = odom.pose.pose.position.y

        dx = xi - x0
        dy = yi - y0

        x_robot = dx * np.cos(-yaw0) - dy * np.sin(-yaw0)
        y_robot = dx * np.sin(-yaw0) + dy * np.cos(-yaw0)

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
        waypoints_dir = dataset_dir / 'waypoints'
        pointclouds_dir = dataset_dir / 'pointclouds'

        images_dir.mkdir(parents=True, exist_ok=True)
        waypoints_dir.mkdir(parents=True, exist_ok=True)
        pointclouds_dir.mkdir(parents=True, exist_ok=True)

        for idx, (image, waypoints, point_cloud) in enumerate(self.collected_data, start=1):
            image_path = images_dir / f'{idx:05d}.png'
            waypoints_path = waypoints_dir / f'{idx:05d}.csv'
            pointcloud_path = pointclouds_dir / f'{idx:05d}.npy'

            cv2.imwrite(str(image_path), image)

            with open(str(waypoints_path), 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['x', 'y'])
                for x, y in waypoints:
                    csv_writer.writerow([x, y])

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

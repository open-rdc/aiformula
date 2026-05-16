#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Joy, NavSatFix, NavSatStatus, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import csv
import copy
import sys
import re
import subprocess
import threading
from pathlib import Path
from collections import deque
from typing import Any, Optional, List, Tuple, Deque, Union
from rclpy.qos import qos_profile_sensor_data

sl = None


def _load_zed_sdk():
    global sl
    if sl is not None:
        return sl

    try:
        import pyzed.sl as zed_sl
    except ImportError as exc:
        raise RuntimeError('ZED SDK not available. Install pyzed package.') from exc

    sl = zed_sl
    return sl

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 6
DEFAULT_COMMAND = 1
COMMAND_LABELS = {
    0: 'roadside',
    1: 'straight',
    2: 'left',
    3: 'right',
}

@dataclass
class GnssPose:
    latitude: float
    longitude: float
    altitude: float
    yaw: float

PoseSample = Union[PoseWithCovarianceStamped, GnssPose, Tuple[float, float, float]]

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))

def default_save_dir() -> Path:
    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        source_data_dir = parent / 'src' / 'aiformula' / 'e2e_planner' / 'data'
        if source_data_dir.is_dir():
            return source_data_dir
        package_data_dir = parent / 'data'
        if parent.name == 'e2e_planner' and package_data_dir.is_dir():
            return package_data_dir
    return current_file.parent.parent / 'data'

class Sample:
    def __init__(
        self,
        image: np.ndarray,
        timestamp: float,
        reference_pose: PoseSample,
        reference_pose_source: str,
        command: int,
        point_cloud: Optional[np.ndarray] = None,
    ):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        self.reference_pose: PoseSample = reference_pose
        self.reference_pose_source: str = reference_pose_source
        self.command: int = command
        self.point_cloud: Optional[np.ndarray] = point_cloud
        self.waypoints: List[Tuple[float, float]] = []
        self.debug_waypoints: List[dict] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]

class DataCollectionNode(Node):
    def __init__(self, simulator_mode: bool = False) -> None:
        super().__init__('data_collection_node')

        self.declare_parameter('simulator_mode', simulator_mode)
        self.declare_parameter('sdk_flag', True)
        self.declare_parameter('save_dir', str(default_save_dir()))
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('pointcloud_topic', '/zed/zed_node/pointcloud')
        self.declare_parameter('real_pose_source', 'gnss')
        self.declare_parameter('pose_topic', '/vectornav/pose')
        self.declare_parameter('gnss_topic', '/vectornav/gnss')
        self.declare_parameter('imu_topic', '/vectornav/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ground_truth_pose_topic', '/world/car_world/pose/info')
        self.declare_parameter('ground_truth_frame', 'ai_car1')
        self.declare_parameter('use_ground_truth_pose', True)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('save_interval_sec', SAMPLE_INTERVAL)
        self.declare_parameter('toggle_button_index', 2)
        self.declare_parameter('left_button_index', 6)
        self.declare_parameter('right_button_index', 7)

        self.simulator_mode = bool(self.get_parameter('simulator_mode').value)
        self.sdk_flag_ = bool(self.get_parameter('sdk_flag').value) and not self.simulator_mode
        self.save_base_dir = Path(self.get_parameter('save_dir').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.real_pose_source = str(self.get_parameter('real_pose_source').value).lower()
        self.pose_topic = self.get_parameter('pose_topic').value
        self.gnss_topic = self.get_parameter('gnss_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.ground_truth_frame = self.get_parameter('ground_truth_frame').value
        self.use_ground_truth_pose = bool(self.get_parameter('use_ground_truth_pose').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.save_interval = float(self.get_parameter('save_interval_sec').value)
        self.toggle_button_index = int(self.get_parameter('toggle_button_index').value)
        self.left_button_index = int(self.get_parameter('left_button_index').value)
        self.right_button_index = int(self.get_parameter('right_button_index').value)

        self.bridge: CvBridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.latest_pose: Optional[PoseSample] = None
        self.latest_pose_source: str = 'none'
        self.latest_gnss: Optional[NavSatFix] = None
        self.latest_imu_yaw: Optional[float] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_command: int = DEFAULT_COMMAND
        self.ground_truth_process: Optional[subprocess.Popen] = None
        self.ground_truth_thread: Optional[threading.Thread] = None
        self._stop_ground_truth_reader = threading.Event()

        self.samples: List[Sample] = []
        self.pose_history: Deque[Tuple[float, PoseSample, str]] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]], int, Optional[np.ndarray], List[dict]]] = []
        self.last_sample_time: Optional[float] = None

        self.is_paused: bool = True
        self.prev_toggle_button_state: int = 0

        self.zed_camera: Optional[Any] = None
        self.zed_image: Optional[Any] = None
        self.zed_point_cloud: Optional[Any] = None
        self.zed_pose: Optional[Any] = None
        self.zed_runtime_params: Optional[Any] = None

        if self.sdk_flag_:
            _load_zed_sdk()
            self._initialize_zed_camera()
        else:
            image_topic = self.image_topic if self.simulator_mode else '/zed/zed_node/rgb/image_rect_color'
            self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
            if not self.simulator_mode:
                self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, qos_profile_sensor_data)

        if self.simulator_mode:
            if self.use_ground_truth_pose:
                self._start_ground_truth_pose_reader()
            self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        else:
            if self.real_pose_source == 'gnss':
                self.create_subscription(NavSatFix, self.gnss_topic, self.gnss_callback, qos_profile_sensor_data)
                self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
                self.get_logger().info(
                    f'Using GNSS pose from {self.gnss_topic} with yaw from {self.imu_topic}'
                )
            elif self.real_pose_source == 'pose':
                self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, qos_profile_sensor_data)
                self.get_logger().info(f'Using pose from {self.pose_topic}')
            else:
                raise ValueError('real_pose_source must be "gnss" or "pose"')
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Create data ready: buttons[2] toggles collection, buttons[6] labels left, buttons[7] labels right'
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
        self.latest_pose_source = 'pose'

    def gnss_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if not all(np.isfinite([msg.latitude, msg.longitude, msg.altitude])):
            return

        self.latest_gnss = msg
        self._update_latest_gnss_pose()

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self.latest_imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._update_latest_gnss_pose()

    def _update_latest_gnss_pose(self) -> None:
        if self.latest_gnss is None or self.latest_imu_yaw is None:
            return
        self.latest_pose = GnssPose(
            latitude=float(self.latest_gnss.latitude),
            longitude=float(self.latest_gnss.longitude),
            altitude=float(self.latest_gnss.altitude),
            yaw=float(self.latest_imu_yaw),
        )
        self.latest_pose_source = 'gnss'

    def odom_callback(self, msg: Odometry) -> None:
        if self.simulator_mode and self.use_ground_truth_pose and self.latest_pose is not None:
            return
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.latest_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.latest_pose_source = 'odom'

    def _start_ground_truth_pose_reader(self) -> None:
        command = ['ign', 'topic', '-e', '-t', self.ground_truth_pose_topic]
        try:
            self.ground_truth_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self.get_logger().error(f'Failed to start Gazebo ground truth reader: {exc}')
            return

        self.ground_truth_thread = threading.Thread(
            target=self._read_ground_truth_pose_loop,
            daemon=True,
        )
        self.ground_truth_thread.start()
        self.get_logger().info(
            f'Using Gazebo ground truth pose from {self.ground_truth_pose_topic}, frame "{self.ground_truth_frame}"'
        )

    def _read_ground_truth_pose_loop(self) -> None:
        if self.ground_truth_process is None or self.ground_truth_process.stdout is None:
            return

        in_pose = False
        brace_depth = 0
        pose_lines: List[str] = []

        for line in self.ground_truth_process.stdout:
            if self._stop_ground_truth_reader.is_set():
                break

            stripped = line.strip()
            if not in_pose and stripped == 'pose {':
                in_pose = True
                brace_depth = 1
                pose_lines = [line]
                continue

            if not in_pose:
                continue

            pose_lines.append(line)
            brace_depth += line.count('{') - line.count('}')
            if brace_depth == 0:
                self._update_ground_truth_pose_from_block(''.join(pose_lines))
                in_pose = False
                pose_lines = []

    def _update_ground_truth_pose_from_block(self, block: str) -> None:
        name_match = re.search(r'name:\s*"([^"]+)"', block)
        if name_match is None or not self._is_ground_truth_frame(name_match.group(1)):
            return

        position_block = self._extract_named_block(block, 'position')
        orientation_block = self._extract_named_block(block, 'orientation')
        x = self._extract_float_field(position_block, 'x', 0.0)
        y = self._extract_float_field(position_block, 'y', 0.0)
        qx = self._extract_float_field(orientation_block, 'x', 0.0)
        qy = self._extract_float_field(orientation_block, 'y', 0.0)
        qz = self._extract_float_field(orientation_block, 'z', 0.0)
        qw = self._extract_float_field(orientation_block, 'w', 1.0)
        yaw = yaw_from_quaternion(qx, qy, qz, qw)
        frame_name = name_match.group(1)
        self.latest_pose = (x, y, yaw)
        self.latest_pose_source = f'ground_truth:{frame_name}'

    def _extract_named_block(self, text: str, name: str) -> str:
        match = re.search(rf'{name}\s*\{{(.*?)\n\s*\}}', text, re.DOTALL)
        return match.group(1) if match else ''

    def _extract_float_field(self, text: str, name: str, default: float) -> float:
        match = re.search(rf'\b{name}:\s*([-+0-9.eE]+)', text)
        return float(match.group(1)) if match else default

    def _is_ground_truth_frame(self, child_frame: str) -> bool:
        frame = self.ground_truth_frame.strip('/')
        candidates = {
            frame,
            f'{frame}/chassis',
            f'{frame}::chassis',
        }
        return (
            child_frame.strip('/') in candidates
            or child_frame.endswith(f'/{frame}')
            or child_frame.endswith(f'::{frame}')
            or child_frame.endswith(f'/{frame}/chassis')
            or child_frame.endswith(f'::{frame}::chassis')
        )

    def destroy_node(self) -> bool:
        self._stop_ground_truth_reader.set()
        if self.ground_truth_process is not None and self.ground_truth_process.poll() is None:
            self.ground_truth_process.terminate()
        return super().destroy_node()

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg

    def _convert_pointcloud2_to_array(self, pointcloud_msg: PointCloud2) -> np.ndarray:
        import sensor_msgs_py.point_cloud2 as pc2
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
            self.pose_history.append((current_time, copy.deepcopy(self.latest_pose), self.latest_pose_source))

        if self.sdk_flag_:
            image, point_cloud = self._capture_data_from_zed()
            if image is None or self.latest_pose is None:
                return

            if self.last_sample_time is None or current_time - self.last_sample_time >= self.save_interval:
                sample = Sample(
                    image,
                    current_time,
                    copy.deepcopy(self.latest_pose),
                    self.latest_pose_source,
                    self.latest_command,
                    point_cloud,
                )
                self.samples.append(sample)
                self.last_sample_time = current_time
        else:
            if self.latest_image is not None and self.latest_pose is not None:
                if self.last_sample_time is None or current_time - self.last_sample_time >= self.save_interval:
                    encoding = 'bgr8' if self.simulator_mode else 'bgra8'
                    cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding=encoding)
                    point_cloud = None
                    if not self.simulator_mode and self.latest_pointcloud is not None:
                        point_cloud = self._convert_pointcloud2_to_array(self.latest_pointcloud)
                    sample = Sample(
                        cv_image,
                        current_time,
                        copy.deepcopy(self.latest_pose),
                        self.latest_pose_source,
                        self.latest_command,
                        point_cloud,
                    )
                    self.samples.append(sample)
                    self.last_sample_time = current_time

        for sample in self.samples:
            self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((
                sample.image,
                sample.waypoints,
                sample.command,
                sample.point_cloud,
                sample.debug_waypoints,
            ))
            self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]

        self.cleanup_pose_history()

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            pose_record = self.find_closest_pose(target_time)
            if pose_record is not None:
                pose, pose_source = pose_record
                x, y, debug_row = self.transform_to_robot_frame(sample.reference_pose, pose)
                debug_row.update({
                    'waypoint_index': i,
                    'sample_time': sample.timestamp,
                    'target_time': target_time,
                    'reference_pose_source': sample.reference_pose_source,
                    'target_pose_source': pose_source,
                })
                sample.waypoints.append((x, y))
                sample.debug_waypoints.append(debug_row)
            else:
                break

    def find_closest_pose(self, target_time: float) -> Optional[Tuple[PoseSample, str]]:
        for t, pose, pose_source in self.pose_history:
            if t >= target_time:
                return pose, pose_source
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

    def transform_to_robot_frame(self, reference_pose: PoseSample, current_pose: PoseSample) -> Tuple[float, float, dict]:
        if self.simulator_mode:
            x0, y0, yaw0 = reference_pose
            x, y, _ = current_pose
            dx = x - x0
            dy = y - y0
            x_robot = np.cos(yaw0) * dx + np.sin(yaw0) * dy
            y_robot = -np.sin(yaw0) * dx + np.cos(yaw0) * dy
            debug_row = {
                'ref_x': x0,
                'ref_y': y0,
                'ref_yaw': yaw0,
                'cur_x': x,
                'cur_y': y,
                'cur_yaw': current_pose[2],
                'dx_world': dx,
                'dy_world': dy,
                'x_robot': x_robot,
                'y_robot': y_robot,
            }
            return x_robot, y_robot, debug_row

        if isinstance(reference_pose, GnssPose) and isinstance(current_pose, GnssPose):
            import pymap3d as pm

            e, n, _ = pm.geodetic2enu(
                current_pose.latitude,
                current_pose.longitude,
                current_pose.altitude,
                reference_pose.latitude,
                reference_pose.longitude,
                reference_pose.altitude,
            )
            yaw0 = reference_pose.yaw
            x_robot = -e * np.sin(yaw0) + n * np.cos(yaw0)
            y_robot = -e * np.cos(yaw0) - n * np.sin(yaw0)

            debug_row = {
                'ref_x': reference_pose.latitude,
                'ref_y': reference_pose.longitude,
                'ref_yaw': yaw0,
                'cur_x': current_pose.latitude,
                'cur_y': current_pose.longitude,
                'cur_yaw': current_pose.yaw,
                'dx_world': e,
                'dy_world': n,
                'x_robot': x_robot,
                'y_robot': y_robot,
            }
            return x_robot, y_robot, debug_row

        x0_ecef = reference_pose.pose.pose.position.x
        y0_ecef = reference_pose.pose.pose.position.y
        z0_ecef = reference_pose.pose.pose.position.z
        import pymap3d as pm
        lat0, lon0, alt0 = pm.ecef2geodetic(x0_ecef, y0_ecef, z0_ecef)

        q0 = reference_pose.pose.pose.orientation
        yaw0 = yaw_from_quaternion(q0.x, q0.y, q0.z, q0.w)

        xi_ecef = current_pose.pose.pose.position.x
        yi_ecef = current_pose.pose.pose.position.y
        zi_ecef = current_pose.pose.pose.position.z
        e, n, u = pm.ecef2enu(xi_ecef, yi_ecef, zi_ecef, lat0, lon0, alt0)

        x_robot = -e * np.sin(yaw0) + n * np.cos(yaw0)
        y_robot = -e * np.cos(yaw0) - n * np.sin(yaw0)

        debug_row = {
            'ref_x': x0_ecef,
            'ref_y': y0_ecef,
            'ref_yaw': yaw0,
            'cur_x': xi_ecef,
            'cur_y': yi_ecef,
            'cur_yaw': yaw_from_quaternion(
                current_pose.pose.pose.orientation.x,
                current_pose.pose.pose.orientation.y,
                current_pose.pose.pose.orientation.z,
                current_pose.pose.pose.orientation.w,
            ),
            'dx_world': e,
            'dy_world': n,
            'x_robot': x_robot,
            'y_robot': y_robot,
        }
        return x_robot, y_robot, debug_row

    def save_data(self) -> None:
        if len(self.collected_data) == 0:
            self.get_logger().info('🔴No data to save')
            return

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        dataset_dir = self.save_base_dir / f'{timestamp}_dataset'
        images_dir = dataset_dir / 'images'
        path_dir = dataset_dir / 'path'
        commands_dir = dataset_dir / 'commands'
        debug_dir = dataset_dir / 'debug_waypoints'
        pointclouds_dir = dataset_dir / 'pointclouds'

        images_dir.mkdir(parents=True, exist_ok=True)
        path_dir.mkdir(parents=True, exist_ok=True)
        commands_dir.mkdir(parents=True, exist_ok=True)
        debug_dir.mkdir(parents=True, exist_ok=True)
        pointclouds_dir.mkdir(parents=True, exist_ok=True)

        for idx, (image, waypoints, command, point_cloud, debug_waypoints) in enumerate(self.collected_data, start=1):
            image_path = images_dir / f'{idx:05d}.png'
            waypoints_path = path_dir / f'{idx:05d}.csv'
            command_path = commands_dir / f'{idx:05d}.csv'
            debug_path = debug_dir / f'{idx:05d}.csv'
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

            if debug_waypoints:
                with open(str(debug_path), 'w', newline='') as csvfile:
                    fieldnames = list(debug_waypoints[0].keys())
                    csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    csv_writer.writeheader()
                    csv_writer.writerows(debug_waypoints)

            if point_cloud is not None:
                np.save(str(pointcloud_path), point_cloud)

        self.get_logger().info(f'🔵Saved {len(self.collected_data)} samples to {dataset_dir}')

def _parse_args(args: Optional[List[str]]) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--sim', action='store_true', help='Use simulator image and odometry topics')
    source_args = sys.argv[1:] if args is None else list(args)
    return parser.parse_known_args(source_args)

def main(args=None) -> None:
    cli_args, ros_args = _parse_args(args)
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = DataCollectionNode(simulator_mode=cli_args.sim)

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

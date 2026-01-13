from typing import Optional

import cv2
import numpy as np
import pyzed.sl as sl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class ZedSdk:
    def __init__(self, logger) -> None:
        self._logger = logger
        self._camera = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER

        err = self._camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        tracking_params = sl.PositionalTrackingParameters()
        err = self._camera.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to enable positional tracking: {err}')

        self._image = sl.Mat()
        self._pointcloud = sl.Mat()
        self._pose = sl.Pose()
        self._runtime = sl.RuntimeParameters()

        self._logger.info('ZED camera initialized successfully')

    def grab(self) -> bool:
        return self._camera.grab(self._runtime) == sl.ERROR_CODE.SUCCESS

    def get_image(self) -> Optional[np.ndarray]:
        self._camera.retrieve_image(self._image, sl.VIEW.LEFT)
        image = self._image.get_data()
        if image is None:
            return None
        height, width = image.shape[:2]
        return cv2.resize(image, (width // 2, height // 2))

    def get_odom(self, header) -> Optional[Odometry]:
        tracking_state = self._camera.get_position(self._pose, sl.REFERENCE_FRAME.WORLD)
        if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return None

        translation = self._pose.get_translation().get()
        orientation = self._pose.get_orientation().get()

        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(translation[0])
        odom_msg.pose.pose.position.y = float(translation[1])
        odom_msg.pose.pose.position.z = float(translation[2])
        odom_msg.pose.pose.orientation.x = float(orientation[0])
        odom_msg.pose.pose.orientation.y = float(orientation[1])
        odom_msg.pose.pose.orientation.z = float(orientation[2])
        odom_msg.pose.pose.orientation.w = float(orientation[3])
        return odom_msg

    def get_pointcloud(self, header) -> Optional[PointCloud2]:
        self._camera.retrieve_measure(self._pointcloud, sl.MEASURE.XYZRGBA)
        pointcloud = self._pointcloud.get_data()
        if pointcloud is None:
            return None

        points = pointcloud.reshape(-1, 4).astype(np.float32, copy=False)
        finite = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1]) & np.isfinite(points[:, 2])
        points = points[finite]
        if points.size == 0:
            return None

        y = points[:, 1]
        in_range = (y >= -0.3) & (y <= 0.5)
        points = points[in_range]
        if points.size == 0:
            return None

        if points.shape[0] > 1024:
            indices = np.random.choice(points.shape[0], 1024, replace=False)
            points = points[indices]

        rgba = points[:, 3].view(np.uint32)
        cloud = np.zeros(points.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgba', np.uint32),
        ])
        cloud['x'] = points[:, 0]
        cloud['y'] = points[:, 1]
        cloud['z'] = points[:, 2]
        cloud['rgba'] = rgba

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]

        pointcloud_msg = point_cloud2.create_cloud(header, fields, cloud)
        pointcloud_msg.header.frame_id = 'zed_camera_center'
        return pointcloud_msg

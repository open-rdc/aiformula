from typing import Optional

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2

import cv2
import numpy as np


class ZedRosMsg:
    def __init__(self, node: Node) -> None:
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.latest_image: Optional[Image] = None

        self.sub_pointcloud = node.create_subscription(PointCloud2, '/zed/zed_node/sim/pointcloud', self.pointcloud_callback, qos_profile_sensor_data)
        self.sub_image = node.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile_sensor_data)

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self.latest_pointcloud = msg

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg
        

class ZedSdk:
    def __init__(self, node: Node, sim_flag: bool) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._ros_msg = ZedRosMsg(node)
        self._bridge = CvBridge()
        self.sim_flag = sim_flag
        self._sl = None
        self._camera = None
        self._image = None
        self._pointcloud = None
        self._runtime = None
        if sim_flag:
            self._logger.info('Using ZED camera in simulation mode')
            return

        import pyzed.sl as sl
        self._sl = sl
        self._camera = sl.Camera()
        
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER

        err = self._camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        self._image = sl.Mat()
        self._pointcloud = sl.Mat()
        self._runtime = sl.RuntimeParameters()

        self._logger.info('ZED camera initialized successfully')

    def grab(self) -> bool:
        if self.sim_flag:
            return True
        return self._camera.grab(self._runtime) == self._sl.ERROR_CODE.SUCCESS

    def get_image(self) -> Optional[np.ndarray]:
        image: Optional[np.ndarray] = None
        if self.sim_flag:
            msg = self._ros_msg.latest_image
            if msg is None:
                return None
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
        else:
            self._camera.retrieve_image(self._image, self._sl.VIEW.LEFT)
            image = self._image.get_data()
            height, width = image.shape[:2]
            cv2.resize(image, (width // 2, height // 2))
        if image is None:
            return None
        return image

    def get_pointcloud(self, header) -> Optional[PointCloud2]:
        if self.sim_flag:
            msg = self._ros_msg.latest_pointcloud
            header = msg.header
            points_list = [point for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)]
            pointcloud = np.array(points_list, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgba', np.float32)])
        else:
            self._camera.retrieve_measure(self._pointcloud, self._sl.MEASURE.XYZRGBA)
            pointcloud = self._pointcloud.get_data()
        filtered_pcl = self.filtered_pointcloud(header, pointcloud)
        return filtered_pcl


    def filtered_pointcloud(self, header, pointcloud: np.ndarray) -> Optional[PointCloud2]:
        points = pointcloud.reshape(-1, pointcloud.shape[-1]).astype(np.float32, copy=False)
        non_nan = (~np.isnan(points[:, 0])) & (~np.isnan(points[:, 1])) & (~np.isnan(points[:, 2]))
        finite = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1]) & np.isfinite(points[:, 2])
        valid = finite & non_nan
        points = points[valid]

        y = points[:, 1]
        x = points[:, 0]
        in_range = (y >= -0.0) & (y <= 0.2) & (x <= 5.0) & (x >= -5.0)
        points = points[in_range]
        
        if points.size == 0:
            return None
        min_idx = np.argmin(points[:, 2])
        min_x = points[min_idx, 0]
        min_z = points[min_idx, 2]
        in_range = (np.abs(points[:, 0] - min_x) <= 0.5) & (np.abs(points[:, 2] - min_z) <= 0.5)
        points = points[in_range]

        if points.size == 0:
            return None
        
        if points.shape[0] > 1024:
            indices = np.random.choice(points.shape[0], 1024, replace=False)
            points = points[indices]

        if points.shape[1] >= 4:
            rgba = points[:, 3].view(np.uint32)
        else:
            rgba = np.zeros(points.shape[0], dtype=np.uint32)
        cloud = np.zeros(points.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgba', np.uint32),
        ])
        cloud['x'] = points[:, 2]
        cloud['y'] = -1 * points[:, 0]
        cloud['z'] = -1 * points[:, 1]
        cloud['rgba'] = rgba

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]

        pointcloud_msg = point_cloud2.create_cloud(header, fields, cloud)
        return pointcloud_msg

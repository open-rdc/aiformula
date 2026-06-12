from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

POINTCLOUD_Y_RANGE = (0.0, 0.5)
POINTCLOUD_MAX_POINTS = 1024


@dataclass(frozen=True)
class SourceConfig:
    resolution: str = 'HD1080'
    fps: int = 30
    depth_mode: str = 'NEURAL'
    image_topic: str = '/zed/zed_node/rgb/image_rect_color'
    pointcloud_topic: str = '/zed/zed_node/pointcloud'


class SensorSource(ABC):
    @abstractmethod
    def grab(self) -> bool:
        ...

    @abstractmethod
    def get_image(self) -> Optional[np.ndarray]:
        ...

    @abstractmethod
    def get_pointcloud(self) -> Optional[np.ndarray]:
        ...


class ZedSdkSource(SensorSource):
    def __init__(self, node: Node, config: SourceConfig) -> None:
        self._node = node
        self._config = config
        self._logger = node.get_logger()

        import pyzed.sl as sl
        self._sl = sl
        self._camera = sl.Camera()
        self._image = sl.Mat()
        self._pointcloud = sl.Mat()
        self.init_param()

        self._logger.info('ZED camera initialized successfully (SDK source)')

    def init_param(self) -> None:
        sl = self._sl
        init_params = sl.InitParameters()
        init_params.camera_resolution = getattr(sl.RESOLUTION, self._config.resolution)
        init_params.camera_fps = self._config.fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = getattr(sl.DEPTH_MODE, self._config.depth_mode)
        init_params.depth_minimum_distance = 0.5
        init_params.depth_maximum_distance = 10.0

        err = self._camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        self._runtime = sl.RuntimeParameters()
        self._runtime.confidence_threshold = 20
        self._runtime.texture_confidence_threshold = 100

    def grab(self) -> bool:
        return self._camera.grab(self._runtime) == self._sl.ERROR_CODE.SUCCESS

    def get_image(self) -> Optional[np.ndarray]:
        self._camera.retrieve_image(self._image, self._sl.VIEW.LEFT, resolution=())
        return self._image.get_data()

    def get_pointcloud(self) -> Optional[np.ndarray]:
        self._camera.retrieve_measure(self._pointcloud, self._sl.MEASURE.XYZRGBA, self._sl.MEM.CPU)
        return self._pointcloud.get_data()


class RosTopicSource(SensorSource):
    def __init__(self, node: Node, config: SourceConfig) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._bridge = CvBridge()
        self._latest_image: Optional[Image] = None
        self._latest_pointcloud: Optional[PointCloud2] = None
        self._image_updated = False

        self.subscription_image = node.create_subscription(
            Image, config.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.subscription_pointcloud = node.create_subscription(
            PointCloud2, config.pointcloud_topic, self.pointcloud_callback, qos_profile_sensor_data
        )

        self._logger.info(
            f'Using ROS topic source (image={config.image_topic}, '
            f'pointcloud={config.pointcloud_topic})'
        )

    def image_callback(self, msg: Image) -> None:
        self._latest_image = msg
        self._image_updated = True

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        self._latest_pointcloud = msg

    def grab(self) -> bool:
        if not self._image_updated:
            return False
        self._image_updated = False
        return True

    def get_image(self) -> Optional[np.ndarray]:
        msg = self._latest_image
        if msg is None:
            return None
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')

    def get_pointcloud(self) -> Optional[np.ndarray]:
        msg = self._latest_pointcloud
        if msg is None:
            return None
        cloud = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True
        )
        if cloud.size == 0:
            return None
        return cloud


def create_sensor_source(node: Node, config: SourceConfig = SourceConfig()) -> SensorSource:
    try:
        import pyzed.sl  # noqa: F401
    except ImportError:
        node.get_logger().warn(
            'pyzed (ZED SDK) not found; falling back to ROS topic source'
        )
        return RosTopicSource(node, config)
    return ZedSdkSource(node, config)


def filtered_pointcloud(header, pointcloud: np.ndarray) -> Optional[PointCloud2]:
    points = pointcloud.reshape(-1, pointcloud.shape[-1]).astype(np.float32, copy=False)
    finite = np.isfinite(points[:, :3]).all(axis=1)
    points = points[finite]

    y = points[:, 1]
    y_min, y_max = POINTCLOUD_Y_RANGE
    points = points[(y >= y_min) & (y <= y_max)]

    if points.size == 0:
        return None

    if points.shape[0] > POINTCLOUD_MAX_POINTS:
        indices = np.random.choice(points.shape[0], POINTCLOUD_MAX_POINTS, replace=False)
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

    return point_cloud2.create_cloud(header, fields, cloud)

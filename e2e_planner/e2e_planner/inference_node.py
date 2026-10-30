import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from cv_bridge import CvBridge
import torch
import numpy as np
import os
import threading
from pathlib import Path as FilePath
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from typing import Optional, Tuple

from util.yolop_processor import YOLOPv2Processor
from util.pylon_mask import is_pylon_near
from util.preprocess import (
    denormalize_waypoints,
    preprocess_mask,
    preprocess_rgb,
    resize_for_debug,
    to_bgr,
)
from .pylon_input import fresh_pylon_mask
from .speed_limit import SpeedLimiter


class InferenceNode(Node):
    """deploy の zed_wrapper の画像と pylon_detector のパイロンマスクから経路と速度上限を出す。

    経路は vision_lane_planner の代わりに /planner/vision_path に出し、その先は deploy の
    path_smoother -> local_planner -> speed_path_planner -> controller がそのまま使う
    """

    def __init__(self) -> None:
        super().__init__('inference_node')

        self.declare_parameter('model_name', 'model.pt')
        self.declare_parameter('interval_ms', 100)
        # 重みの入力に合わせる。2 = 白線マスク + パイロンマスク、1 = 白線マスクだけ（rgb_mask の重み）
        self.declare_parameter('num_mask_channels', 2)
        self.declare_parameter('publish_topic', '/planner/vision_path')
        self.declare_parameter('speed_limit_mps', 4.0)
        self.declare_parameter('turn_threshold_deg', 15.0)
        self.declare_parameter('speed_limit_hold_s', 1.0)
        # 実機は cuda。GPU の無い PC で bag を流して確かめるときだけ cpu にする
        self.declare_parameter('device', 'cuda')

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value
        self.num_mask_channels = int(self.get_parameter('num_mask_channels').value)

        self.bridge = CvBridge()
        self.device = torch.device(self.get_parameter('device').value)

        self.lock = threading.Lock()
        self.image_msg: Optional[Image] = None
        self.pylon_mask: Optional[np.ndarray] = None
        self.pylon_mask_time = None
        self.warned_no_pylon_mask = False

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        yolop_weight_path = FilePath(package_share_directory) / 'weights' / 'yolopv2.pt'

        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None

        self.yolop_processor = YOLOPv2Processor(yolop_weight_path, self.device)
        self.speed_limiter = SpeedLimiter(
            self.get_parameter('speed_limit_mps').value,
            self.get_parameter('turn_threshold_deg').value,
            self.get_parameter('speed_limit_hold_s').value)

        self.pub = self.create_publisher(Path, self.get_parameter('publish_topic').value, qos_profile_system_default)
        self.pub_speed_limit = self.create_publisher(Float32, '/planning/speed_limit', qos_profile_system_default)
        self.pub_debug_image = self.create_publisher(Image, 'e2e_planner/debug_image', qos_profile_system_default)

        self.subscription_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback,
                                 qos_profile_sensor_data, callback_group=self.subscription_cb_group)
        self.create_subscription(Image, '/perception/pylon_mask', self.pylon_mask_callback,
                                 qos_profile_sensor_data, callback_group=self.subscription_cb_group)

        self.torch_timer = self.create_timer(
            interval_ms / 1000.0,
            self.torch_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def image_callback(self, msg: Image) -> None:
        with self.lock:
            self.image_msg = msg

    def pylon_mask_callback(self, msg: Image) -> None:
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        with self.lock:
            self.pylon_mask = mask
            self.pylon_mask_time = self.get_clock().now()

    def current_pylon_mask(self) -> np.ndarray:
        with self.lock:
            mask, received = self.pylon_mask, self.pylon_mask_time
        age_s = None if received is None else (self.get_clock().now() - received).nanoseconds * 1e-9
        selected = fresh_pylon_mask(mask, age_s)
        if (mask is None or age_s is None or age_s > 0.5) and not self.warned_no_pylon_mask:
            self.get_logger().warn('/perception/pylon_mask が届いていないので、パイロン無しとして推論します')
            self.warned_no_pylon_mask = True
        return selected

    def preprocess_image(self, image: np.ndarray, pylon_mask: np.ndarray) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray, np.ndarray]:
        """白線マスク（とパイロンマスク）と RGB の 2 入力を作る。学習時 (train_curve_weight.py) と同じ
        「元解像度(640x360)でマスク生成 -> 256x144 リサイズ」の順序に揃えている"""
        bgr_image = to_bgr(image)
        height, width = bgr_image.shape[:2]

        mask_full = self.yolop_processor.process_image(bgr_image, (width, height))

        mask_channels = [preprocess_mask(mask_full)]
        if self.num_mask_channels == 2:
            mask_channels.append(preprocess_mask((pylon_mask > 127).astype(np.uint8)))
        mask_input = np.concatenate(mask_channels, axis=0)
        rgb_input = preprocess_rgb(bgr_image)

        mask_tensor = torch.from_numpy(mask_input).unsqueeze(0).to(self.device)
        rgb_tensor = torch.from_numpy(rgb_input).unsqueeze(0).to(self.device)

        return mask_tensor, rgb_tensor, mask_input, bgr_image

    def torch_callback(self) -> None:
        with self.lock:
            image_msg = self.image_msg
        if image_msg is None or self.model is None:
            return

        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        pylon_mask = self.current_pylon_mask()
        mask_tensor, rgb_tensor, mask_input, bgr_image = self.preprocess_image(image, pylon_mask)

        # path_smoother は時刻つきで tf を引くので、画像を撮った時刻を付ける
        header = image_msg.header
        header.frame_id = 'base_link'

        if self.pub_debug_image.get_subscription_count() > 0:
            debug_image = resize_for_debug(bgr_image)
            debug_image[mask_input[0] == 1] = [0, 0, 255]
            if self.num_mask_channels == 2:
                debug_image[mask_input[1] == 1] = [0, 165, 255]
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.pub_debug_image.publish(debug_msg)

        with torch.no_grad():
            output = self.model(mask_tensor, rgb_tensor)

        waypoints = denormalize_waypoints(output.cpu().numpy().flatten()).reshape(-1, 2)
        self.pub.publish(self.create_path(waypoints, header))

        now_s = self.get_clock().now().nanoseconds * 1e-9
        limit = self.speed_limiter.update(waypoints, is_pylon_near(pylon_mask), now_s)
        if limit is not None:
            self.pub_speed_limit.publish(Float32(data=float(limit)))

    @staticmethod
    def create_path(waypoints: np.ndarray, header) -> Path:
        path_msg = Path()
        path_msg.header = header
        path_msg.poses.append(PoseStamped(header=header, pose=Pose(position=Point(x=0.0, y=0.0))))
        path_msg.poses.extend(PoseStamped(header=header, pose=Pose(position=Point(x=float(x), y=float(y)))) for x, y in waypoints)
        return path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

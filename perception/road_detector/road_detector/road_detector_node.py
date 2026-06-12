import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os
from rclpy.qos import qos_profile_system_default
from ament_index_python.packages import get_package_share_directory

from .utils.utils import letterbox, lane_line_mask

INPUT_SHAPE = (640, 640)
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class RoadDetectorNode(Node):
    def __init__(self, context=None):
        super().__init__('road_detector_node', context=context)
        self.logger = self.get_logger()
        self.logger.info(f"Using device: {DEVICE}")

        self.declare_parameter('input_image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('output_mask_topic', '/perception/lane_mask')
        self.declare_parameter('capture_width', 640)
        self.declare_parameter('capture_height', 360)
        self.declare_parameter('visualize', False)

        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value
        self.capture_size = (
            int(self.get_parameter('capture_width').value),
            int(self.get_parameter('capture_height').value),
        )
        self.visualize_flag = bool(self.get_parameter('visualize').value)

        if self.capture_size[0] <= 0 or self.capture_size[1] <= 0:
            raise ValueError('capture_width and capture_height must be greater than 0')

        self.ll_seg_publisher = self.create_publisher(Image, self.output_mask_topic, qos_profile_system_default)
        self.image_subscription = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_system_default,
        )
        self.bridge = CvBridge()

        package_share_directory = get_package_share_directory('road_detector')
        model_path = os.path.join(package_share_directory, 'data', 'weights', 'yolopv2.pt')

        self.logger.info(f'Loading PyTorch model from: {model_path}')
        self.model = torch.jit.load(model_path, map_location=DEVICE)
        self.model.to(DEVICE)
        self.model.eval()
        self.logger.info('PyTorch model loaded successfully!')
        self.logger.info(f'Subscribed image topic: {self.input_image_topic}')
        self.logger.info(f'Publishing mask topic: {self.output_mask_topic}')

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_img = cv2.resize(cv_img, self.capture_size, interpolation=cv2.INTER_LINEAR)

        origin_shape = cv_img.shape[:2]

        img_resized, ratio, (pad_left, pad_top) = letterbox(cv_img, INPUT_SHAPE)

        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        ll_seg_mask = lane_line_mask(ll)
        ll_seg_mask = ll_seg_mask.astype(np.uint8)

        ll_seg_resize_mask = cv2.resize(ll_seg_mask, (origin_shape[1], origin_shape[0]), interpolation=cv2.INTER_NEAREST)

        self.ll_seg_publish(ll_seg_resize_mask)
        if self.visualize_flag:
            self.visualize(cv_img, ll_seg_resize_mask)

    def visualize(self, img, mask):
        vis = img.copy()
        vis[mask == 1] = [0, 0, 255]
        cv2.imshow('road_detector', vis)
        cv2.waitKey(1)

    def ll_seg_publish(self, ll_seg_mask):
        ll_seg_mask = (ll_seg_mask * 255).astype(np.uint8)
        ll_seg_msg = self.bridge.cv2_to_imgmsg(ll_seg_mask, encoding="mono8")
        ll_seg_msg.header.stamp = self.get_clock().now().to_msg()
        self.ll_seg_publisher.publish(ll_seg_msg)

def main(args=None):
    rclpy.init(args=args)
    road_detector_node = RoadDetectorNode()
    rclpy.spin(road_detector_node)
    road_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

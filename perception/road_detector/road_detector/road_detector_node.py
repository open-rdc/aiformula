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

from .utils.utils import letterbox, lane_line_mask, unletterbox_mask

MODEL_INPUT_SHAPE = (640, 640)
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class RoadDetectorNode(Node):
    def __init__(self, context=None):
        super().__init__('road_detector_node', context=context)
        self.logger = self.get_logger()
        self.logger.info(f"Using device: {DEVICE}")

        self.declare_parameter('input_image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('output_mask_topic', '/perception/lane_mask')
        self.declare_parameter('camera.size', 'nHD')

        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value

        camera_size = self.get_parameter('camera.size').value
        camera_dict = {"nHD": (640, 360), "SVGA": (960, 600)}
        self.image_size = camera_dict[camera_size]
        
        self.ll_seg_publisher = self.create_publisher(Image, self.output_mask_topic, qos_profile_system_default)
        self.visualize_publisher = self.create_publisher(Image, self.output_mask_topic + '_visualize', qos_profile_system_default)
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
        cv_img = cv2.resize(cv_img, self.image_size, interpolation=cv2.INTER_LINEAR)

        origin_shape = cv_img.shape[:2]

        img_resized, ratio, (pad_left, pad_top) = letterbox(cv_img, MODEL_INPUT_SHAPE)

        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        ll_seg_mask = lane_line_mask(ll)
        ll_seg_mask = ll_seg_mask.astype(np.uint8)

        ll_seg_resize_mask = unletterbox_mask(
            ll_seg_mask, img_resized.shape[:2], ratio, (pad_left, pad_top), origin_shape)

        self.ll_seg_publish(ll_seg_resize_mask, msg.header)
        if self.visualize_publisher.get_subscription_count() > 0:
            self.visualize(cv_img, ll_seg_resize_mask, msg.header)

    def visualize(self, img, mask, header):
        vis = img.copy()
        vis[mask == 1] = [0, 0, 255]
        vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        vis_msg.header = header
        self.visualize_publisher.publish(vis_msg)

    def ll_seg_publish(self, ll_seg_mask, header):
        ll_seg_mask = (ll_seg_mask * 255).astype(np.uint8)
        ll_seg_msg = self.bridge.cv2_to_imgmsg(ll_seg_mask, encoding="mono8")
        ll_seg_msg.header = header
        self.ll_seg_publisher.publish(ll_seg_msg)

def main(args=None):
    rclpy.init(args=args)
    road_detector_node = RoadDetectorNode()
    rclpy.spin(road_detector_node)
    road_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

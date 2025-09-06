import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os
from ament_index_python.packages import get_package_share_directory

# Utility functions
from .utils.utils import (
    select_device, non_max_suppression,
    split_for_trace_model, lane_line_mask
)

# パラメータ設定
INPUT_SHAPE = (640, 640)
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class RoadDetectorNode(Node):
    def __init__(self, context=None, sim_flag=False):
        super().__init__('road_detector_node', context=context)
        self.logger = self.get_logger()

        self.logger.info(f"Using device: {DEVICE}")


        image_topic = '/zed/zed_node/rgb/image_rect_color'

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.ll_seg_publisher = self.create_publisher(Image, '/yolopv2/image/ll_seg_mask', 10)
        self.result_publisher = self.create_publisher(Image, '/yolopv2/image/result_image', 10)
        self.bridge = CvBridge()

        # モデルパス
        package_share_directory = get_package_share_directory('road_detector')
        model_path = os.path.join(package_share_directory, 'data', 'weights', 'yolopv2.pt')

        # モデルのロード
        self.logger.info(f'Loading PyTorch model from: {model_path}')
        self.model = torch.jit.load(model_path, map_location=DEVICE)
        self.model.to(DEVICE)
        self.model.eval()
        self.logger.info('PyTorch model loaded successfully!')


    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img0 = cv_img.copy()
        origin_shape = img0.shape[:2]

        img_resized, ratio, (pad_left, pad_top) = self.letterbox(cv_img, INPUT_SHAPE)

        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        ll_seg_mask = lane_line_mask(ll)
        ll_seg_mask = ll_seg_mask.astype(np.uint8)
        
        ll_seg_resize_mask = cv2.resize(ll_seg_mask, (origin_shape[1], origin_shape[0]), interpolation=cv2.INTER_NEAREST)
        result_image = self.show_seg_result(img0, ll_seg_resize_mask)

        self.ll_seg_publish(ll_seg_resize_mask)
        self.result_publish(result_image)


    def letterbox(self, img, new_shape, color=(114,114,114), stride=32):
        shape = img.shape[:2]
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        new_unpad = int(round(shape[0]*r)), int(round(shape[1]*r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        dw, dh = np.mod(dw, stride)/2, np.mod(dh, stride)/2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)

        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

        return img, r, (dw, dh)


    def show_seg_result(self, img, result):
        color_area = np.zeros((result.shape[0], result.shape[1], 3), dtype=np.uint8)
        
        color_area[result == 1] = [255, 0, 0]
        color_seg = color_area

        color_seg = color_seg[..., ::-1]
        color_mask = np.mean(color_seg, 2)
        img[color_mask != 0] = img[color_mask != 0] * 0.5 + color_seg[color_mask != 0] * 0.5

        return img


    def ll_seg_publish(self, ll_seg_mask):
        ll_seg_mask_bgr = cv2.cvtColor(ll_seg_mask * 255, cv2.COLOR_GRAY2BGR)
        ll_seg_mask_bgr[ll_seg_mask == 1] = [0, 0, 255]
        ll_seg_msg = self.bridge.cv2_to_imgmsg(ll_seg_mask_bgr, encoding="bgr8")
        ll_seg_msg.header.stamp = self.get_clock().now().to_msg()
        self.ll_seg_publisher.publish(ll_seg_msg)


    def result_publish(self, result_image):
        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
        result_msg.header.stamp = self.get_clock().now().to_msg()
        self.result_publisher.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)
    road_detector_node = RoadDetectorNode()
    rclpy.spin(road_detector_node)
    road_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
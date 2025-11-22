import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

import pyzed.sl as sl

from .utils.utils import letterbox, lane_line_mask

INPUT_SHAPE = (640, 640)
CAPTURE_SIZE = (480, 300)
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class RoadDetectorNode(Node):
    def __init__(self, context=None, sim_flag=False):
        super().__init__('road_detector_node', context=context)
        self.logger = self.get_logger()
        self.logger.info(f"Using device: {DEVICE}")

        self.ll_seg_publisher = self.create_publisher(Image, '/yolopv2/image/ll_seg_mask', qos_profile_sensor_data)
        self.bridge = CvBridge()

        package_share_directory = get_package_share_directory('road_detector')
        model_path = os.path.join(package_share_directory, 'data', 'weights', 'yolopv2.pt')

        self.logger.info(f'Loading PyTorch model from: {model_path}')
        self.model = torch.jit.load(model_path, map_location=DEVICE)
        self.model.to(DEVICE)
        self.model.eval()
        self.logger.info('PyTorch model loaded successfully!')

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f'Failed to open ZED camera: {status}')
            raise RuntimeError(f'ZED camera open failed: {status}')

        self.logger.info('ZED camera opened successfully!')

        self.image_zed = sl.Mat()
        self.runtime_params = sl.RuntimeParameters()

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        frame = self.image_zed.get_data()

        cv_img = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        cv_img = cv2.resize(cv_img, CAPTURE_SIZE, interpolation=cv2.INTER_LINEAR)

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

    def destroy_node(self):
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    road_detector_node = RoadDetectorNode()
    rclpy.spin(road_detector_node)
    road_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

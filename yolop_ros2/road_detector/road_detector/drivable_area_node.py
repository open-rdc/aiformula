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
# from .utils.utils import driving_area_mask

# パラメータ設定
INPUT_SHAPE = (320, 320) # Reduced from 640 for CPU speed
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ... (imports)

class DrivableAreaNode(Node):
    def __init__(self, context=None, sim_flag=False):
        super().__init__('drivable_area_node', context=context)
        self.logger = self.get_logger()
        self.logger.info(f"Using device: {DEVICE}")
        
        # Optimize for CPU
        if DEVICE.type == 'cpu':
            torch.set_num_threads(4)
            self.logger.info("Set torch num threads to 4 for CPU optimization")

        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('segmentation_topic', '/yolopv2/image/drivable_area_mask')
        self.declare_parameter('annotated_topic', '/yolopv2/image/drivable_area_visual')
        self.declare_parameter('gemini_topic', '/gemini/drivable_area_image')
        self.declare_parameter('lane_line_topic', '/yolopv2/image/lane_line_mask')

        image_topic = self.get_parameter('input_topic').value
        segmentation_topic = self.get_parameter('segmentation_topic').value
        lane_line_topic = self.get_parameter('lane_line_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        gemini_topic = self.get_parameter('gemini_topic').value or ''

        # Use Best Effort QoS to avoid queue buildup and latency
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile
        )
        self.da_seg_publisher = self.create_publisher(Image, segmentation_topic, 10)
        self.ll_seg_publisher = self.create_publisher(Image, lane_line_topic, 10)
        self.result_publisher = self.create_publisher(Image, annotated_topic, 10)
        self.gemini_publisher = (
            self.create_publisher(Image, gemini_topic, 10) if gemini_topic else None
        )
        self.bridge = CvBridge()

        self.logger.info(
            f'Subscribed to {image_topic}; publishing segmentation to {segmentation_topic}, '
            f'lane lines to {lane_line_topic}, '
            f'annotated images to {annotated_topic}'
            + (f', and Gemini input to {gemini_topic}.' if self.gemini_publisher else '.')
        )

        # ... (model loading code remains same) ...
        package_share_directory = get_package_share_directory('road_detector')
        model_path = os.path.join(package_share_directory, 'data', 'weights', 'yolopv2.pt')

        # モデルのロード
        self.logger.info(f'Loading PyTorch model from: {model_path}')
        self.model = torch.jit.load(model_path, map_location=DEVICE)
        self.model.to(DEVICE)
        
        # Removed FP16 for CPU compatibility
        
        self.model.eval()
        self.logger.info('PyTorch model loaded successfully!')


    def image_callback(self, msg):
        start_time = self.get_clock().now()
        
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img0 = cv_img.copy()
        origin_shape = img0.shape[:2]

        img_resized, ratio, (pad_left, pad_top) = self.letterbox(cv_img, INPUT_SHAPE)

        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(DEVICE)
        
        # Removed FP16 conversion

        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        # --- Drivable Area Processing ---
        # seg shape: [1, 2, H, W] (2 classes: background, drivable area)
        da_seg_mask = torch.nn.functional.interpolate(seg, scale_factor=2, mode='bilinear')
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
        
        # --- Lane Line Processing ---
        # ll shape: [1, 1, H, W] (binary classification per pixel)
        ll_seg_mask = torch.nn.functional.interpolate(ll, scale_factor=2, mode='bilinear')
        ll_seg_mask = torch.round(ll_seg_mask).squeeze(1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()

        # --- Cropping Padding ---
        h, w = da_seg_mask.shape
        top = int(round(pad_top - 0.1))
        bottom = int(round(pad_top + 0.1))
        left = int(round(pad_left - 0.1))
        right = int(round(pad_left + 0.1))

        if h > top + bottom and w > left + right:
            da_cropped = da_seg_mask[top : h-bottom, left : w-right]
            ll_cropped = ll_seg_mask[top : h-bottom, left : w-right]
        else:
            da_cropped = da_seg_mask
            ll_cropped = ll_seg_mask

        # --- Resizing to Original ---
        da_seg_resize_mask = cv2.resize(da_cropped, (origin_shape[1], origin_shape[0]), interpolation=cv2.INTER_NEAREST)
        ll_seg_resize_mask = cv2.resize(ll_cropped, (origin_shape[1], origin_shape[0]), interpolation=cv2.INTER_NEAREST)

        # --- Visualization ---
        result_image = self.show_seg_result(img0, da_seg_resize_mask, ll_seg_resize_mask)

        self.da_seg_publish(da_seg_resize_mask)
        self.ll_seg_publish(ll_seg_resize_mask)
        self.result_publish(result_image)
        
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.logger.info(f"Inference time: {duration:.3f}s")


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


    def show_seg_result(self, img, da_mask, ll_mask):
        # Create a colored overlay
        # Drivable Area: Yellow [0, 255, 255]
        # Lane Lines: Red [0, 0, 255] (BGR)
        
        color_overlay = np.zeros_like(img)
        
        # Apply Yellow for Drivable Area
        color_overlay[da_mask == 1] = [0, 255, 255]
        
        # Apply Red for Lane Lines
        color_overlay[ll_mask == 1] = [0, 0, 255]
        
        # Create a mask for where we have any segmentation
        mask_indices = (da_mask == 1) | (ll_mask == 1)
        
        # Blend: 0.5 * Original + 0.5 * Overlay
        img[mask_indices] = img[mask_indices] * 0.5 + color_overlay[mask_indices] * 0.5

        return img


    def da_seg_publish(self, da_seg_mask):
        da_seg_mask = (da_seg_mask * 255).astype(np.uint8)
        stamp = self.get_clock().now().to_msg()
        da_seg_msg = self.bridge.cv2_to_imgmsg(da_seg_mask, encoding="mono8")
        da_seg_msg.header.stamp = stamp
        self.da_seg_publisher.publish(da_seg_msg)

    def ll_seg_publish(self, ll_seg_mask):
        ll_seg_mask = (ll_seg_mask * 255).astype(np.uint8)
        stamp = self.get_clock().now().to_msg()
        ll_seg_msg = self.bridge.cv2_to_imgmsg(ll_seg_mask, encoding="mono8")
        ll_seg_msg.header.stamp = stamp
        self.ll_seg_publisher.publish(ll_seg_msg)


    def result_publish(self, result_image):
        stamp = self.get_clock().now().to_msg()
        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
        result_msg.header.stamp = stamp
        self.result_publisher.publish(result_msg)
        if self.gemini_publisher is not None:
            gemini_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
            gemini_msg.header.stamp = stamp
            self.gemini_publisher.publish(gemini_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrivableAreaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

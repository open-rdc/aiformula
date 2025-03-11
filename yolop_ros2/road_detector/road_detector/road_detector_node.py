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
    select_device, scale_coords, non_max_suppression,
    split_for_trace_model, lane_line_mask
)

# パラメータ設定
INPUT_SHAPE = (320, 320)
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
CONF_THRES = 0.3
IOU_THRES = 0.45

class RoadDetectorNode(Node):
    def __init__(self, context=None):
        super().__init__('road_detector_node', context=context)
        self.logger = self.get_logger()

        self.logger.info(f"Using device: {DEVICE}")

        self.subscription = self.create_subscription(
            Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10
        )
        self.ll_seg_publisher = self.create_publisher(Image, '/yolopv2/image/ll_seg_mask', 10)
        self.bridge = CvBridge()

        # モデルパス
        package_share_directory = get_package_share_directory('road_detector')
        model_path = os.path.join(package_share_directory, 'data', 'weights', 'yolopv2.pt')

        # モデルのロード
        self.logger.info(f'Loading PyTorch model from: {model_path}')
        try:
            self.model = torch.jit.load(model_path, map_location=DEVICE)
            self.model.to(DEVICE)
            self.model.eval()
            self.logger.info('PyTorch model loaded successfully!')
        except FileNotFoundError:
            self.logger.error(f"Model file not found: {model_path}")
            return
        except Exception as e:
            self.logger.error(f"Failed to load model: {e}")
            return

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_h, original_w = cv_image.shape[:2]
        except Exception as e:
            self.logger.error(f"Failed to convert image: {e}")
            return

        # アスペクト比維持リサイズ（パディング追加）
        img_resized, scale, (pad_left, pad_top) = self.letterbox_resize(cv_image, INPUT_SHAPE)

        # 前処理
        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(DEVICE)

        # 推論
        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        # NMS
        pred = split_for_trace_model(pred, anchor_grid)
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES)

        # バウンディングボックスの座標変換
        for det in pred:
            if len(det):
                det[:, :4] = scale_coords(INPUT_SHAPE, det[:, :4], (original_h, original_w)).round()

        # レーンラインのマスク画像処理
        ll_seg_mask = (lane_line_mask(ll) > 0).astype(np.uint8)
        ll_seg_mask_resized = cv2.resize(ll_seg_mask, (INPUT_SHAPE[1], INPUT_SHAPE[0]), interpolation=cv2.INTER_NEAREST)
        ll_seg_mask_cropped = ll_seg_mask_resized[pad_top:INPUT_SHAPE[0] - pad_top, pad_left:INPUT_SHAPE[1] - pad_left]
        ll_seg_mask = cv2.resize(ll_seg_mask_cropped, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

        # 3チャンネル画像に変換
        ll_seg_mask_bgr = cv2.cvtColor(ll_seg_mask * 255, cv2.COLOR_GRAY2BGR)
        ll_seg_mask_bgr[ll_seg_mask == 1] = [0, 0, 255]  # 赤色マスク適用

        # 結果の配信
        try:
            from rclpy.time import Time
            ll_seg_msg = self.bridge.cv2_to_imgmsg(ll_seg_mask_bgr, encoding="bgr8")
            ll_seg_msg.header.stamp = self.get_clock().now().to_msg()
            self.ll_seg_publisher.publish(ll_seg_msg)

            # self.logger.info(f"Published lane line mask. Using device: {DEVICE}")
        except Exception as e:
            self.logger.error(f"Failed to publish results: {e}")

    def letterbox_resize(self, img, target_size):
        """アスペクト比を維持したリサイズ処理"""
        h, w = img.shape[:2]
        scale = min(target_size[0] / h, target_size[1] / w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_top = (target_size[0] - new_h) // 2
        pad_bottom = target_size[0] - new_h - pad_top
        pad_left = (target_size[1] - new_w) // 2
        pad_right = target_size[1] - new_w - pad_left

        img_padded = cv2.copyMakeBorder(
            resized, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )
        return img_padded, scale, (pad_left, pad_top)


def main(args=None):
    rclpy.init(args=args)
    road_detector_node = RoadDetectorNode()
    rclpy.spin(road_detector_node)
    road_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageBinarizerYolop(Node):
    def __init__(self):
        super().__init__('image_binarizer_yolop')

        # CvBridge初期化
        self.bridge = CvBridge()

        # パラメータ設定
        self.declare_parameter('threshold_value', 240)
        self.declare_parameter('max_value', 255)
        self.threshold_value = self.get_parameter('threshold_value').value
        self.max_value = self.get_parameter('max_value').value

        # サブスクライバ設定
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        # パブリッシャ設定
        self.publisher = self.create_publisher(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            10
        )

        self.get_logger().info(f'Image Binarizer YOLOP node started')
        self.get_logger().info(f'Subscribing to: /image_raw')
        self.get_logger().info(f'Publishing to: /zed/zed_node/rgb/image_rect_color')
        self.get_logger().info(f'Threshold: {self.threshold_value}, Max value: {self.max_value}')

    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV画像変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # グレースケールに変換
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 2値化処理
            _, binary_image = cv2.threshold(
                gray_image,
                self.threshold_value,
                self.max_value,
                cv2.THRESH_BINARY
            )

            # 黒背景のBGR画像を作成
            red_image = np.zeros((binary_image.shape[0], binary_image.shape[1], 3), dtype=np.uint8)

            # 白ピクセル（255）を赤ピクセル（BGR: 0, 0, 255）に変換
            red_image[binary_image == 255] = [0, 0, 255]
            
            # y軸0~10の範囲を黒く塗りつぶす
            red_image[0:10, :] = [0, 0, 0]

            # OpenCV画像 → ROS Image変換
            red_msg = self.bridge.cv2_to_imgmsg(red_image, encoding='bgr8')
            
            # ヘッダー情報をコピー
            red_msg.header = msg.header

            # パブリッシュ
            self.publisher.publish(red_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageBinarizerYolop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

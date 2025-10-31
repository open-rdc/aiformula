#!/usr/bin/env python3
"""
Real-time traffic cone detection using camera
Subscribes to camera image topic, detects cones, and publishes bounding box information
"""

import warnings
warnings.filterwarnings('ignore', category=FutureWarning)
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from obstacle_detector_msgs.msg import RectArray, Rect
from traffic_cone_detector.cone_detector import ConeDetector


class DetectCameraNode(Node):
    def __init__(self):
        super().__init__('detect_camera_node')

        # Parameters
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('output_topic', '/obstacle_pixel')
        self.declare_parameter('weights_path', 'weights/best.pt')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.45)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        weights_path = self.get_parameter('weights_path').value
        conf_thres = self.get_parameter('conf_thres').value
        iou_thres = self.get_parameter('iou_thres').value

        # Initialize detector
        self.get_logger().info('Loading YOLOv5 model...')
        try:
            self.detector = ConeDetector(
                weights_path=weights_path,
                conf_thres=conf_thres,
                iou_thres=iou_thres
            )
            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.obstacle_pub = self.create_publisher(RectArray, output_topic, 10)

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)

        results = self.detector.detect(cv_image)
        detections = self.detector.get_detections(results)

        rect_array_msg = RectArray()
        rect_array_msg.header = msg.header

        for idx, row in detections.iterrows():
            # Extract bounding box coordinates
            xmin = int(row['xmin'])
            ymin = int(row['ymin'])
            xmax = int(row['xmax'])
            ymax = int(row['ymax'])

            bbox_width = xmax - xmin
            bbox_height = ymax - ymin

            center_x = (xmin + xmax) / 2.0
            bottom_y = float(ymax)

            rect = Rect()
            rect.x = center_x
            rect.y = bottom_y
            rect.width = float(bbox_width)
            rect.height = float(bbox_height)

            rect_array_msg.rects.append(rect)

        # Publish
        self.obstacle_pub.publish(rect_array_msg)

        if len(rect_array_msg.rects) > 0:
            self.get_logger().debug(f'Detected {len(rect_array_msg.rects)} cones')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DetectCameraNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

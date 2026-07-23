#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageEncodingConverter(Node):
    def __init__(self):
        super().__init__("sim_image_encoding_converter_node")
        self.declare_parameter("input_topic", "/image_raw")
        self.declare_parameter("output_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("encoding", "bgr8")

        self.output_topic = self.get_parameter("output_topic").value
        self.encoding = self.get_parameter("encoding").value
        self.bridge = CvBridge()

        self.create_subscription(Image, self.get_parameter("input_topic").value, self.callback_image, 10)
        self.publisher = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(f"シミュレータ画像を {self.encoding} に変換して {self.output_topic} へ出版します．")

    def callback_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        out = self.bridge.cv2_to_imgmsg(cv_image, encoding=self.encoding)
        out.header = msg.header
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = ImageEncodingConverter()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from pathlib import Path
import sys
# sys.path.append(str(Path(__file__).parent.parent / 'scripts'))
from e2e_planner.feature_extractor import ImageFeatureExtractor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

PUBLISH_INTERVAL = 0.1   # 10Hz（収集時のSAMPLE_INTERVALと合わせる）
FEAT_DIM = 1280


class FeaturesNode(Node):
    def __init__(self) -> None:
        super().__init__('feature_node')

        self.feature_extractor = ImageFeatureExtractor()
        self.feature_extractor.eval()

        self.bridge = CvBridge()
        self.latest_image = None

        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(Float32MultiArray, '/pfoe/features', 10)

        self.create_timer(PUBLISH_INTERVAL, self.timer_callback)

        self.get_logger().info('feature_node started')

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def timer_callback(self) -> None:
        if self.latest_image is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='rgb8')

        feat = self.feature_extractor.extract(cv_image).astype(np.float32)
        assert feat.shape[0] == FEAT_DIM, f'unexpected feature dim: {feat.shape}'

        msg = Float32MultiArray()
        msg.data = feat.tolist()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FeaturesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

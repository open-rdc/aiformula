import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from steered_drive_msg.msg import SteeredDrive

from .pilot_net_controller_core import PilotNetControllerCore


class PilotNetControllerNode(Node):
    def __init__(self):
        super().__init__('pilot_net_controller_node')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('weights_path', 'pilotnet_weights.npy')
        self.declare_parameter('steering_max', 15.0)
        self.declare_parameter('velocity_max', 5.0)

        image_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        weights_path = self.get_parameter('weights_path').value
        steering_max = self.get_parameter('steering_max').value
        velocity_max = self.get_parameter('velocity_max').value

        share_dir = get_package_share_directory('pilot_net_controller')
        self.core = PilotNetControllerCore(f'{share_dir}/weights/{weights_path}', steering_max, velocity_max)

        self.autonomous_flag = False

        self.bridge = CvBridge()
        self.cmd_publisher = self.create_publisher(SteeredDrive, cmd_topic, 10)
        self.image_subscription = self.create_subscription(Image, image_topic, self.image_callback, 
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1))
        self.autonomous_subscription = self.create_subscription(Bool, '/autonomous', self.autonomous_callback, 10)

    def autonomous_callback(self, msg: Bool):
        self.autonomous_flag = msg.data

    def image_callback(self, msg: Image):
        if not self.autonomous_flag:
            return

        bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        steering_angle, velocity = self.core.infer(bgr_image)

        cmd_msg = SteeredDrive()
        cmd_msg.steering_angle = steering_angle
        cmd_msg.velocity = velocity
        self.cmd_publisher.publish(cmd_msg)


def main():
    rclpy.init()
    node = PilotNetControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

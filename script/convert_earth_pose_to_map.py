import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import copy

class Converter(Node):
    def __init__(self):
        super().__init__("earth_pose_to_map_converter_node")

        self.create_subscription(PoseWithCovarianceStamped, "/vectornav/pose", self.callback, 10)
        self.initial_pose = Pose()
        self.is_callbacked = False

        self.publisher = self.create_publisher(PoseStamped, '/vectornav/mappose', 10)

        self.get_logger().info("vctornav/poseをmap座標系にしたposeに変換して出版します。")



    def callback(self, msg):
        tx = PoseStamped()
        tx.header.frame_id = "map"
        tx.header.stamp = self.get_clock().now().to_msg()


        if not self.is_callbacked:
            self.initial_pose = copy.deepcopy(msg.pose.pose)
            self.is_callbacked = True


        tx.pose = msg.pose.pose
        tx.pose.position.x -= self.initial_pose.position.x
        tx.pose.position.y -= self.initial_pose.position.y
        tx.pose.position.z -= self.initial_pose.position.z

        self.publisher.publish(tx)



def main():
    rclpy.init()
    node = Converter()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

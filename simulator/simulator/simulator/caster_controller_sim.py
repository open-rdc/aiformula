import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class CasterControllerSim(Node):
    def __init__(self):
        super().__init__("caster_controller_sim_node")
        self.wheelbase = 0.65
        self.create_subscription(Twist, "/cmd_vel", self.callback_cmd_vel, 1)
        self.publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 1)
        self.get_logger().info("caster_controller_sim_node has been started.")

    def callback_cmd_vel(self, msg):
        pub_msg = Float64MultiArray()
        if msg.linear.x == 0:
            pub_msg.data = [0.0]
        else:
            print("self.wheelbase: " + str(self.wheelbase))
            print("msg.angular.z: " + str(msg.angular.z))
            print("msg.linear.x: " + str(msg.linear.x))
            print("self.wheelbase * msg.angular.z / msg.linear.x: " + str(self.wheelbase * msg.angular.z / msg.linear.x))
            pub_msg.data = [-math.asin(max(min(self.wheelbase * msg.angular.z / msg.linear.x, 1.0), -1.0))]
        self.publisher.publish(pub_msg)

def main():
    rclpy.init()
    node = CasterControllerSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

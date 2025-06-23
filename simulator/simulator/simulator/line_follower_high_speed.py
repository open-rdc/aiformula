#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.odom_subscriber = self.create_subscription(Odometry, '/p3d/odom', self.odom_callback, 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.lookahead_distance = 1.0
        self.linear_velocity = 5.0
        self.goal_threshold = 0.1

        # 状態: 'accelerating', 'changing_lane', 'finished'
        self.state = 'accelerating'

        # 車線切り替えのしきい値
        self.accel_distance = 5.0

        # 加速区間では y = 0 に直進
        self.accel_line_start = (0, 0)
        self.accel_line_end = (100, 0)

        # 車線変更後のライン y = 1
        self.target_line_start = (0, 1)
        self.target_line_end = (100, 1)

        self.current_position = None
        self.current_orientation = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        self.current_orientation = atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.current_position is None or self.current_orientation is None:
            return

        robot_x = self.current_position.x
        robot_y = self.current_position.y

        # 状態遷移処理
        if self.state == 'accelerating' and robot_x >= self.accel_distance:
            self.get_logger().info('Switching to lane change (changing_lane)')
            self.state = 'changing_lane'

        if self.state == 'finished':
            self.publish_cmd_vel(0.0, 0.0)
            return

        # 状態に応じたライン設定
        if self.state == 'accelerating':
            start, end = self.accel_line_start, self.accel_line_end
        elif self.state == 'changing_lane':
            start, end = self.target_line_start, self.target_line_end
        else:
            return

        goal_x, goal_y = self.calculate_lookahead_point(robot_x, robot_y, start, end)
        distance_to_goal = sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

        if self.state == 'changing_lane' and distance_to_goal < self.goal_threshold:
            self.get_logger().info('Goal reached.')
            self.state = 'finished'
            self.publish_cmd_vel(0.0, 0.0)
            return

        heading_to_goal = atan2(goal_y - robot_y, goal_x - robot_x)
        angle_difference = heading_to_goal - self.current_orientation
        angular_velocity = 2.0 * angle_difference
        self.publish_cmd_vel(self.linear_velocity, angular_velocity)

    def calculate_lookahead_point(self, robot_x, robot_y, line_start, line_end):
        x1, y1 = line_start
        x2, y2 = line_end
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        denom = sqrt(A ** 2 + B ** 2)

        if denom == 0:
            return x1, y1

        dx = x2 - x1
        dy = y2 - y1
        line_length = sqrt(dx ** 2 + dy ** 2)

        if line_length == 0:
            return x1, y1

        dx /= line_length
        dy /= line_length

        closest_point_x = robot_x - A * (A * robot_x + B * robot_y + C) / (denom ** 2)
        closest_point_y = robot_y - B * (A * robot_x + B * robot_y + C) / (denom ** 2)

        goal_x = closest_point_x + dx * self.lookahead_distance
        goal_y = closest_point_y + dy * self.lookahead_distance
        return goal_x, goal_y

    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
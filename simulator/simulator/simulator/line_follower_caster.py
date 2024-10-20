import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, sin, cos, asin
from sensor_msgs.msg import JointState

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # オドメトリのサブスクライバー
        self.odom_subscriber = self.create_subscription(Odometry, '/p3d/odom', self.odom_callback, 1)

        # joint_stateのサブスクライバー
        self.joint_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 1)

        # cmd_velのパブリッシャー
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Pure Pursuitアルゴリズムのパラメータ
        self.lookahead_distance = 1.0  # Lookahead distance (meters)
        self.linear_velocity = 0.5     # 前進速度 (m/s)
        self.goal_threshold = 0.1      # ゴール位置の閾値 (meters)

        # 直線の始点と終点を設定 (例: x=0, y=0 から x=10, y=0)
        self.target_line_start = (0, 1)
        self.target_line_end = (100, 1)

        self.current_position = None
        self.current_orientation = None
        self.caster_steering_angle = None
        self.caster_steering_angular_velocity = None

        # 制御ループ
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """オドメトリのコールバック"""
        self.current_position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 四元数からヨー角に変換（ここでは2Dなのでyaw角のみ使う）
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_orientation = atan2(siny_cosp, cosy_cosp)

    def joint_callback(self, msg):
        """joint_stateのコールバック"""
        if 'wheel_caster_steering_joint' in msg.name:
            index = msg.name.index('wheel_caster_steering_joint')
            self.caster_steering_angle = msg.position[index]
            self.caster_steering_angular_velocity = msg.velocity[index]

    def control_loop(self):
        """Pure Pursuitアルゴリズムの実行ループ"""
        if self.current_position is None or self.current_orientation is None:
            return

        # 現在位置
        robot_x = self.current_position.x
        robot_y = self.current_position.y

        # Lookahead点を計算
        goal_x, goal_y = self.calculate_lookahead_point(robot_x, robot_y)

        # ゴールまでの距離
        distance_to_goal = sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

        # ゴールに近づいた場合は停止
        if distance_to_goal < self.goal_threshold:
            self.publish_cmd_vel(0.0, 0.0)
            self.get_logger().info('Goal reached')
            return

        # ロボットのヨー角と目標点までの方位角を計算
        heading_to_goal = atan2(goal_y - robot_y, goal_x - robot_x)

        # 進行方向との差分
        angle_difference = heading_to_goal - self.current_orientation

        # cmd_velの生成
        angular_velocity = 2.0 * angle_difference  # 回転速度制御

        # caster steering の角度による補正
        wheelbase = 0.65
        target_caster_steering_angle = -asin(max(min(wheelbase * angular_velocity / self.linear_velocity, 1.0), -1.0))
        diff_steering_angle = target_caster_steering_angle - self.caster_steering_angle
        print("target: "+str(target_caster_steering_angle)+", current: "+str(self.caster_steering_angle)+", diff_steering_angle: "+str(diff_steering_angle))
        if abs(diff_steering_angle) > 0.2:
            self.linear_velocity = 0.1
        else:
            self.linear_velocity = 0.5
        self.publish_cmd_vel(self.linear_velocity, angular_velocity)

    def calculate_lookahead_point(self, robot_x, robot_y):
        """直線上のLookaheadポイントを計算"""
        x1, y1 = self.target_line_start
        x2, y2 = self.target_line_end

        # 直線のパラメータ (Ax + By + C = 0の形式)
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2

        # ロボットから直線への垂線の距離
        distance_to_line = (A * robot_x + B * robot_y + C) / sqrt(A**2 + B**2)

        # 直線に沿ったLookaheadポイントの位置を計算
        # ロボットから直線上の最も近い点を見つけ、その点からlookahead距離を加えた点
        dx = x2 - x1
        dy = y2 - y1
        line_length = sqrt(dx**2 + dy**2)

        if line_length == 0:
            return x1, y1

        # 直線上の進行方向のベクトルを正規化
        dx /= line_length
        dy /= line_length

        # ロボットの現在位置とLookahead距離を使って目標点を計算
        closest_point_x = robot_x - A * distance_to_line / sqrt(A**2 + B**2)
        closest_point_y = robot_y - B * distance_to_line / sqrt(A**2 + B**2)

        # Lookahead距離分進んだ点を目標点とする
        goal_x = closest_point_x + dx * self.lookahead_distance
        goal_y = closest_point_y + dy * self.lookahead_distance

        return goal_x, goal_y

    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        """cmd_velトピックにTwistメッセージをパブリッシュ"""
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

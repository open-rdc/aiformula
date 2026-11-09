#!/usr/bin/env python3
import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from speed_path_msgs.msg import SpeedPath, StopPoint
from std_msgs.msg import Bool, String
from steered_drive_msg.msg import SteeredDrive
from tf2_ros import Buffer, TransformListener

from course_replay_plot import plot

SCENARIOS = {
    'out_curve': {'l1': 20.0, 'radius': 15.0, 'angle': 180.0, 'l2': 20.0, 'command': None},
    'branch_curve': {'l1': 30.0, 'radius': 6.0, 'angle': 90.0, 'l2': 15.0, 'command': 'left'},
}
FORBIDDEN_NODES = ['vision_lane_planner_node', 'topomap_localizer_node', 'traffic_signal_stop_node']
REQUIRED_NODES = ['path_smoother_node', 'local_planner_server_node', 'speed_path_planner_node', 'controller_server_node', 'odom_tf_node']
EXCLUSIVE_TOPICS = ['/planner/vision_path', '/planning/nav_cmd']
STEP_M = 0.1
RUNOFF_M = 15.0
RANGE_M = 15.0
COMMAND_LEAD_M = 20.0
PREVIEW_S = 1.0
STEER_MAX = math.radians(15.0)


def build_course(l1, radius, angle, l2):
    theta = math.radians(angle)
    arc = radius * theta
    s = np.arange(0.0, l1 + arc + l2 + RUNOFF_M, STEP_M)
    x = np.empty_like(s)
    y = np.empty_like(s)
    yaw = np.empty_like(s)
    for i, si in enumerate(s):
        if si < l1:
            x[i], y[i], yaw[i] = si, 0.0, 0.0
        elif si < l1 + arc:
            phi = (si - l1) / radius
            x[i], y[i], yaw[i] = l1 + radius * math.sin(phi), radius * (1.0 - math.cos(phi)), phi
        else:
            d = si - l1 - arc
            x[i] = l1 + radius * math.sin(theta) + d * math.cos(theta)
            y[i] = radius * (1.0 - math.cos(theta)) + d * math.sin(theta)
            yaw[i] = theta
    return s, x, y, yaw


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CourseReplay(Node):
    def __init__(self, scenario):
        super().__init__('course_replay_node')
        self.cfg = SCENARIOS[scenario]
        self.s, self.cx, self.cy, self.cyaw = build_course(self.cfg['l1'], self.cfg['radius'], self.cfg['angle'], self.cfg['l2'])
        self.ox, self.oy = self.cx, self.cy
        self.turn_start = self.cfg['l1']
        self.turn_end = self.cfg['l1'] + self.cfg['radius'] * math.radians(self.cfg['angle'])
        self.goal = self.turn_end + self.cfg['l2']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.anchored = False
        self.command = None
        self.velocity = 0.0
        self.steering = 0.0
        self.speed_path = None
        self.rows = []
        self.path_file = f'course_replay_{scenario}_{time.strftime("%Y%m%d_%H%M%S")}.csv'

    def check(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            names = self.get_node_names()
            if all(n in names for n in REQUIRED_NODES) and self.tf_buffer.can_transform('odom', 'base_link', Time()):
                break
        names = self.get_node_names()
        errors = [f'{n} を off にしてください' for n in FORBIDDEN_NODES if n in names]
        errors += [f'{t} に他の publisher があります' for t in EXCLUSIVE_TOPICS if self.count_publishers(t) > 0]
        errors += [f'{n} が起動していません' for n in REQUIRED_NODES if n not in names]
        if not self.tf_buffer.can_transform('odom', 'base_link', Time()):
            errors.append('odom→base_link が取れません')
        return errors

    def start(self):
        self.path_pub = self.create_publisher(Path, '/planner/vision_path', 1)
        self.cmd_pub = self.create_publisher(String, '/planning/nav_cmd', 10)
        self.stop_pub = self.create_publisher(StopPoint, '/planning/stop_point', 1)
        self.create_subscription(Bool, '/autonomous', self.autonomous_callback, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/chassis/velocity_body', self.velocity_callback, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(SteeredDrive, '/cmd_vel', self.steering_callback, 10)
        self.create_subscription(SpeedPath, '/planner/speed_path', self.speed_path_callback, rclpy.qos.qos_profile_sensor_data)
        self.publish_command('straight')
        self.create_timer(1.0 / 15.0, self.timer_callback)

    def velocity_callback(self, msg):
        self.velocity = msg.twist.twist.linear.x

    def steering_callback(self, msg):
        self.steering = msg.steering_angle

    def speed_path_callback(self, msg):
        self.speed_path = msg

    def autonomous_callback(self, msg):
        if not msg.data or self.anchored:
            return
        tf = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        x0, y0 = tf.transform.translation.x, tf.transform.translation.y
        yaw0 = yaw_of(tf.transform.rotation)
        c, s = math.cos(yaw0), math.sin(yaw0)
        self.cx, self.cy = x0 + c * self.cx - s * self.cy, y0 + s * self.cx + c * self.cy
        self.cyaw = self.cyaw + yaw0
        self.origin = (x0, y0, yaw0)
        self.anchored = True
        self.get_logger().info('コースを固定しました')

    def publish_command(self, command):
        if command != self.command:
            self.command = command
            self.cmd_pub.publish(String(data=command))

    def project(self, x, y):
        i = int(np.argmin(np.hypot(self.cx - x, self.cy - y)))
        e = -math.sin(self.cyaw[i]) * (x - self.cx[i]) + math.cos(self.cyaw[i]) * (y - self.cy[i])
        return i, e

    def timer_callback(self):
        if not self.anchored:
            return
        tf = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        x, y = tf.transform.translation.x, tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        i, e_y = self.project(x, y)
        s_ego = self.s[i]

        c, s = math.cos(yaw), math.sin(yaw)
        path = Path()
        path.header.stamp = tf.header.stamp
        path.header.frame_id = 'base_link'
        for j in range(i, len(self.s), 5):
            dx, dy = self.cx[j] - x, self.cy[j] - y
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if self.s[j] - s_ego > RANGE_M + 1.0:
                break
            if 0.0 < bx <= RANGE_M:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = bx, by
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_pub.publish(path)

        command = self.cfg['command']
        self.publish_command(command if command and self.turn_start - COMMAND_LEAD_M <= s_ego < self.turn_end else 'straight')
        self.stop_pub.publish(StopPoint(distance=max(self.goal - s_ego, 0.0)))

        v_ref, offset = math.nan, math.nan
        if self.speed_path and self.speed_path.points:
            pts = self.speed_path.points
            d = [math.hypot(p.pose.position.x - x, p.pose.position.y - y) for p in pts]
            k = int(np.argmin(d))
            v_ref = pts[k].linear_velocity
            acc = 0.0
            while k + 1 < len(pts) and acc < max(self.velocity, 1.0) * PREVIEW_S:
                acc += math.hypot(pts[k + 1].pose.position.x - pts[k].pose.position.x, pts[k + 1].pose.position.y - pts[k].pose.position.y)
                k += 1
            offset = self.project(pts[k].pose.position.x, pts[k].pose.position.y)[1]

        section = 'straight_in' if s_ego < self.turn_start else 'curve' if s_ego < self.turn_end else 'straight_out'
        e_psi = math.remainder(yaw - self.cyaw[i], 2.0 * math.pi)
        x0, y0, yaw0 = self.origin
        dx, dy = x - x0, y - y0
        xc, yc = math.cos(yaw0) * dx + math.sin(yaw0) * dy, -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        self.rows.append([self.get_clock().now().nanoseconds * 1e-9, section, s_ego, e_y, e_psi, self.velocity, v_ref, offset, self.steering, abs(self.steering) >= STEER_MAX - 1e-3, xc, yc, self.ox[i], self.oy[i]])

    def report(self):
        if not self.rows:
            return
        with open(self.path_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'section', 's', 'e_y', 'e_psi', 'v', 'v_ref', 'plan_offset', 'steering', 'saturated', 'x', 'y', 'course_x', 'course_y'])
            w.writerows(self.rows)
        print(f'{self.path_file} に保存しました')
        print(f'{"section":<13}{"e_y rms":>9}{"e_y max":>9}{"e_psi rms":>11}{"dv rms":>9}{"offset max":>12}{"sat %":>8}')
        for name in ['straight_in', 'curve', 'straight_out']:
            r = [row for row in self.rows if row[1] == name and row[2] < self.goal]
            if not r:
                continue
            e_y = np.array([row[3] for row in r])
            e_psi = np.array([row[4] for row in r])
            dv = np.array([row[5] - row[6] for row in r])
            off = np.array([row[7] for row in r])
            sat = np.array([row[9] for row in r])
            print(f'{name:<13}{np.sqrt(np.mean(e_y**2)):9.3f}{np.max(np.abs(e_y)):9.3f}{math.degrees(np.sqrt(np.mean(e_psi**2))):11.2f}'
                  f'{np.sqrt(np.nanmean(dv**2)):9.3f}{np.nanmax(np.abs(off)):12.3f}{100.0 * np.mean(sat):8.1f}')
        plot([self.path_file], self.path_file.replace('.csv', '.png'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=SCENARIOS.keys())
    args = parser.parse_args()
    if args.scenario is None:
        sys.exit(f'軌道を選択してください: {", ".join(SCENARIOS)}')
    rclpy.init()
    node = CourseReplay(args.scenario)
    errors = node.check()
    if errors:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit('\n'.join(errors))
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

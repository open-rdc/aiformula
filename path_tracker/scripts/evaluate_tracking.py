#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TrackingEvaluator(Node):
    def __init__(self):
        super().__init__('tracking_evaluator')
        self.sub_path = self.create_subscription(
            Path, '/e2e_planner/path', self.on_path, 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/mpc/estimated_pose', self.on_pose, 10)
            
        self.reference_path = None # Format: [[x, y, yaw], ...]
        self.path_received = False
        
        # Logs
        self.time_history = []
        self.lateral_errors = []
        self.heading_errors = []
        self.robot_trajectory = [] # [x, y]
        
        self.start_time = None

    def on_path(self, msg):
        if self.path_received:
            return # Only take first path for static evaluation
            
        self.get_logger().info(f"Received Reference Path: {len(msg.poses)} points")
        path_data = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            yaw = quaternion_to_yaw(p.pose.orientation)
            path_data.append([x, y, yaw])
        self.reference_path = np.array(path_data)
        self.path_received = True

    def on_pose(self, msg):
        if not self.path_received:
            return
            
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = current_time - self.start_time
        
        rx = msg.pose.position.x
        ry = msg.pose.position.y
        ryaw = quaternion_to_yaw(msg.pose.orientation)
        
        self.robot_trajectory.append([rx, ry])
        
        # Find nearest point on reference path
        # 1. Calculate distances to all points
        dists = np.hypot(self.reference_path[:, 0] - rx, self.reference_path[:, 1] - ry)
        idx_min = np.argmin(dists)
        
        # 2. Get nearest point state
        px, py, pyaw = self.reference_path[idx_min]
        
        # 3. Calculate Errors
        # Lateral Error: Distance
        # To determine sign (left/right), transform robot pos to path frame or cross product
        # Vector Path->Robot: [rx-px, ry-py]
        # Path tangent vector: [cos(pyaw), sin(pyaw)]
        # Cross product 2D: dx*sy - dy*sx
        # Or simple distance if we don't care about sign, but sign is useful
        
        dx = rx - px
        dy = ry - py
        cross_prod = dx * math.sin(pyaw) - dy * math.cos(pyaw) # Positive if right? 
        # Actually standard Cross Product (Z) of (PathDir x RobotVec)
        # PathDir = (cos, sin)
        # RobotVec = (dx, dy)
        # Z = cos*dy - sin*dx
        # If Z > 0 (Left), Z < 0 (Right)
        
        lat_error = math.cos(pyaw) * dy - math.sin(pyaw) * dx
        
        # Heading Error
        head_error = ryaw - pyaw
        # Normalize
        while head_error > math.pi: head_error -= 2*math.pi
        while head_error < -math.pi: head_error += 2*math.pi
        
        self.time_history.append(t)
        self.lateral_errors.append(lat_error)
        self.heading_errors.append(head_error)
        
        # self.get_logger().info(f"LatErr: {lat_error:.3f}, HeadErr: {head_error:.3f}")

    def save_results(self):
        if not self.time_history:
            self.get_logger().info("No data to save.")
            return

        # Plot
        fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        
        # 1. Trajectories
        axs[0].plot(self.reference_path[:, 0], self.reference_path[:, 1], 'k--', label='Reference')
        traj = np.array(self.robot_trajectory)
        axs[0].plot(traj[:, 0], traj[:, 1], 'r-', label='Robot (Estimated)')
        axs[0].set_title('Trajectory comparison')
        axs[0].set_xlabel('X [m]')
        axs[0].set_ylabel('Y [m]')
        axs[0].legend()
        axs[0].axis('equal')
        
        # 2. Lateral Error
        axs[1].plot(self.time_history, self.lateral_errors, 'b-')
        axs[1].set_title('Lateral Error (Cross Track)')
        axs[1].set_ylabel('Error [m]')
        axs[1].set_xlabel('Time [s]')
        axs[1].grid(True)
        
        # 3. Heading Error
        axs[2].plot(self.time_history, np.degrees(self.heading_errors), 'g-')
        axs[2].set_title('Heading Error')
        axs[2].set_ylabel('Error [deg]')
        axs[2].set_xlabel('Time [s]')
        axs[2].grid(True)
        
        output_file = 'tracking_evaluation.png'
        plt.tight_layout()
        plt.savefig(output_file)
        self.get_logger().info(f"Saved plot to {output_file}")
        
        # Calculate RMSE
        rmse_lat = np.sqrt(np.mean(np.array(self.lateral_errors)**2))
        rmse_head = np.sqrt(np.mean(np.array(self.heading_errors)**2))
        print(f"RMSE Lateral Error: {rmse_lat:.4f} m")
        print(f"RMSE Heading Error: {np.degrees(rmse_head):.4f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

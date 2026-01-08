#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import sys
import math

class CsvPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')
        self.declare_parameter('csv_file', '')
        self.declare_parameter('topic_name', '/e2e_planner/path')
        self.declare_parameter('frame_id', 'base_link')

        csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        if not csv_file:
            self.get_logger().error("Please provide 'csv_file' parameter.")
            sys.exit(1)

        self.publisher_ = self.create_publisher(Path, topic_name, 10)
        self.timer = self.create_timer(1.0, self.publish_path) # Publish every 1s
        
        self.path_msg = self.load_path(csv_file)
        self.get_logger().info(f"Loaded {len(self.path_msg.poses)} points from {csv_file}")

    def load_path(self, filename):
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    
                    x = float(row['x'])
                    y = float(row['y'])
                    yaw = float(row['yaw'])
                    
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    
                    # Yaw to Quaternion
                    # Manual calculation to avoid dependency if transforms3d missing, 
                    # but usually transforms3d is nicer. 
                    # Since we removed dependency in extract script, let's allow it here if present, else manual.
                    # Or just use manual to be safe since user environment seems strictly minimal.
                     
                    cy = math.cos(yaw * 0.5)
                    sy = math.sin(yaw * 0.5)
                    # cr = 1, cp = 1 (roll, pitch = 0)
                    # q = [w, x, y, z] order for many libs, ROS uses x,y,z,w
                    
                    pose.pose.orientation.w = cy
                    pose.pose.orientation.z = sy
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    
                    path_msg.poses.append(pose)
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV: {e}")
            
        return path_msg

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.path_msg.poses:
            p.header.stamp = self.path_msg.header.stamp
        self.publisher_.publish(self.path_msg)
        # self.get_logger().info("Published path")

def main(args=None):
    rclpy.init(args=args)
    node = CsvPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

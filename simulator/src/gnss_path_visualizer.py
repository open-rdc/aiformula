#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
import csv
import math
from pyproj import Transformer


class GNSSPathVisualizer(Node):
    def __init__(self):
        super().__init__('gnss_path_visualizer')
        
        self.declare_parameter('csv_file', 'shihou_gnssnav.csv')
        csv_file = self.get_parameter('csv_file').value
        
        self.path_pub = self.create_publisher(Path, 'gnss_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'gnss_path_markers', 10)
        
        self.origin_lat = 36.11339585571819
        self.origin_lon = 139.9815010597697
        
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:4326", always_xy=True)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        
        self.marker_array = MarkerArray()
        
        self.load_csv_path(csv_file)
        
        self.timer = self.create_timer(1.0, self.publish_path)
        
    def lat_lon_to_xy(self, lat, lon):
        EARTH_RADIUS = 6371000.0
        
        lat_diff = math.radians(lat - self.origin_lat)
        lon_diff = math.radians(lon - self.origin_lon)
        
        x = EARTH_RADIUS * lon_diff * math.cos(math.radians(self.origin_lat))
        y = EARTH_RADIUS * lat_diff
        
        return x, y
    
    def load_csv_path(self, csv_file):
        try:
            with open(csv_file, 'r') as file:
                csv_reader = csv.DictReader(file)
                
                points = []
                for row in csv_reader:
                    lat = float(row['Latitude'])
                    lon = float(row['Longitude'])
                    
                    x, y = self.lat_lon_to_xy(lat, lon)
                    
                    pose = PoseStamped()
                    pose.header.frame_id = 'world'
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.5
                    pose.pose.orientation.w = 1.0
                    
                    self.path_msg.poses.append(pose)
                    points.append((x, y))
                
                # Create line strip marker
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.id = 0
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.1  # Line width
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                
                for x, y in points:
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = 0.5
                    marker.points.append(point)
                
                self.marker_array.markers.append(marker)
                    
            self.get_logger().info(f'Loaded {len(self.path_msg.poses)} GNSS points from {csv_file}')
            
        except FileNotFoundError:
            self.get_logger().error(f'CSV file {csv_file} not found!')
        except Exception as e:
            self.get_logger().error(f'Error loading CSV: {str(e)}')
    
    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        
        # Update marker timestamp
        for marker in self.marker_array.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        
        self.marker_pub.publish(self.marker_array)
        

def main(args=None):
    rclpy.init(args=args)
    node = GNSSPathVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
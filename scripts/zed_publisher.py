#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: pyzed SDK not found. Please install ZED SDK and Python API.")
    sl = None

class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        
        self.declare_parameter('fps', 30)
        self.declare_parameter('resolution', 'HD720')
        
        fps = self.get_parameter('fps').value
        res_str = self.get_parameter('resolution').value
        
        if sl:
            self.camera = sl.Camera()
            init_params = sl.InitParameters()
            
            if res_str == 'HD2K':
                init_params.camera_resolution = sl.RESOLUTION.HD2K
            elif res_str == 'HD1080':
                init_params.camera_resolution = sl.RESOLUTION.HD1080
            elif res_str == 'HD720':
                init_params.camera_resolution = sl.RESOLUTION.HD720
            elif res_str == 'VGA':
                init_params.camera_resolution = sl.RESOLUTION.VGA
            else:
                self.get_logger().warn(f"Unknown resolution {res_str}, defaulting to HD720")
                init_params.camera_resolution = sl.RESOLUTION.HD720
            
            init_params.camera_fps = fps
            
            err = self.camera.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open ZED camera: {err}")
                self.camera = None
            else:
                self.get_logger().info(f"ZED camera opened with resolution {res_str} at {fps} FPS")
                self.image_sl = sl.Mat()
                self.runtime_params = sl.RuntimeParameters()
        else:
            self.camera = None
            self.get_logger().error("ZED SDK not available.")

        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.camera and self.camera.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.camera.retrieve_image(self.image_sl, sl.VIEW.LEFT)
            # Get numpy array (RGBA)
            image_rgba = self.image_sl.get_data()
            # Convert to BGR for ROS image_raw compatibility
            image_bgr = cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2BGR)
            
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'zed_left_camera_frame'
            
            self.publisher_.publish(msg)
        elif not self.camera:
            # For testing/placeholder if camera is not connected
            # self.get_logger().warn("Camera not initialized.")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ZedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.camera:
            node.camera.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

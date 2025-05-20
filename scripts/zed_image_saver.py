import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.cv_image = None
        self.save_directory = '/home/nvidia/image'
        os.makedirs(self.save_directory, exist_ok=True)
        self.image_counter = self.get_next_image_index()
        
        self.get_logger().info('ImageSaver Node Initialized. Press Enter to save an image.')
        
        # 別スレッドでキー入力を監視
        self.thread = threading.Thread(target=self.key_listener, daemon=True)
        self.thread.start()

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def key_listener(self):
        while True:
            input()  # Enterキーを待機
            self.save_image()

    def save_image(self):
        if self.cv_image is not None:
            while True:
                image_path = os.path.join(self.save_directory, f'image_{self.image_counter:04d}.png')
                if not os.path.exists(image_path):
                    break
                self.image_counter += 1
            cv2.imwrite(image_path, self.cv_image)
            self.get_logger().info(f'Saved: {image_path}')
            self.image_counter += 1
        else:
            self.get_logger().warn('No image received yet.')
    
    def get_next_image_index(self):
        existing_files = [f for f in os.listdir(self.save_directory) if f.startswith('image_') and f.endswith('.png')]
        existing_indices = [int(f[6:10]) for f in existing_files if f[6:10].isdigit()]
        return max(existing_indices) + 1 if existing_indices else 0

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

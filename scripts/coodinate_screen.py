import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthPixelReader(Node):
    def __init__(self):
        super().__init__('depth_pixel_reader')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.x = None
        self.y = None
        self.fov_x = 110.0  # 水平方向の視野角 (度)
        self.image_width = 480
        self.image_height = 300
        self.camera_height = 0.6  # カメラの地面からの高さ [m]
        self.scale_factor = (2 * np.tan(np.radians(self.fov_x / 2))) / self.image_width
        cv2.namedWindow("Depth Image")
        cv2.setMouseCallback("Depth Image", self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x, self.y = x, y
            self.get_logger().info(f'ピクセル選択: ({self.x}, {self.y})')

    def process_key_input(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('w'):
            self.y = max(0, self.y - 1)
        elif key == ord('s'):
            self.y = min(self.image_height - 1, self.y + 1)
        elif key == ord('a'):
            self.x = max(0, self.x - 1)
        elif key == ord('d'):
            self.x = min(self.image_width - 1, self.x + 1)

    def screen_to_camera(self, x, y, depth):
        u, v = x, y
        w, h = self.image_width, self.image_height

        x_d = (u - (w/2))/733.33
        y_d = (v - (h/2))/733.33
        
        x_cam = depth * x_d
        y_cam = depth * y_d - self.camera_height
        z_cam = depth
        
        return x_cam, y_cam - self.camera_height, z_cam

    def image_callback(self, msg):
        if self.x is None or self.y is None:
            try:
                self.x = int(input("X座標を入力してください: "))
                self.y = int(input("Y座標を入力してください: "))
            except ValueError:
                self.get_logger().error("無効な入力です。整数を入力してください。")
                return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if 0 <= self.x < depth_image.shape[1] and 0 <= self.y < depth_image.shape[0]:
                depth_value = depth_image[self.y, self.x]
                x_cam, y_cam, z_cam = self.screen_to_camera(self.x, self.y, depth_value)
                self.get_logger().info(f'カメラ座標系: X={x_cam:.3f}, Y={y_cam:.3f}, Z={z_cam:.3f} (m)')
                
                depth_viz = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
                cv2.circle(depth_viz, (self.x, self.y), 5, (0, 0, 255), -1)
                cv2.imshow("Depth Image", depth_viz)
                self.process_key_input()
            else:
                self.get_logger().warn('指定されたピクセル座標が画像の範囲外です')
        except Exception as e:
            self.get_logger().error(f'画像処理中にエラー発生: {e}')


def main():
    rclpy.init()
    node = DepthPixelReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

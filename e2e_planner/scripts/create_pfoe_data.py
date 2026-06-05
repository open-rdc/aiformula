import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import csv
from pathlib import Path
from typing import Optional
from rclpy.qos import qos_profile_sensor_data
from feature_extractor import ImageFeatureExtractor

try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

SAMPLE_INTERVAL = 0.1

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')

        self.declare_parameter('sdk_flag', True)
        self.sdk_flag_ = self.get_parameter('sdk_flag').value

        self.bridge = CvBridge()
        self.latest_image = None

        self.zed_camera: Optional[sl.Camera] = None
        self.zed_image: Optional[sl.Mat] = None
        self.zed_runtime_params: Optional[sl.RuntimeParameters] = None

        self.feature_extractor = ImageFeatureExtractor()
        self.feature_extractor.eval()

        self.is_paused = True
        self.prev_button_state = 0
        self.joy_value = None
        self.last_sample_time = None

        package_root = Path(__file__).parent.parent
        data_base_dir = package_root / 'data'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        dataset_dir = data_base_dir / f'{timestamp}_dataset'
        dataset_dir.mkdir(parents=True, exist_ok=True)
        self.bin_path = dataset_dir / 'data.bin'
        self.bin_file = open(str(self.bin_path), 'wb')
        self.row_count = 0
        self.log_path = dataset_dir / 'log.csv'
        self.log_file = open(str(self.log_path), 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['row', 'joy_value'])

        if self.sdk_flag_:
            if not ZED_SDK_AVAILABLE:
                self.get_logger().error('ZED SDK not available. Install pyzed package.')
                raise RuntimeError('ZED SDK not available')
            self._initialize_zed_camera()
        else:
            self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile_sensor_data)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_timer(0.1, self.timer_callback)

    def _initialize_zed_camera(self)->None:
        self.zed_camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA  # 解像度をSVGA(800x600)に設定
        init_params.camera_fps = 30

        err = self.zed_camera.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'ZEDカメラの起動に失敗しました: {err}')
            raise RuntimeError(f'ZEDカメラの起動に失敗しました: {err}')
        
        self.zed_image = sl.Mat()

        self.get_logger().info('ZEDカメラの初期化')

    def _capture_data_from_zed(self):
        # 1. シャッターを切る（※1）
        if self.zed_camera.grab() != sl.ERROR_CODE.SUCCESS:
            return None

        self.zed_camera.retrieve_image(self.zed_image, sl.VIEW.LEFT)
        image = self.zed_image.get_data()
        resized_image = cv2.resize(image, (224, 224))
        final_image = cv2.cvtColor(resized_image, cv2.COLOR_BGRA2RGB)
        
        return final_image

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg
    
    def joy_callback(self, msg: Joy) -> None:
        if len(msg.buttons) > 2:
            current_button_state = msg.buttons[2]
            if current_button_state == 1 and self.prev_button_state == 0:
                self.is_paused = not self.is_paused
                if self.is_paused:
                    self.get_logger().info('⏸️ Data collection paused')
                else:
                    self.last_sample_time = None
                    self.get_logger().info('▶️ Data collection resumed')
            self.prev_button_state = current_button_state

        if len(msg.axes) > 4:
            left_stick = msg.axes[0]
            right_stick = msg.axes[4]

            self.joy_value = np.arctan2(right_stick, left_stick)

    def timer_callback(self)->None:
        if self.is_paused:
            return
        
        current_time = time.time()

        if self.sdk_flag_:
            image = self._capture_data_from_zed()
            if image is None or self.joy_value is None:
                return
            
            if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                get_image_tensor = self.feature_extractor.extract(image).astype(np.float32)
                row = np.concatenate([
                    get_image_tensor,
                    np.array([self.joy_value], dtype=np.float32)
                ])
                self.bin_file.write(row.tobytes())
                self.log_writer.writerow([self.row_count, self.joy_value])
                self.row_count += 1
                self.last_sample_time = current_time

        else:
            if self.latest_image is None or self.joy_value is None:
                return
            if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='rgb8')
                get_image_tensor = self.feature_extractor.extract(cv_image).astype(np.float32)
                row = np.concatenate([
                    get_image_tensor,
                    np.array([self.joy_value], dtype=np.float32)
                ])
                self.bin_file.write(row.tobytes())
                self.log_writer.writerow([self.row_count, self.joy_value])
                self.row_count += 1
                self.last_sample_time = current_time

    def save_data(self):
        self.bin_file.flush()
        self.bin_file.close()
        self.log_file.flush()
        self.log_file.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataCollectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
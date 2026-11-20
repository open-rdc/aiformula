"""YOLOX の ONNX でパイロンと動的障害物のマスクを出す (/perception/pylon_mask)。

実機は object_detection の pylon_detector (C++、TensorRT) が同じトピックを出す。
TensorRT の無い PC (シミュレーション) ではそれがビルドされないので、代わりにこれを起動する
(e2e_planner.launch.py の pylon_onnx: auto が、pylon_detector が無いときだけ起動する)。

検出と塗り方は、学習データのマスクを作った util.pylon_mask (YoloxOnnx / draw_pylon_mask) をそのまま使う。
util.pylon_mask は C++ 側 (yolox_tensorrt.cpp / pylon_mask.cpp) を写したもの。
"""

import os
import threading
from typing import Optional

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from util.pylon_mask import YoloxOnnx, draw_pylon_mask


def default_onnx_path() -> str:
    try:
        share = get_package_share_directory('object_detection')
    except PackageNotFoundError:
        return ''
    return os.path.join(share, 'weights', 'yolox_s_aiformula.onnx')


class PylonMaskOnnxNode(Node):
    def __init__(self) -> None:
        super().__init__('pylon_mask_onnx_node')
        self.declare_parameter('onnx_path', default_onnx_path())
        # YOLOX は CPU で回すので、届いた画像を全部ではなく最新の 1 枚をこの周期で処理する
        self.declare_parameter('interval_ms', 100)

        onnx_path = self.get_parameter('onnx_path').value
        if not onnx_path or not os.path.exists(onnx_path):
            raise FileNotFoundError(
                f'YOLOX の onnx が無い: {onnx_path!r}。'
                'perception/object_detection/weights/yolox_s_aiformula.onnx を置いてビルドすること')
        self.detector = YoloxOnnx(onnx_path)
        self.get_logger().info(f'YOLOX (onnx, CPU) を読み込みました: {onnx_path}')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image_msg: Optional[Image] = None
        self.last_stamp = None

        self.publisher = self.create_publisher(Image, '/perception/pylon_mask', qos_profile_sensor_data)
        self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback,
                                 qos_profile_sensor_data)
        self.create_timer(self.get_parameter('interval_ms').value / 1000.0, self.timer_callback)

    def image_callback(self, msg: Image) -> None:
        with self.lock:
            self.image_msg = msg

    def timer_callback(self) -> None:
        with self.lock:
            msg = self.image_msg
        if msg is None or msg.header.stamp == self.last_stamp:
            return
        self.last_stamp = msg.header.stamp

        # C++ の pylon_detector と同じく BGR8 にしてから検出し、画像と同じ大きさのマスクを出す
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        mask = draw_pylon_mask(self.detector.detect(bgr), bgr.shape[0], bgr.shape[1])
        out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        out.header = msg.header
        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PylonMaskOnnxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

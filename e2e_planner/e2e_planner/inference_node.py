import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistWithCovarianceStamped
from cv_bridge import CvBridge
import torch
import numpy as np
import os
import threading
from pathlib import Path as FilePath
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from typing import List, Optional, Tuple

from util.yolop_processor import YOLOPv2Processor
from util.pylon_mask import is_pylon_near
from util.preprocess import (
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    PAST_MASK_MODES,
    denormalize_waypoints,
    draw_boxes_on_rgb,
    make_past_masks,
    preprocess_mask,
    preprocess_rgb,
    resize_for_debug,
    to_bgr,
)
from .frame_history import DistanceFrameHistory, Odometer
from .pylon_input import fresh_pylon_mask
from .speed_limit import SpeedLimiter


def parse_distances(text: str) -> List[float]:
    """'1.5,3.0' -> [1.5, 3.0]。空なら []（過去フレームを使わない重み）"""
    return [float(v) for v in str(text).replace(' ', '').split(',') if v]


class InferenceNode(Node):
    """deploy の zed_wrapper の画像と pylon_detector のパイロンマスクから経路と速度上限を出す。

    経路は vision_lane_planner の代わりに /planner/vision_path に出し、その先は deploy の
    path_smoother -> local_planner -> speed_path_planner -> controller がそのまま使う
    """

    def __init__(self) -> None:
        super().__init__('inference_node')

        self.declare_parameter('model_name', 'model.pt')
        self.declare_parameter('interval_ms', 100)
        # 重みの入力に合わせる。2 = 白線マスク + パイロンマスク、1 = 白線マスクだけ（rgb_mask の重み）
        self.declare_parameter('num_mask_channels', 2)
        self.declare_parameter('publish_topic', '/planner/vision_path')
        self.declare_parameter('speed_limit_mps', 4.0)
        self.declare_parameter('turn_threshold_deg', 15.0)
        self.declare_parameter('speed_limit_hold_s', 1.0)
        # 実機は cuda。GPU の無い PC で bag を流して確かめるときだけ cpu にする
        self.declare_parameter('device', 'cuda')
        # 過去フレーム (fusion: early の重み)。学習設定の frame_stack_distances / past_mask_distances /
        # past_mask_mode と同じ値にする。'1.5,3.0' のように m をカンマ区切りで。空なら今のフレームだけ
        self.declare_parameter('frame_stack_distances', '')
        self.declare_parameter('past_mask_distances', '')
        self.declare_parameter('past_mask_mode', 'merged')
        # 学習設定の pylon_bbox_on_rgb。true の重みは RGB にパイロンの枠線 (マゼンタ) を描いて学習している
        self.declare_parameter('pylon_bbox_on_rgb', False)
        # 過去フレームを貯める間隔 [m]。学習データのフレーム間隔 (2 m/s で約 0.5 m) より細かくする
        self.declare_parameter('history_spacing_m', 0.25)
        self.declare_parameter('velocity_topic', '/vectornav/velocity_body')

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value
        self.num_mask_channels = int(self.get_parameter('num_mask_channels').value)
        self.frame_stack_distances = parse_distances(self.get_parameter('frame_stack_distances').value)
        self.past_mask_distances = parse_distances(self.get_parameter('past_mask_distances').value)
        self.past_mask_mode = str(self.get_parameter('past_mask_mode').value)
        self.pylon_bbox_on_rgb = bool(self.get_parameter('pylon_bbox_on_rgb').value)
        if self.past_mask_mode not in PAST_MASK_MODES:
            raise ValueError(f'past_mask_mode は {PAST_MASK_MODES} のどれか: {self.past_mask_mode}')
        self.use_history = bool(self.frame_stack_distances or self.past_mask_distances)
        self.history = DistanceFrameHistory(self.frame_stack_distances + self.past_mask_distances,
                                            float(self.get_parameter('history_spacing_m').value))
        self.odometer = Odometer()
        self.history_lock = threading.Lock()
        self.warned_no_velocity = False

        self.bridge = CvBridge()
        self.device = torch.device(self.get_parameter('device').value)

        self.lock = threading.Lock()
        self.image_msg: Optional[Image] = None
        self.pylon_mask: Optional[np.ndarray] = None
        self.pylon_mask_time = None
        self.warned_no_pylon_mask = False

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        yolop_weight_path = FilePath(package_share_directory) / 'weights' / 'yolopv2.pt'

        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
            self.check_model_input()
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None

        self.yolop_processor = YOLOPv2Processor(yolop_weight_path, self.device)
        self.speed_limiter = SpeedLimiter(
            self.get_parameter('speed_limit_mps').value,
            self.get_parameter('turn_threshold_deg').value,
            self.get_parameter('speed_limit_hold_s').value)

        self.pub = self.create_publisher(Path, self.get_parameter('publish_topic').value, qos_profile_system_default)
        self.pub_speed_limit = self.create_publisher(Float32, '/planning/speed_limit', qos_profile_system_default)
        self.pub_debug_image = self.create_publisher(Image, 'e2e_planner/debug_image', qos_profile_system_default)

        self.subscription_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback,
                                 qos_profile_sensor_data, callback_group=self.subscription_cb_group)
        self.create_subscription(Image, '/perception/pylon_mask', self.pylon_mask_callback,
                                 qos_profile_sensor_data, callback_group=self.subscription_cb_group)
        if self.use_history:
            self.create_subscription(TwistWithCovarianceStamped, self.get_parameter('velocity_topic').value,
                                     self.velocity_callback, qos_profile_sensor_data,
                                     callback_group=self.subscription_cb_group)
            self.get_logger().info(
                f'過去フレームを使う: RGB {self.frame_stack_distances} m 前、'
                f'マスク {self.past_mask_distances} m 前 ({self.past_mask_mode})、'
                f'{self.history.spacing_m} m ごとに貯める')

        self.torch_timer = self.create_timer(
            interval_ms / 1000.0,
            self.torch_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def check_model_input(self) -> None:
        """重みの入力 ch と、パラメータから組み立てる入力 ch が合うかを起動時に確かめる。
        合わないと推論のたびに落ちるので、ここで分かる言葉で止める"""
        past_ch = len(self.past_mask_distances) * (2 if self.past_mask_mode == 'separate' else 1)
        mask_ch = self.num_mask_channels + past_ch
        rgb_ch = 3 * (1 + len(self.frame_stack_distances))
        mask = torch.zeros(1, mask_ch, IMAGE_HEIGHT, IMAGE_WIDTH, device=self.device)
        rgb = torch.zeros(1, rgb_ch, IMAGE_HEIGHT, IMAGE_WIDTH, device=self.device)
        try:
            with torch.no_grad():
                self.model(mask, rgb)
        except RuntimeError as error:
            raise RuntimeError(
                f'重みの入力とパラメータが合わない: マスク {mask_ch} ch (今 {self.num_mask_channels} + 過去 {past_ch}) + '
                f'RGB {rgb_ch} ch (今 + 過去 {len(self.frame_stack_distances)} 枚) で組み立てたが、重みが受け付けない。'
                'num_mask_channels / frame_stack_distances / past_mask_distances / past_mask_mode を '
                f'学習設定と同じにすること。元のエラー: {error}') from error
        self.get_logger().info(f'重みの入力: マスク {mask_ch} ch + RGB {rgb_ch} ch')

    def image_callback(self, msg: Image) -> None:
        with self.lock:
            self.image_msg = msg

    def velocity_callback(self, msg: TwistWithCovarianceStamped) -> None:
        linear = msg.twist.twist.linear
        speed = float(np.hypot(linear.x, linear.y))
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            self.odometer.update(speed, now_s)

    def pylon_mask_callback(self, msg: Image) -> None:
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        with self.lock:
            self.pylon_mask = mask
            self.pylon_mask_time = self.get_clock().now()

    def current_pylon_mask(self) -> np.ndarray:
        with self.lock:
            mask, received = self.pylon_mask, self.pylon_mask_time
        age_s = None if received is None else (self.get_clock().now() - received).nanoseconds * 1e-9
        selected = fresh_pylon_mask(mask, age_s)
        if (mask is None or age_s is None or age_s > 0.5) and not self.warned_no_pylon_mask:
            self.get_logger().warn('/perception/pylon_mask が届いていないので、パイロン無しとして推論します')
            self.warned_no_pylon_mask = True
        return selected

    def preprocess_image(self, image: np.ndarray, pylon_mask: np.ndarray) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray, np.ndarray]:
        """白線マスク（とパイロンマスク）と RGB の 2 入力を作る。学習時 (train_curve_weight.py) と同じ
        「元解像度(640x360)でマスク生成 -> 256x144 リサイズ」の順序に揃えている。

        過去フレームを使う重みでは、学習と同じ順に足す:
        マスク = [今の白線, 今のパイロン, 過去マスク (past_mask_distances の順)]、
        RGB = [今, frame_stack_distances の順の過去]"""
        bgr_image = to_bgr(image)
        height, width = bgr_image.shape[:2]

        mask_full = self.yolop_processor.process_image(bgr_image, (width, height))
        pylon_binary = (pylon_mask > 127).astype(np.uint8)

        mask_channels = [preprocess_mask(mask_full)]
        if self.num_mask_channels == 2:
            mask_channels.append(preprocess_mask(pylon_binary))
        # 枠線は今のフレームに描いてから前処理する。過去フレームはこの結果を貯めて使うので、
        # 学習と同じく各フレームにそのときの枠が描かれる
        rgb_source = draw_boxes_on_rgb(bgr_image, pylon_binary) if self.pylon_bbox_on_rgb else bgr_image
        rgb_input = preprocess_rgb(rgb_source)

        if self.use_history:
            # 今のフレームの前処理結果を、そのまま後で過去フレームとして使い回す
            past_mask_now = (np.concatenate([preprocess_mask(m) for m in
                                             make_past_masks(mask_full, pylon_binary, self.past_mask_mode)], axis=0)
                             if self.past_mask_distances else None)
            past_rgb, past_masks = self.past_inputs(rgb_input, past_mask_now)
            rgb_input = np.concatenate([rgb_input] + past_rgb, axis=0)
            mask_channels.extend(past_masks)
        mask_input = np.concatenate(mask_channels, axis=0)

        mask_tensor = torch.from_numpy(mask_input).unsqueeze(0).to(self.device)
        rgb_tensor = torch.from_numpy(rgb_input).unsqueeze(0).to(self.device)

        # デバッグ表示は重みに入れたのと同じ枠線入りの画像にする
        return mask_tensor, rgb_tensor, mask_input, rgb_source

    def past_inputs(self, rgb_now: np.ndarray, past_mask_now: Optional[np.ndarray]) -> Tuple[list, list]:
        """過去の RGB と過去マスクを、走った距離でバッファから引く。

        速度が届いていない (走った距離が分からない) ときは過去の ch を 0 にする。
        学習の history_dropout (過去の ch を 0 にする) と同じ形なので、重みはこの入力を知っている"""
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            fresh = self.odometer.is_fresh(now_s)
            arc_m = self.odometer.arc_m
        if not fresh:
            if not self.warned_no_velocity:
                self.get_logger().warn(
                    f'{self.get_parameter("velocity_topic").value} が届いていないので、過去フレームを 0 にして推論します')
                self.warned_no_velocity = True
            past_rgb = [np.zeros_like(rgb_now) for _ in self.frame_stack_distances]
            past_masks = [np.zeros_like(past_mask_now) for _ in self.past_mask_distances]
            return past_rgb, past_masks
        if self.warned_no_velocity:
            self.get_logger().info('速度が届いたので、過去フレームを使います')
            self.warned_no_velocity = False

        frame = (rgb_now, past_mask_now)
        # torch_callback は ReentrantCallbackGroup なので、推論が周期より遅いと 2 本同時に来うる
        with self.history_lock:
            self.history.push(arc_m, frame)
            past_rgb = [f[0] for f in self.history.lookup(arc_m, frame, self.frame_stack_distances)]
            past_masks = [f[1] for f in self.history.lookup(arc_m, frame, self.past_mask_distances)]
            buffered = len(self.history)
        self.get_logger().info(f'走行距離 {arc_m:.1f} m、過去フレーム {buffered} 枚',
                               throttle_duration_sec=5.0)
        return past_rgb, past_masks

    def torch_callback(self) -> None:
        with self.lock:
            image_msg = self.image_msg
        if image_msg is None or self.model is None:
            return

        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        pylon_mask = self.current_pylon_mask()
        mask_tensor, rgb_tensor, mask_input, bgr_image = self.preprocess_image(image, pylon_mask)

        # path_smoother は時刻つきで tf を引くので、画像を撮った時刻を付ける
        header = image_msg.header
        header.frame_id = 'base_link'

        if self.pub_debug_image.get_subscription_count() > 0:
            debug_image = resize_for_debug(bgr_image)
            debug_image[mask_input[0] == 1] = [0, 0, 255]
            # パイロンのマスク ch (枠を塗ったもの) は上塗りしない。RGB に描いたマゼンタの枠線が
            # 塗りで隠れ、重みに入れた画像と見た目が変わるため
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.pub_debug_image.publish(debug_msg)

        with torch.no_grad():
            output = self.model(mask_tensor, rgb_tensor)

        waypoints = denormalize_waypoints(output.cpu().numpy().flatten()).reshape(-1, 2)
        self.pub.publish(self.create_path(waypoints, header))

        now_s = self.get_clock().now().nanoseconds * 1e-9
        limit = self.speed_limiter.update(waypoints, is_pylon_near(pylon_mask), now_s)
        if limit is not None:
            self.pub_speed_limit.publish(Float32(data=float(limit)))

    @staticmethod
    def create_path(waypoints: np.ndarray, header) -> Path:
        path_msg = Path()
        path_msg.header = header
        path_msg.poses.append(PoseStamped(header=header, pose=Pose(position=Point(x=0.0, y=0.0))))
        path_msg.poses.extend(PoseStamped(header=header, pose=Pose(position=Point(x=float(x), y=float(y)))) for x, y in waypoints)
        return path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

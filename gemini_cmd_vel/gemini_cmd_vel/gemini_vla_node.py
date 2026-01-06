import math
import os
import threading
import time
from collections import deque
from typing import Optional, List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, PoseStamped, Point
from google import genai
from google.genai import types
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class GeminiVLANode(Node):
    """カメラ画像を使ってGeminiに問い合わせ、軌道に基づいて /cmd_vel コマンドを生成するROS2ノード。"""

    def __init__(self) -> None:
        super().__init__('gemini_vla_node')

        default_api_key = os.environ.get('GEMINI_API_KEY', 'AIzaSyACSzAAldqST8jdvexMxES6ZDefGykdiaY')
        default_model = os.environ.get('GEMINI_MODEL', 'gemini-robotics-er-1.5-preview')

        self.declare_parameter('api_key', default_api_key)
        self.declare_parameter('model', default_model)
        self.declare_parameter('subscribe_topic', '/image_raw')
        self.declare_parameter('publish_topic', '/cmd_vel')
        self.declare_parameter('request_interval_sec', 6.1)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('control_period_sec', 0.1)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('failsafe_timeout_sec', 20.0)
        self.declare_parameter('angular_gain', 20)
        self.declare_parameter(
            'base_prompt',
            'You are a robot navigation assistant. '
            'Analyze the image and generate a driving trajectory for the robot. '
            'Look far ahead (at least 10 meters) and follow the curvature of the white lane lines. '
            'The trajectory should be smooth and keep the robot centered in the lane. '
            'Output a JSON object with a "trajectory" key, which is a list of [x, y] coordinates relative to the robot. '
            'x is forward (meters), y is left (meters). '
            'Provide 5-10 points spaced evenly (e.g., every 1-2 meters) starting from near the robot to far ahead. '
            'Example: {"trajectory": [[1.0, 0.0], [3.0, 0.1], [5.0, 0.3], [7.0, 0.8], [9.0, 1.5]]}'
        )

        self.api_key = self.get_parameter('api_key').value
        self.model = self.get_parameter('model').value
        self.subscribe_topic = self.get_parameter('subscribe_topic').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.request_interval = float(self.get_parameter('request_interval_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.control_period = float(self.get_parameter('control_period_sec').value)
        self.stop_on_failure = bool(self.get_parameter('stop_on_failure').value)
        self.failsafe_timeout = float(self.get_parameter('failsafe_timeout_sec').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.base_prompt = self.get_parameter('base_prompt').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, self.publish_topic, 10)
        self.path_publisher = self.create_publisher(Path, '~/trajectory', 10)
        self.image_publisher = self.create_publisher(Image, '~/annotated_image', 10)
        
        self.subscription = self.create_subscription(
            Image, self.subscribe_topic, self._image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self._camera_info_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        
        self.timer = self.create_timer(self.request_interval, self._timer_callback)
        
        failsafe_period = max(0.5, self.failsafe_timeout / 2.0) if self.stop_on_failure else None
        self.failsafe_timer = (
            self.create_timer(failsafe_period, self._failsafe_check)
            if failsafe_period
            else None
        )
        
        self.control_timer = self.create_timer(self.control_period, self._control_loop)

        self.add_on_set_parameters_callback(self._on_parameters_set)

        self._latest_image_jpeg: Optional[bytes] = None
        self._latest_cv_image: Optional[np.ndarray] = None
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._current_speed_linear = 0.0
        self._current_speed_angular = 0.0
        self._crop_offset_y = 0
        
        self._lock = threading.Lock()
        self._processing = False
        self._key_warning_sent = False
        
        self._current_trajectory: List[Tuple[float, float]] = []
        self._trajectory_start_time = self.get_clock().now()
        self._last_trajectory_update_time = self.get_clock().now()

        # Geminiクライアントの初期化
        self.client: Optional[genai.Client] = None
        if self.api_key:
            self.client = genai.Client(api_key=self.api_key)

        self.get_logger().info(
            f'Started Gemini VLA controller. Model={self.model} '
            f'subscription={self.subscribe_topic} publication={self.publish_topic}'
        )

    def _on_parameters_set(self, params):
        for param in params:
            if param.name == 'api_key':
                self.api_key = param.value
                if self.api_key:
                    self.client = genai.Client(api_key=self.api_key)
                self._key_warning_sent = False
            elif param.name == 'model':
                self.model = param.value
            elif param.name == 'request_interval_sec':
                self.request_interval = float(param.value)
                if self.timer:
                    self.timer.cancel()
                    self.destroy_timer(self.timer)
                self.timer = self.create_timer(self.request_interval, self._timer_callback)
            elif param.name == 'max_linear_speed':
                self.max_linear_speed = float(param.value)
            elif param.name == 'angular_gain':
                self.angular_gain = float(param.value)
            elif param.name == 'base_prompt':
                self.base_prompt = param.value
        return SetParametersResult(successful=True)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        with self._lock:
            self._camera_matrix = np.array(msg.k).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d)

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_speed_linear = msg.twist.twist.linear.x
        self._current_speed_angular = msg.twist.twist.angular.z

    def _image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 画像の上半分を切り捨て、下半分を使用（道路面などの重要な情報が含まれるため）
            height, width = cv_image.shape[:2]
            crop_y = height // 2
            cropped_image = cv_image[crop_y:, :]
            
            success, buffer = cv2.imencode('.jpg', cropped_image)
            if not success:
                self.get_logger().warning('Failed to JPEG-encode incoming frame.')
                return
            with self._lock:
                self._latest_image_jpeg = buffer.tobytes()
                self._latest_cv_image = cropped_image
                self._crop_offset_y = crop_y
        except CvBridgeError as exc:
            self.get_logger().warning(f'CvBridge conversion failed: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Unexpected image processing error: {exc}')

    def _timer_callback(self) -> None:
        with self._lock:
            if self._processing or self._latest_image_jpeg is None:
                return
            image_bytes = self._latest_image_jpeg
            self._processing = True

        thread = threading.Thread(
            target=self._invoke_gemini, args=(image_bytes,), daemon=True
        )
        thread.start()

    def _invoke_gemini(self, image_bytes: bytes) -> None:
        try:
            if not self.client:
                self._warn_once('Gemini API key is empty. Set the api_key parameter.')
                return

            response = self.client.models.generate_content(
                model=self.model,
                contents=[
                    types.Content(
                        role="user",
                        parts=[
                            types.Part(text=self.base_prompt),
                            types.Part(
                                inline_data=types.Blob(
                                    mime_type='image/jpeg',
                                    data=image_bytes
                                )
                            )
                        ]
                    )
                ],
                config=types.GenerateContentConfig(
                    response_mime_type='application/json',
                    temperature=0.2,
                )
            )

            if not response.text:
                self.get_logger().warning('Empty response from Gemini.')
                return

            import json
            try:
                data = json.loads(response.text)
                trajectory = data.get('trajectory', [])
                if trajectory:
                    with self._lock:
                        self._current_trajectory = [(float(p[0]), float(p[1])) for p in trajectory]
                        self._trajectory_start_time = self.get_clock().now()
                        self._last_trajectory_update_time = self.get_clock().now()
                        current_cv_image = self._latest_cv_image.copy() if self._latest_cv_image is not None else None
                        camera_matrix = self._camera_matrix
                        dist_coeffs = self._dist_coeffs
                        crop_offset_y = self._crop_offset_y
                    
                    self.get_logger().info(f'Received trajectory with {len(trajectory)} points.')
                    
                    # Publish Path
                    self._publish_path(self._current_trajectory)
                    
                    # Publish Annotated Image
                    if current_cv_image is not None and camera_matrix is not None:
                        self._publish_annotated_image(current_cv_image, self._current_trajectory, camera_matrix, dist_coeffs, crop_offset_y)
                        
                else:
                    self.get_logger().warning('No trajectory found in response.')
            except json.JSONDecodeError:
                self.get_logger().error(f'Failed to parse JSON: {response.text}')

        except Exception as exc:
            self.get_logger().error(f'Gemini request failed: {exc}')
        finally:
            with self._lock:
                self._processing = False

    def _publish_path(self, trajectory: List[Tuple[float, float]]) -> None:
        path_msg = Path()
        path_msg.header.frame_id = 'base_link' # 軌道はロボット座標系（相対座標）
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # 単位クォータニオン（回転なし）
            path_msg.poses.append(pose)
            
        self.path_publisher.publish(path_msg)

    def _publish_annotated_image(self, image: np.ndarray, trajectory: List[Tuple[float, float]], camera_matrix: np.ndarray, dist_coeffs: np.ndarray, crop_offset_y: int) -> None:
        # 画像クロップに合わせてカメラ行列を調整
        # 光学中心（主点）cy を -crop_offset_y だけずらす
        camera_matrix_cropped = camera_matrix.copy()
        camera_matrix_cropped[1, 2] -= crop_offset_y

        # 3次元点を画像平面に投影
        # ロボット座標系: X-前方, Y-左, Z-上
        # カメラ座標系 (標準的なROS定義): Z-前方, X-右, Y-下
        # カメラが前方を向いていると仮定した場合の変換:
        # TF（座標変換）がないため、簡易的な変換を行う。
        # カメラは base_link に対して (0, 0, 0) にあると仮定し、回転のみ考慮する。
        # 回転関係:
        # カメラのZ軸 = ロボットのX軸
        # カメラのX軸 = -ロボットのY軸
        # カメラのY軸 = -ロボットのZ軸
        
        # ロボット座標系の点: (x, y, 0)
        # カメラ座標系への変換:
        # xc = -y
        # yc = 0 (もしカメラが地面レベルにある場合) -> しかし実際には高さが必要 (zc=xとなるため)
        # カメラの高さを仮定:
        # yc = - (-H) = H? いや、カメラ座標系のY軸は下向き正。
        # カメラの高さを 0.5m と仮定する
        cam_height = 0.5
        
        object_points = []
        for x, y in trajectory:
            # ロボット座標系からカメラ光学座標系への変換
            # xc = -y （横方向）
            # yc = 0.0 # 水平線？いいえ、ycはカメラ中心からの垂直距離。
            # カメラが高さHにある場合、地面は Y = H (Y軸下向きが正のため)。
            # zc = x （奥行き方向）
            
            xc = -y
            yc = cam_height 
            zc = x
            
            object_points.append([xc, yc, zc])
            
        object_points = np.array(object_points, dtype=np.float32)
        
        if len(object_points) > 0:
            # 投影を実行
            rvec = np.zeros((3, 1), dtype=np.float32) # 単位回転行列（すでに手動で座標変換済みのため）
            tvec = np.zeros((3, 1), dtype=np.float32) # 単位並進ベクトル
            
            img_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix_cropped, dist_coeffs)
            
            # 描画
            for i in range(len(img_points) - 1):
                pt1 = (int(img_points[i][0][0]), int(img_points[i][0][1]))
                pt2 = (int(img_points[i+1][0][0]), int(img_points[i+1][0][1]))
                cv2.line(image, pt1, pt2, (0, 255, 0), 2)
                cv2.circle(image, pt1, 3, (0, 0, 255), -1)
            
            # 最後の点を描画（終点）
            pt_last = (int(img_points[-1][0][0]), int(img_points[-1][0][1]))
            cv2.circle(image, pt_last, 3, (0, 0, 255), -1)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            img_msg.header.frame_id = 'camera_link' # または適切なカメラフレーム名
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')


    def _control_loop(self) -> None:
        # シンプルな Pure Pursuit アルゴリズム / Follow the Carrot (人参を追いかけるような制御)
        twist = Twist()
        
        with self._lock:
            if not self._current_trajectory:
                self.publisher.publish(twist) # 停止
                return

            # 指定した前方注視距離（Lookahead distance）にある目標点を探す
            lookahead_dist = 1.5 # メートル
            target_x = 0.0
            target_y = 0.0
            found_target = False
            
            for pt in self._current_trajectory:
                dist = math.sqrt(pt[0]**2 + pt[1]**2)
                if dist >= lookahead_dist:
                    target_x = pt[0]
                    target_y = pt[1]
                    found_target = True
                    break
            
            # すべての点が注視距離より近い場合は、最後の点を採用する
            if not found_target and self._current_trajectory:
                target_pt = self._current_trajectory[-1]
                target_x = target_pt[0]
                target_y = target_pt[1]
            
            lookahead_dist_sq = target_x**2 + target_y**2
            
            if lookahead_dist_sq > 0.01:
                # 一定の並進速度を設定
                linear_vel = self.max_linear_speed
                
                # Pure Pursuit の曲率計算: curvature = 2y / L^2
                curvature = 2.0 * target_y / lookahead_dist_sq
                
                # 旋回をより積極的にするためのゲインを適用
                angular_vel = linear_vel * curvature * self.angular_gain
                
                # 値の制限（クランプ）
                angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
                
                twist.linear.x = linear_vel
                twist.angular.z = angular_vel
                
                # デバッグログ（大量に出ないよう調整が必要な場合はthrottleを使用）
                # self.get_logger().info(f'Target: ({target_x:.2f}, {target_y:.2f}), Curv: {curvature:.2f}, AngVel: {angular_vel:.2f}')
            else:
                # 近すぎるか無効な場合、停止するかゆっくり動く
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.publisher.publish(twist)

    def _failsafe_check(self) -> None:
        if not self.stop_on_failure:
            return
        
        # しばらく軌道の更新がない場合、安全のため停止する
        elapsed = (self.get_clock().now() - self._last_trajectory_update_time).nanoseconds * 1e-9
        if elapsed > self.failsafe_timeout:
            self.get_logger().warning('Failsafe triggered: No trajectory update.')
            self.publisher.publish(Twist()) # 停止
            with self._lock:
                self._current_trajectory = []

    def _warn_once(self, message: str) -> None:
        if not self._key_warning_sent:
            self.get_logger().warn(message)
            self._key_warning_sent = True

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeminiVLANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

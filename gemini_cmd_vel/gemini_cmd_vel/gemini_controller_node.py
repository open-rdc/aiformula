import asyncio
import json
import os
import threading
from collections import deque
from concurrent.futures import TimeoutError as FuturesTimeoutError
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from google import genai
from google.genai import errors as genai_errors
from google.genai import types as genai_types
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image


class GeminiControllerNode(Node):
    """ROS2 node that uses the Gemini Live API to produce /cmd_vel commands from camera images."""

    # ノード立ち上げ時にパラメータや通信、内部状態を設定する
    def __init__(self) -> None:
        super().__init__('gemini_controller')

        default_api_key = os.environ.get('GEMINI_API_KEY', '')
        default_model = os.environ.get('GEMINI_MODEL', 'models/gemini-2.5-pro') # models/gemini-2.5-flash

        self.declare_parameter('api_key', default_api_key)
        self.declare_parameter('model', default_model)
        self.declare_parameter('subscribe_topic', '/gemini/annotated_image') #gemini/annotated_image、gemini/drivable_area_image
        self.declare_parameter('publish_topic', '/cmd_vel')
        self.declare_parameter('request_interval_sec', 0.1)
        self.declare_parameter('max_linear_speed', 0.9)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('control_period_sec', 0.1)
        self.declare_parameter('max_linear_slew', 1.0)
        self.declare_parameter('max_angular_slew', 0.7)
        self.declare_parameter('min_linear_speed', 1.1)
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 1)#2.5 #1.7
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('failsafe_timeout_sec', 20.0)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('smoothing_window', 1)
        self.declare_parameter('response_timeout_sec', 20.0)
        self.declare_parameter('history_lookback_sec', 2.0)
        self.declare_parameter('history_second_lookback_sec', 4.0)
        self.declare_parameter('use_history_frames', True)
        self.declare_parameter(
                      'base_prompt',                  #script 
            'You control a simulated race car. Using the provided forward-facing camera image, '
            'decide speed and steering to stay centered between the left solid white boundary and the center dashed lane line, '
            'keeping the car within the left lane at all times without stopping. '
            'Favor the left-hand side of the circuit, but avoid approaching or crossing the red-annotated white boundary lines. '
            'If the car drifts right, steer smoothly back left to recenter. '
            'When the lane curves, begin turning early. '
            'IMPORTANT: Maintain the steering angle until you have completely exited the curve. Do not straighten out prematurely. '
            'Use the provided past images to determine the optimal steering angle and observe the evolution of the curve over time. '
            'Reduce speed before the turn, and accelerate again once realigned with the lane center after exiting. '
            'After exiting a curve, quickly reduce steering toward zero to maintain a straight trajectory. '
            'If an orange cone is detected, avoid it smoothly while STRICTLY remaining within the white lane boundaries. '
            'If avoiding on one side would cause you to cross the line, avoid it from the other side. '
            'If an orange cone is detected, avoid it smoothly while STRICTLY remaining within the white lane boundaries. '
            'If avoiding on one side would cause you to cross the line, avoid it from the other side. '
            'Avoid driving onto green surface areas; remain on the asphalt lane. '
            'Respond ONLY with a JSON object containing "linear_x" (m/s) and "angular_z" (rad/s). '
            'Positive angular_z turns left, and negative angular_z turns right.' 
            'Keep values within the given limits, maintain smooth forward motion, '
            'and avoid reversing or oscillating. Output only the JSON object, no narration or markdown.'     
          
          
           # 'base_prompt',
            #'You are driving the race car. The RED segmented lines differentiate the drivable road area. '
            #'Your goal is to follow these RED lines. '
            #'Look ahead at the curvature of the RED lines. If they curve left, turn LEFT immediately (POSITIVE angular_z). '
            #'If they curve right, turn RIGHT immediately (NEGATIVE angular_z). '
            #'IMPORTANT: POSITIVE angular_z = LEFT turn. NEGATIVE angular_z = RIGHT turn. '
            #'Do not wait for the curve to get close; anticipate it by observing the distant red lines. '
            #'Respond ONLY with JSON {"linear_x": m/s, "angular_z": rad/s}. Forward only.'
        
        )

        self.api_key = self.get_parameter('api_key').value
        self.model = self.get_parameter('model').value
        self.subscribe_topic = self.get_parameter('subscribe_topic').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.request_interval = max(0.1, float(self.get_parameter('request_interval_sec').value))
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.control_period = float(self.get_parameter('control_period_sec').value)
        self.max_linear_slew = float(self.get_parameter('max_linear_slew').value)
        self.max_angular_slew = float(self.get_parameter('max_angular_slew').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.linear_gain = max(0.0, float(self.get_parameter('linear_gain').value))
        self.angular_gain = max(0.0, float(self.get_parameter('angular_gain').value))
        self.stop_on_failure = bool(self.get_parameter('stop_on_failure').value)
        self.failsafe_timeout = float(self.get_parameter('failsafe_timeout_sec').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.smoothing_window = max(1, int(self.get_parameter('smoothing_window').value))
        self.base_prompt = self.get_parameter('base_prompt').value
        self.response_timeout = max(1.0, float(self.get_parameter('response_timeout_sec').value))
        self.history_lookback = max(0.0, float(self.get_parameter('history_lookback_sec').value))
        self.history_second_lookback = max(
            0.0, float(self.get_parameter('history_second_lookback_sec').value)
        )
        self.use_history_frames = bool(self.get_parameter('use_history_frames').value)
        self._normalise_failsafe_timeout()

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, self.publish_topic, 10)
        self.subscription = self.create_subscription(
            Image, self.subscribe_topic, self._image_callback, 10
        )
        self.command_timer = self._create_command_timer(self.request_interval)
        failsafe_period = max(0.5, self.failsafe_timeout / 2.0) if self.stop_on_failure else None
        self.failsafe_timer = (
            self.create_timer(failsafe_period, self._failsafe_check)
            if failsafe_period
            else None
        )
        control_period = max(0.02, self.control_period)
        self.control_timer = self.create_timer(control_period, self._control_loop)

        self.add_on_set_parameters_callback(self._on_parameters_set)

        self._latest_image_jpeg: Optional[bytes] = None
        self._frame_history: deque[tuple[float, bytes]] = deque(maxlen=240)
        self._lock = threading.Lock()
        self._processing = False
        self._pending_request_token: Optional[int] = None
        self._pending_request_counter = 0
        self._key_warning_sent = False
        self._last_publish_time = self.get_clock().now()
        self._last_stop_reason: Optional[str] = None
        self._last_target_time = self._last_publish_time
        self._target_twist = Twist()
        self._current_twist = Twist()
        self._target_history: deque[tuple[float, float]] = deque(maxlen=self.smoothing_window)
        self._have_valid_target = False
        self._reset_target_history(Twist(), valid=False)

        # Gemini Live API を扱うイベントループとセッションを常駐化する
        self._async_loop = asyncio.new_event_loop()
        self._async_thread = threading.Thread(
            target=self._async_loop.run_forever, daemon=True
        )
        self._async_thread.start()
        self._live_client: Optional[genai.Client] = None

        self.get_logger().info(
            f'Started Gemini controller. Model={self.model} '
            f'subscription={self.subscribe_topic} publication={self.publish_topic}'
        )
        self._publish_stop('initialised')

    # パラメータ更新イベントを処理し、必要に応じてタイマーや内部状態を更新する
    def _on_parameters_set(self, params):
        for param in params:
            if param.name == 'api_key':
                self.api_key = param.value
                self._key_warning_sent = False
                self._schedule_live_client_reset()
            elif param.name == 'model':
                self.model = param.value
                self._schedule_live_client_reset()
            elif param.name == 'subscribe_topic':
                self.subscribe_topic = param.value
            elif param.name == 'publish_topic':
                self.publish_topic = param.value
            elif param.name == 'request_interval_sec':
                new_interval = max(0.1, float(param.value))
                if abs(new_interval - self.request_interval) > 1e-6:
                    self.request_interval = new_interval
                    if self.command_timer is not None:
                        self.command_timer.cancel()
                        self.destroy_timer(self.command_timer)
                    self.command_timer = self._create_command_timer(self.request_interval)
                self._normalise_failsafe_timeout()
            elif param.name == 'max_linear_speed':
                self.max_linear_speed = float(param.value)
                self._apply_limits_to_state()
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = float(param.value)
                self._apply_limits_to_state()
            elif param.name == 'control_period_sec':
                self.control_period = max(0.02, float(param.value))
                if self.control_timer:
                    self.control_timer.cancel()
                    self.destroy_timer(self.control_timer)
                self.control_timer = self.create_timer(self.control_period, self._control_loop)
            elif param.name == 'max_linear_slew':
                self.max_linear_slew = float(param.value)
            elif param.name == 'max_angular_slew':
                self.max_angular_slew = float(param.value)
            elif param.name == 'min_linear_speed':
                self.min_linear_speed = float(param.value)
                self._apply_limits_to_state()
            elif param.name == 'linear_gain':
                self.linear_gain = max(0.0, float(param.value))
            elif param.name == 'angular_gain':
                self.angular_gain = max(0.0, float(param.value))
            elif param.name == 'stop_on_failure':
                self.stop_on_failure = bool(param.value)
                if not self.stop_on_failure and self.failsafe_timer:
                    self.failsafe_timer.cancel()
                    self.destroy_timer(self.failsafe_timer)
                    self.failsafe_timer = None
                elif self.stop_on_failure and self.failsafe_timeout > 0.0 and not self.failsafe_timer:
                    period = max(0.5, self.failsafe_timeout / 2.0)
                    self.failsafe_timer = self.create_timer(period, self._failsafe_check)
            elif param.name == 'failsafe_timeout_sec':
                self.failsafe_timeout = float(param.value)
                self._normalise_failsafe_timeout()
                if self.failsafe_timer:
                    self.failsafe_timer.cancel()
                    self.destroy_timer(self.failsafe_timer)
                    self.failsafe_timer = None
                if self.stop_on_failure and self.failsafe_timeout > 0.0:
                    period = max(0.5, self.failsafe_timeout / 2.0)
                    self.failsafe_timer = self.create_timer(period, self._failsafe_check)
            elif param.name == 'allow_reverse':
                self.allow_reverse = bool(param.value)
                self._apply_limits_to_state()
            elif param.name == 'smoothing_window':
                self.smoothing_window = max(1, int(param.value))
                self._target_history = deque(self._target_history, maxlen=self.smoothing_window)
                self._apply_limits_to_state()
            elif param.name == 'response_timeout_sec':
                self.response_timeout = max(1.0, float(param.value))
            elif param.name == 'history_lookback_sec':
                self.history_lookback = max(0.0, float(param.value))
            elif param.name == 'history_second_lookback_sec':
                self.history_second_lookback = max(0.0, float(param.value))
            elif param.name == 'use_history_frames':
                self.use_history_frames = bool(param.value)
            elif param.name == 'base_prompt':
                self.base_prompt = param.value
        return SetParametersResult(successful=True)

    # カメラ画像をJPEGに変換し履歴へ蓄積するサブスクライバのコールバック
    def _image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, buffer = cv2.imencode('.jpg', cv_image)
            if not success:
                self.get_logger().warning('Failed to JPEG-encode incoming frame.')
                return
            with self._lock:
                jpeg_bytes = buffer.tobytes()
                self._latest_image_jpeg = jpeg_bytes
                timestamp = self.get_clock().now().nanoseconds * 1e-9
                self._frame_history.append((timestamp, jpeg_bytes))
                max_lookback = max(self.history_lookback, self.history_second_lookback)
                retention = max(5.0, max_lookback * 3.0 + 1.0)
                oldest_allowed = timestamp - retention
                while self._frame_history and self._frame_history[0][0] < oldest_allowed:
                    self._frame_history.popleft()
        except CvBridgeError as exc:
            self.get_logger().warning(f'CvBridge conversion failed: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Unexpected image processing error: {exc}')

    # Gemini問い合わせ用のタイマーを指定周期で生成する
    def _create_command_timer(self, interval: float):
        interval = max(0.1, interval)
        return self.create_timer(interval, self._timer_callback)

    # タイマー発火時にGeminiリクエストをトリガーする
    def _timer_callback(self) -> None:
        if not self._trigger_request():
            with self._lock:
                self._pending_request_counter += 1
                self._pending_request_token = self._pending_request_counter

    # 実行中でない場合に非同期処理スレッドを起動する
    def _trigger_request(self) -> bool:
        with self._lock:
            if self._processing or self._latest_image_jpeg is None:
                return False
            self._processing = True

        thread = threading.Thread(
            target=self._invoke_gemini, daemon=True
        )
        thread.start()
        return True

    # 最新フレームを使ってGemini Live APIへコマンド生成を依頼する
    def _invoke_gemini(self) -> None:
        try:
            if not self.api_key:
                self._warn_once('Gemini API key is empty. Set the api_key parameter to enable requests.')
                self._publish_stop('missing api key')
                return
            frame_parts, frame_descriptors = self._prepare_frame_parts()
            if not frame_parts:
                self.get_logger().warning('No frame data available for Gemini request.')
                self._publish_stop('missing frame data')
                return

            prompt = self._build_prompt(frame_descriptors)
            model_id = self._normalize_model_name(self.model)

            try:
                raw_text = self._run_live_interaction(prompt, frame_parts, model_id)
            except FuturesTimeoutError:
                self.get_logger().error(
                    'Gemini live response future exceeded timeout while waiting on loop.'
                )
                self._publish_stop('live timeout')
                return
            except asyncio.TimeoutError:
                self.get_logger().error(
                    f'Gemini live response timed out after {self.response_timeout:.1f}s.'
                )
                self._publish_stop('live timeout')
                return
            except genai_errors.APIError as exc:
                self.get_logger().error(f'Gemini live API error: {exc}')
                self._publish_stop('live api error')
                return
            except genai_errors.ClientError as exc:
                self.get_logger().error(f'Gemini live client error: {exc}')
                self._publish_stop('live client error')
                return
            except Exception as exc:
                self.get_logger().error(f'Gemini live request failed: {exc}')
                self._publish_stop('live request failure')
                return

            if not raw_text:
                self.get_logger().warning('Gemini live response was empty.')
                self._publish_stop('empty response')
                return

            twist = self._parse_twist(raw_text)
            if twist:
                self._set_target_twist(twist)
            else:
                self.get_logger().warning('Gemini response produced no usable Twist.')
                self._publish_stop('invalid response')
        finally:
            with self._lock:
                self._processing = False
                pending = self._pending_request_token is not None
                self._pending_request_token = None
            if pending:
                self._trigger_request()

    # 基本プロンプトに速度制限や過去フレーム情報を追記する
    def _build_prompt(self, frame_descriptors: list[str]) -> str:
        prompt = (
            f'{self.base_prompt}\n\n'
            f'Maximum linear speed: {self.max_linear_speed:.2f} m/s.\n'
            f'Maximum angular speed: {self.max_angular_speed:.2f} rad/s.\n'
        )
        if frame_descriptors:
            prompt += (
                'Frames provided in order:\n'
                '1) Current camera view.\n'
            )
            for descriptor in frame_descriptors:
                prompt += f'{descriptor}\n'
        else:
            prompt += 'Only the current image is available for this request.\n'
        return prompt

    @staticmethod
    # Gemini Live API用にJPEGバイト列をPartへ梱包する
    def _build_live_image_part(image_bytes: bytes) -> genai_types.Part:
        return genai_types.Part(
            inline_data=genai_types.Blob(
                mimeType='image/jpeg',
                data=image_bytes,
            )
        )

    # 現在フレームと指定した過去フレームを選択しAPI送信用のPartを構築する
    def _prepare_frame_parts(self) -> tuple[list[genai_types.Part], list[str]]:
        with self._lock:
            if not self._frame_history:
                return ([], [])
            history_list = list(self._frame_history)
            current_timestamp, current_bytes = history_list[-1]
            additional_requests: list[tuple[str, float]] = []
            if self.use_history_frames:
                if self.history_lookback > 0.0:
                    additional_requests.append(('previous', self.history_lookback))
                if self.history_second_lookback > 0.0:
                    additional_requests.append(('earlier', self.history_second_lookback))

            selected_frames: list[tuple[int, str, float, float, bytes]] = []
            for idx, (label, lookback) in enumerate(additional_requests, start=2):
                if len(history_list) < 2:
                    break
                target_time = current_timestamp - lookback
                fallback_data: Optional[bytes] = None
                fallback_ts: Optional[float] = None
                chosen_data: Optional[bytes] = None
                chosen_ts: Optional[float] = None
                for ts, data in reversed(history_list[:-1]):
                    if fallback_data is None:
                        fallback_data = data
                        fallback_ts = ts
                    if ts <= target_time:
                        chosen_data = data
                        chosen_ts = ts
                        break
                if chosen_data is None:
                    chosen_data = fallback_data
                    chosen_ts = fallback_ts
                if chosen_data is None or chosen_ts is None:
                    continue
                actual_delta = max(0.0, current_timestamp - chosen_ts)
                selected_frames.append((idx, label, lookback, actual_delta, chosen_data))

        parts: list[genai_types.Part] = [
            genai_types.Part(text='FRAME_1_CURRENT'),
            self._build_live_image_part(current_bytes),
        ]
        descriptors: list[str] = []
        for idx, label, requested_lookback, actual_delta, data in selected_frames:
            label_token = (
                f'FRAME_{idx}_{label.upper()}_REQUESTED_{requested_lookback:.1f}_S_ACTUAL_{actual_delta:.2f}_S'
            )
            parts.extend(
                [
                    genai_types.Part(text=label_token),
                    self._build_live_image_part(data),
                ]
            )
            descriptors.append(
                f'{idx}) {label} view from ≈{actual_delta:.2f} s earlier (requested {requested_lookback:.1f} s).'
            )

        return parts, descriptors

    # Gemini標準API (generate_content) を使って応答テキストを受信する
    def _run_live_interaction(
        self, prompt: str, frame_parts: list[genai_types.Part], model_id: str
    ) -> Optional[str]:
        if not getattr(self, '_async_loop', None):
            raise RuntimeError('Async loop is not available.')
        future = asyncio.run_coroutine_threadsafe(
            self._async_generate_content(prompt, frame_parts, model_id),
            self._async_loop,
        )
        timeout = max(0.1, self.response_timeout + 2.0)
        return future.result(timeout=timeout)

    async def _async_generate_content(
        self, prompt: str, frame_parts: list[genai_types.Part], model_id: str
    ) -> Optional[str]:
        client = self._ensure_live_client()
        # Gemini 3.0 Flash PreviewなどはLive API (Bidi) 未対応の場合があるため
        # 通常の generate_content を使用する実装に切り替える
        
        config = {
            'response_modalities': ['TEXT'], # snake_case for standard API? or camelCase for types? SDK usually handles snake_case
            'temperature': 0.2,
            'top_p': 0.8,
            'top_k': 32,
            'max_output_tokens': 1000,
            'response_mime_type': 'application/json',
        }
        
        # SDKのバージョンやConfigの仕様に合わせて調整
        # google.genai.types.GenerateContentConfig を使うのが確実
        generation_config = genai_types.GenerateContentConfig(
            temperature=0.2,
            top_p=0.8,
            top_k=32,
            max_output_tokens=1000,
            response_mime_type='application/json',
            safety_settings=[
                genai_types.SafetySetting(
                    category='HARM_CATEGORY_HATE_SPEECH',
                    threshold='BLOCK_NONE'
                ),
                genai_types.SafetySetting(
                    category='HARM_CATEGORY_DANGEROUS_CONTENT',
                    threshold='BLOCK_NONE'
                ),
                genai_types.SafetySetting(
                    category='HARM_CATEGORY_SEXUALLY_EXPLICIT',
                    threshold='BLOCK_NONE'
                ),
                genai_types.SafetySetting(
                    category='HARM_CATEGORY_HARASSMENT',
                    threshold='BLOCK_NONE'
                ),
            ]
        )

        try:
            # content = genai_types.Content(role='user', parts=[genai_types.Part(text=prompt), *frame_parts])
            # generate_content は contents=[...] を受け取る
            contents = [genai_types.Content(role='user', parts=[genai_types.Part(text=prompt), *frame_parts])]
            
            response = await client.aio.models.generate_content(
                model=model_id,
                contents=contents,
                config=generation_config
            )
            
            return response.text
        except Exception as e:
            self.get_logger().error(f"Generate content failed: {e}")
            raise

    def _ensure_live_client(self) -> genai.Client:
        if self._live_client is None:
            self._live_client = genai.Client(api_key=self.api_key, http_options={'api_version': 'v1beta'})
        return self._live_client

    async def _close_live_client(self) -> None:
        if self._live_client:
            # Client doesn't strictly need closing if just shared, but good practice
            self._live_client = None

    def _schedule_live_client_reset(self) -> None:
        loop = getattr(self, '_async_loop', None)
        if not loop:
            return
        future = asyncio.run_coroutine_threadsafe(self._close_live_client(), loop)
        try:
            future.result(timeout=5.0)
        except Exception as exc:
            self.get_logger().warn(f'Failed to reset Gemini live session: {exc}')

    # Geminiから返されたJSON文字列をTwist型へ変換する
    def _parse_twist(self, raw_text: str) -> Optional[Twist]:
        raw_text = raw_text.strip()
        if not raw_text:
            self.get_logger().warning('Gemini response contained no text content.')
            return None

        json_snippet = self._extract_json(raw_text)
        if json_snippet is None:
            self.get_logger().warning(f'Unable to extract JSON from response: {raw_text}')
            return None

        try:
            data = json.loads(json_snippet)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'JSON parsing failed: {exc}. Raw={json_snippet}')
            return None

        raw_linear = float(data.get('linear_x', 0.0))
        raw_angular = float(data.get('angular_z', 0.0))
        scaled_linear = raw_linear * self.linear_gain
        # ユーザー要求: 正の値（左旋回）のときのみゲインを適用し、負の値（右旋回）のときはゲインをかけない
        if raw_angular > 0.0:
            scaled_angular = raw_angular * self.angular_gain
        else:
            scaled_angular = raw_angular
            
        linear_x = scaled_linear
        angular_z = scaled_angular
        if not self.allow_reverse and linear_x < 0.0:
            linear_x = 0.0
        linear_x = self._limit_linear(linear_x)
        angular_z = self._limit_angular(angular_z)

        if self.linear_gain != 1.0 or (self.angular_gain != 1.0 and raw_angular > 0.0):
            self.get_logger().info(
                'LLM cmd (raw→scaled→clipped) '
                f'linear_x: {raw_linear:.3f} → {scaled_linear:.3f} → {linear_x:.3f}, '
                f'angular_z: {raw_angular:.3f} → {scaled_angular:.3f} → {angular_z:.3f}'
            )

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    @staticmethod
    # テキストの中から最初と最後の波括弧で囲まれたJSON部分を抽出する
    def _extract_json(text: str) -> Optional[str]:
        start = text.find('{')
        end = text.rfind('}')
        if start == -1 or end == -1 or end <= start:
            return None
        return text[start : end + 1]

    @staticmethod
    # モデル名がmodels/で始まる場合にAPI向けにトリムする
    def _normalize_model_name(model: str) -> str:
        """Strip any leading 'models/' so API paths stay valid."""
        return model.split('/', maxsplit=1)[-1] if model.startswith('models/') else model

    # 新しいターゲット速度を平滑化し内部状態として保持する
    def _set_target_twist(self, twist: Twist) -> None:
        limited_entry = (self._limit_linear(twist.linear.x), self._limit_angular(twist.angular.z))
        self._target_history.append(limited_entry)
        smoothed = self._average_target()
        smoothed.linear.x = self._limit_linear(smoothed.linear.x)
        smoothed.angular.z = self._limit_angular(smoothed.angular.z)
        self._target_twist = smoothed
        self._last_publish_time = self.get_clock().now()
        self._last_target_time = self._last_publish_time
        self._last_stop_reason = None
        self._have_valid_target = True
        self.get_logger().info(
            f'Target cmd_vel linear_x={smoothed.linear.x:.3f} angular_z={smoothed.angular.z:.3f}'
        )

    # 制御周期ごとに現在速度を目標へスルーしながらcmd_velをPublishする
    def _control_loop(self) -> None:
        dt = self.control_period
        linear_delta = self.max_linear_slew * dt
        angular_delta = self.max_angular_slew * dt

        target = self._target_twist
        current = Twist()
        current.linear.x = self._slew_towards(self._current_twist.linear.x, target.linear.x, linear_delta)
        current.angular.z = self._slew_towards(self._current_twist.angular.z, target.angular.z, angular_delta)
        current = self._clip_twist(current)
        self.publisher.publish(current)
        self._current_twist = current

    # 目標履歴を平均して揺らぎを抑えたTwistを生成する
    def _average_target(self) -> Twist:
        if not self._target_history:
            return self._target_twist
        total_lin = sum(item[0] for item in self._target_history)
        total_ang = sum(item[1] for item in self._target_history)
        count = len(self._target_history)
        twist = Twist()
        twist.linear.x = total_lin / count
        twist.angular.z = total_ang / count
        return twist

    # 線速度を設定範囲内に納め、前進のみ設定する場合は負値を切り捨てる
    def _limit_linear(self, value: float) -> float:
        value = max(-self.max_linear_speed, min(self.max_linear_speed, value))
        if not self.allow_reverse and value < 0.0:
            value = 0.0
        if value > 0.0:
            min_allowed = min(self.min_linear_speed, self.max_linear_speed)
            value = max(min_allowed, value)
            value = min(self.max_linear_speed, value)
        return value

    # 角速度を設定範囲内にクリップする
    def _limit_angular(self, value: float) -> float:
        return max(-self.max_angular_speed, min(self.max_angular_speed, value))

    # パブリッシュ直前のTwistを限界値に収め、安全なメッセージへ整形する
    def _clip_twist(self, twist: Twist) -> Twist:
        """Saturate twist values to configured limits before publishing."""
        clipped = Twist()
        clipped.linear.x = self._limit_linear(twist.linear.x)
        clipped.linear.y = 0.0
        clipped.linear.z = 0.0
        clipped.angular.x = 0.0
        clipped.angular.y = 0.0
        clipped.angular.z = self._limit_angular(twist.angular.z)
        if clipped.linear.x != twist.linear.x or clipped.angular.z != twist.angular.z:
            self.get_logger().debug(
                f'Clipping cmd_vel to linear_x={clipped.linear.x:.3f}, angular_z={clipped.angular.z:.3f}'
            )
        return clipped

    # 新しいターゲット値で履歴と状態をリセットし、即座に制限を適用する
    def _reset_target_history(self, twist: Twist, *, valid: bool = False) -> None:
        self._target_history.clear()
        entry = (self._limit_linear(twist.linear.x), self._limit_angular(twist.angular.z))
        self._target_history.append(entry)

        target = Twist()
        target.linear.x, target.angular.z = entry
        current = Twist()
        current.linear.x, current.angular.z = entry

        self._target_twist = target
        self._current_twist = current
        self._have_valid_target = valid
        self._apply_limits_to_state()

    # 異常時に停止コマンドを発行するか直前の速度を維持するかを判断する
    def _publish_stop(self, reason: Optional[str] = None) -> None:
        if not self.stop_on_failure:
            return

        zero_reasons = {'initialised', 'missing api key', 'failsafe timeout'}
        now = self.get_clock().now()

        if reason in zero_reasons:
            if reason and reason != self._last_stop_reason:
                self.get_logger().info(f'Publishing zero cmd_vel ({reason}).')
            zero = Twist()
            self._reset_target_history(zero, valid=False)
            self.publisher.publish(self._clip_twist(zero))
            self._last_publish_time = now
            self._last_target_time = now
            self._have_valid_target = False
        else:
            if reason and reason != self._last_stop_reason:
                self.get_logger().info(f'Holding last cmd_vel ({reason}).')
            self._last_publish_time = now
            self._last_target_time = now

        self._last_stop_reason = reason

    # 最終目標更新から一定時間経過した場合に安全停止する
    def _failsafe_check(self) -> None:
        if not self.stop_on_failure or not self._have_valid_target:
            return
        elapsed = (self.get_clock().now() - self._last_target_time).nanoseconds * 1e-9
        if elapsed > self.failsafe_timeout:
            self._publish_stop('failsafe timeout')

    # APIキー未設定の警告を一度だけ出す
    def _warn_once(self, message: str) -> None:
        if not self._key_warning_sent:
            self.get_logger().warn(message)
            self._key_warning_sent = True

    # フェイルセーフタイムアウトをリクエスト周期と整合性が取れるよう補正する
    def _normalise_failsafe_timeout(self) -> None:
        minimum_timeout = max(0.0, self.request_interval * 1.6)
        if self.failsafe_timeout > 0.0 and self.failsafe_timeout < minimum_timeout:
            self.get_logger().debug(
                f'Extending failsafe timeout from {self.failsafe_timeout:.2f}s '
                f'to {minimum_timeout:.2f}s to accommodate request interval.'
            )
            self.failsafe_timeout = minimum_timeout

    @staticmethod
    # 現在値から指定ステップ内で目標値へ近づくスルーレート制御
    def _slew_towards(current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return target
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + max_delta * (1.0 if delta > 0.0 else -1.0)

    # 内部状態全体に速度制限を適用し履歴も再構築する
    def _apply_limits_to_state(self) -> None:
        self._target_twist.linear.x = self._limit_linear(self._target_twist.linear.x)
        self._target_twist.angular.z = self._limit_angular(self._target_twist.angular.z)
        self._current_twist.linear.x = self._limit_linear(self._current_twist.linear.x)
        self._current_twist.angular.z = self._limit_angular(self._current_twist.angular.z)

        limited_history = deque(maxlen=self.smoothing_window)
        for lin, ang in self._target_history:
            limited_history.append((self._limit_linear(lin), self._limit_angular(ang)))
        self._target_history = limited_history

    def destroy_node(self) -> None:
        self._shutdown_async_components()
        super().destroy_node()

    def _shutdown_async_components(self) -> None:
        loop = getattr(self, '_async_loop', None)
        if not loop:
            return
        try:
            future = asyncio.run_coroutine_threadsafe(self._close_live_client(), loop)
            future.result(timeout=5.0)
        except Exception as exc:
            self.get_logger().warn(f'Error while shutting down Gemini client: {exc}')
        finally:
            loop.call_soon_threadsafe(loop.stop)
            thread = getattr(self, '_async_thread', None)
            if thread and thread.is_alive():
                thread.join(timeout=5.0)
            self._async_loop = None
            self._async_thread = None

# rclpyノードを起動するエントリーポイント
def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeminiControllerNode()
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

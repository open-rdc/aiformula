import base64
import json
import os
import threading
from collections import deque
from datetime import datetime, timezone
from email.utils import parsedate_to_datetime
from typing import Optional

import cv2
import requests
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image


class GeminiControllerNode(Node):
    """ROS2 node that queries Gemini with camera images to produce /cmd_vel commands."""

    def __init__(self) -> None:
        super().__init__('gemini_controller')

        default_api_key = os.environ.get('GEMINI_API_KEY', '')
        default_model = os.environ.get('GEMINI_MODEL', 'gemini-flash-latest')

        self.declare_parameter('api_key', default_api_key)
        self.declare_parameter('model', default_model)
        self.declare_parameter('subscribe_topic', '/image_raw')
        self.declare_parameter('publish_topic', '/cmd_vel')
        self.declare_parameter('request_interval_sec', 5.0)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.3)
        self.declare_parameter('control_period_sec', 0.1)
        self.declare_parameter('max_linear_slew', 0.4)
        self.declare_parameter('max_angular_slew', 2.0)
        self.declare_parameter('min_linear_speed', 0.2)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('failsafe_timeout_sec', 10.0)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('smoothing_window', 3)
        self.declare_parameter(
            'base_prompt',
            'You control a simulated race car. Using the provided forward-facing camera image, '
            'decide speed and steering to stay at the center between two white lane lines and keep moving without stopping. '
            'Avoid driving onto the green surface areas; remain on the asphalt lane. If a white lane line appears ahead '
            'on the current heading, adjust steering to stay between the lines and prevent crossing them. Respond ONLY '
            'with a JSON object containing "linear_x" (m/s) and "angular_z" (rad/s). Positive angular_z turns left. '
            'Keep absolute values within the supplied speed limits, maintain smooth forward motion, and avoid reversing '
            'or oscillating. No narration or markdown. '
        )

        self.api_key = self.get_parameter('api_key').value
        self.model = self.get_parameter('model').value
        self.subscribe_topic = self.get_parameter('subscribe_topic').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.request_interval = float(self.get_parameter('request_interval_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.control_period = float(self.get_parameter('control_period_sec').value)
        self.max_linear_slew = float(self.get_parameter('max_linear_slew').value)
        self.max_angular_slew = float(self.get_parameter('max_angular_slew').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.stop_on_failure = bool(self.get_parameter('stop_on_failure').value)
        self.failsafe_timeout = float(self.get_parameter('failsafe_timeout_sec').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.smoothing_window = max(1, int(self.get_parameter('smoothing_window').value))
        self.base_prompt = self.get_parameter('base_prompt').value
        self._normalise_failsafe_timeout()

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, self.publish_topic, 10)
        self.subscription = self.create_subscription(
            Image, self.subscribe_topic, self._image_callback, 10
        )
        self.timer = self.create_timer(self.request_interval, self._timer_callback)
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
        self._lock = threading.Lock()
        self._processing = False
        self._key_warning_sent = False
        self._last_publish_time = self.get_clock().now()
        self._last_stop_reason: Optional[str] = None
        self._last_target_time = self._last_publish_time
        self._target_twist = Twist()
        self._current_twist = Twist()
        self._target_history: deque[tuple[float, float]] = deque(maxlen=self.smoothing_window)
        self._have_valid_target = False
        self._reset_target_history(Twist(), valid=False)
        self._cooldown_until: Optional[rclpy.time.Time] = None

        self.get_logger().info(
            f'Started Gemini controller. Model={self.model} '
            f'subscription={self.subscribe_topic} publication={self.publish_topic}'
        )
        self._publish_stop('initialised')

    def _on_parameters_set(self, params):
        for param in params:
            if param.name == 'api_key':
                self.api_key = param.value
                self._key_warning_sent = False
            elif param.name == 'model':
                self.model = param.value
            elif param.name == 'subscribe_topic':
                self.subscribe_topic = param.value
            elif param.name == 'publish_topic':
                self.publish_topic = param.value
            elif param.name == 'request_interval_sec':
                self.request_interval = float(param.value)
                self._normalise_failsafe_timeout()
            elif param.name == 'max_linear_speed':
                self.max_linear_speed = float(param.value)
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = float(param.value)
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
            elif param.name == 'smoothing_window':
                self.smoothing_window = max(1, int(param.value))
                self._target_history = deque(self._target_history, maxlen=self.smoothing_window)
                self._reset_target_history(self._target_twist, valid=self._have_valid_target)
            elif param.name == 'base_prompt':
                self.base_prompt = param.value
        return SetParametersResult(successful=True)

    def _image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, buffer = cv2.imencode('.jpg', cv_image)
            if not success:
                self.get_logger().warning('Failed to JPEG-encode incoming frame.')
                return
            with self._lock:
                self._latest_image_jpeg = buffer.tobytes()
        except CvBridgeError as exc:
            self.get_logger().warning(f'CvBridge conversion failed: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Unexpected image processing error: {exc}')

    def _timer_callback(self) -> None:
        if self._cooldown_until is not None:
            now = self.get_clock().now()
            if now < self._cooldown_until:
                return
            self._cooldown_until = None

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
            if not self.api_key:
                self._warn_once('Gemini API key is empty. Set the api_key parameter to enable requests.')
                self._publish_stop('missing api key')
                return

            payload = self._build_payload(image_bytes)
            model_id = self._normalize_model_name(self.model)
            url = (
                f'https://generativelanguage.googleapis.com/v1beta/models/'
                f'{model_id}:generateContent?key={self.api_key}'
            )
            response = requests.post(url, json=payload, timeout=15.0)
            if response.status_code == 429:
                delay = self._handle_rate_limit(response)
                self.get_logger().warning(
                    f'Gemini rate limited. Cooling down for {delay:.1f}s before retrying.'
                )
                self._publish_stop('rate limit cooldown')
                return

            if response.status_code >= 400:
                response.raise_for_status()

            twist = self._parse_twist(response.json())
            if twist:
                self._set_target_twist(twist)
            else:
                self.get_logger().warning('Gemini response produced no usable Twist.')
                self._publish_stop('invalid response')
        except requests.RequestException as exc:
            self.get_logger().error(f'Gemini request failed: {exc}')
            self._publish_stop('request failure')
        except Exception as exc:
            self.get_logger().error(f'Failed to handle Gemini response: {exc}')
            self._publish_stop('processing error')
        finally:
            with self._lock:
                self._processing = False

    def _build_payload(self, image_bytes: bytes) -> dict:
        prompt = (
            f'{self.base_prompt}\n\n'
            f'Maximum linear speed: {self.max_linear_speed:.2f} m/s.\n'
            f'Maximum angular speed: {self.max_angular_speed:.2f} rad/s.'
        )
        image_b64 = base64.b64encode(image_bytes).decode('utf-8')
        return {
            'contents': [
                {
                    'role': 'user',
                    'parts': [
                        {'text': prompt},
                        {
                            'inline_data': {
                                'mime_type': 'image/jpeg',
                                'data': image_b64,
                            }
                        },
                    ],
                }
            ],
            'generationConfig': {
                'temperature': 0.2,
                'topP': 0.8,
                'topK': 32,
                'maxOutputTokens': 512,
                'responseMimeType': 'application/json',
            },
        }

    def _parse_twist(self, payload: dict) -> Optional[Twist]:
        candidates = payload.get('candidates', [])
        if not candidates:
            self.get_logger().warning('Gemini response did not contain candidates.')
            return None

        texts = []
        first_candidate = candidates[0]
        for part in first_candidate.get('content', {}).get('parts', []):
            text = part.get('text')
            if text:
                texts.append(text)
        raw_text = ''.join(texts).strip()
        if not raw_text:
            self.get_logger().warning(
                f'Gemini response contained no text content. candidate='
                f'{json.dumps(first_candidate, ensure_ascii=False)}'
            )
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

        linear_x = float(data.get('linear_x', 0.0))
        angular_z = float(data.get('angular_z', 0.0))
        if not self.allow_reverse and linear_x < 0.0:
            linear_x = 0.0
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        if linear_x > 0.0:
            linear_x = max(self.min_linear_speed, linear_x)
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    @staticmethod
    def _extract_json(text: str) -> Optional[str]:
        start = text.find('{')
        end = text.rfind('}')
        if start == -1 or end == -1 or end <= start:
            return None
        return text[start : end + 1]

    @staticmethod
    def _normalize_model_name(model: str) -> str:
        """Strip any leading 'models/' so API paths stay valid."""
        return model.split('/', maxsplit=1)[-1] if model.startswith('models/') else model

    def _set_target_twist(self, twist: Twist) -> None:
        self._target_history.append((twist.linear.x, twist.angular.z))
        smoothed = self._average_target()
        self._target_twist = smoothed
        self._last_publish_time = self.get_clock().now()
        self._last_target_time = self._last_publish_time
        self._last_stop_reason = None
        self._have_valid_target = True
        self.get_logger().info(
            f'Target cmd_vel linear_x={smoothed.linear.x:.3f} angular_z={smoothed.angular.z:.3f}'
        )

    def _control_loop(self) -> None:
        dt = self.control_period
        linear_delta = self.max_linear_slew * dt
        angular_delta = self.max_angular_slew * dt

        target = self._target_twist
        current = Twist()
        current.linear.x = self._slew_towards(self._current_twist.linear.x, target.linear.x, linear_delta)
        current.angular.z = self._slew_towards(self._current_twist.angular.z, target.angular.z, angular_delta)
        self.publisher.publish(current)
        self._current_twist = current

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

    def _reset_target_history(self, twist: Twist, *, valid: bool = False) -> None:
        self._target_history.clear()
        entry = (twist.linear.x, twist.angular.z)
        self._target_history.append(entry)

        target = Twist()
        target.linear.x, target.angular.z = entry
        current = Twist()
        current.linear.x, current.angular.z = entry

        self._target_twist = target
        self._current_twist = current
        self._have_valid_target = valid

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
            self.publisher.publish(zero)
            self._last_publish_time = now
            self._last_target_time = now
            self._have_valid_target = False
        else:
            if reason and reason != self._last_stop_reason:
                self.get_logger().info(f'Holding last cmd_vel ({reason}).')
            self._last_publish_time = now
            self._last_target_time = now

        self._last_stop_reason = reason

    def _failsafe_check(self) -> None:
        if not self.stop_on_failure or not self._have_valid_target:
            return
        elapsed = (self.get_clock().now() - self._last_target_time).nanoseconds * 1e-9
        if elapsed > self.failsafe_timeout:
            self._publish_stop('failsafe timeout')

    def _warn_once(self, message: str) -> None:
        if not self._key_warning_sent:
            self.get_logger().warn(message)
            self._key_warning_sent = True

    def _normalise_failsafe_timeout(self) -> None:
        minimum_timeout = max(0.0, self.request_interval * 1.6)
        if self.failsafe_timeout > 0.0 and self.failsafe_timeout < minimum_timeout:
            self.get_logger().debug(
                f'Extending failsafe timeout from {self.failsafe_timeout:.2f}s '
                f'to {minimum_timeout:.2f}s to accommodate request interval.'
            )
            self.failsafe_timeout = minimum_timeout

    @staticmethod
    def _slew_towards(current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return target
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + max_delta * (1.0 if delta > 0.0 else -1.0)

    def _handle_rate_limit(self, response: requests.Response) -> float:
        default_delay = max(60.0, self.request_interval * 2.0)
        delay = default_delay

        retry_after = response.headers.get('Retry-After')
        if retry_after:
            try:
                delay = float(retry_after)
            except ValueError:
                try:
                    retry_dt = parsedate_to_datetime(retry_after)
                    if retry_dt.tzinfo is None:
                        retry_dt = retry_dt.replace(tzinfo=timezone.utc)
                    now_dt = datetime.now(timezone.utc)
                    delay = max(0.0, (retry_dt - now_dt).total_seconds())
                except Exception:
                    delay = default_delay
        else:
            try:
                body = response.json()
                details = body.get('error', {}).get('details', [])
                for detail in details:
                    retry = detail.get('retryDelay') or detail.get('retry_delay')
                    if isinstance(retry, str) and retry.endswith('s'):
                        delay = float(retry[:-1])
                        break
            except Exception:
                delay = default_delay

        delay = max(delay, self.request_interval * 1.5)
        self._cooldown_until = self.get_clock().now() + Duration(seconds=delay)
        return delay


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

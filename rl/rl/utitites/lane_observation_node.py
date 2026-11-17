import cv2
import numpy as np
import rclpy
import message_filters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int8

from rl.utitites.lane_departure import DepartureConfig, DepartureDetector
from rl.utitites.lane_kf import KalmanConfig, LineKalmanFilter
from rl.utitites.lane_observation import LaneObserver, ObserverConfig


class LaneObservationNode(Node):
    def __init__(self):
        super().__init__('lane_observation_node')
        cfg = ObserverConfig(**{
            k: type(v)(self.declare_parameter(k, v).value) for k, v in vars(ObserverConfig()).items()
        })
        self.observer = LaneObserver(cfg)
        kf_cfg = KalmanConfig(**{
            k: type(v)(self.declare_parameter(k, v).value) for k, v in vars(KalmanConfig()).items()
        })
        self.filters = [LineKalmanFilter(kf_cfg), LineKalmanFilter(kf_cfg)]
        dep_cfg = DepartureConfig(**{
            k: type(v)(self.declare_parameter(k, v).value) for k, v in vars(DepartureConfig()).items()
        })
        self.departure = DepartureDetector(dep_cfg, cfg)
        self.last_stamp = None

        self.Qos = rclpy.qos.QoSProfile(depth=10)
        self.Qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        self.Qos.durability = rclpy.qos.DurabilityPolicy.VOLATILE
        self.Qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST

        self.image_sub = message_filters.Subscriber(self, Image, '/perception/lane_mask', self.Qos)
        self.imu_sub = message_filters.Subscriber(self, Imu, '/vectornav/imu', self.Qos)
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odom', self.Qos)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.imu_sub, self.odom_sub], self.Qos.depth, 0.1)
        self.ts.registerCallback(self.callback)

        self.pub = self.create_publisher(Float32MultiArray, '/lane_observation', self.Qos)
        self.state_pub = self.create_publisher(Float32MultiArray, '/lane_state', self.Qos)
        self.debug_pub = self.create_publisher(Image, '/lane_observation/debug', self.Qos)
        self.departure_pub = self.create_publisher(Int8, '/lane_departure', 10)

    def callback(self, image_msg, imu_msg, odom_msg):
        if image_msg.encoding not in ('mono8', '8UC1'):
            self.get_logger().error(f'lane_mask は mono8 を想定しています: {image_msg.encoding}', throttle_duration_sec=5.0)
            return
        mask = np.frombuffer(image_msg.data, np.uint8).reshape(image_msg.height, image_msg.step)[:, :image_msg.width]
        stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_stamp is None else stamp - self.last_stamp
        self.last_stamp = stamp

        for kf in self.filters:
            kf.predict(dt)
        obs = self.observer.observe(mask, *(kf.prior() for kf in self.filters))
        for kf, o in zip(self.filters, obs):
            kf.update(o)

        out = Float32MultiArray()
        out.data = obs[0].to_list() + obs[1].to_list()
        self.pub.publish(out)
        state = Float32MultiArray()
        state.data = self.filters[0].to_list() + self.filters[1].to_list()
        self.state_pub.publish(state)

        events = [kf.pop_event() for kf in self.filters]
        result = self.departure.check(*events, width=mask.shape[1], height=mask.shape[0])
        self.departure_pub.publish(Int8(data=result))
        if any(e is not None for e in events):
            detail = ', '.join(
                f'{side}: {e.reason} track={e.track_time:.2f}s x_ref={e.state[0]:.1f} slope={e.state[1]:.2f} '
                f'x_bottom={self.departure.bottom_x(e, mask.shape[0]):.1f}'
                for side, e in zip(('L', 'R'), events) if e is not None)
            self.get_logger().info(f'lane_departure={result} ({detail})')

        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(mask, obs, image_msg.header)

    def publish_debug(self, mask, obs, header):
        cfg = self.observer.cfg
        img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) // 2
        cv2.rectangle(img, (0, cfg.y_start), (img.shape[1] - 1, cfg.y_end - 1), (80, 80, 80), 1)
        for o, color in zip(obs, [(255, 128, 0), (0, 128, 255)]):
            if not o.valid:
                continue
            for y, x in o.points:
                img[int(y), int(round(x))] = color
            self._draw_line(img, o.x_ref, o.slope, color, 2)
            cv2.circle(img, (int(o.x_ref), cfg.y_ref), max(2, int(np.sqrt(o.var_x))), (0, 255, 0), 1)
        for kf in self.filters:
            if kf.initialized:
                self._draw_line(img, kf.s[0], kf.s[1], (255, 255, 255), 1)
        out = Image(header=header, height=img.shape[0], width=img.shape[1], encoding='bgr8', step=img.shape[1] * 3)
        out.data = np.ascontiguousarray(img).tobytes()
        self.debug_pub.publish(out)

    def _draw_line(self, img, x_ref, slope, color, thickness):
        cfg = self.observer.cfg
        p0 = (int(x_ref + slope * (cfg.y_start - cfg.y_ref)), cfg.y_start)
        p1 = (int(x_ref + slope * (cfg.y_end - 1 - cfg.y_ref)), cfg.y_end - 1)
        cv2.line(img, p0, p1, color, thickness)


def main(args=None):
    rclpy.init(args=args)
    node = LaneObservationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

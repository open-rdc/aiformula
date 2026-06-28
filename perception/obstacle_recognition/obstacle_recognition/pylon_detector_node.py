#!/usr/bin/env python3
import math
import numpy as np
import torch
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from object_detection_msgs.msg import ObjectInfo, ObjectInfoArray

import cv2
from cv_bridge import CvBridge

from yolox.exp import get_exp
from yolox.utils import postprocess
from yolox.data.data_augment import ValTransform


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx

OPT_TO_BODY = np.array([[0.0, 0.0, 1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0]])


class PylonDetectorNode(Node):
    def __init__(self):
        pkg = get_package_share_directory('obstacle_recognition')
        super().__init__('pylon_detector_node')

        self.declare_parameter('exp_file', os.path.join(pkg, 'YOLOX/exps/yolox_s_cone_nhd.py'))
        self.declare_parameter('ckpt', os.path.join(pkg, 'data/weights/best_ckpt_nhd.pth'))
        self.declare_parameter('conf_thre', 0.3)
        self.declare_parameter('nms_thre', 0.45)
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('objects_topic', '/perception/pylons')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('pylon_class_id', 0)
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('cam_xyz', [0.055, 0.0, 0.54])   # カメラ取付位置[m] (base_link基準)
        self.declare_parameter('cam_rpy', [0.0, 0.0, 0.0])      # 取付姿勢[rad] (点群frame X前/Y左/Z上 → base)
        self.declare_parameter('ground_z', 0.0)

        g = lambda k: self.get_parameter(k).value
        self.target_frame = g('target_frame')
        self.pylon_class_id = g('pylon_class_id')
        self.fx = self.fy = self.cx = self.cy = None    # camera_info受信で埋まる
        self.ground_z = float(g('ground_z'))
        self._t = np.array(g('cam_xyz'), dtype=float)
        self._R = rpy_to_matrix(*g('cam_rpy')) @OPT_TO_BODY

        # YOLOXのロード
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.exp = get_exp(g('exp_file'), None)
        self.exp.test_conf = g('conf_thre')
        self.exp.nmsthre = g('nms_thre')
        self.model = self.exp.get_model().to(self.device).eval()
        ckpt = torch.load(g('ckpt'), map_location=self.device)
        self.model.load_state_dict(ckpt['model'] if 'model' in ckpt else ckpt)
        self.preproc = ValTransform(legacy=False)
        self.get_logger().info(f'Loaded YOLOX model from {g("ckpt")}')

        # ros入出力
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, g('camera_info_topic'), self.camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, g('image_topic'), self.image_cb, qos_profile_sensor_data)
        self.object_pub = self.create_publisher(ObjectInfoArray, g('objects_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/pylons_visualize', 10)
        self.debug_image_pub = self.create_publisher(Image, '/perception/pylons_debug_image', 10)

    
    # YOLOX推論
    def detect(self, img):
        h, w = img.shape[:2]
        ratio = min(self.exp.test_size[0] / h, self.exp.test_size[1] / w)
        inputs, _ = self.preproc(img, None, self.exp.test_size)
        inputs = torch.from_numpy(inputs).unsqueeze(0).float().to(self.device)
        with torch.no_grad():
            outputs = self.model(inputs)
            outputs = postprocess(outputs, self.exp.num_classes, self.exp.test_conf, self.exp.nmsthre, class_agnostic=True)[0]
        boxes_out = []
        if outputs is not None:
            boxes = outputs.cpu()[:, 0:4] / ratio
            for x1, y1, x2, y2 in boxes:
                boxes_out.append((float(x1), float(y1), float(x2), float(y2)))
        return boxes_out
            
    # camera_info から内部パラメータを取得
    def camera_info_cb(self, msg):
        k = msg.k
        self.fx, self.fy = k[0], k[4]
        self.cx, self.cy = k[2], k[5]

    # ピクセル(u,v)の視線を地面平面と交差させ base_link 座標を求める
    def pixel_to_ground(self, u, v):
        d_opt = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        d = self._R @ d_opt
        if d[2] >= -1e-6:
            return None
        s = (self.ground_z - self._t[2]) / d[2]
        if s <= 0:
            return None
        p = self._t + s * d
        return (float(p[0]), float(p[1]), float(p[2]))

    def image_cb(self, img_msg):
        if self.fx is None:
            self.get_logger().warn('waiting camera_info...', throttle_duration_sec=2.0)
            return
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        boxes = self.detect(img)

        cones = []
        for (x1, y1, x2, y2) in boxes:
            v = y2                                       # bbox底辺(接地点)
            pC = self.pixel_to_ground((x1 + x2) / 2, v)
            pL = self.pixel_to_ground(x1, v)
            pR = self.pixel_to_ground(x2, v)

            # デバッグ描画(bbox)
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 165, 255), 2)
            if pC is None or pL is None or pR is None:
                continue
            width = math.hypot(pL[0] - pR[0], pL[1] - pR[1])
            dist = math.hypot(pC[0], pC[1])
            cones.append((pC[0], pC[1], width))
            cv2.putText(img, f'{dist:.2f}m {width:.2f}m', (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            self.get_logger().info(f'pylon dist={dist:.2f}m width={width:.2f}m', throttle_duration_sec=1.0)

        debug_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        debug_msg.header = img_msg.header
        self.debug_image_pub.publish(debug_msg)

        self.object_pub.publish(self.make_objects(cones, img_msg.header.stamp))
        self.marker_pub.publish(self.make_markers(cones, img_msg.header.stamp))
        self.get_logger().info(f'pylons: {len(cones)}', throttle_duration_sec=1.0)

    def make_objects(self, cones, stamp):
        arr = ObjectInfoArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.target_frame
        for (x, y, width) in cones:
            o = ObjectInfo()
            o.x = float(x)
            o.y = float(y)
            o.width = float(width)
            o.id = int(self.pylon_class_id)
            arr.objects.append(o)
        return arr
    
    def make_markers(self, cones, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y, width) in enumerate(cones):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = stamp
            m.ns = 'pylon'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = float(width)
            m.scale.z = 0.4
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 1.0
            arr.markers.append(m)
        return arr

def main(args=None):
    rclpy.init(args=args)
    node = PylonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
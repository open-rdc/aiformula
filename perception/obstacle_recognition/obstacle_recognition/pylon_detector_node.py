#!/usr/bin/env python3
import math
import numpy as np
import torch
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from object_detection_msgs.msg import ObjectInfo, ObjectInfoArray

from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
import tf2_ros
import tf2_geometry_msgs

from yolox.exp import get_exp
from yolox.utils import postprocess
from yolox.data.data_augment import ValTransform

class PylonDetectorNode(Node):
    def __init__(self):
        pkg = get_package_share_directory('obstacle_recognition')
        super().__init__('pylon_detector_node')

        self.declare_parameter('exp_file', os.path.join(pkg, 'YOLOX/exps/yolox_s_cone_full.py'))
        self.declare_parameter('ckpt', os.path.join(pkg, 'data/weights/best_ckpt_full.pth'))
        self.declare_parameter('conf_thre', 0.3)
        self.declare_parameter('nms_thre', 0.45)
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('pointcloud_topic', '/zed/zed_node/point_cloud')
        self.declare_parameter('objects_topic', '/perception/pylons')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('pylon_width_m', 0.3)
        self.declare_parameter('pylon_class_id', 0)
        self.declare_parameter('patch_radius', 2)

        g = lambda k: self.get_parameter(k).value
        self.target_frame = g('target_frame')
        self.pylon_width_m = g('pylon_width_m')
        self.pylon_class_id = g('pylon_class_id')
        self.patch_radius = g('patch_radius')

        # YOLOXのロード
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.exp = get_exp(g('exp_file'), None)
        self.exp.test_conf = g('conf_thre')
        self.exp.nmsthre = g('nms_thre')
        self.model = self.exp.get_model().to(self.device).eval()
        ckpt = torch.load(g('ckpt'), map_location=self.device, weights_only=False)
        self.model.load_state_dict(ckpt['model'] if 'model' in ckpt else ckpt)
        self.preproc = ValTransform(legacy=False)
        self.get_logger().info(f'Loaded YOLOX model from {g("ckpt")}')

        # ros入出力
        self.bridge = CvBridge()
        self.latest_cloud = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, g('pointcloud_topic'), self.pointcloud_cb, qos_profile_sensor_data)
        self.create_subscription(Image, g('image_topic'), self.image_cb, qos_profile_sensor_data)
        self.object_pub = self.create_publisher(ObjectInfoArray, g('objects_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/pylons_visualize', 10)

    def pointcloud_cb(self, msg):
        self.latest_cloud = msg
    
    # YOLOX推論
    def detect(self, img):
        h, w = img.shape[:2]
        ratio = min(self.exp.test_size[0] / h, self.exp.test_size[1] / w)
        inputs, _ = self.preproc(img, None, self.exp.test_size)
        inputs = torch.from_numpy(inputs).unsqueeze(0).float().to(self.device)
        with torch.no_grad():
            outputs = self.model(inputs)
            outputs = postprocess(outputs, self.exp.num_classes, self.exp.test_conf, self.exp.nmsthre, class_agnostic=True)[0]
        centers = []
        if outputs is not None:
            boxes = outputs.cpu()[:, 0:4] / ratio
            for x1, y1, x2, y2 in boxes:
                centers.append((float((x1 + x2) / 2), float((y1 + y2))))
        return centers, (w, h)
    
    # 座標変換
    def pixel_to_xyz(self, pc, u, v):
        pts = list(point_cloud2.read_points(
            pc, field_names=['x', 'y', 'z'], skip_nans=False, uvs=[(int(u), int(v))]
        ))
        if not pts:
            return None
        x, y, z = pts[0]
        if any(math.isnan(c) or math.isinf(c) for c in (x, y, z)):
            return None
        return (float(x), float(y), float(z))
    
    def patch_xyz(self, pc, u, v):
        r = self.patch_radius
        xs, ys, zs = [], [], []
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                p = self.pixel_to_xyz(pc, u + du, v + dv)
                if p:
                    xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
        if not xs:
            return None
        return (float(np.median(xs)), float(np.median(ys)), float(np.median(zs)))
    
    def transform_point(self, x, y, z, src_frame, stamp):
        ps = PointStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = stamp
        ps.point.x, ps.point.y, ps.point.z = x, y, z
        try:
            o = self.tf_buffer.transform(ps, self.target_frame, timeout=Duration(seconds=0.1))
            return (o.point.x, o.point.y, o.point.z)
        except Exception as e:
            self.get_logger().warn(f'TF {src_frame}->{self.target_frame} fail: {e}',
                                    throttle_duration_sec=2.0)
            return None
        
    def image_cb(self, img_msg):
        if self.latest_cloud is None:
            self.get_logger().warn('waiting point cloud....', throttle_duration_sec=2.0)
            return
        pc = self.latest_cloud
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        centers, (img_w, img_h) = self.detect(img)

        cones = []
        for (u, v) in centers:
            pu = u * pc.width / img_w
            pv = v * pc.height / img_h
            xyz = self.patch_xyz(pc, pu, pv)
            if xyz is None:
                continue
            tp = self.transform_point(*xyz, pc.header.frame_id, pc.header.stamp)
            if tp is not None:
                cones.append(tp)
        
        self.object_pub.publish(self.make_objects(cones, img_msg.header.stamp))
        self.marker_pub.publish(self.make_markers(cones, img_msg.header.stamp))
        self.get_logger().info(f'pylons: {len(cones)}', throttle_duration_sec=1.0)

    def make_objects(self, cones, stamp):
        arr = ObjectInfoArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.target_frame
        for (x, y, z) in cones:
            o = ObjectInfo()
            o.x = float(x)
            o.y = float(y)
            o.width = float(self.pylon_width_m)
            o.id = int(self.pylon_class_id)
            arr.objects.append(o)
        return arr
    
    def make_markers(self, cones, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y, z) in enumerate(cones):
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
            m.scale.x = m.scale.y = float(self.pylon_width_m)
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
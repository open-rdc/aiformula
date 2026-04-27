import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rclpy.init()
node = rclpy.create_node('img_saver')
bridge = CvBridge()

def cb(msg):
    cv2.imwrite('sim_image.png', bridge.imgmsg_to_cv2(msg, 'bgr8'))
    print('Saved')
    raise SystemExit

sub = node.create_subscription(Image, '/image_raw', cb, 10)
try:
    rclpy.spin(node)
except SystemExit:
    pass
rclpy.shutdown()

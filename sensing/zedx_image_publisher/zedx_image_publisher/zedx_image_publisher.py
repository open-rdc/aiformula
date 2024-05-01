import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage,Image
import pyzed.sl as sl
import cv2
from cv_bridge import CvBridge
class ZedXImagePublisher(Node):
    def __init__(self):
        super().__init__('zedx_image_publisher')
        #ZEDX
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 60
        self.zed = sl.Camera()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit(-1)

        self.runtime = sl.RuntimeParameters()
        # self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        self.image_size = self.zed.get_camera_information().camera_configuration.resolution
        self.image_size.width = self.image_size.width /2
        self.image_size.height = self.image_size.height /2
        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        #ROS2
        self.raw_img_pub_ = self.create_publisher(Image, '/aiformula_sensing/zedx_image_publisher/image_raw', 1)
        self.commpressed_img_pub_ = self.create_publisher(CompressedImage, '/aiformula_sensing/zedx_image_publisher/image_raw/compressed', 10)
        timer_raw = 1/30
        self.timer_image = self.create_timer(timer_raw, self.image_callback)
        self.br = CvBridge()
      
    def image_callback(self):
        self.err = self.zed.grab(self.runtime)
        if self.err == sl.ERROR_CODE.SUCCESS :
            image_sl_left = sl.Mat()
            self.zed.retrieve_image(image_sl_left, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
            image_cv_left = image_sl_left.get_data()
            pub_img = self.br.cv2_to_imgmsg(cv2.cvtColor(image_cv_left, cv2.COLOR_BGRA2BGR), 'bgr8')

            msg = CompressedImage()
            msg.format = "jpeg"
            ret, compressed_img = cv2.imencode(".jpg", image_cv_left, [int(cv2.IMWRITE_JPEG_QUALITY),30])
            msg.data = compressed_img.tostring()

            pub_img.header.stamp = self.get_clock().now().to_msg()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.raw_img_pub_.publish(pub_img)
            self.commpressed_img_pub_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    zedx_image_publisher = ZedXImagePublisher()
    rclpy.spin(zedx_image_publisher)
    zedx_image_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

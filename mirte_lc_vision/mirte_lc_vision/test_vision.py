'''
ros2 launch mirte_lc_vision vision_test
'''

import open3d
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Image
from cv_bridge import CvBridge

class MirteTestVision(Node):

    def __init__(self):
        super().__init__('vision_test')

        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.get_img_callback, 1
        )

        self.img_pub = self.create_publisher(
            Image, "/camera/processed_img", 10
        )

        self.img_timer = self.create_timer(0.02, self.img_callback)
        self.img = None
        self.bridge = CvBridge()
    
    def get_img_callback(self, msg:Image):
        self.get_logger().info(f'Received image  of size {msg.height}x{msg.width} ')
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'e')

    def img_callback(self):
        if self.img is not None:
            overlay = self.img.copy()
            overlay[100:200, 100:200] = (0, 255, 0)

            overlayed = cv2.addWeighted(overlay, 0.3, self.img, 1 - 0.3, 0, self.img)

            img_msg = Image()
            img_msg = self.bridge.cv2_to_imgmsg(overlayed, encoding='bgr8')
            self.img_pub.publish(img_msg)
            self.get_logger().info(f'Overlayed Image: {overlayed}')

        else:
            self.get_logger().info('Did not receive image from topic')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MirteTestVision())
    rclpy.shutdown()

if __name__== '__main__':
    main()
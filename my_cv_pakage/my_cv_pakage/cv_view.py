#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CvViewNode(Node):
    def __init__(self):
        super().__init__('cv_view')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.get_logger().info('cv_view node started and subscribed to /camera/image_raw')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CvViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


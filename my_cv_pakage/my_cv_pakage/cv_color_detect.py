#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CvColorDetect(Node):
    def __init__(self):
        super().__init__('color_detect')
        self.get_logger().info("Node has started!")

        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # ✅ Make sure this matches the simulator!
            self.convert_callback,
            10
        )

        # HSV thresholds
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])

        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 25, 255])

    def convert_callback(self, msg):
        print("Callback triggered!")

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Apply masks
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
            combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

            # Apply mask to original image
            result = cv2.bitwise_and(cv_image, cv_image, mask=combined_mask)

            # Display the result
            cv2.imshow("Yellow and White Lines", result)
            cv2.waitKey(1)

        except:
            print("Something went wrong during image processing.")

def main(args=None):
    rclpy.init(args=args)
    node = CvColorDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


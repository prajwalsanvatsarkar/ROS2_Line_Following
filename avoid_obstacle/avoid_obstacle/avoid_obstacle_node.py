# !/usr/bin/env/ python3

import rclpy
import rclpy.node as Node
import geometry_msgs.msg as Twist
import sensor_msgs.msg as iMage, Laserscan
from cv_bridge import CvBridge
import numpy as np
import cv2

class AvoidObstacleNode(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Image,'/image/raw',self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)

        self.bridge = CvBridge()

        self.state = 'FOLLOW_LINE'

    def image_callback(self, msg):
   
    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    height, width, _ = frame.shape
    center_x = width // 2

   
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

   
    white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

   
    roi = white_mask[height // 2 :, :]

   
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
       
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])  # x-coordinate of centroid
            cy = int(M["m01"] / M["m00"])
            self.line_center_x = cx
            self.line_detected = True
            return
   
    self.line_center_x = -1
    self.line_detected = False

        

    def scan_callback(self,msg):
        center_index = len(msg.ranges) // 2
        window = msg.ranges[center_index - 10, center_index +10]
        valid_ranges = [r for r in window if not np.isnan(r) and not np.isinf(r)]
        self.obstacle_in_front = any(r < 0.3 for r in valid_ranges)


def main():
    rclpy.init(args=args)
    node= AvoidObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




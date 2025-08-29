#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge


class FollowCenterLaneNode(Node):
    def __init__(self):
        super().__init__('follow_center_lane_node')
        self.get_logger().info('Lane Centering (Contour-based) with Obstacle Fix Started')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.obstacle_in_front = False

        self.lower_yellow = np.array([10, 70, 70])
        self.upper_yellow = np.array([40, 255, 255])
        self.lower_white = np.array([0, 0, 160])
        self.upper_white = np.array([180, 40, 255])

    def lidar_callback(self, msg):
        center_index = len(msg.ranges) // 2
        window = msg.ranges[center_index - 10:center_index + 10]
        valid_ranges = [r for r in window if not np.isnan(r) and not np.isinf(r)]
        self.obstacle_in_front = any(r < 0.3 for r in valid_ranges)

    def get_contour_data(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        else:
            return None

    def image_callback(self, msg):
        twist = Twist()

        if self.obstacle_in_front:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn("Obstacle detected! Robot stopped.")
            self.pub.publish(twist)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape
        center_x = width // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

        yellow_center = self.get_contour_data(yellow_mask[:, :width // 2])
        white_center = self.get_contour_data(white_mask[:, width // 2:])

        if yellow_center and white_center:
            yellow_x = yellow_center[0]
            white_x = white_center[0] + width // 2
            line_position = (yellow_x + white_x) // 2
        elif yellow_center:
            line_position = yellow_center[0]
        elif white_center:
            line_position = white_center[0] + width // 2
        else:
            line_position = -1

        if line_position != -1:
            error = center_x - line_position
            twist.linear.x = 0.05
            twist.angular.z = error * 0.002
            self.get_logger().info(f"Line Center: {line_position}, Error: {error}")
            cv2.circle(frame, (line_position, height - 30), 5, (0, 0, 255), -1)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self.get_logger().warn("No line detected. Searching...")

        self.pub.publish(twist)

        # Optional debug view
        # cv2.imshow("View", frame)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FollowCenterLaneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

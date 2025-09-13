#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class FastBotController(Node):
    def __init__(self):
        super().__init__('fastbot_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/fastbot_1/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/fastbot_1/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        self.last_error = 0.0
        self.max_error = 0.0  # ✅ Fix: Initialize max_error

        self.get_logger().info("✅ FastBotController initialized.")

    def image_callback(self, msg: Image):
        try:
            # Convert and resize image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            scale = 2.0
            image = cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            height, width, _ = image.shape

            # Convert to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # HSV mask
            hsv_ranges = [
                ((97, 120, 98), (115, 154, 121)),
                ((97, 120, 97), (115, 146, 121)),
                ((100, 132, 100), (106, 152, 125)),
                ((100, 139, 100), (106, 155, 124)),
                ((100, 135, 98), (106, 153, 124)),
                ((99, 122, 86), (103, 163, 121)),
                ((99, 126, 86), (104, 168, 118)),
                ((99, 151, 82), (105, 179, 95)),
                ((100, 134, 81), (103, 179, 119))
            ]

            combined_mask = None
            for lower, upper in hsv_ranges:
                mask = cv2.inRange(hsv, lower, upper)
                combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            twist = Twist()

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)

                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Visual overlay
                    cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(image, f"Centroid: ({cx}, {cy})", (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                    # === PID Control ===
                    error_x = cx - width // 2
                    derivative = error_x - self.last_error
                    self.last_error = error_x

                    # Tuned Parameters
                    Kp = 0.0156 #increase this 
                    Kd = 0.00734225
                    max_speed = 0.69
                    min_speed = 0.25

                    angular_z = -Kp * error_x - Kd * derivative
                    angular_z = max(min(angular_z, 2.4), -2.4)

                    # Adaptive speed
                    error_ratio = min(abs(error_x) / (width / 2), 1.0)
                    linear_x = max_speed - (max_speed - min_speed) * (error_ratio ** 1.5)

                    # Track max error
                    if abs(error_x) > self.max_error:
                        self.max_error = abs(error_x)

                    print(f"[DEBUG] error_x: {error_x}, max_error: {self.max_error}")

                    twist.linear.x = linear_x
                    twist.angular.z = angular_z

                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)

            # Show preview
            cv2.imshow("Track Follower", image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FastBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

#colcon build
#source install/setup.bash
#ros2 run follow_track follow_node
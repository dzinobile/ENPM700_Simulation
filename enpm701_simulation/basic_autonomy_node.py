import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 2.0

class BasicAutonomyNode(Node):
    def __init__(self):
        super().__init__('basic_autonomy_node')
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self._grip_pub = self.create_publisher(Twist, 'gripper_vel', 1)
        self._lift_pub = self.create_publisher(Twist, 'gripper_vel', 1)
        self._image_pub = self.create_publisher(Image, 'processed_image', 1)
        self._image_sub = self.create_subscription(Image, 'my_robot/camera/image_color', self.grab_block_callback, 1)

        self._bridge = CvBridge()
        self._grip_closed = False
        self._lift_up = False

    def publish_move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    def publish_grip(self):
        msg = Twist()
        msg.linear.x = 0.03 if self._grip_closed else 0.0
        msg.linear.y = 0.005 if self._lift_up else 0.0
        self._grip_pub.publish(msg)

    def publish_lift(self):
        msg = Twist()
        msg.linear.x = 0.03 if self._grip_closed else 0.0
        msg.linear.y = 0.005 if self._lift_up else 0.0
        self._lift_pub.publish(msg)

    def grab_block_callback(self, ros_img):
        cv_img = self._bridge.imgmsg_to_cv2(ros_img)
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # hsv_img[0:220,:] = (0,0,0)
        blue_lower = np.array([80, 115, 62])
        blue_upper = np.array([140, 255, 255])
        blue_mask = hsv_img.copy()
        blue_mask = cv2.inRange(blue_mask, blue_lower, blue_upper)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(blue_contours) > 0:
            for blue_contour in blue_contours:
                rect = cv2.minAreaRect(blue_contour)
                (rect_x, rect_y),(rect_w, rect_h), rect_a = rect
                if rect_w > rect_h:
                    rect_w, rect_h = rect_h, rect_w
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv_img = cv2.drawContours(cv_img, [box], 0, (0, 0, 0), 1)
                centroid = (int(rect_x), int(rect_y))
                cv2.circle(cv_img, centroid, 3, (0, 0, 0), -1)
                cv2.putText(cv_img, f"({centroid[0]}, {centroid[1]})", (centroid[0] + 5, centroid[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        ros_img = self._bridge.cv2_to_imgmsg(cv_img)
        self._image_pub.publish(ros_img)

def main(args=None):
    rclpy.init(args=args)
    node = BasicAutonomyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
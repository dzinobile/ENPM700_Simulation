import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.duration import Duration

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 2.0

class BasicAutonomyNode(Node):
    def __init__(self):
        super().__init__('basic_autonomy_node')
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self._grip_pub = self.create_publisher(Twist, 'gripper_vel', 1)
        self._image_pub = self.create_publisher(Image, 'processed_image', 1)
        self._image_sub = self.create_subscription(Image, 'my_robot/camera/image_color', self.camera_callback, 1)
        self._left_encoder_sub = self.create_subscription(Float64, 'wheel_position/left', self.left_encoder_callback, 1)
        self._right_encoder_sub = self.create_subscription(Float64, 'wheel_position/right', self.right_encoder_callback, 1)
        self._imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 1)
        self._bridge = CvBridge()
        self._left_wheel_pos = 0.0
        self._right_wheel_pos = 0.0
        self._imu_msg = Imu()
        self._robot_pos = [-1.2192, -1.2192, 0]
        self._linear_speed = 0.5
        self._angular_speed = 2.0
        self._current_state = "go to block"
        self._checkpoint_time = self.get_clock().now()


    def left_encoder_callback(self, msg):
        self._left_wheel_pos = msg
    
    def right_encoder_callback(self, msg):
        self._right_wheel_pos = msg

    def imu_callback(self, msg):
        self._imu_msg = msg

    def state_behavior(self, rect_x, rect_y, cols):
        cmd_msg = Twist()
        grip_msg = Twist()

        if self._current_state == "go to block":
            if rect_x:
                if rect_y >= 420:
                    self._current_state = "grab block"
                    cmd_msg.linear.x = 0.0
                    cmd_msg.angular.z = 0.0
                    grip_msg.linear.x = 0.03
                    self._checkpoint_time = self.get_clock().now()
                else:
                    normalised_angle_error = (rect_x - cols / 2.0) / (cols / 2.0)
                    normalised_speed_error = ((1 - abs(normalised_angle_error))/2.0) * ((420 - rect_y)/420)
                    cmd_msg.linear.x = self._linear_speed * normalised_speed_error
                    cmd_msg.angular.z = -self._angular_speed * normalised_angle_error
                    grip_msg.linear.x = 0.0
            else:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = self._angular_speed
                grip_msg.linear.x = 0.0

        elif self._current_state == "grab block":
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            grip_msg.linear.x = 0.03
            if (self.get_clock().now() - self._checkpoint_time).nanoseconds * 1e-9 > 1.0:
                self._checkpoint_time = self.get_clock().now()
                self._current_state = "go to drop off"

        elif self._current_state == "go to drop off":
            cmd_msg.linear.x = self._linear_speed
            cmd_msg.angular.z = 0.0
            grip_msg.linear.x = 0.03
            if (self.get_clock().now() - self._checkpoint_time).nanoseconds * 1e-9 > 2.0:
                self._checkpoint_time = self.get_clock().now()
                self._current_state = "drop off block"


        elif self._current_state == "drop off block":
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            grip_msg.linear.x = 0.0
            if (self.get_clock().now() - self._checkpoint_time).nanoseconds * 1e-9 > 1.0:
                self._checkpoint_time = self.get_clock().now()
                self._current_state = "back away from block"

        elif self._current_state == "back away from block":
            cmd_msg.linear.x = -self._linear_speed
            cmd_msg.angular.z = 0.0
            grip_msg.linear.x = 0.0
            if (self.get_clock().now() - self._checkpoint_time).nanoseconds * 1e-9 > 2.0:
                self._current_state = "go to block"
        
        self._cmd_vel_pub.publish(cmd_msg)
        self._grip_pub.publish(grip_msg)
        self.get_logger().info(self._current_state)


    def camera_callback(self, ros_img):
        cmd_msg = Twist()

        cv_img = self._bridge.imgmsg_to_cv2(ros_img)

        rows, cols, _ = cv_img.shape
        rect_x = None
        rect_y = None

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        blue_lower = np.array([108, 105, 10])
        blue_upper = np.array([130, 255, 255])
        red_lower1 = np.array([190, 100, 100])
        red_upper1 = np.array([255, 255, 255])
        red_lower2 = np.array([0, 190, 195])
        red_upper2 = np.array([255, 255, 255])
        green_lower = np.array([40, 0, 0])
        green_upper = np.array([100, 255, 255])


        blue_mask = hsv_img.copy()
        blue_mask = cv2.inRange(blue_mask, blue_lower, blue_upper)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(blue_contours) > 0:
            rect = cv2.minAreaRect(blue_contours[0])
            (rect_x, rect_y),(rect_w, rect_h), rect_a = rect
            if rect_w > rect_h:
                rect_w, rect_h = rect_h, rect_w
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv_img = cv2.drawContours(cv_img, [box], 0, (0, 0, 0), 1)
            centroid = (int(rect_x), int(rect_y))
            cv2.circle(cv_img, centroid, 3, (0, 0, 0), -1)
            cv2.putText(cv_img, f"robot pos: {self._robot_pos}", (20, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        
        
        self.state_behavior(rect_x, rect_y, cols)
        
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
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
        self._robot_pos = [-1.2192, -1.2192, 0.0]
        self._linear_speed = 0.5
        self._angular_speed = 2.0
        self._current_state = "go to block"
        self._checkpoint_time = self.get_clock().now()
        self._left_wheel_distance = 0.0
        self._right_wheel_distance = 0.0
        self._dropoff_coords = [-0.9144, 0.9144]
        self._start_angle = 0.0


    def left_encoder_callback(self, msg):
        self._left_wheel_pos = msg.data

    def right_encoder_callback(self, msg):
        self._right_wheel_pos = msg.data

    def imu_callback(self, msg):
        self._imu_msg = msg

    def quaternion_to_euler(selfl, x, y, z, w):
        norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def update_position(self):
        new_left_total_distance = (0.032 * self._left_wheel_pos)
        new_right_total_distance = (0.032 * self._right_wheel_pos)
        left_distance_traveled = new_left_total_distance - self._left_wheel_distance
        right_distance_traveled = new_right_total_distance - self._right_wheel_distance
        average_distance_traveled = (left_distance_traveled + right_distance_traveled) / 2.0
        self._left_wheel_distance = new_left_total_distance
        self._right_wheel_distance = new_right_total_distance

        q = self._imu_msg.orientation
        yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self._robot_pos[2] = round(yaw, 4)
        x = self._robot_pos[0] + (average_distance_traveled * np.cos(yaw))
        y = self._robot_pos[1] + (average_distance_traveled * np.sin(yaw))
        self._robot_pos[0] = round(x, 4)
        self._robot_pos[1] = round(y, 4)

    def turn_to_angle(self, angle):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        angle_error = self._robot_pos[2] - angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        normalized_angle_error = angle_error / np.pi
        cmd_msg.angular.z = -self._angular_speed * normalized_angle_error
        return cmd_msg
    
    def move_forward_straight(self, distance, angle):
        cmd_msg = Twist()
        angle_error = self._robot_pos[2] - angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        normalized_angle_error = angle_error / np.pi
        cmd_msg.angular.z = -self._angular_speed * normalized_angle_error

        cmd_msg.linear.x = min(self._linear_speed, self._linear_speed * distance)

    def turn_to_point(self, point):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        x2, y2 = point[0], point[1]
        x1, y1 = self._robot_pos[0], self._robot_pos[1]
        angle = np.arctan2((y2-y1),(x2-x1))
        angle_error = self._robot_pos[2] - angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        normalized_angle_error = angle_error / np.pi
        cmd_msg.angular.z = -self._angular_speed * normalized_angle_error
        return angle_error, cmd_msg
    
    def forward_to_point(self, point, angle):
        cmd_msg = Twist()
        x2, y2 = point[0], point[1]
        x1, y1 = self._robot_pos[0], self._robot_pos[1]
        angle_error = self._robot_pos[2] - angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        normalized_angle_error = angle_error / np.pi
        cmd_msg.angular.z = -self._angular_speed * normalized_angle_error
        distance_error = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        cmd_msg.linear.x = min(self._linear_speed, distance_error * self._linear_speed)
        return distance_error, cmd_msg
        
    # def move_to_point(self, point):
    #     x2, y2 = point[0], point[1]
    #     x1, y1 = self._robot_pos[0], self._robot_pos[1]
    #     req_angle = np.arctan2((y2-y1),(x2-x1))
    #     cmd_msg = Twist()
    #     angle_error = self._robot_pos[2] - req_angle
    #     angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
    #     distance_error = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    #     normalized_angle_error = angle_error / np.pi
    #     cmd_msg.angular.z = -self._angular_speed * normalized_angle_error

    #     cmd_msg.linear.x = distance_error * self._linear_speed * np.exp(-30 * abs(normalized_angle_error))

    #     return distance_error, cmd_msg

    def state_behavior(self, rect_x, rect_y, cols):
        cmd_msg = Twist()
        grip_msg = Twist()
        self.update_position()
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
                self._current_state = "turn to drop off"

        elif self._current_state == "turn to drop off":
            grip_msg.linear.x = 0.03
            angle_error, cmd_msg = self.turn_to_point(self._dropoff_coords)
            if abs(angle_error) < 0.03:
                self._current_state = "go to drop off"
                self._checkpoint_time = self.get_clock().now()
                self._start_angle = self._robot_pos[2]


        elif self._current_state == "go to drop off":
            distance_error, cmd_msg = self.forward_to_point(self._dropoff_coords, self._start_angle)
            grip_msg.linear.x = 0.03
            print(distance_error)
            if distance_error < 0.06:
                self._current_state = "drop off block"
                self._checkpoint_time = self.get_clock().now()



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
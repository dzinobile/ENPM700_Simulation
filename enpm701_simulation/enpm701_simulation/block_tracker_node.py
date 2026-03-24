import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float64


BOUNDS = {
    'red':   {'lower': np.array([171, 145, 47]), 'upper': np.array([255, 255, 255]),
              'lower2': np.array([0, 175, 193]),  'upper2': np.array([10, 255, 255])},
    'green': {'lower': np.array([39, 0, 10]),     'upper': np.array([92, 255, 255])},
    'blue':  {'lower': np.array([100, 109, 5]),     'upper': np.array([130, 255, 255])},
}



class BlockTracker(Node):
    def __init__(self):
        super().__init__('block_tracker_node')
        self._image_pub = self.create_publisher(Image, 'block_tracker_image', 1)
        self._image_sub = self.create_subscription(
            Image, 'my_robot/camera/image_color', self._callback, 1)
        self._bridge = CvBridge()
        self._left_encoder_sub = self.create_subscription(Float64, 'wheel_position/left', self.left_encoder_callback, 1)
        self._right_encoder_sub = self.create_subscription(Float64, 'wheel_position/right', self.right_encoder_callback, 1)
        self._imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 1)
        self._left_wheel_pos = 0.0
        self._right_wheel_pos = 0.0
        self._imu_msg = Imu()
        self._robot_pos = [-1.2192, -1.2192, 0.0]
        self._left_wheel_distance = 0.0
        self._right_wheel_distance = 0.0

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

    def estimate_block_position(self, rect_x, rect_h):
        # Radial distance from empirical fit to rect_h (bounding box height in pixels)
        block_distance = 24.805 * (rect_h ** -0.821)

        # Lateral bearing: pixels right of centre → clockwise (negative in robot frame)
        # FOV = 0.7854 rad horizontal over 640 px
        bearing = -(rect_x - 320.0) * (0.7854 / 640.0)

        yaw = self._robot_pos[2]
        block_angle = yaw + bearing
        block_x = self._robot_pos[0] + (block_distance * np.cos(block_angle))
        block_y = self._robot_pos[1] + (block_distance * np.sin(block_angle))
        return block_x, block_y

    def _callback(self, ros_img):
        self.update_position()
        cv_img = self._bridge.imgmsg_to_cv2(ros_img)
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        for color in ['red', 'green', 'blue']:
            b = BOUNDS[color]
            if color == 'red':
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, b['lower'], b['upper']),
                    cv2.inRange(hsv, b['lower2'], b['upper2'])
                )
                n = 24.805
                e = -0.821
            else:
                mask = cv2.inRange(hsv, b['lower'], b['upper'])

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                for i in range(0, min(3, len(contours))):
                    rect = cv2.minAreaRect(contours[i])
                    (rect_x, rect_y),(rect_w, rect_h), rect_a = rect
                    if rect_a > 25:
                        if rect_w > rect_h:
                            rect_w, rect_h = rect_h, rect_w
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv_img = cv2.drawContours(cv_img, [box], 0, (0, 0, 0), 1)
                        centroid = (int(rect_x), int(rect_y))
                        cv2.circle(cv_img, centroid, 3, (0, 0, 0), -1)
                        block_position = self.estimate_block_position(rect_x, rect_h)
                        cv2.putText(cv_img, f"{color} block at ({block_position[0]:.2f}, {block_position[1]:.2f})", (centroid[0]+5, centroid[1]-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                self._image_pub.publish(self._bridge.cv2_to_imgmsg(cv_img))


def main(args=None):
    rclpy.init(args=args)
    node = BlockTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

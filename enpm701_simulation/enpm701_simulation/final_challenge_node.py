import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Pose2D, Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
import itertools
import time
import cv2
from cv_bridge import CvBridge
import numpy as np

BOUNDS = {
    'red':   {'lower': np.array([171, 145, 47]), 'upper': np.array([255, 255, 255]),
              'lower2': np.array([0, 175, 193]),  'upper2': np.array([10, 255, 255])},
    'green': {'lower': np.array([39, 0, 10]),     'upper': np.array([92, 255, 255])},
    'blue':  {'lower': np.array([100, 109, 5]),     'upper': np.array([130, 255, 255])},
}

class FinalChallengeNode(Node):
    def __init__(self):
        super().__init__('final_challenge_node')

        # ReentrantCallbackGroup allows spin_until_future_complete inside a timer callback
        self._reentrant = ReentrantCallbackGroup()

        self._position_sub = self.create_subscription(Pose2D, 'robot_position', self._position_cb, 1)
        self._image_sub = self.create_subscription(Image, 'my_robot/camera/image_color', self._camera_callback, 1)
        self.create_subscription(MarkerArray, 'block_markers', self._markers_cb, 1)
        self.create_timer(0.1, self._execute, callback_group=self._reentrant)  # 10 Hz

        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self._grip_pub = self.create_publisher(Float64, 'cmd_grip', 1)
        self._plan_client = self.create_client(GetPlan, 'plan_path',
                                               callback_group=self._reentrant)
        self._tracking_active_pub = self.create_publisher(Bool, 'block_tracking_active', 1)
        self._remove_block_pub = self.create_publisher(Marker, 'remove_block', 1)

        self._robot_pos = [-1.2192, -1.2192, 0.0]
        self._linear_speed = 0.5
        self._angular_speed = 2.0
        self._bridge = CvBridge()
        self._current_state = "scan for blocks"
        self._color_cycle = itertools.cycle(['red', 'green', 'blue'])
        self._current_color = next(self._color_cycle)
        self._dropoff_distance = 0.6096
        self._block_frame_position = None
        self._target_block_pos = None   # world (x, y) of block being approached
        self._checkpoint_time = self.get_clock().now()
        self._checkpoint_pos = [-1.2192, -1.2192, 0.0]
        self._blocks = {'red': [], 'green': [], 'blue': []}
        self._waypoints = []   # list of (x, y) world coords for current path
        self._waypoint_idx = 0
        self._planning = False

    def _position_cb(self, msg):
        self._robot_pos = [msg.x, msg.y, msg.theta]

    def _markers_cb(self, msg):
        blocks = {'red': [], 'green': [], 'blue': []}
        for marker in msg.markers:
            if marker.ns in blocks:
                blocks[marker.ns].append((marker.pose.position.x, marker.pose.position.y))
        self._blocks = blocks

    def _camera_callback(self, ros_img):
        cv_img = self._bridge.imgmsg_to_cv2(ros_img)
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        b = BOUNDS[self._current_color]

        if self._current_color == 'red':
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, b['lower'], b['upper']),
                cv2.inRange(hsv, b['lower2'], b['upper2'])
            )
        else:
            mask = cv2.inRange(hsv, b['lower'], b['upper'])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            rect = cv2.minAreaRect(contours[0])
            (rect_x, rect_y), (rect_w, rect_h), rect_a = rect
            self._block_frame_position = (rect_x, rect_y)
        else:
            self._block_frame_position = None
        


    def _execute(self):
        print(self._current_state)
        if self._current_state == "scan for blocks":
            self._tracking_active_pub.publish(Bool(data=True))
            self._scan_for_blocks()
            self.get_logger().info("scanning for blocks...")
        elif self._current_state == "go to block":
            self._tracking_active_pub.publish(Bool(data=True))
            self._go_to_block()
            self.get_logger().info("going to block")

        elif self._current_state == "align with block":
            self._tracking_active_pub.publish(Bool(data=False))
            self._align_with_block()
            self.get_logger().info("aligning with block")

        elif self._current_state == "grip block":
            self._tracking_active_pub.publish(Bool(data=False))
            self._grip_block()
            self.get_logger().info("gripping block")

        elif self._current_state == "go to dropoff":
            self._tracking_active_pub.publish(Bool(data=False))
            self._go_to_dropoff()
            self.get_logger().info("going to dropoff")

        elif self._current_state == "align with dropoff":
            self._tracking_active_pub.publish(Bool(data=False))
            self._align_with_dropoff()
            self.get_logger().info("aligning with dropoff")

        elif self._current_state == "release block":
            self._tracking_active_pub.publish(Bool(data=False))
            self._release_block()
            self.get_logger().info("releasing block")

        elif self._current_state == "back away from dropoff":
            self._tracking_active_pub.publish(Bool(data=False))
            self._back_away_from_dropoff()
            self.get_logger().info("backing away from dropoff")
        else:
            self.get_logger().error(f"Unknown state: {self._current_state}")

    def _scan_for_blocks(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        if abs(self._robot_pos[2] - self._checkpoint_pos[2]) >= np.pi - 0.1:  
            self._current_state = "go to block"
            cmd_msg.angular.z = 0.0
        else:
            cmd_msg.angular.z = self._angular_speed/2.0

        self._cmd_vel_pub.publish(cmd_msg)

    def _back_away_from_dropoff(self):
        cmd_msg = Twist()
        if abs(self._robot_pos[1] - self._checkpoint_pos[1]) >= 0.3048:
            self._current_state = "scan for blocks"
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(cmd_msg)
            if self._current_color == "blue":
                self._dropoff_distance -= 0.3048  # next block needs less distance to dropoff since stack grows
            self._current_color = next(self._color_cycle)
        else:
            cmd_msg.linear.x = -self._linear_speed
            cmd_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(cmd_msg)
    
    def request_path(self, goal_x, goal_y): 
        if not self._plan_client.service_is_ready():
            self.get_logger().warn('plan_path service not available')
            return None
        req = GetPlan.Request()
        req.goal = PoseStamped()
        req.goal.pose.position.x = goal_x
        req.goal.pose.position.y = goal_y
        future = self._plan_client.call_async(req)
        # busy-wait — lets the MultiThreadedExecutor process the response on its other thread
        while rclpy.ok() and not future.done():
            time.sleep(0.005)
        if future.result() is not None:
            return future.result().plan.poses
        return None
        
    
    def _grip_block(self):

        grip_msg = Float64()
        grip_msg.data = 0.03  # Adjust as needed for your gripper
        self._grip_pub.publish(grip_msg)
        if (self.get_clock().now() - self._checkpoint_time).nanoseconds * 1e-9 > 1.0:
            self._checkpoint_time = self.get_clock().now()
            self._current_state = "go to dropoff"
    
    def _align_with_block(self):
        if self._block_frame_position is None:
            return  # no block visible yet — wait for camera callback
        cmd_msg = Twist()
        grip_msg = Float64()

        rect_x, rect_y = self._block_frame_position
        if rect_y >= 450:
            self._current_state = "grip block"
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(cmd_msg)
            self._checkpoint_time = self.get_clock().now()
            if self._target_block_pos is not None:
                m = Marker()
                m.ns = self._current_color
                m.pose.position.x = self._target_block_pos[0]
                m.pose.position.y = self._target_block_pos[1]
                self._remove_block_pub.publish(m)

        else:
            normalised_angle_error = (rect_x - 640 / 2.0) / (640 / 2.0)
            normalised_speed_error = ((1 - abs(normalised_angle_error))/2.0) * ((450 - rect_y)/450)
            cmd_msg.linear.x = self._linear_speed * normalised_speed_error
            cmd_msg.angular.z = -self._angular_speed * normalised_angle_error
            grip_msg.data = 0.0
            self._grip_pub.publish(grip_msg)
            self._cmd_vel_pub.publish(cmd_msg)


    def _align_with_dropoff(self):
        if self._checkpoint_pos is None:
            self._checkpoint_pos = list(self._robot_pos)  # capture position on first entry

        cmd_msg = Twist()
        grip_msg = Float64()
        grip_msg.data = 0.03
        self._grip_pub.publish(grip_msg)

        if (np.pi/2.0) - 0.05 <= self._robot_pos[2] <= (np.pi/2.0) + 0.05:
            distance_traveled = self._robot_pos[1] - self._checkpoint_pos[1]  # heading north → +y
            if distance_traveled >= self._dropoff_distance:
                self._current_state = "release block"
                self._checkpoint_pos = None  # reset for next dropoff
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self._cmd_vel_pub.publish(cmd_msg)
                self._checkpoint_time = self.get_clock().now()
            else:
                cmd_msg.linear.x = self._linear_speed/3.0
                cmd_msg.angular.z = 0.0
                self._cmd_vel_pub.publish(cmd_msg)
        else:
            angle_error = (np.pi/2.0) - self._robot_pos[2]
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self._angular_speed * angle_error
            self._cmd_vel_pub.publish(cmd_msg)

    def _release_block(self):
        cmd_msg = Twist()
        grip_msg = Float64()
        grip_msg.data = 0.0
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self._cmd_vel_pub.publish(cmd_msg)
        self._grip_pub.publish(grip_msg)
        if (self.get_clock().now() - self._checkpoint_time).nanoseconds * 1e-9 > 1.0:

            
            self._checkpoint_pos = self._robot_pos
            self._current_state = "back away from dropoff"


    def _go_to_block(self):
        if not self._waypoints:
            targets = self._blocks.get(self._current_color, [])
            if not targets:
                self._current_state = "scan for blocks"
                return  # no confirmed block yet — stay in state

            # pick nearest block of current color
            rx, ry = self._robot_pos[0], self._robot_pos[1]
            bx, by = min(targets, key=lambda p: np.hypot(p[0] - rx, p[1] - ry))
            self._target_block_pos = (bx, by)

            # approach point: 0.3048 m from block along the robot→block axis
            dx, dy = rx - bx, ry - by
            dist = max(np.hypot(dx, dy), 1e-6)
            ax = bx + (dx / dist) * 0.3048
            ay = by + (dy / dist) * 0.3048

            poses = self.request_path(ax, ay)
            if not poses:
                return
            self._waypoints = [(p.pose.position.x, p.pose.position.y) for p in poses]
            self._waypoint_idx = 0

        self._follow_waypoints(next_state='align with block')

    def _go_to_dropoff(self):
        self._target_block_pos = None  # no facing step after dropoff path
        if not self._waypoints:
            if self._planning:
                return 
            self._planning = True

            if self._current_color == 'red':
                goal_x = -1.2192
            elif self._current_color == 'green':
                goal_x = -0.9144
            else:
                goal_x = -0.6096
            goal_y = 0.6096

            poses = self.request_path(goal_x, goal_y)
            self._planning = False
            if not poses:
                return
            self._waypoints = [(p.pose.position.x, p.pose.position.y) for p in poses]
            self._waypoint_idx = 0

        self._follow_waypoints(next_state='align with dropoff')

    def _follow_waypoints(self, next_state):
        """Drive toward the current waypoint; advance when within tolerance."""
        WAYPOINT_TOL = 0.1   # metres — close enough to advance to next waypoint
        HEADING_TOL  = 0.2   # radians — rotate in place if heading error exceeds this

        if self._waypoint_idx >= len(self._waypoints):
            # If a target block is set, rotate to face it before transitioning
            if self._target_block_pos is not None:
                rx, ry, ryaw = self._robot_pos
                bx, by = self._target_block_pos
                target_heading = np.arctan2(by - ry, bx - rx)
                heading_error = (target_heading - ryaw + np.pi) % (2 * np.pi) - np.pi
                if abs(heading_error) > 0.1:
                    cmd = Twist()
                    cmd.angular.z = self._angular_speed * np.clip(heading_error, -1.0, 1.0)
                    self._cmd_vel_pub.publish(cmd)
                    return  # keep rotating until aligned
            self._waypoints = []
            self._current_state = next_state

            self._checkpoint_pos = self._robot_pos
            self._cmd_vel_pub.publish(Twist())  # stop
            return

        wx, wy = self._waypoints[self._waypoint_idx]
        rx, ry, ryaw = self._robot_pos
        dx, dy = wx - rx, wy - ry
        dist = np.hypot(dx, dy)

        if dist < WAYPOINT_TOL:
            self._waypoint_idx += 1
            return

        target_heading = np.arctan2(dy, dx)
        heading_error = (target_heading - ryaw + np.pi) % (2 * np.pi) - np.pi

        cmd = Twist()
        if abs(heading_error) > HEADING_TOL:
            # rotate in place to face the waypoint before moving
            cmd.angular.z = self._angular_speed * np.clip(heading_error, -1.0, 1.0)
        else:
            cmd.linear.x  = self._linear_speed * min(dist, 1.0)
            cmd.angular.z = self._angular_speed * heading_error
        self._cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FinalChallengeNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





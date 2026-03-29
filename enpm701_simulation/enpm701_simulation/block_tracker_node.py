import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Bool, Float64
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D

CONSTRUCTION_X = (-1.524, -0.3048)
CONSTRUCTION_Y = (0.3048, 1.524)
CLUSTER_THRESHOLD = 0.3  # metres — detections within this radius merge into one cluster
CONFIRM_FRAMES = 10      # consecutive frames a detection must be stable before counting
EDGE_MARGIN = 40         # pixels — centroids closer than this to any edge are skipped

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
        self._map_pub = self.create_publisher(Image, 'block_map_image', 1)
        self._position_pub = self.create_publisher(Pose2D, 'robot_position', 1)
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
        # Each entry: {'x_sum': float, 'y_sum': float, 'count': int}
        self._block_positions = {'red': [], 'green': [], 'blue': []}
        # Pending detections not yet confirmed; each entry: {'x': float, 'y': float, 'streak': int}
        self._candidates = {'red': [], 'green': [], 'blue': []}
        self._marker_array_pub = self.create_publisher(MarkerArray, 'block_markers', 1)
        self._tracker_active_sub = self.create_subscription(Bool, 'block_tracking_active', self._tracking_active_callback, 1)
        self.create_subscription(Marker, 'remove_block', self._remove_block_cb, 1)
        self._tracker_active = True
    def _tracking_active_callback(self, msg):
        self._tracker_active = msg.data
    
    def left_encoder_callback(self, msg):
        self._left_wheel_pos = msg.data

    def right_encoder_callback(self, msg):
        self._right_wheel_pos = msg.data

    def imu_callback(self, msg):
        self._imu_msg = msg

    def quaternion_to_euler(self, x, y, z, w):
        norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
    def publish_marker_array(self, block_positions):
        marker_array = MarkerArray()
        for i, (color, x, y) in enumerate(block_positions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            if color == (255, 0, 0):    # blue in BGR
                marker.color.b = 1.0
                marker.ns = "blue"
            elif color == (0, 255, 0):  # green in BGR
                marker.color.g = 1.0
                marker.ns = "green"
            elif color == (0, 0, 255):  # red in BGR
                marker.color.r = 1.0
                marker.ns = "red"
            else:
                continue
            marker.color.a = 1.0  
            marker_array.markers.append(marker)
        self._marker_array_pub.publish(marker_array)

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
        position_msg = Pose2D()
        position_msg.x = self._robot_pos[0]
        position_msg.y = self._robot_pos[1]
        position_msg.theta = self._robot_pos[2]
        self._position_pub.publish(position_msg)

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
    
    def update_block_position(self, color, x, y):
        clusters = self._block_positions[color]
        best, best_dist = None, float('inf')
        for cluster in clusters:
            cx = cluster['x_sum'] / cluster['count']
            cy = cluster['y_sum'] / cluster['count']
            d = np.hypot(x - cx, y - cy)
            if d < best_dist:
                best, best_dist = cluster, d
        if best is not None and best_dist < CLUSTER_THRESHOLD:
            best['x_sum'] += x
            best['y_sum'] += y
            best['count'] += 1
        else:
            clusters.append({'x_sum': x, 'y_sum': y, 'count': 1})

    def _remove_block_cb(self, marker):
        color = marker.ns
        x, y = marker.pose.position.x, marker.pose.position.y
        if color not in self._block_positions:
            return
        clusters = self._block_positions[color]
        if not clusters:
            return
        nearest = min(clusters, key=lambda c: np.hypot(x - c['x_sum'] / c['count'],
                                                        y - c['y_sum'] / c['count']))
        clusters.remove(nearest)
        self._candidates[color] = [
            c for c in self._candidates[color]
            if np.hypot(x - c['x'], y - c['y']) > CLUSTER_THRESHOLD
        ]
        self.get_logger().info(f'Removed {color} block at ({x:.2f}, {y:.2f})')

    def xy_to_pixel(self, x, y):
        pixel_x = int((x + (3.048/2)) / 3.048 * 500)
        pixel_y = 500 - int((y + (3.048/2)) / 3.048 * 500)
        return pixel_x, pixel_y

    def publish_map(self, block_positions):
        # map_img = np.full((500, 500, 3), 255, dtype=np.uint8)
        map_img = np.zeros((500, 500, 3), dtype=np.uint8)
        robot_x, robot_y = self.xy_to_pixel(self._robot_pos[0], self._robot_pos[1])
        cv2.rectangle(map_img, (0, 0), (199, 200), (100, 100, 100), -1)
        cv2.rectangle(map_img, (0, 500), (100, 400), (100, 100, 100), -1)
        cv2.circle(map_img, (robot_x, robot_y), 10, (255, 255, 255), -1)
        cv2.line(map_img, (robot_x, robot_y), (robot_x + int(10 * np.cos(self._robot_pos[2])), robot_y - int(10 * np.sin(self._robot_pos[2]))), (0, 0, 0), 2) 
        for block_position in block_positions:
            c, block_x, block_y = block_position
            pixel_x, pixel_y = self.xy_to_pixel(block_x, block_y)
            cv2.circle(map_img, (pixel_x, pixel_y), 5, c, -1)
        self._map_pub.publish(self._bridge.cv2_to_imgmsg(map_img))

    def _callback(self, ros_img):
        
        self.update_position()
        cv_img = self._bridge.imgmsg_to_cv2(ros_img)
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        color_bgr = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}

        for color in ['red', 'green', 'blue']:
            b = BOUNDS[color]
            bgr = color_bgr[color]
            if color == 'red':
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, b['lower'], b['upper']),
                    cv2.inRange(hsv, b['lower2'], b['upper2'])
                )
            else:
                mask = cv2.inRange(hsv, b['lower'], b['upper'])

            # Collect valid detections for this color this frame
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            frame_detections = []  # (rect, rect_x, rect_h, raw_x, raw_y)
            for i in range(min(3, len(contours))):
                rect = cv2.minAreaRect(contours[i])
                (rect_x, rect_y), (rect_w, rect_h), rect_a = rect
                if rect_a > 25:
                    if rect_w > rect_h:
                        rect_w, rect_h = rect_h, rect_w
                    if (rect_x < EDGE_MARGIN or rect_x > 640 - EDGE_MARGIN or
                            rect_y < EDGE_MARGIN or rect_y > 480 - EDGE_MARGIN):
                        continue  # centroid too close to edge — block likely partially out of frame
                    raw_x, raw_y = self.estimate_block_position(rect_x, rect_h)
                    if (CONSTRUCTION_X[0] <= raw_x <= CONSTRUCTION_X[1] and
                            CONSTRUCTION_Y[0] <= raw_y <= CONSTRUCTION_Y[1]):
                        continue  # block in construction zone — ignore
                    frame_detections.append((rect, rect_x, rect_h, raw_x, raw_y))

            # Match each candidate to the nearest unmatched detection this frame.
            # Unmatched candidates lose their streak and are dropped.
            claimed = set()
            for cand in self._candidates[color]:
                best_di, best_dist = None, float('inf')
                for di, (_, _, _, raw_x, raw_y) in enumerate(frame_detections):
                    if di in claimed:
                        continue
                    d = np.hypot(raw_x - cand['x'], raw_y - cand['y'])
                    if d < best_dist:
                        best_di, best_dist = di, d
                if best_di is not None and best_dist < CLUSTER_THRESHOLD:
                    _, _, _, raw_x, raw_y = frame_detections[best_di]
                    cand['x'], cand['y'] = raw_x, raw_y
                    cand['streak'] += 1
                    claimed.add(best_di)
                else:
                    cand['streak'] = 0  # missed this frame — reset

            if self._tracker_active:
                self._candidates[color] = [c for c in self._candidates[color] if c['streak'] > 0]

                # Unmatched detections start new candidates
                for di, (_, _, _, raw_x, raw_y) in enumerate(frame_detections):
                    if di not in claimed:
                        self._candidates[color].append({'x': raw_x, 'y': raw_y, 'streak': 1})

                # Confirmed candidates (streak >= CONFIRM_FRAMES) update the cluster average
                for cand in self._candidates[color]:
                    if cand['streak'] >= CONFIRM_FRAMES:
                        self.update_block_position(color, cand['x'], cand['y'])

            # Draw all detections; show streak for pending, cluster avg for confirmed
            for rect, rect_x, rect_h, raw_x, raw_y in frame_detections:
                (rect_cx, rect_cy), _, _ = rect
                centroid = (int(rect_cx), int(rect_cy))
                cv_img = cv2.drawContours(cv_img, [np.int0(cv2.boxPoints(rect))], 0, bgr, 1)
                cv2.circle(cv_img, centroid, 3, (0, 0, 0), -1)

                # find the candidate this detection belongs to (nearest)
                cand = min(self._candidates[color],
                        key=lambda c: np.hypot(raw_x - c['x'], raw_y - c['y']),
                        default=None)
                if cand is not None and cand['streak'] >= CONFIRM_FRAMES and self._block_positions[color]:
                    nearest = min(
                        self._block_positions[color],
                        key=lambda cl: np.hypot(raw_x - cl['x_sum'] / cl['count'],
                                                raw_y - cl['y_sum'] / cl['count'])
                    )
                    avg_x = nearest['x_sum'] / nearest['count']
                    avg_y = nearest['y_sum'] / nearest['count']
                    cv2.putText(cv_img, f"{color} ({avg_x:.2f},{avg_y:.2f}) n={nearest['count']}",
                                (centroid[0] + 5, centroid[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                elif cand is not None:
                    cv2.putText(cv_img, f"{color} streak={cand['streak']}/{CONFIRM_FRAMES}",
                                (centroid[0] + 5, centroid[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

        # publish camera image and map from all averaged clusters
        map_positions = [
            (color_bgr[color], cl['x_sum'] / cl['count'], cl['y_sum'] / cl['count'])
            for color, clusters in self._block_positions.items()
            for cl in clusters
        ]
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(cv_img))
        self.publish_map(map_positions)
        self.publish_marker_array(map_positions)


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

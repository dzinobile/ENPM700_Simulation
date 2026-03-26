import heapq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import numpy as np


# Arena is 3.048 x 3.048 m, centred at origin
ARENA_SIZE = 3.048
RESOLUTION = 0.05                            # metres per grid cell
GRID_N = int(ARENA_SIZE / RESOLUTION)        # 60 cells
ORIGIN = -ARENA_SIZE / 2.0                   # -1.524 m

INFLATE_RADIUS = 3   # cells — inflates obstacles by ~15 cm (robot half-width)


def world_to_cell(x, y):
    col = int((x - ORIGIN) / RESOLUTION)
    row = int((y - ORIGIN) / RESOLUTION)
    return col, row


def cell_to_world(col, row):
    x = ORIGIN + (col + 0.5) * RESOLUTION
    y = ORIGIN + (row + 0.5) * RESOLUTION
    return x, y


def astar(grid, start, goal):
    """A* on a flat occupancy grid. Returns list of (col, row) cells or None."""
    if grid[goal[1] * GRID_N + goal[0]] >= 100:
        return None  # goal is inside an obstacle

    open_set = [(0.0, start)]
    came_from = {}
    g = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        cx, cy = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < GRID_N and 0 <= ny < GRID_N):
                continue
            if grid[ny * GRID_N + nx] >= 100:
                continue
            step = 1.414 if dx != 0 and dy != 0 else 1.0
            ng = g[current] + step
            neighbor = (nx, ny)
            if ng < g.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g[neighbor] = ng
                h = np.hypot(goal[0] - nx, goal[1] - ny)
                heapq.heappush(open_set, (ng + h, neighbor))

    return None  # no path found


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # Which color block to navigate to
        self.declare_parameter('target_color', 'red')
        self._target_color = self.get_parameter('target_color').get_parameter_value().string_value

        # Robot odometry state (mirrored from block_tracker logic)
        self._robot_pos = [-1.2192, -1.2192, 0.0]   # [x, y, yaw]
        self._left_wheel_pos = 0.0
        self._right_wheel_pos = 0.0
        self._left_wheel_distance = 0.0
        self._right_wheel_distance = 0.0
        self._imu_msg = Imu()

        # Block positions received from block_tracker_node, keyed by color name
        self._blocks = {'red': [], 'green': [], 'blue': []}  # list of (x, y) per color

        # Subscriptions
        self.create_subscription(MarkerArray, 'block_markers', self._markers_cb, 1)
        self.create_subscription(Float64, 'wheel_position/left',  self._left_enc_cb,  1)
        self.create_subscription(Float64, 'wheel_position/right', self._right_enc_cb, 1)
        self.create_subscription(Imu, 'imu', self._imu_cb, 1)

        # Publishers
        self._path_pub = self.create_publisher(Path, 'planned_path', 1)
        self._grid_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 1)

        # Re-plan at 1 Hz whenever marker data is available
        self.create_timer(1.0, self._plan)

    # ── odometry callbacks ────────────────────────────────────────────────────

    def _left_enc_cb(self, msg):
        self._left_wheel_pos = msg.data

    def _right_enc_cb(self, msg):
        self._right_wheel_pos = msg.data

    def _imu_cb(self, msg):
        self._imu_msg = msg

    def _update_position(self):
        new_left  = 0.032 * self._left_wheel_pos
        new_right = 0.032 * self._right_wheel_pos
        dl = new_left  - self._left_wheel_distance
        dr = new_right - self._right_wheel_distance
        self._left_wheel_distance  = new_left
        self._right_wheel_distance = new_right

        q = self._imu_msg.orientation
        norm = np.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        if norm == 0:
            return
        w, x, y, z = q.w/norm, q.x/norm, q.y/norm, q.z/norm
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        dist = (dl + dr) / 2.0
        self._robot_pos[0] += dist * np.cos(yaw)
        self._robot_pos[1] += dist * np.sin(yaw)
        self._robot_pos[2]  = yaw

    # ── marker callback ───────────────────────────────────────────────────────

    def _markers_cb(self, msg):
        blocks = {'red': [], 'green': [], 'blue': []}
        for marker in msg.markers:
            if marker.ns in blocks:
                blocks[marker.ns].append((marker.pose.position.x,
                                          marker.pose.position.y))
        self._blocks = blocks

    # ── grid building ─────────────────────────────────────────────────────────

    def _build_grid(self, target_color):
        """Return flat OccupancyGrid data with non-target blocks as obstacles."""
        data = [0] * (GRID_N * GRID_N)

        for color, positions in self._blocks.items():
            if color == target_color:
                continue  # target blocks are goals, not obstacles
            for bx, by in positions:
                cc, rc = world_to_cell(bx, by)
                for dc in range(-INFLATE_RADIUS, INFLATE_RADIUS + 1):
                    for dr in range(-INFLATE_RADIUS, INFLATE_RADIUS + 1):
                        nc, nr = cc + dc, rc + dr
                        if 0 <= nc < GRID_N and 0 <= nr < GRID_N:
                            data[nr * GRID_N + nc] = 100

        return data

    def _publish_grid(self, data):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = RESOLUTION
        grid.info.width  = GRID_N
        grid.info.height = GRID_N
        grid.info.origin.position.x = ORIGIN
        grid.info.origin.position.y = ORIGIN
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        self._grid_pub.publish(grid)

    # ── path planning ─────────────────────────────────────────────────────────

    def _plan(self):
        self._update_position()

        targets = self._blocks.get(self._target_color, [])
        if not targets:
            return  # no confirmed blocks of target color yet

        grid = self._build_grid(self._target_color)
        self._publish_grid(grid)

        start = world_to_cell(self._robot_pos[0], self._robot_pos[1])

        # Try each target block, keep the shortest path
        best_path = None
        for tx, ty in targets:
            goal = world_to_cell(tx, ty)
            path = astar(grid, start, goal)
            if path is not None:
                if best_path is None or len(path) < len(best_path):
                    best_path = path

        if best_path is None:
            self.get_logger().warn(f'No path found to any {self._target_color} block')
            return

        self._publish_path(best_path)

    def _publish_path(self, cell_path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for col, row in cell_path:
            wx, wy = cell_to_world(col, row)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)
        self.get_logger().info(
            f'Path to {self._target_color} block: {len(cell_path)} cells '
            f'({len(cell_path) * RESOLUTION:.2f} m)')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

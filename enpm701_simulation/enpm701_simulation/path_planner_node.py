import heapq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Pose2D
from visualization_msgs.msg import MarkerArray
import numpy as np




# Arena is 3.048 x 3.048 m, centred at origin
ARENA_SIZE = 3.048
RESOLUTION = 0.01                            # metres per grid cell
GRID_N = int(ARENA_SIZE / RESOLUTION)        # 60 cells
ORIGIN = -ARENA_SIZE / 2.0                   # -1.524 m

INFLATE_RADIUS = 15   # cells — inflates obstacles by ~15 cm (robot half-width)


def world_to_cell(x, y):
    col = int((x - ORIGIN) / RESOLUTION)
    row = int((y - ORIGIN) / RESOLUTION)
    return col, row


def cell_to_world(col, row):
    x = ORIGIN + (col + 0.5) * RESOLUTION
    y = ORIGIN + (row + 0.5) * RESOLUTION
    return x, y


def nearest_free_cell(grid, cell):
    """BFS from cell outward; return the closest unoccupied cell."""
    from collections import deque
    visited = {cell}
    queue = deque([cell])
    while queue:
        c = queue.popleft()
        if grid[c[1] * GRID_N + c[0]] < 100:
            return c
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nc = (c[0] + dx, c[1] + dy)
            if 0 <= nc[0] < GRID_N and 0 <= nc[1] < GRID_N and nc not in visited:
                visited.add(nc)
                queue.append(nc)
    return None


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

        self._robot_pos = [0.0, 0.0, 0.0]  # x, y, theta

        # Block positions received from block_tracker_node, keyed by color name
        self._blocks = {'red': [], 'green': [], 'blue': []}  # list of (x, y) per color

        # Subscriptions
        self.create_subscription(MarkerArray, 'block_markers', self._markers_cb, 1)
        self.create_subscription(Pose2D, 'robot_position', self._position_cb, 1)
        # Publishers (visualisation only)
        self._path_pub = self.create_publisher(Path, 'planned_path', 1)
        self._grid_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 1)

        # Service — autonomous_routine calls this to request a path to a goal pose
        self.create_service(GetPlan, 'plan_path', self._plan_cb)

    # ── odometry callbacks ────────────────────────────────────────────────────
    def _position_cb(self, msg):
        self._robot_pos = [msg.x, msg.y, msg.theta]

    def _left_enc_cb(self, msg):
        self._left_wheel_pos = msg.data

    def _right_enc_cb(self, msg):
        self._right_wheel_pos = msg.data

    def _imu_cb(self, msg):
        self._imu_msg = msg


    # ── marker callback ───────────────────────────────────────────────────────

    def _markers_cb(self, msg):
        blocks = {'red': [], 'green': [], 'blue': []}
        for marker in msg.markers:
            if marker.ns in blocks:
                blocks[marker.ns].append((marker.pose.position.x,
                                          marker.pose.position.y))
        self._blocks = blocks

    # ── grid building ─────────────────────────────────────────────────────────

    def _build_grid(self, exclude_pos=None):
        """Return flat OccupancyGrid data with all blocks as obstacles.

        exclude_pos: (x, y) world position of the goal block — its cell is left
        free so A* can reach it.
        """
        data = [0] * (GRID_N * GRID_N)
        exclude_cell = world_to_cell(*exclude_pos) if exclude_pos else None

        for positions in self._blocks.values():
            for bx, by in positions:
                cc, rc = world_to_cell(bx, by)
                if (cc, rc) == exclude_cell:
                    continue
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

    # ── service callback ──────────────────────────────────────────────────────

    def _plan_cb(self, request, response):
        """GetPlan service: request.goal is the target pose; returns response.plan."""
 

        gx = request.goal.pose.position.x
        gy = request.goal.pose.position.y

        grid = self._build_grid(exclude_pos=(gx, gy))
        self._publish_grid(grid)

        start = world_to_cell(self._robot_pos[0], self._robot_pos[1])
        goal  = world_to_cell(gx, gy)

        # If goal cell is occupied (e.g. drop-off inside an obstacle zone),
        # find the nearest free cell instead
        if grid[goal[1] * GRID_N + goal[0]] >= 100:
            goal = nearest_free_cell(grid, goal)

        if goal is None:
            self.get_logger().warn('No reachable goal cell found')
            return response  # empty path

        cell_path = astar(grid, start, goal)
        if cell_path is None:
            self.get_logger().warn(f'No path found to ({gx:.2f}, {gy:.2f})')
            return response  # empty path

        response.plan.header.frame_id = 'map'
        response.plan.header.stamp = self.get_clock().now().to_msg()
        for col, row in cell_path:
            wx, wy = cell_to_world(col, row)
            pose = PoseStamped()
            pose.header = response.plan.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            response.plan.poses.append(pose)

        self.get_logger().info(
            f'Path to ({gx:.2f},{gy:.2f}): {len(cell_path)} cells '
            f'({len(cell_path) * RESOLUTION:.2f} m)')
        self._path_pub.publish(response.plan)  # visualisation
        return response


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

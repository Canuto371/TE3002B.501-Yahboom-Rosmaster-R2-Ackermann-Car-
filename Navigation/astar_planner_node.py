import math
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('inflation_radius', 0.16)
        self.declare_parameter('unknown_as_obstacle', True)
        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('plan_once', True)
        self.declare_parameter('waypoint_stride', 5)

        # Si start/goal caen en celda inflada, buscar celda libre cercana.
        self.declare_parameter('allow_start_goal_recovery', True)
        self.declare_parameter('recovery_radius_cells', 40)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.unknown_as_obstacle = bool(self.get_parameter('unknown_as_obstacle').value)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.plan_once = bool(self.get_parameter('plan_once').value)
        self.waypoint_stride = int(self.get_parameter('waypoint_stride').value)

        self.allow_start_goal_recovery = bool(
            self.get_parameter('allow_start_goal_recovery').value
        )

        self.recovery_radius_cells = int(
            self.get_parameter('recovery_radius_cells').value
        )

        self.has_planned_once = False
        self.map_msg = None
        self.robot_pose = None
        self.inflated_grid = None

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        path_qos = QoSProfile(depth=1)
        path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', path_qos)

        self.timer = self.create_timer(self.replan_period, self.plan_and_publish)

        self.get_logger().info(
            f'A* planner started | goal=({self.goal_x:.2f}, {self.goal_y:.2f}) | '
            f'inflation_radius={self.inflation_radius:.2f} | '
            f'plan_once={self.plan_once} | '
            f'allow_start_goal_recovery={self.allow_start_goal_recovery}'
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.inflated_grid = self.build_inflated_grid(msg)

        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'res={msg.info.resolution:.3f}',
            throttle_duration_sec=2.0
        )

    def pose_callback(self, msg):
        self.robot_pose = msg

    def plan_and_publish(self):
        if self.plan_once and self.has_planned_once:
            return

        if self.map_msg is None:
            self.get_logger().warn('Waiting for /map...', throttle_duration_sec=2.0)
            return

        if self.robot_pose is None:
            self.get_logger().warn('Waiting for /robot_pose...', throttle_duration_sec=2.0)
            return

        start_world = (
            self.robot_pose.pose.position.x,
            self.robot_pose.pose.position.y
        )

        goal_world = (
            self.goal_x,
            self.goal_y
        )

        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)

        if start is None:
            self.get_logger().warn(f'Start outside map: {start_world}')
            return

        if goal is None:
            self.get_logger().warn(f'Goal outside map: {goal_world}')
            return

        if self.is_occupied(*start):
            if not self.allow_start_goal_recovery:
                self.get_logger().warn(f'Start occupied/inflated: {start}')
                return

            new_start = self.find_nearest_free_cell(
                start,
                max_radius_cells=self.recovery_radius_cells
            )

            if new_start is None:
                self.get_logger().warn(
                    f'Start occupied/inflated and no nearby free cell found: {start}'
                )
                return

            old_world = self.grid_to_world(*start)
            new_world = self.grid_to_world(*new_start)

            self.get_logger().warn(
                f'Start occupied/inflated: {start} '
                f'world=({old_world[0]:.2f}, {old_world[1]:.2f}). '
                f'Using nearest free cell: {new_start} '
                f'world=({new_world[0]:.2f}, {new_world[1]:.2f})'
            )

            start = new_start

        if self.is_occupied(*goal):
            if not self.allow_start_goal_recovery:
                self.get_logger().warn(f'Goal occupied/inflated: {goal}')
                return

            new_goal = self.find_nearest_free_cell(
                goal,
                max_radius_cells=self.recovery_radius_cells
            )

            if new_goal is None:
                self.get_logger().warn(
                    f'Goal occupied/inflated and no nearby free cell found: {goal}'
                )
                return

            old_world = self.grid_to_world(*goal)
            new_world = self.grid_to_world(*new_goal)

            self.get_logger().warn(
                f'Goal occupied/inflated: {goal} '
                f'world=({old_world[0]:.2f}, {old_world[1]:.2f}). '
                f'Using nearest free cell: {new_goal} '
                f'world=({new_world[0]:.2f}, {new_world[1]:.2f})'
            )

            goal = new_goal

        grid_path = self.astar(start, goal)

        if not grid_path:
            self.get_logger().warn(f'No path found from {start} to {goal}')
            return

        self.print_path_debug(grid_path, start_world, goal_world, goal)

        path_msg = self.grid_path_to_ros_path(grid_path)
        self.path_pub.publish(path_msg)

        self.has_planned_once = True

        self.get_logger().info(
            f'Path published: {len(path_msg.poses)} waypoints | '
            f'start=({start_world[0]:.2f}, {start_world[1]:.2f}) | '
            f'goal=({goal_world[0]:.2f}, {goal_world[1]:.2f})'
        )

    def find_nearest_free_cell(self, cell, max_radius_cells=40):
        cx, cy = cell

        if self.in_bounds(cx, cy) and not self.is_occupied(cx, cy):
            return cell

        best_cell = None
        best_dist_sq = None

        for radius in range(1, max_radius_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    # Solo revisar el borde del cuadrado para no repetir celdas internas.
                    if abs(dx) != radius and abs(dy) != radius:
                        continue

                    nx = cx + dx
                    ny = cy + dy

                    if not self.in_bounds(nx, ny):
                        continue

                    if self.is_occupied(nx, ny):
                        continue

                    dist_sq = dx * dx + dy * dy

                    if best_dist_sq is None or dist_sq < best_dist_sq:
                        best_dist_sq = dist_sq
                        best_cell = (nx, ny)

            if best_cell is not None:
                return best_cell

        return None

    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed.add(current)

            for neighbor, step_cost in self.get_neighbors(current):
                if neighbor in closed:
                    continue

                tentative_g = g_score[current] + step_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))

        return None

    def get_neighbors(self, cell):
        x, y = cell

        directions = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1,  1, math.sqrt(2.0)),
            ( 1, -1, math.sqrt(2.0)),
            ( 1,  1, math.sqrt(2.0)),
        ]

        neighbors = []

        for dx, dy, cost in directions:
            nx = x + dx
            ny = y + dy

            if not self.in_bounds(nx, ny):
                continue

            if self.is_occupied(nx, ny):
                continue

            # Evitar cortar esquinas entre obstáculos.
            if dx != 0 and dy != 0:
                if self.is_occupied(x + dx, y) or self.is_occupied(x, y + dy):
                    continue

            neighbors.append(((nx, ny), cost))

        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def print_path_debug(self, grid_path, start_world, goal_world, goal_cell):
        debug_points = []

        sample_count = min(10, len(grid_path))
        step = max(1, len(grid_path) // sample_count)

        for cell in grid_path[::step]:
            wx, wy = self.grid_to_world(*cell)
            debug_points.append(f'({wx:.2f}, {wy:.2f})')

        if grid_path[-1] != grid_path[::step][-1]:
            wx, wy = self.grid_to_world(*grid_path[-1])
            debug_points.append(f'({wx:.2f}, {wy:.2f})')

        gx_w, gy_w = self.grid_to_world(*goal_cell)

        self.get_logger().info(
            'Path debug: '
            f'start_world=({start_world[0]:.2f}, {start_world[1]:.2f}) | '
            f'goal_world=({goal_world[0]:.2f}, {goal_world[1]:.2f}) | '
            f'goal_cell_world=({gx_w:.2f}, {gy_w:.2f}) | '
            f'samples={" -> ".join(debug_points)}'
        )

    def build_inflated_grid(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution

        raw = list(map_msg.data)
        inflated = [False] * (width * height)

        inflation_cells = int(math.ceil(self.inflation_radius / resolution))
        occupied_cells = []

        for y in range(height):
            for x in range(width):
                index = y * width + x
                value = raw[index]

                occupied = value > 50

                if self.unknown_as_obstacle and value < 0:
                    occupied = True

                if occupied:
                    occupied_cells.append((x, y))

        for ox, oy in occupied_cells:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx = ox + dx
                    ny = oy + dy

                    if not self.in_bounds(nx, ny, width, height):
                        continue

                    dist = math.sqrt(dx * dx + dy * dy) * resolution

                    if dist <= self.inflation_radius:
                        inflated[ny * width + nx] = True

        return inflated

    def world_to_grid(self, x_world, y_world):
        info = self.map_msg.info

        gx = int((x_world - info.origin.position.x) / info.resolution)
        gy = int((y_world - info.origin.position.y) / info.resolution)

        if not self.in_bounds(gx, gy):
            return None

        return gx, gy

    def grid_to_world(self, gx, gy):
        info = self.map_msg.info

        x_world = info.origin.position.x + (gx + 0.5) * info.resolution
        y_world = info.origin.position.y + (gy + 0.5) * info.resolution

        return x_world, y_world

    def grid_path_to_ros_path(self, grid_path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        stride = max(1, self.waypoint_stride)
        sampled_path = grid_path[::stride]

        if sampled_path[-1] != grid_path[-1]:
            sampled_path.append(grid_path[-1])

        for i, cell in enumerate(sampled_path):
            x_world, y_world = self.grid_to_world(*cell)

            pose = PoseStamped()
            pose.header = path_msg.header

            pose.pose.position.x = x_world
            pose.pose.position.y = y_world
            pose.pose.position.z = 0.0

            if i < len(sampled_path) - 1:
                next_x, next_y = self.grid_to_world(*sampled_path[i + 1])
                yaw = math.atan2(next_y - y_world, next_x - x_world)
            else:
                yaw = 0.0

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path_msg.poses.append(pose)

        return path_msg

    def is_occupied(self, x, y):
        if not self.in_bounds(x, y):
            return True

        if self.inflated_grid is None:
            return True

        return self.inflated_grid[y * self.map_msg.info.width + x]

    def in_bounds(self, x, y, width=None, height=None):
        if width is None:
            if self.map_msg is None:
                return False
            width = self.map_msg.info.width

        if height is None:
            if self.map_msg is None:
                return False
            height = self.map_msg.info.height

        return 0 <= x < width and 0 <= y < height


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

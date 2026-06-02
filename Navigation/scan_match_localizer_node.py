import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32

from tf2_ros import Buffer, TransformListener


class ScanMatchLocalizer(Node):
    def __init__(self):
        super().__init__('scan_match_localizer')

        self.declare_parameter('initial_x', 0.4)
        self.declare_parameter('initial_y', 0.4)
        self.declare_parameter('initial_yaw', 0.0)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('process_rate_limit', 3.0)

        self.declare_parameter('laser_min_range', 0.15)
        self.declare_parameter('laser_max_range', 3.2)
        self.declare_parameter('max_beams', 45)

        self.declare_parameter('search_xy_range', 0.10)
        self.declare_parameter('search_yaw_range_deg', 12.0)

        self.declare_parameter('coarse_xy_step', 0.04)
        self.declare_parameter('coarse_yaw_step_deg', 4.0)

        self.declare_parameter('fine_xy_range', 0.025)
        self.declare_parameter('fine_yaw_range_deg', 3.0)

        self.declare_parameter('fine_xy_step', 0.0125)
        self.declare_parameter('fine_yaw_step_deg', 1.5)

        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('max_score_distance', 0.35)
        self.declare_parameter('score_sigma', 0.08)

        self.declare_parameter('pose_alpha_xy', 0.75)
        self.declare_parameter('pose_alpha_yaw', 0.75)

        self.declare_parameter('min_accept_score', 0.08)

        self.pose_x = float(self.get_parameter('initial_x').value)
        self.pose_y = float(self.get_parameter('initial_y').value)
        self.pose_yaw = float(self.get_parameter('initial_yaw').value)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.process_rate_limit = float(self.get_parameter('process_rate_limit').value)

        self.laser_min_range = float(self.get_parameter('laser_min_range').value)
        self.laser_max_range = float(self.get_parameter('laser_max_range').value)
        self.max_beams = int(self.get_parameter('max_beams').value)

        self.search_xy_range = float(self.get_parameter('search_xy_range').value)
        self.search_yaw_range = math.radians(float(self.get_parameter('search_yaw_range_deg').value))

        self.coarse_xy_step = float(self.get_parameter('coarse_xy_step').value)
        self.coarse_yaw_step = math.radians(float(self.get_parameter('coarse_yaw_step_deg').value))

        self.fine_xy_range = float(self.get_parameter('fine_xy_range').value)
        self.fine_yaw_range = math.radians(float(self.get_parameter('fine_yaw_range_deg').value))

        self.fine_xy_step = float(self.get_parameter('fine_xy_step').value)
        self.fine_yaw_step = math.radians(float(self.get_parameter('fine_yaw_step_deg').value))

        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.max_score_distance = float(self.get_parameter('max_score_distance').value)
        self.score_sigma = float(self.get_parameter('score_sigma').value)

        self.pose_alpha_xy = float(self.get_parameter('pose_alpha_xy').value)
        self.pose_alpha_yaw = float(self.get_parameter('pose_alpha_yaw').value)

        self.min_accept_score = float(self.get_parameter('min_accept_score').value)

        self.map_msg = None
        self.distance_field = None
        self.max_score_cells = None

        self.last_process_time = 0.0
        self.last_score = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.score_pub = self.create_publisher(Float32, '/scan_match_score', 10)

        self.pose_timer = self.create_timer(
            1.0 / max(0.1, self.publish_rate),
            self.publish_pose
        )

        self.score_timer = self.create_timer(
            1.0,
            self.publish_score_timer
        )

        self.get_logger().info(
            f'Scan-match localizer started | initial=({self.pose_x:.2f}, {self.pose_y:.2f}, {self.pose_yaw:.2f})'
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.max_score_cells = int(math.ceil(
            self.max_score_distance / msg.info.resolution
        ))
        self.distance_field = self.build_distance_field(msg)

        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}'
        )

    def initial_pose_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.get_logger().info(
            f'Initial pose received: x={self.pose_x:.3f}, y={self.pose_y:.3f}, yaw={self.pose_yaw:.3f}'
        )

    def scan_callback(self, msg):
        if self.map_msg is None or self.distance_field is None:
            self.get_logger().warn('Waiting for /map...', throttle_duration_sec=2.0)
            return

        now = time.time()
        min_dt = 1.0 / max(0.1, self.process_rate_limit)
        if now - self.last_process_time < min_dt:
            return
        self.last_process_time = now

        laser_to_base = self.get_laser_to_base_transform(msg.header.frame_id)
        if laser_to_base is None:
            return

        points_base = self.laserscan_to_base_points(msg, laser_to_base)

        if len(points_base) < 10:
            self.get_logger().warn(
                f'Not enough valid scan points: {len(points_base)}',
                throttle_duration_sec=1.0
            )
            return

        old_pose = (self.pose_x, self.pose_y, self.pose_yaw)
        best_pose, best_score = self.match_scan_to_map(points_base, old_pose)

        if best_score < self.min_accept_score:
            self.last_score = best_score
            self.get_logger().warn(
                f'Low scan-match score={best_score:.3f}. Keeping previous pose.',
                throttle_duration_sec=1.0
            )
            return

        self.pose_x = self.pose_alpha_xy * best_pose[0] + (1.0 - self.pose_alpha_xy) * self.pose_x
        self.pose_y = self.pose_alpha_xy * best_pose[1] + (1.0 - self.pose_alpha_xy) * self.pose_y

        yaw_error = self.angle_wrap(best_pose[2] - self.pose_yaw)
        self.pose_yaw = self.angle_wrap(
            self.pose_yaw + self.pose_alpha_yaw * yaw_error
        )

        self.last_score = best_score

        dx = self.pose_x - old_pose[0]
        dy = self.pose_y - old_pose[1]
        dyaw = self.angle_wrap(self.pose_yaw - old_pose[2])

        self.get_logger().info(
            f'ScanMatch pose: x={self.pose_x:.3f}, y={self.pose_y:.3f}, '
            f'yaw={math.degrees(self.pose_yaw):.1f} deg | '
            f'score={best_score:.3f} | '
            f'delta=({dx:.3f}, {dy:.3f}, {math.degrees(dyaw):.1f} deg)',
            throttle_duration_sec=0.5
        )

    def match_scan_to_map(self, points_base, seed_pose):
        coarse_pose, coarse_score = self.search_around_pose(
            points_base,
            seed_pose,
            self.search_xy_range,
            self.search_yaw_range,
            self.coarse_xy_step,
            self.coarse_yaw_step
        )

        fine_pose, fine_score = self.search_around_pose(
            points_base,
            coarse_pose,
            self.fine_xy_range,
            self.fine_yaw_range,
            self.fine_xy_step,
            self.fine_yaw_step
        )

        if fine_score >= coarse_score:
            return fine_pose, fine_score

        return coarse_pose, coarse_score

    def search_around_pose(self, points_base, center_pose, xy_range, yaw_range, xy_step, yaw_step):
        cx, cy, cyaw = center_pose

        best_pose = center_pose
        best_score = -1.0

        for dx in self.frange(-xy_range, xy_range, xy_step):
            x = cx + dx
            for dy in self.frange(-xy_range, xy_range, xy_step):
                y = cy + dy
                for dyaw in self.frange(-yaw_range, yaw_range, yaw_step):
                    yaw = self.angle_wrap(cyaw + dyaw)
                    score = self.score_pose(points_base, x, y, yaw)

                    if score > best_score:
                        best_score = score
                        best_pose = (x, y, yaw)

        return best_pose, best_score

    def score_pose(self, points_base, x, y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        total = 0.0
        valid = 0

        sigma2 = self.score_sigma * self.score_sigma

        for bx, by in points_base:
            mx = x + cos_yaw * bx - sin_yaw * by
            my = y + sin_yaw * bx + cos_yaw * by

            cell = self.world_to_grid(mx, my)
            if cell is None:
                continue

            gx, gy = cell
            dist_cells = self.distance_field[self.index(gx, gy)]

            if dist_cells is None:
                continue

            dist_m = dist_cells * self.map_msg.info.resolution
            total += math.exp(-(dist_m * dist_m) / (2.0 * sigma2))
            valid += 1

        if valid == 0:
            return 0.0

        return total / float(valid)

    def build_distance_field(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        data = list(map_msg.data)

        distance = [None] * (width * height)
        q = deque()

        for y in range(height):
            for x in range(width):
                idx = y * width + x
                value = data[idx]

                if value >= self.occupied_threshold:
                    distance[idx] = 0
                    q.append((x, y))

        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        while q:
            x, y = q.popleft()
            idx = y * width + x
            current_d = distance[idx]

            if current_d is not None and current_d >= self.max_score_cells:
                continue

            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy

                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue

                nidx = ny * width + nx

                if distance[nidx] is not None:
                    continue

                distance[nidx] = current_d + 1
                q.append((nx, ny))

        return distance

    def get_laser_to_base_transform(self, laser_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                laser_frame,
                rclpy.time.Time()
            )

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = self.quaternion_to_yaw(tf.transform.rotation)

            return tx, ty, yaw

        except Exception as e:
            self.get_logger().warn(
                f'Waiting for TF {self.base_frame} <- {laser_frame}: {e}',
                throttle_duration_sec=2.0
            )
            return None

    def laserscan_to_base_points(self, scan_msg, laser_to_base):
        tx, ty, laser_yaw = laser_to_base

        cos_l = math.cos(laser_yaw)
        sin_l = math.sin(laser_yaw)

        ranges = scan_msg.ranges
        n = len(ranges)

        if n == 0:
            return []

        step = max(1, n // max(1, self.max_beams))

        points = []

        for i in range(0, n, step):
            r = ranges[i]

            if not math.isfinite(r):
                continue

            if r < self.laser_min_range or r > self.laser_max_range:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            bx = tx + cos_l * lx - sin_l * ly
            by = ty + sin_l * lx + cos_l * ly

            points.append((bx, by))

        return points

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = self.pose_x
        msg.pose.position.y = self.pose_y
        msg.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(self.pose_yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)

    def publish_score_timer(self):
        msg = Float32()
        msg.data = float(self.last_score)
        self.score_pub.publish(msg)

    def world_to_grid(self, x_world, y_world):
        info = self.map_msg.info

        gx = int((x_world - info.origin.position.x) / info.resolution)
        gy = int((y_world - info.origin.position.y) / info.resolution)

        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return None

        return gx, gy

    def index(self, gx, gy):
        return gy * self.map_msg.info.width + gx

    def quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def yaw_to_quaternion(self, yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def angle_wrap(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def frange(self, start, stop, step):
        values = []

        if step <= 0.0:
            return [start]

        x = start
        eps = step * 0.5

        while x <= stop + eps:
            values.append(x)
            x += step

        return values


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatchLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

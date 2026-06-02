import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        path_qos = QoSProfile(depth=1)
        path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, path_qos)
        self.create_subscription(String, '/detected_signal', self.vision_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.declare_parameter('lookahead_distance', 0.16)
        self.declare_parameter('final_goal_tolerance', 0.08)

        self.declare_parameter('max_linear', 0.06)
        self.declare_parameter('min_linear', 0.025)
        self.declare_parameter('max_angular', 0.45)

        self.declare_parameter('angular_bias', 0.05)
        self.declare_parameter('pose_timeout', 3.0)
        self.declare_parameter('max_search_backwards', 3)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.final_goal_tolerance = float(self.get_parameter('final_goal_tolerance').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.min_linear = float(self.get_parameter('min_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.angular_bias = float(self.get_parameter('angular_bias').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.max_search_backwards = int(self.get_parameter('max_search_backwards').value)

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        self.has_pose = False
        self.last_pose_time = None

        self.path = []
        self.has_path = False

        self.last_closest_index = 0

        self.signal = "unknown"

        self.get_logger().info(
            'Pure pursuit path follower started | '
            f'max_linear={self.max_linear:.2f}, '
            f'max_angular={self.max_angular:.2f}, '
            f'lookahead_distance={self.lookahead_distance:.2f}, '
            f'angular_bias={self.angular_bias:.3f}, '
            f'pose_timeout={self.pose_timeout:.1f}'
        )

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.phi = self.quaternion_to_yaw(msg.pose.orientation)

        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            self.has_path = False
            self.path = []
            self.last_closest_index = 0
            self.get_logger().warn('Received empty path.')
            return

        self.path = msg.poses
        self.has_path = True
        self.last_closest_index = 0

        first = self.path[0].pose.position
        last = self.path[-1].pose.position

        self.get_logger().info(
            f'New path received: {len(self.path)} waypoints | '
            f'first=({first.x:.2f}, {first.y:.2f}) | '
            f'last=({last.x:.2f}, {last.y:.2f})'
        )

    def vision_callback(self, msg):
        try:
            label, _ = msg.data.split(',')
            self.signal = label
        except ValueError:
            self.signal = msg.data

    def control_loop(self):
        twist = Twist()

        if not self.has_pose:
            self.cmd_pub.publish(twist)
            self.get_logger().warn('Waiting for /robot_pose...', throttle_duration_sec=1.0)
            return

        now = self.get_clock().now()
        pose_age = (now - self.last_pose_time).nanoseconds / 1e9

        if pose_age > self.pose_timeout:
            self.cmd_pub.publish(twist)
            self.get_logger().warn(
                f'/robot_pose timeout: {pose_age:.2f}s. Stopping.',
                throttle_duration_sec=1.0
            )
            return

        if not self.has_path:
            self.cmd_pub.publish(twist)
            self.get_logger().warn('Waiting for /planned_path...', throttle_duration_sec=1.0)
            return

        if self.signal == "stop":
            self.cmd_pub.publish(twist)
            self.get_logger().info('STOP signal detected. Robot stopped.', throttle_duration_sec=1.0)
            return

        final_pose = self.path[-1].pose.position
        dist_to_goal = self.distance(self.x, self.y, final_pose.x, final_pose.y)

        if dist_to_goal < self.final_goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f'FINAL GOAL REACHED | x={self.x:.2f}, y={self.y:.2f}',
                throttle_duration_sec=1.0
            )
            return

        closest_index = self.find_closest_index()
        target_index = self.find_lookahead_index(closest_index)
        target = self.path[target_index].pose.position

        dx = target.x - self.x
        dy = target.y - self.y

        target_distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        theta_e = self.angle_wrap(target_angle - self.phi)

        x_local = math.cos(self.phi) * dx + math.sin(self.phi) * dy
        y_local = -math.sin(self.phi) * dx + math.cos(self.phi) * dy

        target_behind = x_local < 0.02

        if target_distance < 0.001:
            curvature = 0.0
        else:
            curvature = 2.0 * y_local / (target_distance * target_distance)

        V = self.max_linear

        abs_theta_e = abs(theta_e)

        if target_behind:
            V = self.min_linear
        elif abs_theta_e > math.radians(80):
            V = self.min_linear
        elif abs_theta_e > math.radians(55):
            V = self.max_linear * 0.35
        elif abs_theta_e > math.radians(35):
            V = self.max_linear * 0.60

        w = V * curvature

        if V > 0.0:
            w += self.angular_bias

        V = max(min(V, self.max_linear), -self.max_linear)
        w = max(min(w, self.max_angular), -self.max_angular)

        twist.linear.x = V
        twist.angular.z = w

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'Pose: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.phi):.1f} deg) | '
            f'closest={closest_index}, target={target_index} '
            f'({target.x:.2f}, {target.y:.2f}) | '
            f'd_goal={dist_to_goal:.2f}, d_target={target_distance:.2f}, '
            f'theta_e={math.degrees(theta_e):.1f} deg, '
            f'x_local={x_local:.2f}, y_local={y_local:.2f} | '
            f'V={V:.2f}, W={w:.2f}',
            throttle_duration_sec=0.5
        )

    def find_closest_index(self):
        if not self.path:
            return 0

        start_index = max(0, self.last_closest_index - self.max_search_backwards)

        best_index = start_index
        best_distance = float('inf')

        for i in range(start_index, len(self.path)):
            p = self.path[i].pose.position
            d = self.distance(self.x, self.y, p.x, p.y)

            if d < best_distance:
                best_distance = d
                best_index = i

        if best_index < self.last_closest_index:
            best_index = self.last_closest_index

        self.last_closest_index = best_index
        return best_index

    def find_lookahead_index(self, closest_index):
        if not self.path:
            return 0

        for i in range(closest_index, len(self.path)):
            p = self.path[i].pose.position
            d = self.distance(self.x, self.y, p.x, p.y)

            if d >= self.lookahead_distance:
                return i

        return len(self.path) - 1

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def angle_wrap(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

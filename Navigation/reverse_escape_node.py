import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ReverseEscapeNode(Node):

    def __init__(self):
        super().__init__('reverse_escape_node')

        self.declare_parameter('reverse_speed', 0.06)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('publish_rate', 20.0)

        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.duration = float(self.get_parameter('duration').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.control_loop
        )

        self.finished = False

        self.get_logger().info(
            f'Reverse escape started | '
            f'reverse_speed={self.reverse_speed:.3f}, '
            f'angular_z={self.angular_z:.3f}, '
            f'duration={self.duration:.2f}s'
        )

    def control_loop(self):
        if self.finished:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed >= self.duration:
            self.stop_robot()
            self.finished = True
            self.get_logger().info('Reverse escape complete. Robot stopped.')
            return

        twist = Twist()

        # reverse_speed se pasa positivo, pero aquí lo convertimos a reversa.
        twist.linear.x = -abs(self.reverse_speed)

        # angular_z define hacia dónde gira mientras va de reversa.
        twist.angular.z = self.angular_z

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'Reversing... t={elapsed:.2f}/{self.duration:.2f}s | '
            f'V={twist.linear.x:.3f}, W={twist.angular.z:.3f}',
            throttle_duration_sec=0.5
        )

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = ReverseEscapeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

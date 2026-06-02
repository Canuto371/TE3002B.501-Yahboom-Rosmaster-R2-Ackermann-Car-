import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', 0.4)
        self.declare_parameter('y', 0.4)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('interval', 0.5)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.interval = float(self.get_parameter('interval').value)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.sent_count = 0
        self.started = False

        self.start_timer = self.create_timer(
            self.start_delay,
            self.start_publishing
        )

        self.publish_timer = None

        self.get_logger().info(
            f'Initial pose scheduled: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}'
        )

    def start_publishing(self):
        if self.started:
            return

        self.started = True
        self.start_timer.cancel()

        self.publish_timer = self.create_timer(
            self.interval,
            self.publish_initial_pose
        )

    def publish_initial_pose(self):
        if self.sent_count >= self.publish_count:
            self.get_logger().info('Initial pose publication complete.')
            self.publish_timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        ]

        self.publisher.publish(msg)
        self.sent_count += 1

        self.get_logger().info(
            f'Published initial pose {self.sent_count}/{self.publish_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

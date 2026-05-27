import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class PoseRepublisher(Node):
    def __init__(self):
        super().__init__('pose_republisher')

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )

        self.get_logger().info('Pose republisher started: /amcl_pose -> /robot_pose')

    def pose_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose

        self.publisher.publish(pose_msg)

        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y

        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        self.get_logger().info(
            f'Robot pose: x={x:.3f}, y={y:.3f}, theta={yaw:.3f} rad',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

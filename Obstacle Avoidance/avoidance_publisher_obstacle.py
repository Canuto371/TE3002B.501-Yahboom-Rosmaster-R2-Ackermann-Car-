import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.publisher_ = self.create_publisher(
            String,
            '/obstacle_info',
            10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.safe_distance = 0.3

        self.get_logger().info('Obstacle avoidance node started')

    def scan_callback(self, msg):

        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        closest_distance = 999.0
        closest_angle = 0.0

        for i, distance in enumerate(ranges):

            angle = angle_min + i * angle_increment
            angle_deg = math.degrees(angle)

            if math.isinf(distance) or math.isnan(distance) or distance < 0.05:
                continue

            if distance < closest_distance:
                closest_distance = distance
                closest_angle = angle_deg

        msg_out = String()

        if closest_distance < self.safe_distance:
            msg_out.data = f"1,{closest_angle},{closest_distance}"
        else:
            msg_out.data = "0,0,0"

        self.publisher_.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

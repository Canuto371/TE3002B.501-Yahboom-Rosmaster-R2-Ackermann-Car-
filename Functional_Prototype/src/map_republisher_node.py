import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid


class MapRepublisherNode(Node):
    def __init__(self):
        super().__init__('map_republisher_node')

        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_for_planning')
        self.declare_parameter('publish_rate', 1.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.last_map = None

        self.sub = self.create_subscription(
            OccupancyGrid,
            self.input_topic,
            self.map_callback,
            qos
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            qos
        )

        self.timer = self.create_timer(
            1.0 / max(0.1, self.publish_rate),
            self.publish_map
        )

        self.get_logger().info(
            f'Map republisher started | {self.input_topic} -> {self.output_topic}'
        )

    def map_callback(self, msg):
        self.last_map = msg

        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}',
            throttle_duration_sec=2.0
        )

    def publish_map(self):
        if self.last_map is None:
            self.get_logger().warn(
                f'Waiting for {self.input_topic}...',
                throttle_duration_sec=2.0
            )
            return

        self.last_map.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.last_map)


def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory


class SignalGateNode(Node):
    def __init__(self):
        super().__init__('signal_gate_node')

        self.declare_parameter('config_file', '')

        config_file = str(self.get_parameter('config_file').value)

        if config_file:
            self.config_path = Path(config_file)
        else:
            pkg_share = Path(get_package_share_directory('integration_test_4'))
            self.config_path = pkg_share / 'config' / 'signal_config.json'

        self.config = json.loads(self.config_path.read_text())

        self.pedestrian_scale = float(
            self.config['pedestrians']['speed_scale']
        )

        self.stop_duration = float(
            self.config['stop']['duration']
        )

        self.stop_cooldown = float(
            self.config['stop']['cooldown']
        )

        self.forbidden_area_ratio_stop = float(
            self.config['forbidden']['area_ratio_stop']
        )

        self.parking_enabled = bool(
            self.config['parking']['enabled']
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        self.create_subscription(
            String,
            '/detected_signal',
            self.signal_callback,
            10
        )

        self.create_subscription(
            String,
            '/mission_state',
            self.mission_state_callback,
            10
        )

        self.create_subscription(
            String,
            '/mission_event',
            self.mission_event_callback,
            10
        )

        self.active_label = 'none'
        self.active_until = 0.0

        self.stop_until = 0.0
        self.last_stop_time = -999.0

        self.loading_completed = False
        self.mission_state = 'UNKNOWN'

        self.get_logger().info(
            f'Signal gate started | config={self.config_path} | '
            '/cmd_vel_raw -> /cmd_vel'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize_label(self, label):
        label = label.strip().lower()

        aliases = {
            'forbidden': 'forbidden',
            'restricted': 'forbidden',
            'restricted_area': 'forbidden',
            'restricted area': 'forbidden',

            'pedestrian': 'pedestrians',
            'pedestrians': 'pedestrians',

            'agv': 'agv_area',
            'agv_area': 'agv_area',
            'agv zone': 'agv_area',
            'agv_zone': 'agv_area',

            'stop': 'stop',

            'loading': 'loading',
            'loading_zone': 'loading',
            'loading zone': 'loading',

            'parking': 'parking',
            'parking_zone': 'parking',
            'parking zone': 'parking'
        }

        return aliases.get(label, label)

    def parse_signal(self, data):
        # Expected:
        # label,score,area_ratio
        parts = [p.strip() for p in data.split(',')]

        label = self.normalize_label(parts[0]) if len(parts) >= 1 else 'unknown'

        try:
            score = float(parts[1]) if len(parts) >= 2 else 0.0
        except Exception:
            score = 0.0

        try:
            area_ratio = float(parts[2]) if len(parts) >= 3 else 0.0
        except Exception:
            area_ratio = 0.0

        return label, score, area_ratio

    def mission_state_callback(self, msg):
        self.mission_state = msg.data

    def mission_event_callback(self, msg):
        event = msg.data

        # En esta misión, "loading" real equivale a llegar al cajón
        # y terminar la espera de 5 segundos.
        if event == 'target_wait_complete':
            self.loading_completed = True
            self.get_logger().info(
                'Mission target wait complete. Loading considered completed.'
            )

    def signal_callback(self, msg):
        now = self.now_sec()

        label, score, area_ratio = self.parse_signal(msg.data)

        if label == 'unknown':
            return

        if label == 'pedestrians':
            self.active_label = 'pedestrians'
            self.active_until = now + 2.0

            self.get_logger().info(
                f'Pedestrians detected | score={score:.1f} | speed scale={self.pedestrian_scale:.2f}',
                throttle_duration_sec=1.0
            )
            return

        if label == 'agv_area':
            self.active_label = 'agv_area'
            self.active_until = now + 2.0

            self.get_logger().info(
                f'AGV zone detected | score={score:.1f} | normal speed',
                throttle_duration_sec=1.0
            )
            return

        if label == 'stop':
            if now - self.last_stop_time > self.stop_cooldown:
                self.stop_until = now + self.stop_duration
                self.last_stop_time = now

                self.get_logger().warn(
                    f'STOP detected | stopping for {self.stop_duration:.1f}s'
                )
            return

        if label == 'forbidden':
            if area_ratio >= self.forbidden_area_ratio_stop:
                self.stop_until = now + 0.8

                self.get_logger().warn(
                    f'Forbidden area too close | area_ratio={area_ratio:.3f} | stopping',
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().info(
                    f'Forbidden seen but not close | area_ratio={area_ratio:.3f}',
                    throttle_duration_sec=1.0
                )
            return

        if label == 'loading':
            # Loading no interrumpe la misión.
            # La misión espera 5s al llegar al target.
            self.get_logger().info(
                'Loading sign detected but ignored by gate.',
                throttle_duration_sec=1.0
            )
            return

        if label == 'parking':
            if self.parking_enabled and self.loading_completed:
                self.stop_until = now + 9999.0

                self.get_logger().warn(
                    'Parking detected after loading complete. Final stop.'
                )
            return

    def cmd_callback(self, msg):
        now = self.now_sec()

        out = Twist()

        # Stop modes: stop sign, forbidden close, final parking.
        if now < self.stop_until:
            self.cmd_pub.publish(out)
            return

        scale = 1.0

        if now < self.active_until:
            if self.active_label == 'pedestrians':
                scale = self.pedestrian_scale
            elif self.active_label == 'agv_area':
                scale = 1.0

        out.linear.x = msg.linear.x * scale
        out.linear.y = msg.linear.y * scale
        out.linear.z = msg.linear.z * scale

        # Escalamos angular también para no sobrecorregir cuando va más lento.
        out.angular.x = msg.angular.x * scale
        out.angular.y = msg.angular.y * scale
        out.angular.z = msg.angular.z * scale

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = SignalGateNode()

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

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32

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
            self.config.get('pedestrians', {}).get('speed_scale', 0.5)
        )

        self.stop_duration = float(
            self.config.get('stop', {}).get('duration', 5.0)
        )

        self.stop_cooldown = float(
            self.config.get('stop', {}).get('cooldown', 8.0)
        )

        self.forbidden_area_ratio_stop = float(
            self.config.get('forbidden', {}).get('area_ratio_stop', 0.06)
        )

        self.pause_pub = self.create_publisher(
            Bool,
            '/motion_pause',
            10
        )

        self.speed_pub = self.create_publisher(
            Float32,
            '/speed_scale',
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

        self.timer = self.create_timer(
            0.1,
            self.policy_loop
        )

        self.mission_state = 'UNKNOWN'

        self.stop_until = 0.0
        self.forbidden_pause_until = 0.0
        self.last_stop_time = -999.0

        self.speed_scale = 1.0
        self.speed_scale_until = 0.0

        self.get_logger().info(
            f'Signal gate started | config={self.config_path} | '
            'sub=/detected_signal | pub=/motion_pause,/speed_scale'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def mission_state_callback(self, msg):
        self.mission_state = msg.data

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

            'avg': 'agv_area',
            'avg_area': 'agv_area',
            'avg zone': 'agv_area',
            'avg_zone': 'agv_area',

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

    def signal_callback(self, msg):
        now = self.now_sec()

        label, score, area_ratio = self.parse_signal(msg.data)

        self.get_logger().info(
            f'Received /detected_signal: raw="{msg.data}" | '
            f'label={label}, score={score:.1f}, area_ratio={area_ratio:.3f}, '
            f'mission_state={self.mission_state}'
        )

        if label == 'unknown':
            return

        if label == 'stop':
            if now - self.last_stop_time > self.stop_cooldown:
                self.stop_until = now + self.stop_duration
                self.last_stop_time = now

                self.get_logger().warn(
                    f'STOP accepted. motion_pause TRUE for {self.stop_duration:.1f}s'
                )
            else:
                self.get_logger().info(
                    'STOP ignored due to cooldown.',
                    throttle_duration_sec=1.0
                )
            return

        if label == 'pedestrians':
            self.speed_scale = self.pedestrian_scale
            self.speed_scale_until = now + 2.0

            self.get_logger().warn(
                f'PEDESTRIANS accepted. speed_scale={self.speed_scale:.2f}'
            )
            return

        if label == 'agv_area':
            self.speed_scale = 1.0
            self.speed_scale_until = now + 2.0

            self.get_logger().warn(
                'AGV accepted. speed_scale=1.0'
            )
            return

        if label == 'forbidden':
            if area_ratio >= self.forbidden_area_ratio_stop:
                self.forbidden_pause_until = now + 0.8

                self.get_logger().warn(
                    f'FORBIDDEN close accepted. area_ratio={area_ratio:.3f}. '
                    'motion_pause TRUE briefly.'
                )
            else:
                self.get_logger().info(
                    f'FORBIDDEN seen but not close. area_ratio={area_ratio:.3f}'
                )
            return

        if label == 'loading':
            self.get_logger().info(
                'LOADING detected but ignored by policy.'
            )
            return

        if label == 'parking':
            self.get_logger().info(
                'PARKING detected but ignored by policy.'
            )
            return

    def policy_loop(self):
        now = self.now_sec()

        if now > self.speed_scale_until:
            self.speed_scale = 1.0

        should_pause = False

        if now < self.stop_until:
            should_pause = True

        if now < self.forbidden_pause_until:
            should_pause = True

        pause_msg = Bool()
        pause_msg.data = should_pause

        speed_msg = Float32()
        speed_msg.data = float(self.speed_scale)

        self.pause_pub.publish(pause_msg)
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SignalGateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pause_pub.publish(Bool(data=False))
        node.speed_pub.publish(Float32(data=1.0))

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

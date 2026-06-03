import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SignalGateNode(Node):
    def __init__(self):
        super().__init__('signal_gate_node')

        self.declare_parameter('signal_timeout', 2.0)
        self.declare_parameter('pedestrian_speed_scale', 0.5)
        self.declare_parameter('restricted_area_ratio_stop', 0.06)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('stop_cooldown', 8.0)

        # Si false, loading espera loading_wait_seconds y sigue.
        # Si true, loading espera /manual_continue.
        self.declare_parameter('manual_loading_confirmation', False)
        self.declare_parameter('loading_wait_seconds', 5.0)

        self.signal_timeout = float(self.get_parameter('signal_timeout').value)
        self.pedestrian_speed_scale = float(self.get_parameter('pedestrian_speed_scale').value)
        self.restricted_area_ratio_stop = float(self.get_parameter('restricted_area_ratio_stop').value)
        self.stop_duration = float(self.get_parameter('stop_duration').value)
        self.stop_cooldown = float(self.get_parameter('stop_cooldown').value)
        self.manual_loading_confirmation = bool(self.get_parameter('manual_loading_confirmation').value)
        self.loading_wait_seconds = float(self.get_parameter('loading_wait_seconds').value)

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        self.signal_sub = self.create_subscription(
            String,
            '/detected_signal',
            self.signal_callback,
            10
        )

        self.manual_sub = self.create_subscription(
            String,
            '/manual_continue',
            self.manual_continue_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.loading_status_pub = self.create_publisher(
            String,
            '/loading_status',
            10
        )

        now = self.now_sec()

        self.last_signal_time = 0.0
        self.last_label = 'unknown'
        self.last_score = 0.0
        self.last_area_ratio = 0.0

        self.speed_scale = 1.0
        self.speed_scale_until = 0.0

        self.stop_until = 0.0
        self.last_stop_trigger_time = -999.0

        self.loading_wait = False
        self.loading_started_at = 0.0
        self.loading_completed = False

        self.parking_stop = False

        self.get_logger().info(
            'Signal gate started | /cmd_vel_raw -> /cmd_vel | '
            f'manual_loading_confirmation={self.manual_loading_confirmation}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize_label(self, label):
        label = label.strip().lower()

        aliases = {
            'forbidden': 'restricted',
            'restricted': 'restricted',
            'restricted_area': 'restricted',
            'restricted area': 'restricted',

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
            'parking zone': 'parking',

            'unknown': 'unknown'
        }

        return aliases.get(label, label)

    def parse_signal(self, data):
        # Formatos aceptados:
        # "label,score"
        # "label,score,area_ratio"
        parts = [p.strip() for p in data.split(',')]

        label = parts[0] if len(parts) >= 1 else 'unknown'

        try:
            score = float(parts[1]) if len(parts) >= 2 else 0.0
        except Exception:
            score = 0.0

        try:
            area_ratio = float(parts[2]) if len(parts) >= 3 else 0.0
        except Exception:
            area_ratio = 0.0

        return self.normalize_label(label), score, area_ratio

    def signal_callback(self, msg):
        now = self.now_sec()

        label, score, area_ratio = self.parse_signal(msg.data)

        self.last_label = label
        self.last_score = score
        self.last_area_ratio = area_ratio
        self.last_signal_time = now

        if label == 'unknown':
            return

        if label == 'pedestrians':
            self.speed_scale = self.pedestrian_speed_scale
            self.speed_scale_until = now + self.signal_timeout
            self.get_logger().info(
                f'Pedestrians detected. Speed scale={self.speed_scale:.2f}',
                throttle_duration_sec=1.0
            )
            return

        if label == 'agv_area':
            self.speed_scale = 1.0
            self.speed_scale_until = now + self.signal_timeout
            self.get_logger().info(
                'AGV zone detected. Normal speed enabled.',
                throttle_duration_sec=1.0
            )
            return

        if label == 'stop':
            # No resetear 5 segundos infinitamente si sigue viendo la misma señal.
            if now - self.last_stop_trigger_time > self.stop_cooldown:
                self.stop_until = now + self.stop_duration
                self.last_stop_trigger_time = now
                self.get_logger().warn(
                    f'STOP detected. Stopping for {self.stop_duration:.1f}s.'
                )
            return

        if label == 'restricted':
            # Si el SIFT node todavía no manda area_ratio, area_ratio será 0.
            # En ese caso NO frenamos de golpe por default. Cuando agreguemos area_ratio,
            # frenará solo si la señal está suficientemente cerca.
            if area_ratio >= self.restricted_area_ratio_stop:
                self.stop_until = now + 0.8
                self.get_logger().warn(
                    f'Restricted area too close. area_ratio={area_ratio:.3f}. Stopping.',
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().info(
                    f'Restricted sign seen but not close. area_ratio={area_ratio:.3f}.',
                    throttle_duration_sec=1.0
                )
            return

        if label == 'loading':
            # Para esta misión, loading NO modifica la navegación.
            # El robot debe llegar al cajón A/B/C, esperar 5 segundos ahí,
            # luego hacer reversa y regresar.
            self.get_logger().info(
                'Loading sign detected, but ignored by signal gate for this mission.',
                throttle_duration_sec=1.0
            )
            return

        if label == 'parking':
            if self.loading_completed:
                self.parking_stop = True
                self.get_logger().warn(
                    'Parking zone detected after loading. Stopping.',
                    throttle_duration_sec=1.0
                )
            return

    def manual_continue_callback(self, msg):
        data = msg.data.strip().lower()

        if data in ['continue', 'go', 'done', 'loaded', 'ok', '1', 'true']:
            self.loading_wait = False
            self.loading_completed = True

            status = String()
            status.data = 'loading_complete'
            self.loading_status_pub.publish(status)

            self.get_logger().info('Manual loading confirmation received. Continuing.')

    def cmd_callback(self, msg):
        now = self.now_sec()

        out = Twist()

        # Parking final
        if self.parking_stop:
            self.cmd_pub.publish(out)
            return

        # Loading wait
        if self.loading_wait:
            if self.manual_loading_confirmation:
                self.cmd_pub.publish(out)
                return

            elapsed = now - self.loading_started_at

            if elapsed < self.loading_wait_seconds:
                self.cmd_pub.publish(out)
                return

            self.loading_wait = False
            self.loading_completed = True

            status = String()
            status.data = 'loading_complete'
            self.loading_status_pub.publish(status)

            self.get_logger().info(
                f'Loading wait finished after {self.loading_wait_seconds:.1f}s. Continuing.'
            )

        # Stop sign or restricted close stop
        if now < self.stop_until:
            self.cmd_pub.publish(out)
            return

        # Speed scaling timeout
        scale = self.speed_scale

        if now > self.speed_scale_until:
            scale = 1.0
            self.speed_scale = 1.0

        out.linear.x = msg.linear.x * scale
        out.linear.y = msg.linear.y * scale
        out.linear.z = msg.linear.z * scale

        # Mantengo angular igual para que pueda girar bien aunque vaya lento.
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = SignalGateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
This node sends different velocities to each motor when rotating (due to difference in distance to the ICR). :D
'''

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math


class LyapunovController(Node):

    def __init__(self):

        super().__init__('lyapunov_controller')

        # ==========================================
        # Publisher
        # ==========================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================================
        # Subscriber
        # ==========================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==========================================
        # Timer
        # ==========================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # ==========================================
        # Estados
        # ==========================================

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        # ==========================================
        # Objetivo
        # ==========================================

        self.xd = 1.5
        self.yd = 1.0

        # ==========================================
        # Parámetros físicos
        # ==========================================

        self.a = 0.165
        self.L = 0.235

        # ==========================================
        # Ganancias
        # ==========================================

        self.kv = 1.2
        self.ktheta = 1.5

        self.goal_tolerance = 0.10

        self.get_logger().info(
            'Lyapunov controller started'
        )

    # ==================================================
    # ODOM CALLBACK
    # ==================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion -> yaw

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.phi = math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ==================================================
    # CONTROL LOOP
    # ==================================================

    def control_loop(self):

        twist = Twist()

        # ==========================================
        # Errores
        # ==========================================

        xe = self.xd - self.x
        ye = self.yd - self.y

        d = math.sqrt(
            xe**2 + ye**2
        )

        theta_e = math.atan2(
            ye,
            xe
        ) - self.phi

        # Normalización angular

        theta_e = math.atan2(
            math.sin(theta_e),
            math.cos(theta_e)
        )

        # ==========================================
        # Objetivo alcanzado
        # ==========================================

        if d < self.goal_tolerance:

            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_pub.publish(twist)

            self.get_logger().info(
                'GOAL REACHED'
            )

            return

        # ==========================================
        # Ley de control
        # ==========================================

        Vt = self.kv * math.tanh(d)

        delta = math.atan(
            self.ktheta * theta_e
        )

        # ==========================================
        # Radio de curvatura
        # ==========================================

        # Evitar división por cero

        if abs(math.tan(delta)) < 0.001:

            Rt = 999999.0

        else:

            Rt = self.L / math.tan(delta)

        # ==========================================
        # Velocidad angular
        # ==========================================

        w = Vt / Rt

        # ==========================================
        # Velocidades de llantas
        # ==========================================

        VR = Vt + (w * self.a / 2.0)

        VL = Vt - (w * self.a / 2.0)

        # ==========================================
        # Velocidad equivalente
        # ==========================================

        V = (VR + VL) / 2.0

        # ==========================================
        # Saturaciones
        # ==========================================

        max_linear = 0.30
        max_angular = 1.2

        V = max(
            min(V, max_linear),
            -max_linear
        )

        w = max(
            min(w, max_angular),
            -max_angular
        )

        # ==========================================
        # Publicar
        # ==========================================

        twist.linear.x = V
        twist.angular.z = w

        self.cmd_pub.publish(twist)

        # ==========================================
        # Logs
        # ==========================================

        self.get_logger().info(

            f'X: {self.x:.2f} | '
            f'Y: {self.y:.2f} | '
            f'PHI: {math.degrees(self.phi):.1f} | '
            f'D: {d:.2f} | '
            f'V: {V:.2f} | '
            f'W: {w:.2f} | '
            f'VR: {VR:.2f} | '
            f'VL: {VL:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = LyapunovController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        twist = Twist()

        node.cmd_pub.publish(twist)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()

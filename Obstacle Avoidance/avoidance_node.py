import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import threading
import sys
import termios
import tty
import math


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Timer
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # Estado
        self.stop_requested = False
        self.obstacle_detected = False

        # Máquina de estados
        self.state = "FORWARD"
        self.state_start_time = self.get_clock().now()

        # Distancia mínima detectada
        self.min_distance = 999.0

        # Ángulo del obstáculo más cercano
        self.closest_angle = 0.0

        # Dirección de giro
        # -1 = derecha
        # +1 = izquierda
        self.turn_direction = -1.0

        # Obstáculo detrás
        self.obstacle_behind = False

        # Parámetros
        self.safe_distance = 0.3
        self.critical_distance = 0.2

        self.get_logger().info(
            'Obstacle avoidance node started'
        )

    # ==================================================
    # CALLBACK LIDAR
    # ==================================================

    def scan_callback(self, msg):

        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        closest_distance = 999.0
        closest_angle = 0.0

        self.obstacle_behind = False

        for i, distance in enumerate(ranges):

            angle = angle_min + i * angle_increment
            angle_deg = math.degrees(angle)

            # Filtrar valores inválidos
            if (
                math.isinf(distance)
                or math.isnan(distance)
                or distance < 0.05
            ):
                continue

            # Buscar obstáculo MÁS cercano
            if distance < closest_distance:

                closest_distance = distance
                closest_angle = angle_deg

        # Guardar datos
        self.min_distance = closest_distance
        self.closest_angle = closest_angle

        # Detectar obstáculo
        if self.min_distance < self.safe_distance:

            self.obstacle_detected = True

            # =========================
            # OBSTÁCULO A LA IZQUIERDA
            # -> GIRAR DERECHA
            # =========================

            if -135 <= closest_angle <= -90:

                self.turn_direction = -1.0

            # =========================
            # OBSTÁCULO A LA DERECHA
            # -> GIRAR IZQUIERDA
            # =========================

            elif 90 <= closest_angle <= 135:

                self.turn_direction = 1.0

            # =========================
            # OBSTÁCULO ATRÁS
            # NO RETROCEDER
            # =========================

            elif -45 <= closest_angle <= 45:

                self.obstacle_behind = True

        else:

            self.obstacle_detected = False

        self.get_logger().info(
            f'STATE: {self.state} | '
            f'MIN: {self.min_distance:.2f} m | '
            f'ANGLE: {self.closest_angle:.1f}'
        )

    # ==================================================
    # CONTROL LOOP
    # ==================================================

    def control_loop(self):

        twist = Twist()

        now = self.get_clock().now()
        elapsed = (
            now - self.state_start_time
        ).nanoseconds / 1e9

        # STOP manual
        if self.stop_requested:

            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # =========================
        # FORWARD
        # =========================

        elif self.state == "FORWARD":

            twist.linear.x = 0.20
            twist.angular.z = 0.0

            if self.obstacle_detected:

                self.state = "TURNING"
                self.state_start_time = now

                direction_text = (
                    "LEFT"
                    if self.turn_direction > 0
                    else "RIGHT"
                )

                self.get_logger().info(
                    f"Obstacle detected -> TURNING {direction_text}"
                )

        # =========================
        # TURNING
        # =========================

        elif self.state == "TURNING":

            twist.linear.x = 0.08

            # Girar según lado del obstáculo
            twist.angular.z = 0.8 * self.turn_direction

            # Muy cerca -> reversa
            if self.min_distance < self.critical_distance:

                # Si el obstáculo está atrás
                # NO hacer reversa
                if self.obstacle_behind:

                    self.get_logger().info(
                        "Obstacle behind -> NO BACKUP"
                    )

                else:

                    self.state = "BACKING_UP"
                    self.state_start_time = now

                    self.get_logger().info(
                        "Too close -> BACKING UP"
                    )

            # Terminar giro
            elif elapsed > 1.3:

                self.state = "FORWARD"
                self.state_start_time = now

                self.get_logger().info(
                    "Turn complete -> FORWARD"
                )

        # =========================
        # BACKING UP
        # =========================

        elif self.state == "BACKING_UP":

            # Si hay obstáculo atrás
            # NO retroceder
            if self.obstacle_behind:

                twist.linear.x = 0.0
                twist.angular.z = 0.8 * self.turn_direction

            else:

                twist.linear.x = -0.12
                twist.angular.z = 0.6 * self.turn_direction

            if elapsed > 1.0:

                self.state = "TURNING"
                self.state_start_time = now

                self.get_logger().info(
                    "Backup complete -> TURNING"
                )

        # Publicar
        self.publisher_.publish(twist)


# ======================================================
# TECLADO
# ======================================================

def keyboard_listener(node):

    settings = termios.tcgetattr(sys.stdin)

    try:
        while True:

            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)

            if key == 's':

                node.stop_requested = True
                print("\nSTOP requested")

            elif key == 'g':

                node.stop_requested = False
                print("\nGO")

            elif key == '\x03':
                break

    finally:

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )


# ======================================================
# MAIN
# ======================================================

def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    keyboard_thread = threading.Thread(
        target=keyboard_listener,
        args=(node,),
        daemon=True
    )

    keyboard_thread.start()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        twist = Twist()
        node.publisher_.publish(twist)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math


class SmartController(Node):

    def __init__(self):

        super().__init__('smart_controller')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/obstacle_info', self.obstacle_callback, 10)
        self.create_subscription(String, '/detected_signal', self.vision_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        # Estado robot
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        # Objetivo
        self.xd = 1.5
        self.yd = 1.0

        # Control params
        self.a = 0.165
        self.L = 0.235
        self.kv = 1.2
        self.ktheta = 1.5
        self.goal_tolerance = 0.10

        # Sensores
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = 999.0
        self.signal = "unknown"

        self.get_logger().info('Smart controller started')

    # =============================
    # CALLBACKS
    # =============================

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)

        self.phi = math.atan2(siny_cosp, cosy_cosp)

    def obstacle_callback(self, msg):
        data = msg.data.split(',')

        if data[0] == "1":
            self.obstacle_detected = True
            self.obstacle_angle = float(data[1])
            self.obstacle_distance = float(data[2])
        else:
            self.obstacle_detected = False

    def vision_callback(self, msg):
        label, _ = msg.data.split(',')
        self.signal = label

    # =============================
    # CONTROL LOOP
    # =============================

    def control_loop(self):

        twist = Twist()

        # 🚨 PRIORITY 1: obstacle
        if self.obstacle_detected:
            twist.linear.x = 0.05
            twist.angular.z = -0.8 if self.obstacle_angle > 0 else 0.8
            self.cmd_pub.publish(twist)
            return

        # 🚦 PRIORITY 2: vision
        if self.signal == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # =============================
        # LYAPUNOV CONTROL (original)
        # =============================

        xe = self.xd - self.x
        ye = self.yd - self.y

        d = math.sqrt(xe**2 + ye**2)

        theta_e = math.atan2(ye, xe) - self.phi
        theta_e = math.atan2(math.sin(theta_e), math.cos(theta_e))

        if d < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            return

        Vt = self.kv * math.tanh(d)
        delta = math.atan(self.ktheta * theta_e)

        Rt = self.L / math.tan(delta) if abs(math.tan(delta)) > 0.001 else 999999.0
        w = Vt / Rt

        VR = Vt + (w * self.a / 2.0)
        VL = Vt - (w * self.a / 2.0)

        V = (VR + VL) / 2.0

        # Saturation
        V = max(min(V, 0.3), -0.3)
        w = max(min(w, 1.2), -1.2)

        twist.linear.x = V
        twist.angular.z = w

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SmartController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

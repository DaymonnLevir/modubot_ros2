import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom')

        self.declare_parameter('serial_rx_topic', '/modubot/serial_rx')
        self.declare_parameter('ticks_per_rev_left', 91.0)
        self.declare_parameter('ticks_per_rev_right', 91.0)
        self.declare_parameter('wheel_radius', 0.078)
        self.declare_parameter('wheel_separation', 0.225)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('velocity_timeout', 0.2)
        self.declare_parameter('debug', False)

        serial_rx_topic = str(self.get_parameter('serial_rx_topic').value)
        self.ticks_left = float(
            self.get_parameter('ticks_per_rev_left').value)
        self.ticks_right = float(
            self.get_parameter('ticks_per_rev_right').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(
            self.get_parameter('child_frame_id').value)
        self.publish_rate = float(
            self.get_parameter('publish_rate').value)
        self.velocity_timeout = float(
            self.get_parameter('velocity_timeout').value)
        self.debug = bool(self.get_parameter('debug').value)

        if min(
            self.ticks_left,
            self.ticks_right,
            self.wheel_radius,
            self.wheel_separation,
            self.publish_rate,
            self.velocity_timeout,
        ) <= 0.0:
            raise ValueError(
                'Geometria, ticks e parâmetros temporais devem ser positivos.')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_measurement_ns = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, serial_rx_topic, self.on_serial_line, 100)
        self.create_timer(1.0 / self.publish_rate, self.publish_odometry)

        self.get_logger().info(
            f'Odometria ouvindo {serial_rx_topic}; '
            f'ticks=({self.ticks_left:g}, {self.ticks_right:g})')

    @staticmethod
    def quaternion_from_yaw(yaw):
        half = yaw / 2.0
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def on_serial_line(self, message):
        line = message.data.strip()
        if not line.startswith('O '):
            return

        parts = line.split()
        if len(parts) != 4:
            if self.debug:
                self.get_logger().warn(f'Odometria inválida: {line}')
            return

        try:
            ticks_left = int(parts[1])
            ticks_right = int(parts[2])
            dt = int(parts[3]) / 1000.0
        except ValueError:
            if self.debug:
                self.get_logger().warn(f'Falha ao interpretar: {line}')
            return

        if dt <= 0.0:
            return

        distance_left = (
            2.0 * math.pi * self.wheel_radius
            * ticks_left / self.ticks_left
        )
        distance_right = (
            2.0 * math.pi * self.wheel_radius
            * ticks_right / self.ticks_right
        )
        velocity_left = distance_left / dt
        velocity_right = distance_right / dt
        linear = (velocity_right + velocity_left) / 2.0
        angular = (
            velocity_right - velocity_left
        ) / self.wheel_separation

        delta_theta = angular * dt
        heading_mid = self.theta + delta_theta / 2.0
        self.x += linear * math.cos(heading_mid) * dt
        self.y += linear * math.sin(heading_mid) * dt
        self.theta = math.atan2(
            math.sin(self.theta + delta_theta),
            math.cos(self.theta + delta_theta),
        )
        self.last_linear = linear
        self.last_angular = angular
        self.last_measurement_ns = self.get_clock().now().nanoseconds

        if self.debug:
            self.get_logger().info(
                f'dL={ticks_left} dR={ticks_right} dt={dt:.3f}s '
                f'v={linear:.3f} w={angular:.3f} '
                f'x={self.x:.3f} y={self.y:.3f} theta={self.theta:.3f}')

    def publish_odometry(self):
        now = self.get_clock().now()
        linear = 0.0
        angular = 0.0
        if self.last_measurement_ns is not None:
            age = (now.nanoseconds - self.last_measurement_ns) / 1e9
            if age <= self.velocity_timeout:
                linear = self.last_linear
                angular = self.last_angular

        stamp = now.to_msg()
        qx, qy, qz, qw = self.quaternion_from_yaw(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.1
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

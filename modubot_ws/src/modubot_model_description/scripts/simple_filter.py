#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ModubotFilter(Node):
    def __init__(self):
        super().__init__('modubot_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('minimum_range', 0.15)
        self.declare_parameter('blocked_sector_enabled', True)
        self.declare_parameter('blocked_sector_center', 0.0)
        self.declare_parameter(
            'blocked_sector_half_width', math.radians(15.0))

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.minimum_range = float(
            self.get_parameter('minimum_range').value)
        self.blocked_sector_enabled = bool(
            self.get_parameter('blocked_sector_enabled').value)
        self.blocked_sector_center = float(
            self.get_parameter('blocked_sector_center').value)
        self.blocked_sector_half_width = float(
            self.get_parameter('blocked_sector_half_width').value)

        self.sub = self.create_subscription(
            LaserScan, input_topic, self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        blocked_width = (
            2.0 * self.blocked_sector_half_width
            if self.blocked_sector_enabled else 0.0
        )
        useful_fov = 360.0 - math.degrees(blocked_width)
        self.get_logger().info(
            f'Filtro do ModuBot iniciado: campo de visão útil de '
            f'aproximadamente {useful_fov:.1f} graus e timestamp original '
            'preservado.')

    @staticmethod
    def angular_distance(angle, center):
        return math.atan2(
            math.sin(angle - center), math.cos(angle - center))

    def callback(self, msg):
        filtered_ranges = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + (i * msg.angle_increment)
            blocked = (
                self.blocked_sector_enabled
                and abs(self.angular_distance(
                    angle, self.blocked_sector_center
                )) <= self.blocked_sector_half_width
            )

            if blocked or (math.isfinite(r) and r < self.minimum_range):
                # NaN means "not observed". Using +inf here would make the
                # obstacle layer raytrace the blind sector as free space.
                filtered_ranges.append(float('nan'))
            else:
                filtered_ranges.append(r)

        msg.ranges = filtered_ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModubotFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

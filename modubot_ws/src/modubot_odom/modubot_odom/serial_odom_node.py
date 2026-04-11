import math
import serial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom')

        # Parâmetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('ticks_per_rev_left', 90.0)
        self.declare_parameter('ticks_per_rev_right', 90.0)
        self.declare_parameter('wheel_radius', 0.078)      # m
        self.declare_parameter('wheel_separation', 0.223)  # m
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('debug', False)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.ticks_per_rev_L = float(
            self.get_parameter('ticks_per_rev_left').value)
        self.ticks_per_rev_R = float(
            self.get_parameter('ticks_per_rev_right').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.B = float(self.get_parameter('wheel_separation').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.debug = bool(self.get_parameter('debug').value)

        # Estado de pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw

        # Serial
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f"Abrindo serial {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Falha ao abrir {port}: {e}")
            raise

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer de leitura
        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz

    def quaternion_from_yaw(self, yaw):
        """Retorna (x,y,z,w) para rotação apenas em Z."""
        half = yaw / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def read_serial(self):
        try:
            line_bytes = self.ser.readline()
        except Exception as e:
            self.get_logger().warn(f"Erro lendo serial: {e}")
            return

        if not line_bytes:
            return

        try:
            line = line_bytes.decode('utf-8').strip()
        except UnicodeDecodeError:
            return

        if not line:
            return

        # Linha do tipo: "O dL dR dt_ms"
        if not line.startswith('O'):
            # pode ser log do ESP
            if self.debug:
                self.get_logger().info(f"DBG ESP: {line}")
            return

        parts = line.split()
        if len(parts) != 4:
            if self.debug:
                self.get_logger().warn(f"Linha O inválida: {line}")
            return

        try:
            dL = int(parts[1])
            dR = int(parts[2])
            dt_ms = int(parts[3])
        except ValueError:
            if self.debug:
                self.get_logger().warn(f"Falha no parse: {line}")
            return

        if dt_ms <= 0:
            return

        dt = dt_ms / 1000.0  # s

        # Distâncias percorridas por cada roda
        dist_L = 2.0 * math.pi * self.R * (dL / self.ticks_per_rev_L)
        dist_R = 2.0 * math.pi * self.R * (dR / self.ticks_per_rev_R)

        # Velocidades lineares de cada roda
        vL = dist_L / dt
        vR = dist_R / dt

        # Cinemática diferencial
        v = (vR + vL) / 2.0
        w = (vR - vL) / self.B

        # Integração simples
        self.theta += w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        if self.debug:
            self.get_logger().info(
                f"dL={dL} dR={dR} dt={dt:.3f}s -> v={v:.3f} w={w:.3f} x={self.x:.3f} y={self.y:.3f} th={self.theta:.3f}"
            )

        # Publica Odometry
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariâncias simples (ajustar depois se quiser)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.1

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # Publica TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


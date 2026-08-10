#!/usr/bin/env python3
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('serial_rx_topic', '/modubot/serial_rx')
        self.declare_parameter('wheel_separation', 0.207)
        self.declare_parameter('wheel_radius', 0.078)
        self.declare_parameter('max_wheel_speed', 0.6)
        self.declare_parameter('ticks_per_rev_left', 91.0)
        self.declare_parameter('ticks_per_rev_right', 91.0)
        self.declare_parameter('send_rate', 20.0)
        self.declare_parameter('read_rate', 100.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('kp', 12.0)
        self.declare_parameter('ki', 40.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('kff', 0.0)
        self.declare_parameter('dac_min', 0.0)
        self.declare_parameter('debug', False)

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        serial_rx_topic = str(self.get_parameter('serial_rx_topic').value)

        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value)
        self.ticks_left = float(self.get_parameter('ticks_per_rev_left').value)
        self.ticks_right = float(self.get_parameter('ticks_per_rev_right').value)
        self.send_rate = float(self.get_parameter('send_rate').value)
        self.read_rate = float(self.get_parameter('read_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.closed_loop = bool(self.get_parameter('closed_loop').value)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.kff = float(self.get_parameter('kff').value)
        self.dac_min = float(self.get_parameter('dac_min').value)
        self.debug = bool(self.get_parameter('debug').value)

        self._validate_parameters()

        try:
            self.serial = serial.Serial(
                port,
                baudrate=baud,
                timeout=0,
                write_timeout=0.2,
            )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().fatal(f'Não foi possível abrir {port}: {exc}')
            raise

        self.get_logger().info(f'Porta serial {port} aberta em {baud} baud')
        self.serial_lock = threading.Lock()
        self.rx_buffer = bytearray()
        self.target_rad = (0.0, 0.0)
        self.target_norm = (0.0, 0.0)
        self.last_cmd_time = None
        self.braked = True

        self.serial_pub = self.create_publisher(String, serial_rx_topic, 100)
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.send_timer = self.create_timer(
            1.0 / self.send_rate, self.on_send_timer)
        self.read_timer = self.create_timer(
            1.0 / self.read_rate, self.read_serial)

        # A abertura da USB pode reiniciar a ESP32.
        time.sleep(0.5)
        self.configure_firmware()

    def _validate_parameters(self):
        positive = {
            'wheel_separation': self.wheel_separation,
            'wheel_radius': self.wheel_radius,
            'max_wheel_speed': self.max_wheel_speed,
            'ticks_per_rev_left': self.ticks_left,
            'ticks_per_rev_right': self.ticks_right,
            'send_rate': self.send_rate,
            'read_rate': self.read_rate,
            'cmd_timeout': self.cmd_timeout,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(
                f'Parâmetros devem ser positivos: {", ".join(invalid)}')
        if min(self.kp, self.ki, self.kd, self.kff, self.dac_min) < 0.0:
            raise ValueError('Ganhos, KFF e dac_min não podem ser negativos.')
        if self.dac_min > 255.0:
            raise ValueError('dac_min deve estar entre 0 e 255.')

    def write_line(self, line):
        try:
            with self.serial_lock:
                self.serial.write((line + '\n').encode('ascii'))
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            self.get_logger().error(f'Falha ao escrever na serial: {exc}')

    def configure_firmware(self):
        self.target_rad = (0.0, 0.0)
        self.target_norm = (0.0, 0.0)
        self.last_cmd_time = None
        self.write_line(f'C {self.ticks_left:.3f} {self.ticks_right:.3f}')
        self.write_line(f'K {self.kp:.4f} {self.ki:.4f} {self.kd:.4f}')
        self.write_line(f'F {self.kff:.4f} {self.dac_min:.3f}')
        self.write_line(f'M {1 if self.closed_loop else 0}')
        self.write_line('P 0')
        self.braked = True
        mode = 'fechada' if self.closed_loop else 'aberta'
        self.get_logger().info(
            f'ESP32 configurada: malha {mode}, '
            f'K=({self.kp:g}, {self.ki:g}, {self.kd:g}), KFF={self.kff:g}')

    def on_cmd_vel(self, msg):
        left = float(msg.linear.x) - (
            self.wheel_separation / 2.0) * float(msg.angular.z)
        right = float(msg.linear.x) + (
            self.wheel_separation / 2.0) * float(msg.angular.z)

        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            left *= scale
            right *= scale
            if self.debug:
                self.get_logger().warn(
                    f'Comando saturado com escala proporcional {scale:.3f}')

        self.target_rad = (
            left / self.wheel_radius,
            right / self.wheel_radius,
        )
        self.target_norm = (
            left / self.max_wheel_speed,
            right / self.max_wheel_speed,
        )
        self.last_cmd_time = self.get_clock().now()

    def on_send_timer(self):
        if self.last_cmd_time is None:
            return

        age = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds * 1e-9
        stopped = age > self.cmd_timeout or all(
            abs(value) < 1e-4 for value in self.target_rad)

        if stopped:
            if not self.braked:
                self.write_line('S')
                self.braked = True
            return

        if self.closed_loop:
            left, right = self.target_rad
            self.write_line(f'W {left:.4f} {right:.4f}')
        else:
            left, right = self.target_norm
            self.write_line(f'V {left:.4f} {right:.4f}')
        self.braked = False

    def read_serial(self):
        try:
            waiting = self.serial.in_waiting
            if waiting:
                self.rx_buffer.extend(self.serial.read(waiting))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f'Falha ao ler a serial: {exc}')
            return

        while b'\n' in self.rx_buffer:
            raw, _, self.rx_buffer = self.rx_buffer.partition(b'\n')
            line = raw.rstrip(b'\r').decode('utf-8', errors='replace').strip()
            if not line:
                continue

            message = String()
            message.data = line
            self.serial_pub.publish(message)

            if 'ModubotFirmwarePID READY' in line:
                self.configure_firmware()
            elif self.debug and line.startswith('#'):
                self.get_logger().info(f'ESP32: {line}')

    def shutdown(self):
        try:
            self.write_line('S')
            self.serial.close()
        except (serial.SerialException, OSError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

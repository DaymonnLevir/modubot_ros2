#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, threading, time

def clamp(x,a,b): return max(a, min(b, x))

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')

        # Parâmetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.28)  # B [m]
        self.declare_parameter('wheel_radius', 0.05)       # R [m]
        self.declare_parameter('v_wheel_max', 0.6)         # vel. linear de roda p/ 100% DAC [m/s]
        self.declare_parameter('watchdog_sec', 0.6)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
            self.get_logger().info(f'Conectado em {port} @ {baud}')
        except Exception as e:
            self.get_logger().fatal(f'Não abriu serial: {e}')
            raise

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.lock = threading.Lock()
        self.last_send = time.time()
        self.timer = self.create_timer(0.2, self.watchdog)

    def write_line(self, line: str):
        with self.lock:
            try:
                self.ser.write((line+'\n').encode('utf-8'))
                self.last_send = time.time()
            except Exception as e:
                self.get_logger().error(f'Falha ao enviar: {e}')

    def on_twist(self, msg: Twist):
        B = float(self.get_parameter('wheel_separation').value)
        v_wheel_max = float(self.get_parameter('v_wheel_max').value)

        vx = msg.linear.x         # frente(+)/ré(-)
        wz = msg.angular.z        # yaw (CCW +)
        # (holonômico: linear.y, linear.z, etc. são ignorados para diferencial)

        # cinemâtica diferencial (velocidade linear de cada roda, m/s)
        vL = vx - (B/2.0)*wz
        vR = vx + (B/2.0)*wz

        # normaliza para [-1..1] conforme capacidade de roda
        if v_wheel_max <= 0.0:
            v_wheel_max = 0.6
        nL = clamp(vL / v_wheel_max, -1.0, 1.0)
        nR = clamp(vR / v_wheel_max, -1.0, 1.0)

        self.write_line(f"V {nL:.3f} {nR:.3f}")
        # self.get_logger().debug(f"cmd_vel -> V {nL:.2f} {nR:.2f}")

    def watchdog(self):
        if time.time() - self.last_send > float(self.get_parameter('watchdog_sec').value):
            self.write_line("S")

def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


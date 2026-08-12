#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState


class BatteryMonitorNode(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        # ==========================================
        # PARAMETROS
        # ==========================================

        self.declare_parameter(
            'serial_rx_topic',
            '/modubot/serial_rx'
        )

        self.declare_parameter(
            'battery_voltage_min',
            18.0
        )

        self.declare_parameter(
            'battery_voltage_max',
            25.0
        )

        self.declare_parameter(
            'soc_alpha',
            0.05
        )

        self.declare_parameter(
            'design_capacity',
            5.0
        )

        serial_rx_topic = str(
            self.get_parameter(
                'serial_rx_topic'
            ).value
        )

        self.voltage_min = float(
            self.get_parameter(
                'battery_voltage_min'
            ).value
        )

        self.voltage_max = float(
            self.get_parameter(
                'battery_voltage_max'
            ).value
        )

        self.soc_alpha = float(
            self.get_parameter(
                'soc_alpha'
            ).value
        )

        self.design_capacity = float(
            self.get_parameter(
                'design_capacity'
            ).value
        )

        if self.voltage_max <= self.voltage_min:
            raise ValueError(
                'battery_voltage_max deve ser maior '
                'que battery_voltage_min'
            )

        # ==========================================
        # PUBLISHERS
        # ==========================================

        self.voltage_pub = self.create_publisher(
            Float32,
            '/battery/voltage',
            10
        )

        self.percentage_pub = self.create_publisher(
            Float32,
            '/battery/percentage',
            10
        )

        self.battery_state_pub = self.create_publisher(
            BatteryState,
            '/battery_state',
            10
        )

        # ==========================================
        # SUBSCRICAO DO BARRAMENTO SERIAL
        # ==========================================

        self.create_subscription(
            String,
            serial_rx_topic,
            self.on_serial_line,
            100
        )

        self.filtered_soc = None

        self.get_logger().info(
            f'Monitor de bateria ouvindo '
            f'{serial_rx_topic}'
        )

    # ==============================================
    # SOC PROVISORIO
    # ==============================================

    def calculate_soc(self, voltage):

        soc = (
            (voltage - self.voltage_min)
            /
            (self.voltage_max - self.voltage_min)
        ) * 100.0

        # Limita entre 0 e 100 %
        soc = max(
            0.0,
            min(100.0, soc)
        )

        # Primeira leitura
        if self.filtered_soc is None:
            self.filtered_soc = soc

        else:
            self.filtered_soc = (
                self.soc_alpha * soc
                +
                (1.0 - self.soc_alpha)
                * self.filtered_soc
            )

        return self.filtered_soc

    # ==============================================
    # RECEBE /modubot/serial_rx
    # ==============================================

    def on_serial_line(self, msg):

        line = msg.data.strip()

        # Ignora odometria, telemetria,
        # jitter e qualquer outra mensagem
        if not line.startswith('BAT:'):
            return

        try:
            voltage = float(
                line.split(':', 1)[1]
            )

        except (ValueError, IndexError):
            self.get_logger().warn(
                f'Mensagem BAT invalida: {line}'
            )
            return

        soc = self.calculate_soc(
            voltage
        )

        self.publish_battery(
            voltage,
            soc
        )

    # ==============================================
    # PUBLICACAO
    # ==============================================

    def publish_battery(self, voltage, soc):

        # ------------------------------------------
        # /battery/voltage
        # ------------------------------------------

        voltage_msg = Float32()

        voltage_msg.data = float(
            voltage
        )

        self.voltage_pub.publish(
            voltage_msg
        )

        # ------------------------------------------
        # /battery/percentage
        # Valor de 0 a 100 %
        # ------------------------------------------

        percentage_msg = Float32()

        percentage_msg.data = float(
            soc
        )

        self.percentage_pub.publish(
            percentage_msg
        )

        # ------------------------------------------
        # /battery_state
        # ------------------------------------------

        battery_msg = BatteryState()

        battery_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        battery_msg.voltage = float(
            voltage
        )

        # sensor_msgs/BatteryState:
        # 0.0 = 0 %
        # 1.0 = 100 %
        battery_msg.percentage = float(
            soc / 100.0
        )

        # Duas baterias de 2.5 Ah
        # conectadas em paralelo
        battery_msg.design_capacity = (
            self.design_capacity
        )

        # Ainda não temos sensores
        # para estes valores
        battery_msg.temperature = float('nan')
        battery_msg.current = float('nan')
        battery_msg.charge = float('nan')
        battery_msg.capacity = float('nan')

        battery_msg.present = True

        battery_msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        )

        battery_msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )

        battery_msg.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        )

        self.battery_state_pub.publish(
            battery_msg
        )

        self.get_logger().info(
            f'Bateria: {voltage:.1f} V | '
            f'{soc:.0f} %'
        )


def main(args=None):

    rclpy.init(args=args)

    node = BatteryMonitorNode()

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

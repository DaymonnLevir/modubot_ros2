#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ModubotFilter(Node):
    def __init__(self):
        super().__init__('modubot_filter')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info('Filtro do Modubot iniciado. Corrigindo Time Delay e Filtro.')

    def callback(self, msg):
        # --- PASSO 1: CORREÇÃO DO DELAY (CRÍTICO) ---
        # Forçamos a mensagem filtrada a ter o tempo exato de AGORA na Jetson
        msg.header.stamp = self.get_clock().now().to_msg()
        
        filtered_ranges = []
        
        for i, r in enumerate(msg.ranges):
            # Calcula o ângulo atual baseado no índice do feixe
            angle = msg.angle_min + (i * msg.angle_increment)
            
            # Normaliza o ângulo para ficar entre -pi e pi
            while angle > math.pi: angle -= 2.0 * math.pi
            while angle < -math.pi: angle += 2.0 * math.pi

            # --- PASSO 2: LÓGICA DE CORTE CORRIGIDA ---
            # Frente do robô em ROS é o ângulo 0. 
            # 90° para esquerda é +1.57 rad, 90° para direita é -1.57 rad.
            # Para manter apenas a FRENTE (um cone de 180°), usamos abs(angle) < 1.57
            
            if abs(angle) > 1.57:  # MUDANÇA AQUI: de > para <
                # Filtro de proximidade (evitar ler a própria estrutura do robô)
                if r > 0.15:
                    filtered_ranges.append(r)
                else:
                    filtered_ranges.append(float('inf'))
            else:
                # Tudo que estiver atrás vira infinito
                filtered_ranges.append(float('inf'))
        
        # Atualiza a mensagem e publica
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
from .cmdvel_to_serial import *
def main():
    import rclpy
    rclpy.init()
    from .cmdvel_to_serial import CmdVelToSerial
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

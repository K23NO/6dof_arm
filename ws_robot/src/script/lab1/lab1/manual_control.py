#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Offsets para convertir ángulos relativos a absolutos
SERVO_OFFSETS = [90, 90, 90, 0, 90, 30]

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)

    def send_angles(self, angles_relative):
        # Convertir ángulos relativos a absolutos
        angles_absolute = [
            relative + offset for relative, offset in zip(angles_relative, SERVO_OFFSETS)
        ]

        msg = Int32MultiArray()
        msg.data = angles_absolute
        self.publisher_.publish(msg)

        self.get_logger().info(f'Relativos: {angles_relative} | Absolutos: {angles_absolute}')

def main():
    rclpy.init()
    node = ManualControl()

    try:
        while rclpy.ok():
            entrada = input("Ingrese 6 ángulos relativos (ej: 0 0 0 0 0 0): ")
            try:
                valores = [int(x) for x in entrada.strip().split()]
                if len(valores) != 6:
                    print("⚠️ Debe ingresar exactamente 6 valores.")
                    continue
                node.send_angles(valores)
            except ValueError:
                print("⚠️ Entrada inválida. Use solo números enteros.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

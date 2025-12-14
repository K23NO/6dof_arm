import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Offsets: la posición física de home de cada servo (grados absolutos)
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 30]


class ServoInteractivePublisher(Node):
    def __init__(self):
        super().__init__('servo_interactive_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)

    def send_angles(self, angles_relative):
        angles_absolute = [
            relative + offset for relative, offset in zip(angles_relative, SERVO_HOME_OFFSETS)
        ]

        msg = Int32MultiArray()
        msg.data = angles_absolute
        self.publisher_.publish(msg)

        self.get_logger().info(f'Ángulos relativos: {angles_relative} | Enviados (absolutos): {angles_absolute}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoInteractivePublisher()

    try:
        while rclpy.ok():
            entrada = input("Ingrese 6 ángulos relativos (ej: 0 0 0 0 0 0): ")
            try:
                valores = [int(x) for x in entrada.strip().split()]
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


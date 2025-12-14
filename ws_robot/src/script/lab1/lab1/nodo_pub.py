#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ServoInteractivePublisher(Node):
    def __init__(self):
        super().__init__('servo_interactive_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/servo_commands', 10)
        self.home_position = [80, 10, 90, 0, 110]  # Tu posición home
   
    def send_angles(self, angles):    
        msg = Int32MultiArray()
        # Asegurar que tenemos exactamente 5 valores      
        if len(angles) > 5:
            angles = angles[:5]
        elif len(angles) < 5:
            angles = angles + [0] * (5 - len(angles))
            
        msg.data = angles
        self.publisher_.publish(msg)
        self.get_logger().info(f'Ángulos enviados: {angles}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoInteractivePublisher()

    print("\n=== Control de Brazo Robot ===")
    print("Comandos especiales:")
    print("  'home' - Posición home [80, 10, 90, 0, 110]")
    print("  'exit' - Salir")
    
    try:
        while rclpy.ok():
            entrada = input("\nIngrese 5 ángulos separados por espacio (0-180) o comando: ")
            
            if entrada.lower() == 'exit':
                break
            elif entrada.lower() == 'home':
                node.send_angles(node.home_position)
                continue
                
            try:
                valores = [int(x) for x in entrada.strip().split()]
                if len(valores) != 5:
                    print("⚠ Debe ingresar exactamente 5 valores.")
                    continue
                # Validar rango
                if all(0 <= x <= 180 for x in valores):
                    node.send_angles(valores)
                else:
                    print("⚠ Los ángulos deben estar entre 0 y 180.")
            except ValueError:
                print("⚠ Entrada inválida. Use solo números enteros.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
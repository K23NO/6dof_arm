import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

# Offsets: la posición física de home de cada servo
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 40]

# Secuencia de posiciones relativas (sin offset)
SECUENCIA = [
    # --- Inicio t=0 ---
    [0, 0, 0, 0, 0, 0],		#PUNTO A

    # t=0 a t=1 (Salto grande en J2: 0 a -20, J3: 0 a 10)
    [0, 0, -5, 2, 0, 0],
    [0, 0, -10, 5, 0, 0],
    [0, 0, -15, 8, 0, 0],
    [0, 0, -20, 10, 0, 0],	#PUNTO B

    # t=1 a t=2 (J1: 0 a -10)
    [-5, 0, -20, 10, 0, 0],
    [-10, 0, -20, 10, 0, 0],

    # t=2 (J1: -10 a -20, J3: -20 a -30, J4: 10 a 20)
    [-15, 0, -25, 15, 0, 0],
    [-20, 0, -30, 20, 0, 0],

    # (J1: -20 a -30)
    [-25, 0, -30, 20, 0, 0],
    [-30, 0, -30, 20, 0, 0],	#PUNTO C

    # t=3 (J1: -30 a -40, J4: 20 a 0) -> J4 cambia 20, necesitamos 4 pasos
    [-32, 0, -30, 15, 0, 0],	
    [-35, 0, -30, 10, 0, 0],
    [-38, 0, -30, 5, 0, 0],
    [-40, 0, -30, 0, 0, 0],

    # (J1: -40 a -55) -> Diferencia 15, necesitamos 3 pasos
    [-45, 0, -30, 0, 0, 0],
    [-50, 0, -30, 0, 0, 0],
    [-55, 0, -30, 0, 0, 0],

    # (J3: -30 a -20) -> Diferencia 10, 2 pasos
    [-55, 0, -25, 0, 0, 0],
    [-55, 0, -20, 0, 0, 0],	#PUNTO D

    # (J6: 0 a 40) -> GRAN SALTO DE 40, necesitamos 8 pasos
    [-55, 0, -20, 0, 0, 5],
    [-55, 0, -20, 0, 0, 10],
    [-55, 0, -20, 0, 0, 15],
    [-55, 0, -20, 0, 0, 20],
    [-55, 0, -20, 0, 0, 25],
    [-55, 0, -20, 0, 0, 30],
    [-55, 0, -20, 0, 0, 35],
    [-55, 0, -20, 0, 0, 35],	#PUNTO E

    # Nikolay
    [-55, 0, -25, 0, 0, 35],
    [-55, 0, -30, 0, 0, 35],
    [-55, 0, -30, 0, 0, 35],	#PUNTO F
    # t=4 (J1: -55 a -40) -> Diferencia 15
    [-50, 0, -30, 0, 0, 35],
    [-45, 0, -30, 0, 0, 35],
    [-40, 0, -30, 0, 0, 35],
    
    # t=5 (J1: -40 a -30, J3: -20 a -30) -> Diferencia 10
    [-35, 0, -30, 0, 0, 35],
    [-30, 0, -30, 0, 0, 35],

    # t=6 (J1: -30 a -20) -> Diferencia 10
    [-25, 0, -30, 0, 0, 35],
    [-20, 0, -30, 0, 0, 35],

    # t=7 (J1: -20 a 0) -> Diferencia 20, 4 pasos
    [-15, 0, -30, 0, 0, 35],
    [-10, 0, -30, 0, 0, 35],
    [-5, 0, -30, 0, 0, 35],
    [0, 0, -30, 0, 0, 35],	#PUNTO G

    # (J1: 0 a 10)
    [5, 0, -32, 0, 0, 35],
    [10, 0, -35, 0, 0, 35],

    # t=8 (J1: 10 a 20)
    [15, 0, -35, 0, 0, 35],
    [20, 0, -35, 0, 0, 35],

    # (J1: 20 a 30)
    [25, 0, -35, 0, 0, 35],
    [30, 0, -35, 0, 0, 35],

    # t=9 (J1: 30 a 40)
    [35, 0, -35, 0, 0, 35],
    [40, 0, -35, 0, 0, 35],      #PUNTO H

    # (J1: 40 a 50)
    [45, 0, -35, 0, 0, 35],
    [50, 0, -35, 0, 0, 35],

    # t=10 (J1: 50 a 60, J3: -30 a -35, J4: 0 a 10) -> Max diff 10 (J1 y J4)
    [55, 0, -35, 0, 0, 35],
    [60, 0, -35, 0, 0, 35],

    # (J1: 60 a 70)
    [65, 0, -35, 0, 0, 35],
    [70, 0, -35, 0, 0, 35],

    # t=11 (J1: 70 a 80)
    [75, 0, -35, 0, 0, 35],
    [80, 0, -35, 0, 0, 35],
    # t=12 (J1: 80 a 90, J3: -35 a -45) -> Diferencia 10
    [85, 0, -35, 0, 0, 35],
    [90, 0, -35, 0, 0, 35],	#PUNTO I

    # t=14 (J3: -45 a -40, J4: 10 a 0) -> Diferencia 10 (en J4)
    [90, 0, -30, 0, 0, 35],
    [90, 0, -30, 0, 0, 35],     #PUNTO J
    # t=15 (J6: 40 a 0) -> Soltar objeto. GRAN SALTO DE 40. 8 Pasos.
    [90, 0, -30, 0, 0, 30],
    [90, 0, -30, 0, 0, 30],
    [90, 0, -30, 0, 0, 25],
    [90, 0, -30, 0, 0, 20],
    [90, 0, -30, 0, 0, 15],
    [90, 0, -30, 0, 0, 10],
    [90, 0, -30, 0, 0, 5],
    [90, 0, -30, 0, 0, 0],	#PUNTO K

    # t=16 (J3: -40 a -50) -> Sube un poco. Diferencia 10.
    [90, 0, -35, 0, 0, 0],
    [90, 0, -35, 0, 0, 0],	#PUNTO L

    # t=17 (J1: 90 a 80)
    [85, 0, -35, 0, 0, 0],
    [80, 0, -35, 0, 0, 0],

    # t=18 (J1: 80 a 70)
    [75, 0, -35, 0, 0, 0],
    [70, 0, -35, 0, 0, 0],

    # t=19 (J1: 70 a 60, J3: -50 a -40) -> Diferencia 10
    [65, 0, -35, 0, 0, 0],
    [60, 0, -35, 0, 0, 0],

    # t=20 (J1: 60 a 50, J3: -40 a -30) -> Diferencia 10
    [55, 0, -35, 0, 0, 0],
    [50, 0, -30, 0, 0, 0],	#PUNTO M

    # t=21 (J1: 50 a 40)
    [45, 0, -30, 0, 0, 0],
    [40, 0, -30, 0, 0, 0],

    # t=22 (J1: 40 a 30, J3: -30 a -20) -> Diferencia 10
    [35, 0, -25, 0, 0, 0],
    [30, 0, -20, 0, 0, 0],

    # (J1: 30 a 20)
    [25, 0, -20, 0, 0, 0],	#PUNTO N
    [20, 0, -20, 0, 0, 0],

    # t=23 (J1: 20 a 10, J3: -20 a -10)
    [15, 0, -15, 0, 0, 0],
    [10, 0, -10, 0, 0, 0],

    # t=24 (J1: 10 a 0, J3: -10 a 0) -> Regreso a home
    [5, 0, -5, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]		#PUNTO O
]



class ServoTimedPublisher(Node):
    def __init__(self):
        super().__init__('servo_timed_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        self.get_logger().info("Iniciando publicación automática de secuencia…")
        self.publicar_secuencia()

    def aplicar_offsets(self, angles_relative):
        return [
            relative + offset for relative, offset in zip(angles_relative, SERVO_HOME_OFFSETS)
        ]

    def publicar_secuencia(self):
        for t, angles in enumerate(SECUENCIA):
            angles_absolute = self.aplicar_offsets(angles)

            msg = Int32MultiArray()
            msg.data = angles_absolute
            self.publisher_.publish(msg)

            self.get_logger().info(f"t={t} | Relativos: {angles} | Enviados (absolutos): {angles_absolute}")

            time.sleep(1)  # 0.5 segundos entre cada estado

def main(args=None):
    rclpy.init(args=args)
    node = ServoTimedPublisher()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

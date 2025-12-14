import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time
import numpy as np
from scipy.interpolate import CubicSpline

# Frecuencia de publicación
FREQ = 2           # 50 Hz → 20 ms por punto
DT = 1.0 / FREQ

# Offsets físicos de home
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 30]

# Puntos clave (tiempos en segundos, ángulos relativos)
KEY_TIMES = np.array([0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120], dtype=float) * 3.5/5
#KEY_POINTS = np
KEY_POINTS = [
    [0,0,0,0,0,0],        # t = 0
    [0,0,-20,10,0,0],     # t = 1
    [-20,0,-30,20,0,0],   # t = 2
    [-40,0,-25,0,0,0],    # t = 3
    [-40,0,-25,0,0,20],   # t = 4
    [-40,0,-35,0,0,20],   # t = 5
    [-20,0,-35,0,0,20],   # t = 6
    [0,0,-35,0,0,20],     # t = 7
    [20,0,-35,0,0,20],    # t = 8
    [40,0,-35,0,0,20],    # t = 9
    [60,0,-35,10,0,20],   # t = 10
    [80,0,-35,10,0,20],   # t = 11
    [90,0,-45,10,0,20],   # t = 12
    [90,0,-45,10,0,20],   # t = 13
    [90,0,-40,0,0,20],    # t = 14
    [90,0,-40,0,0,0],     # t = 15
    [90,0,-40,0,0,0],     # t = 16
    [80,0,-40,0,0,0],     # t = 17
    [70,0,-40,0,0,0],     # t = 18
    [60,0,-30,0,0,0],     # t = 19
    [50 ,0,-20,0,0,0],     # t = 20
    [40,0,-10,0,0,0],     # t = 21
    [30,0,0,0,0,0],       # t = 22
    [15,0,0,0,0,0],       # t = 23
    [0,0,0,0,0,0],        # t = 24
]


class ServoSplinePublisher(Node):
    def __init__(self):
        super().__init__('servo_spline_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        
        self.get_logger().info("Generando splines cúbicos por tramos…")
        self.splines = self.generar_splines()
        
        self.get_logger().info("Iniciando movimiento suave…")
        self.run_sequence()

    def generar_splines(self):
        """
        Genera un spline cúbico por cada servo usando los puntos clave.
        """
        puntos = np.array(KEY_POINTS)  # shape (9,6)
        splines = []

        for servo in range(6):
            valores_servo = puntos[:, servo]
            spline = CubicSpline(KEY_TIMES, valores_servo, bc_type='natural')
            splines.append(spline)

        return splines

    def aplicar_offsets(self, angles_relative):
        """
        Convierte los ángulos relativos en absolutos.
        """
        return [rel + off for rel, off in zip(angles_relative, SERVO_HOME_OFFSETS)]

    def publicar(self, valores):
        msg = Int32MultiArray()
        msg.data = valores
        self.publisher_.publish(msg)

    def run_sequence(self):
        tiempo_final = KEY_TIMES[-1]
        t = 0.0

        while t <= tiempo_final:
            valores_rel = [float(spline(t)) for spline in self.splines]
            valores_rel = [int(round(v)) for v in valores_rel]

            valores_abs = self.aplicar_offsets(valores_rel)
            self.publicar(valores_abs)

            self.get_logger().info(f"t={t:.2f}  |  abs={valores_abs}")

            time.sleep(DT)
            t += DT

        self.get_logger().info("Trayectoria spline completa.")

def main(args=None):
    rclpy.init(args=args)
    node = ServoSplinePublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
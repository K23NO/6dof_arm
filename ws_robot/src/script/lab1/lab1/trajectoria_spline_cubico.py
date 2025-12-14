import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time
import numpy as np
from scipy.interpolate import CubicSpline

# Frecuencia de publicación
FREQ = 1           # 50 Hz → 20 ms por punto
DT = 1.0 / FREQ

# Offsets físicos de home
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 35]

# Puntos clave (tiempos en segundos, ángulos relativos)
KEY_TIMES = np.array([0, 3, 6, 9, 12, 15, 21, 27, 33, 36, 39,45,51,57,63], dtype=float)

KEY_POINTS = [
    [0, 0, 0, 0, 0, 5],          # A
    [0, 0, -20, 10, 0, 5],     	 # B
    [-30, 0, -30, 20, 0, 5],     # C
    [-55, 0, -20, 0, 0, 0],      # D
    [-55, 0, -20, 0, 0, 35],     # E
    [-55, 0, -30, 0, 0, 35],     # F
    [0, 0, -30, 0, 0, 35],       # G
    [40, 0, -35, 0, 0, 35],      # H
    [80, 0, -35, 0, 0, 35],      # I
    [80, 0, -30, 0, 0, 35],      # J
    [80, 0, -30, 0, 0, 5],       # K
    [80, 0, -35, 0, 0, 5],       # L
    [50, 0, -30, 0, 0, 5],       # M
    [25, 0, -20, 0, 0, 5],       # N
    [0, 0, 0, 0, 0, 5]	         # O
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
        puntos = np.array(KEY_POINTS)
        splines = []

        for servo in range(6):
            valores_servo = puntos[:, servo]

            if servo == 5:
            # La garra: NO usar spline cúbico
            # Mantener exactamente el valor entre puntos
                spline = lambda t, vals=valores_servo: np.interp(t, KEY_TIMES, vals)
            else:
            # Articulaciones: spline cúbico suave
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


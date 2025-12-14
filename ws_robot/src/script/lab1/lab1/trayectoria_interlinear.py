import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time
import numpy as np

# Frecuencia de actualización (Hz)
FREQ = 5             # 20 Hz → 50 ms
DT = 1.0 / FREQ       # Paso de tiempo

# Número de pasos para interpolar entre cada punto
# (si quieres que cada transición dure 1 segundo, entonces steps = FREQ)
STEPS_PER_TRANSITION = FREQ

# Offsets: home físico de cada servo
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 30]

# Secuencia de posiciones RELATIVAS (sin offset)
SECUENCIA = [
    [0,0,0,0,0,0],        # 0
    [0,0,-30,20,0,0],     # 1
    [-40,0,-30,20,0,0],   # 2
    [-40,0,-25,0,0,0],    # 3
    [-40,0,-25,0,0,20],   # 4
    [-40,0,-35,0,0,20],   # 5
    [-20,0,-35,0,0,20],   # 6
    [0,0,-35,0,0,20],     # 7
    [20,0,-35,0,0,20],    # 8
    [40,0,-35,0,0,20],    # 9
    [60,0,-35,10,0,20],   # 10
    [80,0,-35,10,0,20],   # 11
    [90,0,-45,10,0,20],   # 12
    [90,0,-45,10,0,20],   # 13
    [90,0,-40,0,0,20],    # 14
    [90,0,-40,0,0,0],     # 15
    [90,0,-60,0,0,0],     # 16
    [80,0,-50,0,0,0],     # 17
    [70,0,-40,0,0,0],     # 18
    [60,0,-30,0,0,0],     # 19
    [50,0,-20,0,0,0],     # 20
    [40,0,-10,0,0,0],     # 21
    [30,0,0,0,0,0],       # 22
    [15,0,0,0,0,0],       # 23
    [0,0,0,0,0,0],        # 24
]

class ServoSmoothPublisher(Node):
    def __init__(self):
        super().__init__('servo_smooth_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        self.get_logger().info("Publicación suave con interpolación iniciada…")
        self.run_sequence()

    def aplicar_offsets(self, angles_relative):
        return [relative + offset for relative, offset in zip(angles_relative, SERVO_HOME_OFFSETS)]

    def publicar(self, valores):
        msg = Int32MultiArray()
        msg.data = valores
        self.publisher_.publish(msg)

    def interpolar(self, a, b, steps):
        """Devuelve una lista de vectores interpolados línea por línea"""
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)
        tray = []

        for i in range(steps):
            t = i / steps
            punto = a * (1 - t) + b * t
            tray.append(punto.tolist())

        return tray

    def run_sequence(self):
        # Recorremos cada par de puntos consecutivos
        for i in range(len(SECUENCIA) - 1):
            start = SECUENCIA[i]
            end   = SECUENCIA[i + 1]
            
            # Generar interpolación lineal
            segmentos = self.interpolar(start, end, STEPS_PER_TRANSITION)

            for punto_rel in segmentos:
                # Aplicar offsets
                punto_abs = self.aplicar_offsets([int(round(x)) for x in punto_rel])

                # Publicar
                self.publicar(punto_abs)

                # Mostrar en consola
                self.get_logger().info(f"{punto_abs}")

                # Esperar el tiempo de muestreo
                time.sleep(DT)

        self.get_logger().info("Secuencia suave completa.")

def main(args=None):
    rclpy.init(args=args)
    node = ServoSmoothPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
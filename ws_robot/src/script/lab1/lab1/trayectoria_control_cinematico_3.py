import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
import sympy as sp
import time
from scipy.interpolate import CubicSpline

# Offsets físicos de home
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 40]


# ============================================================
#     CONTROL CINEMÁTICO INVERSO (TU MISMA FUNCIÓN)
# ============================================================
def control_cinematico(q0, xd):
    [q1, q2, q3, q4, q5] = sp.symbols("q1 q2 q3 q4 q5")
    q = sp.Matrix([q1, q2, q3, q4, q5])

    f = sp.Matrix([
        (10.5*sp.sin(q2) - 13*sp.sin(q2-q3+q4) - 14.8*sp.cos(q2-q3))*sp.sin(q1),
        (-10.5*sp.sin(q2) + 13*sp.sin(q2-q3+q4) + 14.8*sp.cos(q2-q3))*sp.cos(q1),
        14.8*sp.sin(q2-q3) + 10.5*sp.cos(q2) - 13*sp.cos(q2-q3+q4) + 8.2
    ])

    q1 = q0
    qret = q0
    J = f.jacobian(q)
    J_fun = sp.lambdify((q,), J, 'numpy')
    f_fun = sp.lambdify((q,), f, 'numpy')

    # ---- evaluación numérica ----
    J_num = J_fun(qret)
    f_num = f_fun(qret).reshape(3,)
    e = f_num - xd

    # Control derivativo simple
    k = 0.3
    e_diff = -k * e

    # Pseudoinversa
    J_inv = np.linalg.pinv(J_num)

    # Update
    q_diff = J_inv @ e_diff
    q1 = q1 + 0.1 * q_diff

    return q1, np.linalg.norm(e)


# ============================================================
#       NODO: SIGUE UNA TRAYECTORIA 3D SUAVIZADA
# ============================================================
class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)

        # q0 inicial
        self.q = np.array([0.1, 0.1, 0.1, 0.1, 0.1])

        # Estado inicial de la garra (abierta)
        self.gripper_value = 0  # sin offset

        # Frecuencia 20 Hz (50 ms)
        self.dt = 0.05

        self.get_logger().info("Generando splines cúbicos de trayectoria…")
        self.traj_fun = self.generar_splines()

        self.get_logger().info("Iniciando seguimiento de trayectoria…")
        self.run_trajectory()


    # =======================================================
    #       Interpolación cúbica (x(t), y(t), z(t))
    # =======================================================
    def generar_splines(self):

        #T = np.array([0, 4], float)
        T = np.array([0, 4, 8, 12, 16, 20, 24, 28, 32, 40, 45], float)

        P = np.array([
            [0, 17, 6],
            [0, 12, 15],
            [15, 8, 15],
            [15, 8, 9],      # cerrar garra después de este punto
            [15, 8, 15],
            [0, 12, 20],
            [-20, 5, 20],
            [-20, 5, 15],
            [-20, 5, 20],
            [0, 10, 12],
            [0, 17, 6]
        ], float)

        x_s = CubicSpline(T, P[:,0])
        y_s = CubicSpline(T, P[:,1])
        z_s = CubicSpline(T, P[:,2])

        return lambda t: np.array([x_s(t), y_s(t), z_s(t)])


    # =======================================================
    #         Seguimiento continuo de la trayectoria
    # =======================================================
    def run_trajectory(self):
        t = 0.0
        T_final = 45

        while t <= T_final and rclpy.ok():

            # --- CAMBIOS DE ESTADO DE LA GARRA ---
            if t >= 12.0 and t < 28.0:
                self.gripper_value = 30   # cerrar garra
            else:
                self.gripper_value = 5    # abrir garra

            # --- posición deseada ---
            xd = self.traj_fun(t)

            # --- controlador IK ---
            self.q, e = control_cinematico(self.q, xd)

            # --- rad → grados ---
            q_deg = np.degrees(self.q)

            # --- aplicar offsets ---
            q_abs = [int(round(a + off)) for a, off in zip(q_deg, SERVO_HOME_OFFSETS)]

            # --- añadir gripper ---
            q_abs.append(self.gripper_value + SERVO_HOME_OFFSETS[5])

            # --- publicar ---
            msg = Int32MultiArray()
            msg.data = q_abs
            self.publisher_.publish(msg)

            self.get_logger().info(
                f"t={t:.2f}  xd={xd}  garra={self.gripper_value}  err={e:.3f}  -> {q_abs}"
            )

            time.sleep(self.dt)
            t += self.dt

        self.get_logger().info("Trayectoria completada.")


# ============================================================
#                        MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


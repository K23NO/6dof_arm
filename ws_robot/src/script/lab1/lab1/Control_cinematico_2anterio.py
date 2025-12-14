#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import sympy as sp
import time

# Offsets: la posición física de home de cada servo
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 0]
GRIPPER_OPEN = 0
GRIPPER_CLOSED = 45

# Se asume que 'control_cinematico' ya está definido en otro archivo o módulo
def control_cinematico(q0, xd):
    [q1, q2, q3, q4, q5] = sp.symbols("q1 q2 q3 q4 q5")
    q = sp.Matrix([q1, q2, q3, q4, q5])
    f = sp.Matrix([
        (10.5*sp.sin(q2) - 13*sp.sin(q2-q3+q4) - 14.8*sp.cos(q2-q3) - 2.5)*sp.sin(q1),
        (-10.5*sp.sin(q2) + 13*sp.sin(q2-q3+q4) + 14.8*sp.cos(q2-q3) + 2.5)*sp.cos(q1),
        14.8*sp.sin(q2-q3) + 10.5*sp.cos(q2) - 13*sp.cos(q2-q3+q4) + 8.2
    ])

    q1 = q0
    qret = q0
    norm_e = 10
    J = f.jacobian(q)
    J_fun = sp.lambdify((q,), J, 'numpy')
    f_fun = sp.lambdify((q,), f, 'numpy')
    [n, m] = J.shape

    J_num = J_fun(qret)
    f_num = f_fun(qret)
    xret = f_num.reshape(n,)
    e = xret - xd
    k = 1
    e_diff = -k * e
    J_inv = np.linalg.pinv(J_num)
    q_diff = J_inv @ e_diff
    t_diff = 0.5
    q1 = q1 + t_diff * q_diff
    norm_e = np.linalg.norm(e)
    return q1, norm_e


class PositionInteractivePublisher(Node):
    def __init__(self):
        super().__init__('position_interactive_publisher')

        # --- Publicador ---
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)

        # --- Suscriptor ---
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # --- Variables internas ---
        self.q0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1])  # estado inicial en radianes
        self.qe = None  # feedback recibido en grados
        self.gripper_angle = GRIPPER_OPEN

    def joint_state_callback(self, msg):
        """Callback para recibir los JointStates (en radianes) y convertirlos a grados."""
        self.qe = np.degrees(np.array(msg.position))  # convertir a sexagesimal
        self.get_logger().info(f"Feedback recibido (grados): {self.qe}")

    def publish_servo_command(self, joint_angles_deg, gripper_angle):
        """Publica el comando combinado para articulaciones y garra."""
        joints_with_offsets = [
            int(round(angle + offset))
            for angle, offset in zip(joint_angles_deg, SERVO_HOME_OFFSETS[:-1])
        ]
        clipped_gripper = np.clip(gripper_angle, 0.0, 180.0)
        gripper_command = int(round(clipped_gripper + SERVO_HOME_OFFSETS[-1]))
        command = joints_with_offsets + [gripper_command]

        msg = Int32MultiArray()
        msg.data = command
        self.publisher_.publish(msg)
        self.gripper_angle = clipped_gripper

    def send_position(self, xd, gripper_angle, tolerance=0.1, max_iterations=120):
        """Resuelve cinemática inversa y mueve el robot hacia xd aplicando la garra."""
        if not isinstance(xd, np.ndarray):
            xd = np.array(xd, dtype=float)

        e = np.inf
        iterations = 0
        target_gripper = float(gripper_angle)

        while e > tolerance and iterations < max_iterations and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.q0, e = control_cinematico(self.q0, xd)
            q_deg = np.degrees(self.q0)

            self.publish_servo_command(q_deg, target_gripper)
            self.get_logger().info(
                f'Posición deseada: {xd} | e={e:.3f} | Garra objetivo: {target_gripper:.1f}'
            )

            time.sleep(0.1)
            iterations += 1

        if e <= tolerance:
            self.get_logger().info('✅ Posición alcanzada. No se envían más comandos.')
        else:
            self.get_logger().warning(
                f'⚠️ No se alcanzó la posición (e={e:.3f}) dentro de {max_iterations} iteraciones.'
            )

        return e

    def execute_trajectory(self, trajectory, steps_per_segment=8, tolerance=0.15):
        """Ejecuta una trayectoria con interpolación suave entre puntos."""
        if not trajectory:
            self.get_logger().warning('⚠️ La trayectoria está vacía, no hay nada que ejecutar.')
            return

        waypoints = [
            (np.array(pose, dtype=float), float(grip))
            for pose, grip in trajectory
        ]

        # Mover al primer punto de la trayectoria.
        start_pose, start_grip = waypoints[0]
        self.get_logger().info(
            f'Iniciando trayectoria en {start_pose.tolist()} con garra {start_grip:.1f}°.'
        )
        self.send_position(start_pose, start_grip, tolerance=tolerance)

        for idx in range(len(waypoints) - 1):
            pose_a, grip_a = waypoints[idx]
            pose_b, grip_b = waypoints[idx + 1]

            for step in range(1, steps_per_segment + 1):
                alpha = step / steps_per_segment
                interp_pose = pose_a + alpha * (pose_b - pose_a)
                interp_grip = grip_a + alpha * (grip_b - grip_a)

                final_target = idx == len(waypoints) - 2 and step == steps_per_segment
                local_tolerance = tolerance if not final_target else 0.1
                max_iters = 60 if not final_target else 160

                self.send_position(
                    interp_pose,
                    interp_grip,
                    tolerance=local_tolerance,
                    max_iterations=max_iters
                )

        self.get_logger().info('✅ Trayectoria completada.')


PREDEFINED_TRAJECTORY = [
    ([0.0, 17.0, 6.0], GRIPPER_OPEN),
    ([0.0, 17.0, 20.0], GRIPPER_OPEN),
    ([20.0, 16.0, 20.0], GRIPPER_OPEN),
    ([20.0, 16.0, 13.0], GRIPPER_CLOSED),  # cerrar garra
    ([20.0, 16.0, 20.0], GRIPPER_CLOSED),
    ([0.0, 16.0, 20.0], GRIPPER_CLOSED),
    ([-20.0, 16.0, 20.0], GRIPPER_CLOSED),
    ([-20.0, 16.0, 13.0], GRIPPER_OPEN),  # abrir garra
    ([-20.0, 16.0, 20.0], GRIPPER_OPEN),
    ([0.0, 16.0, 20.0], GRIPPER_OPEN),
    ([0.0, 17.0, 6.0], GRIPPER_OPEN)
]


def main(args=None):
    rclpy.init(args=args)
    node = PositionInteractivePublisher()

    try:
        node.execute_trajectory(PREDEFINED_TRAJECTORY)

        while rclpy.ok():
            entrada = input(
                "Ingrese posición deseada xd=[x y z] (+garra opcional) o 'traj' para repetir trayectoria: "
            )

            if entrada.strip().lower() == 'traj':
                node.execute_trajectory(PREDEFINED_TRAJECTORY)
                continue

            try:
                valores = [float(x) for x in entrada.strip().split()]
                if len(valores) not in (3, 4):
                    print("⚠️ Debe ingresar 3 valores (x y z) o 4 valores (x y z garra).")
                    continue

                gripper_angle = node.gripper_angle if len(valores) == 3 else valores[3]
                xd = np.array(valores[:3])
                node.send_position(xd, gripper_angle)
            except ValueError:
                print("⚠️ Entrada inválida. Use solo números válidos.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


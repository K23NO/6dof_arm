#!/usr/bin/env python3

import time
import numpy as np
import sympy as sp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


# =====================
# CONFIG SERVOS
# =====================
SERVO_HOME_OFFSETS = np.array([90, 90, 90, 0, 90, 0], dtype=float)
SERVO_MIN_DEG = np.zeros(6)
SERVO_MAX_DEG = np.full(6, 180.0)

GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 40.0


# =====================
# TRAYECTORIA EXACTA
# =====================
TRAJECTORY = [
    {'pos': np.array([0.0, 17.0, 6.0]),   'gripper': GRIPPER_OPEN,  'hold': 0.5},
    {'pos': np.array([0.0, 17.0, 20.0]),  'gripper': GRIPPER_OPEN,  'hold': 0.0},
    {'pos': np.array([20.0,16.0,20.0]),   'gripper': GRIPPER_OPEN,  'hold': 0.0},
    {'pos': np.array([20.0,16.0,13.0]),   'gripper': GRIPPER_OPEN,  'hold': 0.0},
    {'pos': np.array([20.0,16.0,13.0]),   'gripper': GRIPPER_CLOSE, 'hold': 1.0},
    {'pos': np.array([20.0,16.0,20.0]),   'gripper': GRIPPER_CLOSE, 'hold': 0.0},
    {'pos': np.array([0.0,16.0,20.0]),    'gripper': GRIPPER_CLOSE, 'hold': 0.0},
    {'pos': np.array([-20.0,16.0,20.0]),  'gripper': GRIPPER_CLOSE, 'hold': 0.0},
    {'pos': np.array([-20.0,16.0,13.0]),  'gripper': GRIPPER_CLOSE, 'hold': 0.0},
    {'pos': np.array([-20.0,16.0,13.0]),  'gripper': GRIPPER_OPEN,  'hold': 1.0},
    {'pos': np.array([-20.0,16.0,20.0]),  'gripper': GRIPPER_OPEN,  'hold': 0.0},
    {'pos': np.array([0.0,16.0,20.0]),    'gripper': GRIPPER_OPEN,  'hold': 0.0},
    {'pos': np.array([0.0,17.0,6.0]),     'gripper': GRIPPER_OPEN,  'hold': 0.5},
]


# =====================
# IK DEL MODELO (SUAVE)
# =====================
def _prepare_ik():
    q1, q2, q3, q4, q5 = sp.symbols("q1 q2 q3 q4 q5")
    qv = sp.Matrix([q1, q2, q3, q4, q5])

    f = sp.Matrix([
        (10.5*sp.sin(q2) - 13*sp.sin(q2-q3+q4) - 14.8*sp.cos(q2-q3) - 2.5) * sp.sin(q1),
        (-10.5*sp.sin(q2) + 13*sp.sin(q2-q3+q4) + 14.8*sp.cos(q2-q3) + 2.5) * sp.cos(q1),
        14.8*sp.sin(q2-q3) + 10.5*sp.cos(q2) - 13*sp.cos(q2-q3+q4) + 8.2
    ])

    J = f.jacobian(qv)

    return (
        sp.lambdify((qv,), f, "numpy"),
        sp.lambdify((qv,), J, "numpy"),
    )


FK_FUN, JAC_FUN = _prepare_ik()


def ik_step(q, xd):
    """Un solo paso pequeño de IK, suave y seguro"""
    x = np.array(FK_FUN(q)).reshape(3,)
    e = xd - x
    norm_e = np.linalg.norm(e)

    J = np.array(JAC_FUN(q)).astype(float)
    J_inv = np.linalg.pinv(J)  # muy suave

    dq = 0.15 * (J_inv @ (-e))
    dq = np.clip(dq, -np.radians(3), np.radians(3))

    return q + dq, norm_e


# =====================
# CONTROLADOR
# =====================
class SoftTrajectory(Node):
    def __init__(self):
        super().__init__("soft_trajectory")

        self.pub = self.create_publisher(Int32MultiArray, "servo_commands", 10)
        self.timer = self.create_timer(0.18, self.loop)

        self.q = np.radians([0, 17, 6, 0, 0])
        self.target_idx = 0
        self.state = "moving"
        self.hold_until = None

        self.current_gripper = SERVO_HOME_OFFSETS[5]

        self.get_logger().info("Control suave iniciado.")

    def loop(self):
        target = TRAJECTORY[self.target_idx]
        xd = target["pos"]
        desired_gripper = target["gripper"]

        now = time.time()

        # =====================
        # FASE 1: MOVING
        # =====================
        if self.state == "moving":
            q_new, err = ik_step(self.q, xd)

            # suavidad extrema
            if np.linalg.norm(q_new - self.q) > np.radians(6):
                q_new = self.q + np.sign(q_new - self.q) * np.radians(6)

            self.q = q_new

            # convertir a grados físico
            q_deg = np.degrees(self.q)
            cmd = (q_deg + SERVO_HOME_OFFSETS[:5]).tolist()
            cmd = np.clip(cmd, SERVO_MIN_DEG[:5], SERVO_MAX_DEG[:5]).astype(int)

            # garra NO se mueve aún
            cmd = list(cmd) + [int(self.current_gripper)]

            msg = Int32MultiArray()
            msg.data = cmd
            self.pub.publish(msg)

            self.get_logger().info(
                f"[MOV] idx={self.target_idx} err={err:.2f} cmd={cmd}"
            )

            # Llegué?
            if err < 2.0:
                self.state = "hold"
                self.hold_until = now + target["hold"]

                # activar garra SOLO aquí
                self.current_gripper = SERVO_HOME_OFFSETS[5] + desired_gripper

        # =====================
        # FASE 2: HOLD
        # =====================
        elif self.state == "hold":
            q_deg = np.degrees(self.q)
            cmd = (q_deg + SERVO_HOME_OFFSETS[:5]).tolist()
            cmd = np.clip(cmd, SERVO_MIN_DEG[:5], SERVO_MAX_DEG[:5]).astype(int)
            cmd = list(cmd) + [int(self.current_gripper)]

            msg = Int32MultiArray()
            msg.data = cmd
            self.pub.publish(msg)

            if now >= self.hold_until:
                self.state = "moving"
                self.target_idx = (self.target_idx + 1) % len(TRAJECTORY)

                self.get_logger().info(
                    f"→ Siguiente waypoint {self.target_idx}"
                )


# =====================
# MAIN
# =====================
def main():
    rclpy.init()
    node = SoftTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import sympy as sp


SERVO_HOME_OFFSETS = np.array([90, 90, 90, 0, 90, 30], dtype=float)
SERVO_MIN_DEG = np.zeros(6, dtype=float)
SERVO_MAX_DEG = np.full(6, 180.0, dtype=float)

GRIPPER_OPEN_REL = 0.0
GRIPPER_CLOSED_REL = 40.0

HOME_Q_RAD = np.radians([0.0, 17.0, 6.0, 0.0, 0.0])

TRAJECTORY = [
    {'pos': np.array([0.0, 17.0, 6.0], dtype=float),   'gripper': GRIPPER_OPEN_REL,    'hold': 0.6},
    {'pos': np.array([0.0, 17.0, 20.0], dtype=float),  'gripper': GRIPPER_OPEN_REL,    'hold': 0.0},
    {'pos': np.array([20.0, 16.0, 20.0], dtype=float), 'gripper': GRIPPER_OPEN_REL,    'hold': 0.0},
    {'pos': np.array([20.0, 16.0, 13.0], dtype=float), 'gripper': GRIPPER_OPEN_REL,    'hold': 0.3},
    {'pos': np.array([20.0, 16.0, 13.0], dtype=float), 'gripper': GRIPPER_CLOSED_REL,  'hold': 0.8},
    {'pos': np.array([20.0, 16.0, 20.0], dtype=float), 'gripper': GRIPPER_CLOSED_REL,  'hold': 0.0},
    {'pos': np.array([0.0, 16.0, 20.0], dtype=float),  'gripper': GRIPPER_CLOSED_REL,  'hold': 0.0},
    {'pos': np.array([-20.0, 16.0, 20.0], dtype=float), 'gripper': GRIPPER_CLOSED_REL, 'hold': 0.0},
    {'pos': np.array([-20.0, 16.0, 13.0], dtype=float), 'gripper': GRIPPER_CLOSED_REL, 'hold': 0.3},
    {'pos': np.array([-20.0, 16.0, 13.0], dtype=float), 'gripper': GRIPPER_OPEN_REL,   'hold': 0.8},
    {'pos': np.array([-20.0, 16.0, 20.0], dtype=float), 'gripper': GRIPPER_OPEN_REL,   'hold': 0.0},
    {'pos': np.array([0.0, 16.0, 20.0], dtype=float),  'gripper': GRIPPER_OPEN_REL,    'hold': 0.0},
    {'pos': np.array([0.0, 17.0, 6.0], dtype=float),   'gripper': GRIPPER_OPEN_REL,    'hold': 0.6},
]

POSITION_TOLERANCE = 2.0  # cm
CONTROL_STEP = 0.35
TIMER_PERIOD = 0.2
KP_CARTESIAN = np.diag([0.85, 0.85, 0.65])
DAMPING_LAMBDA = 0.12
MAX_Q_DELTA_RAD = np.radians([6.0, 6.0, 6.0, 7.0, 7.0])
MAX_INTERNAL_ITERS = 5


def damped_pseudo_inverse(jac: np.ndarray, damping: float) -> np.ndarray:
    jj_t = jac @ jac.T
    dim = jj_t.shape[0]
    regularized = jj_t + (damping ** 2) * np.eye(dim)
    return jac.T @ np.linalg.inv(regularized)


def control_cinematico(q0: np.ndarray, xd: np.ndarray):
    if not hasattr(control_cinematico, '_prepared'):
        q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
        q_vec = sp.Matrix([q1, q2, q3, q4, q5])
        f = sp.Matrix([
            (10.5 * sp.sin(q2) - 13 * sp.sin(q2 - q3 + q4) - 14.8 * sp.cos(q2 - q3) - 2.5) * sp.sin(q1),
            (-10.5 * sp.sin(q2) + 13 * sp.sin(q2 - q3 + q4) + 14.8 * sp.cos(q2 - q3) + 2.5) * sp.cos(q1),
            14.8 * sp.sin(q2 - q3) + 10.5 * sp.cos(q2) - 13 * sp.cos(q2 - q3 + q4) + 8.2,
        ])
        J = f.jacobian(q_vec)
        control_cinematico._fk_fun = sp.lambdify((q_vec,), f, 'numpy')
        control_cinematico._jac_fun = sp.lambdify((q_vec,), J, 'numpy')
        control_cinematico._prepared = True

    fk_fun = control_cinematico._fk_fun
    jac_fun = control_cinematico._jac_fun

    q_current = np.array(q0, dtype=float).reshape(5,)
    x_actual = np.array(fk_fun(q_current)).reshape(3,).astype(float)
    error_vec = xd - x_actual
    norm_error = float(np.linalg.norm(error_vec))

    for _ in range(MAX_INTERNAL_ITERS):
        if norm_error <= POSITION_TOLERANCE:
            break

        jac = np.array(jac_fun(q_current)).astype(float)
        pseudo_inv = damped_pseudo_inverse(jac, DAMPING_LAMBDA)

        cartesian_drive = KP_CARTESIAN @ error_vec
        q_delta = CONTROL_STEP * (pseudo_inv @ cartesian_drive)
        q_delta = np.clip(q_delta, -MAX_Q_DELTA_RAD, MAX_Q_DELTA_RAD)

        q_current = q_current + q_delta
        x_actual = np.array(fk_fun(q_current)).reshape(3,).astype(float)
        error_vec = xd - x_actual
        norm_error = float(np.linalg.norm(error_vec))

    return q_current, norm_error, x_actual


class KinematicTrajectoryController(Node):
    def __init__(self):
        super().__init__('kinematic_trajectory_controller')

        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
        )

        self.q_feedback = HOME_Q_RAD.copy()
        self.q_working = HOME_Q_RAD.copy()
        self.feedback_ready = False
        self.x_actual = np.zeros(3, dtype=float)

        self.target_idx = 0
        self.hold_deadline = None

        self.get_logger().info('Control cinemático iniciado; esperando feedback para comenzar.')
        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < 5:
            return

        self.q_feedback = np.array(msg.position[:5], dtype=float)
        self.feedback_ready = True

    def current_target(self):
        return TRAJECTORY[self.target_idx]

    def control_loop(self):
        if not self.feedback_ready:
            return

        target = self.current_target()
        xd = target['pos']
        gripper_rel = target['gripper']

        self.q_working = self.q_feedback.copy()
        q_candidate, error_norm, x_actual = control_cinematico(self.q_working, xd)

        q_deg = np.degrees(q_candidate)
        absolute = q_deg + SERVO_HOME_OFFSETS[:5]
        absolute = np.clip(absolute, SERVO_MIN_DEG[:5], SERVO_MAX_DEG[:5])
        self.q_working = np.radians(absolute - SERVO_HOME_OFFSETS[:5])
        self.q_feedback = self.q_working.copy()

        gripper_abs = np.clip(gripper_rel + SERVO_HOME_OFFSETS[5], SERVO_MIN_DEG[5], SERVO_MAX_DEG[5])

        cmd = [int(round(val)) for val in absolute]
        cmd.append(int(round(gripper_abs)))

        msg = Int32MultiArray()
        msg.data = cmd
        self.publisher_.publish(msg)

        self.x_actual = x_actual
        self.get_logger().info(
            f'idx={self.target_idx} | xd={xd} | x_act={self.x_actual} | e={error_norm:.3f} | cmd={cmd}'
        )

        now = self.get_clock().now().nanoseconds / 1e9

        if error_norm <= POSITION_TOLERANCE:
            if self.hold_deadline is None:
                self.hold_deadline = now + target['hold']
            elif now >= self.hold_deadline:
                self.target_idx = (self.target_idx + 1) % len(TRAJECTORY)
                self.hold_deadline = None


def main(args=None):
    rclpy.init(args=args)
    node = KinematicTrajectoryController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


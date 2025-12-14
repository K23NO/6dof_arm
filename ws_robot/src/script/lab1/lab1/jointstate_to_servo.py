#!/usr/bin/env python3

import math
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 0]
SERVO_MIN = 0
SERVO_MAX = 180
SERVO_COUNT = len(SERVO_HOME_OFFSETS)


class JointStateToServo(Node):
    def __init__(self) -> None:
        super().__init__('jointstate_to_servo')

        self.declare_parameter('apply_offsets', True)
        self.apply_offsets = self.get_parameter('apply_offsets').get_parameter_value().bool_value

        self.declare_parameter('joint_names', SERVO_COUNT * [''])
        configured_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        if configured_names:
            self.joint_name_order = list(configured_names)
        else:
            self.joint_name_order = [
                'joint_1',
                'joint_2',
                'joint_3',
                'joint_4',
                'joint_5',
                'joint_6',
            ]

        self.name_to_index: Dict[str, int] = {
            name: idx for idx, name in enumerate(self.joint_name_order[:SERVO_COUNT])
        }

        self.last_commands = [offset if self.apply_offsets else 0 for offset in SERVO_HOME_OFFSETS]

        self.publisher = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
        )

        mode = 'RELATIVO (con offsets)' if self.apply_offsets else 'ABSOLUTO (sin offsets)'
        self.get_logger().info(
            f'jointstate_to_servo activo. Esperando `joint_states` con orden: {self.joint_name_order}. Modo: {mode}'
        )

    def joint_state_callback(self, msg: JointState) -> None:
        # Build temporary list to ensure we always publish SERVO_COUNT values
        commands = list(self.last_commands)

        for name, position in zip(msg.name, msg.position):
            idx = self.name_to_index.get(name)
            if idx is None or idx >= SERVO_COUNT:
                continue

            degrees = math.degrees(position)
            value = degrees
            if self.apply_offsets:
                value += SERVO_HOME_OFFSETS[idx]

            value = int(round(value))
            value = max(SERVO_MIN, min(SERVO_MAX, value))
            commands[idx] = value

        self.last_commands = commands

        outgoing = Int32MultiArray()
        outgoing.data = commands
        self.publisher.publish(outgoing)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

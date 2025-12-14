import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math

SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 30]


class ServoToJointState(Node):
    def __init__(self):
        super().__init__('servo_to_jointstate')

        feedback_topic = self.declare_parameter('feedback_topic', 'servo_feedback')
        command_topic = self.declare_parameter('command_topic', 'servo_commands')

        feedback_topic = feedback_topic.get_parameter_value().string_value or 'servo_feedback'
        command_topic = command_topic.get_parameter_value().string_value or 'servo_commands'

        self.get_logger().info(
            f'Feedback topic: "{feedback_topic}" | Command fallback: "{command_topic}"'
        )

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        self.last_source = 'initial'

        self.subscriptions = []

        if feedback_topic:
            self.subscriptions.append(
                self.create_subscription(
                    Int32MultiArray,
                    feedback_topic,
                    lambda msg: self.cb(msg, 'feedback'),
                    10,
                )
            )

        if command_topic and command_topic != feedback_topic:
            self.subscriptions.append(
                self.create_subscription(
                    Int32MultiArray,
                    command_topic,
                    lambda msg: self.cb(msg, 'command'),
                    10,
                )
            )

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.last_vals = SERVO_HOME_OFFSETS.copy()

        self.timer = self.create_timer(0.05, self.publish_joint_state)  # 20 Hz

    def cb(self, msg: Int32MultiArray, source: str):
        clamped = [max(0, min(180, int(a))) for a in msg.data[:6]]

        while len(clamped) < 6:
            clamped.append(0)

        self.last_vals = clamped
        if self.last_source != source:
            self.get_logger().info(f'Actualizando joint_states desde {source}')
            self.last_source = source

    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [
            (val - offset) * math.pi / 180.0
            for val, offset in zip(self.last_vals, SERVO_HOME_OFFSETS)
        ]
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    n = ServoToJointState()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


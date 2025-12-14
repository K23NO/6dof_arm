#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math

class ServoToJointBridge(Node):
    def __init__(self):
        super().__init__('servo_to_joint_bridge')
        
        # Suscriptor a los comandos de servo
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_commands',
            self.servo_callback,
            10)
        
        # Publisher para JointState (que usa RViz)
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        # Timer para publicar continuamente (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # NOMBRES CORRECTOS de las articulaciones según tu URDF
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        
        # Posición home (en radianes) - TU POSICIÓN
        self.current_position = [math.radians(90), math.radians(0), math.radians(90), 
                                math.radians(0), math.radians(110), math.radians(0)]
        
        self.get_logger().info('Servo to Joint Bridge iniciado')
        self.get_logger().info(f'Usando nombres de joints: {self.joint_names}')

    def servo_callback(self, msg):
        if len(msg.data) >= 5:
            # Actualizar la posición actual
            self.current_position = [math.radians(angle) for angle in msg.data[:5]]
            self.get_logger().info(f'Ángulos recibidos: {msg.data[:5]}')
            
            # Publicar inmediatamente al recibir nuevos ángulos
            self.publish_joint_state()

    def timer_callback(self):
        # Publicar continuamente para mantener la posición
        self.publish_joint_state()

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = self.joint_names
        joint_state.position = self.current_position
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = ServoToJointBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose
import math

class InverseKinematicsController(Node):
    def __init__(self):
        super().__init__('ik_controller')
        
        # Suscriptores
        self.feedback_sub = self.create_subscription(
            Int32MultiArray, 'potentiometer_feedback', self.feedback_callback, 10)
        self.target_sub = self.create_subscription(
            Pose, '/target_pose', self.target_callback, 10)
            
        # Publicador para comandos de servo
        self.servo_pub = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        
        # Estado actual desde potenciómetros
        self.current_angles = [0.0] * 6
        self.target_angles = [0.0] * 6
        self.has_feedback = False
        
        # Control PID (ajustar según necesidad)
        self.kp = [2.0, 2.0, 2.0, 1.5, 1.5, 1.5]
        self.ki = [0.01, 0.01, 0.01, 0.005, 0.005, 0.005]
        self.kd = [0.1, 0.1, 0.1, 0.05, 0.05, 0.05]
        
        self.integral_error = [0.0] * 6
        self.previous_error = [0.0] * 6
        
        # Offsets para convertir a comandos absolutos
        self.servo_offsets = [90, 90, 90, 0, 90, 0]
        
        self.get_logger().info("Controlador IK con feedback iniciado")
        self.get_logger().info("Esperando feedback de potenciómetros...")

    def feedback_callback(self, msg):
        # Convertir feedback de potenciómetros a ángulos
        self.current_angles = [value / 100.0 for value in msg.data]
        if not self.has_feedback:
            self.get_logger().info("✓ Feedback de potenciómetros recibido")
            self.has_feedback = True

    def target_callback(self, msg):
        if not self.has_feedback:
            self.get_logger().warn("⚠️  Esperando feedback de potenciómetros...")
            return
            
        # Cinemática inversa
        self.target_angles = self.inverse_kinematics(msg)
        self.control_loop()

    def inverse_kinematics(self, target_pose):
        """
        IMPLEMENTA TU CINEMÁTICA INVERSA AQUÍ
        Por ahora: ejemplo simple
        """
        x = target_pose.position.x
        y = target_pose.position.y  
        z = target_pose.position.z
        
        # EJEMPLO - REEMPLAZA CON TU ALGORITMO
        angle1 = math.atan2(y, x) * 180.0 / math.pi if abs(x) > 0.01 or abs(y) > 0.01 else 0.0
        angle2 = 45.0
        angle3 = -30.0
        angle4 = 15.0
        angle5 = 0.0
        angle6 = 0.0
        
        return [angle1, angle2, angle3, angle4, angle5, angle6]

    def control_loop(self):
        """Lazo de control PID con feedback"""
        msg = Int32MultiArray()
        
        commands = []
        for i in range(6):
            error = self.target_angles[i] - self.current_angles[i]
            
            # Términos PID
            p_term = self.kp[i] * error
            
            self.integral_error[i] += error
            self.integral_error[i] = max(min(self.integral_error[i], 50), -50)
            i_term = self.ki[i] * self.integral_error[i]
            
            d_term = self.kd[i] * (error - self.previous_error[i])
            self.previous_error[i] = error
            
            # Señal de control
            control_signal = self.current_angles[i] + p_term + i_term + d_term
            
            # Convertir a comando absoluto de servo
            servo_command = control_signal + self.servo_offsets[i]
            servo_command = max(0, min(180, int(servo_command)))
            
            commands.append(servo_command)
        
        msg.data = commands
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Comandos enviados: {commands}')

def main():
    rclpy.init()
    node = InverseKinematicsController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

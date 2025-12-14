#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
import sympy as sp
import time

# Offsets: la posición física de home de cada servo
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 0]

# Se asume que 'control_cinematico' ya está definido en otro archivo o módulo
# from tu_modulo import control_cinematico
def control_cinematico(q0,xd):
	[q1,q2,q3,q4,q5]=sp.symbols("q1 q2 q3 q4 q5")
	q=sp.Matrix([q1,q2,q3,q4,q5])
	f=sp.Matrix([(10.5*sp.sin(q2)-13*sp.sin(q2-q3+q4)-14.8*sp.cos(q2-q3)-2.5)*sp.sin(q1),
             (-10.5*sp.sin(q2)+13*sp.sin(q2-q3+q4)+14.8*sp.cos(q2-q3)+2.5)*sp.cos(q1),
             14.8*sp.sin(q2-q3)+10.5*sp.cos(q2)-13*sp.cos(q2-q3+q4)+8.2])
	q1=q0
	qret=q0
	norm_e=10
	J=f.jacobian(q)
	J_fun=sp.lambdify((q,),J,'numpy')
	f_fun=sp.lambdify((q,),f,'numpy')
	[n,m]=J.shape

	J_num=J_fun(qret)
	f_num=f_fun(qret)
	xret=f_num.reshape(n,)
	e=xret-xd
	k=1
	e_diff=-k*e
	J_inv=np.linalg.pinv(J_num)
	q_diff=J_inv@e_diff
	t_diff=1
	q1=q1+t_diff*q_diff
	norm_e=np.linalg.norm(e)
	return q1,norm_e


class PositionInteractivePublisher(Node):
    def __init__(self):
        super().__init__('position_interactive_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        # Estado inicial de las articulaciones (en radianes)
        self.q0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1])

    def send_position(self, xd):
        e = 1.0  # valor inicial para entrar al bucle
        while e > 0.1 and rclpy.ok():
            # Calcular nueva configuración articular y error
            self.q0, e = control_cinematico(self.q0, xd)

            # Convertir de radianes a grados
            q_deg = np.degrees(self.q0)

            # Aplicar offsets
            q_absolute = [int(round(angle + offset)) for angle, offset in zip(q_deg, SERVO_HOME_OFFSETS)]

            # Publicar
            msg = Int32MultiArray()
            msg.data = q_absolute
            self.publisher_.publish(msg)

            self.get_logger().info(f'Posición deseada: {xd} | e={e:.3f} | Ángulos enviados: {q_absolute}')

            # Esperar 1 segundo antes de la siguiente iteración
            time.sleep(1.0)

        if e <= 0.1:
            self.get_logger().info("✅ Posición alcanzada. No se envían más comandos.")


def main(args=None):
    rclpy.init(args=args)
    node = PositionInteractivePublisher()

    try:
        while rclpy.ok():
            entrada = input("Ingrese posición deseada xd=[x y z]: ")
            try:
                valores = [float(x) for x in entrada.strip().split()]
                if len(valores) != 3:
                    print("⚠️ Debe ingresar exactamente 3 valores para xd.")
                    continue

                xd = np.array(valores)
                node.send_position(xd)
            except ValueError:
                print("⚠️ Entrada inválida. Use solo números válidos.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


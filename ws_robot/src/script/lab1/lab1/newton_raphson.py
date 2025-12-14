import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
import sympy as sp

# Offsets: la posición física de home de cada servo
SERVO_HOME_OFFSETS = [90, 90, 90, 0, 90, 0]

# Se asume que 'q', 'f', 'x0' ya están definidos en tu código principal
# from tu_modulo import q, f, x0, newton_raphson

def newton_raphson(f,q,x0,xd,iter):

  J=f.jacobian(q)
  J_fun=sp.lambdify((q,),J,'numpy')
  f_fun=sp.lambdify((q,),f,'numpy')
  [n,m]=J.shape
  q0=x0

  errores=[]
  for i in range(iter):
    J_num=J_fun(q0)
    f_num=f_fun(q0)
    e=xd.reshape(n,1)-f_num.reshape(n,1)
    q1=q0.reshape(m,1)+np.linalg.pinv(J_num)@e
    q0=q1.reshape(m,)

    errores.append(np.linalg.norm(e))

    if (np.linalg.norm(e)<0.001):
      err=np.array(errores)
      return q0, err


  err=np.array(errores)

  return np.zeros(m,),err



class PositionInteractivePublisher(Node):
    def __init__(self):
        super().__init__('position_interactive_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)

    def send_position(self, xd):
        # xd es un np.array([x, y, z])
        [q1,q2,q3,q4,q5]=sp.symbols("q1 q2 q3 q4 q5")
        q=sp.Matrix([q1,q2,q3,q4,q5])

        f=sp.Matrix([(10.5*sp.sin(q2)-7.5*sp.sin(q2-q3+q4)-14.8*sp.cos(q2-q3)-2.5)*sp.sin(q1),
             (-10.5*sp.sin(q2)+7.5*sp.sin(q2-q3+q4)+14.8*sp.cos(q2-q3)+2.5)*sp.cos(q1),
             14.8*sp.sin(q2-q3)+10.5*sp.cos(q2)-7.5*sp.cos(q2-q3+q4)+8.2])

        x0=np.array([0.1,0.1,0.1,0.1,0.1])
        
        q_sol, errores = newton_raphson(f, q, x0, xd, 10)  # Ajusta iteraciones si quieres

        # Convertir de radianes a grados
        q_deg = np.degrees(q_sol)

        # Aplicar offsets
        q_absolute = [int(round(angle + offset)) for angle, offset in zip(q_deg, SERVO_HOME_OFFSETS)]

        # Publicar
        msg = Int32MultiArray()
        msg.data = q_absolute
        self.publisher_.publish(msg)

        self.get_logger().info(f'Posición deseada: {xd} | Ángulos enviados: {q_absolute}')

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


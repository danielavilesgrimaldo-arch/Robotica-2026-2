#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import time
import matplotlib.pyplot as plt

# Importamos nuestras librerías del paquete
from final_project.scripts.kinematics import RobotKinematics
from final_project.scripts.dynamics import RobotDynamics

class ManipulatorController(Node):
    def __init__(self):
        super().__init__("manipulator_controller")
        
        # 1. Inicializar Cinemática
        self.kin = RobotKinematics()
        self.kin.redirect_print(self.get_logger().info)
        self.kin.direct_kinematics() # Configura símbolos
        
        # 2. Inicializar Dinámica
        self.dyn = RobotDynamics()
        self.dyn.define_kinematics(self.kin)
        self.dyn.define_dynamics()
        
        # 3. Comunicaciones ROS
        self.pub = self.create_publisher(JointState, '/joint_goals', 10)
        
        self.create_subscription(PointStamped, '/clicked_point', self.click_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        
        # Estado inicial
        self.current_q = [0.0, 0.0, 0.0]
        self.get_logger().info("Controlador RRR Listo. Haz click en 'Publish Point'.")

    def state_cb(self, msg):
        """Actualiza la posición actual del robot"""
        # El RRR usa las primeras 3 juntas que lleguen
        if len(msg.position) >= 3: 
            self.current_q = list(msg.position[:3])

    def click_cb(self, msg):
        """Callback al hacer click en Rviz"""
        # Para el RRR Vertical:
        # X de Rviz -> Alcance horizontal del robot
        # Z de Rviz -> Altura del robot
        # Y de Rviz -> Ignorado (o podrías usarlo para rotar la base si fuera 3D real)
        
        target = [msg.point.x, msg.point.z, 0.0] # [x, z, theta_global]
        self.get_logger().info(f"RRR yendo a X={target[0]:.2f}, Z={target[1]:.2f}")
        
        # 1. Calcular Trayectoria y Cinemática Inversa
        self.kin.trajectory_generator(self.current_q, target, duration=4.0)
        self.kin.inverse_kinematics()
        
        # 2. Calcular Dinámica (Torques)
        self.dyn.lagrange_effort_generator()
        
        # 3. Mostrar Gráficas (Requerimiento)
        self.get_logger().info("Mostrando gráficas. CIERRA LA VENTANA para mover el robot.")
        # Podemos llamar a las gráficas individuales o hacer una combinada aquí.
        # Usaremos las de las librerías para mantener el orden.
        self.kin.ws_graph()
        self.kin.q_graph()
        self.dyn.effort_graph()
        
        # 4. Ejecutar Movimiento
        self.execute_trajectory()

    def execute_trajectory(self):
        """Publica los puntos calculados paso a paso"""
        self.get_logger().info("Ejecutando movimiento...")
        dt = self.kin.dt
        samples = self.kin.samples
        
        for i in range(samples):
            msg = JointState()
            # Nombres exactos del URDF RRR
            msg.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
            msg.position = [float(x) for x in self.kin.q_m[:, i]]
            
            self.pub.publish(msg)
            time.sleep(dt) # Respetar el tiempo real
            
        self.get_logger().info("Movimiento finalizado.")

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np
import matplotlib.pyplot as plt
import time

class ScaraController(Node):
    def __init__(self):
        super().__init__("scara_controller")
        # Longitudes del SCARA (según URDF)
        self.L1 = 0.3
        self.L2 = 0.3
        
        # Nombres de las juntas del SCARA
        self.joints = ["shoulder_joint", "forearm_joint", "hand_joint"]
        
        # Publicadores y Suscriptores
        self.pub = self.create_publisher(JointState, '/joint_goals', 10)
        self.create_subscription(PointStamped, '/clicked_point', self.click_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        
        self.current_q = [0.0, 0.0, 0.0]
        self.get_logger().info("Controlador SCARA Listo. Haz click en 'Publish Point'.")

    def state_cb(self, msg):
        """Actualiza la posición actual. El SCARA usa 3 juntas."""
        if len(msg.position) >= 3: 
            self.current_q = list(msg.position[:3])

    def click_cb(self, msg):
        """Callback al hacer click"""
        # SCARA: X, Y son alcance plano. Z es altura (prismática).
        target = [msg.point.x, msg.point.y, msg.point.z]
        self.get_logger().info(f"SCARA yendo a {target}")
        
        # 1. Calcular Trayectoria (Cartesiana y Articular) y Dinámica
        t, xi_m, q_m, tau_m = self.calc_trajectory(self.current_q, target)
        
        # 2. Mostrar Gráficas (Requerimiento)
        self.get_logger().info("Mostrando gráficas. CIERRA LA VENTANA para mover el robot.")
        self.plot_graphs(t, xi_m, q_m, tau_m)
        
        # 3. Ejecutar Movimiento
        self.execute(q_m, t)

    def calc_trajectory(self, q_start, target, duration=4.0):
        """Calcula cinemática inversa y dinámica simple para SCARA"""
        steps = 50
        t = np.linspace(0, duration, steps)
        
        q_m = np.zeros((3, steps))
        tau_m = np.zeros((3, steps))
        xi_m = np.zeros((3, steps)) # Trayectoria Cartesiana
        
        # Posición inicial cartesiana (FK para interpolar)
        x0 = self.L1*np.cos(q_start[0]) + self.L2*np.cos(q_start[0]+q_start[1])
        y0 = self.L1*np.sin(q_start[0]) + self.L2*np.sin(q_start[0]+q_start[1])
        z0 = q_start[2]

        for i in range(steps):
            s = i / (steps - 1)
            k = 3*(s**2) - 2*(s**3) # Polinomio suave
            
            # Interpolación Cartesiana
            x = x0 + k*(target[0] - x0)
            y = y0 + k*(target[1] - y0)
            z = z0 + k*(target[2] - z0)
            # Límite físico de la junta prismática (-0.2 a 0.2 según URDF)
            z = np.clip(z, -0.2, 0.2)
            
            xi_m[:, i] = [x, y, z]
            
            # --- Cinemática Inversa SCARA (RRP) ---
            r_sq = x**2 + y**2
            c2 = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            c2 = np.clip(c2, -1.0, 1.0)
            
            q2 = np.arccos(c2) # Codo
            k1 = self.L1 + self.L2 * np.cos(q2)
            k2 = self.L2 * np.sin(q2)
            q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            q3 = z # Eje Z directo
            
            q_m[:, i] = [q1, q2, q3]
            
            # --- Dinámica Simplificada ---
            # J3 (Z) debe sostener el peso (F = m*g)
            # J1, J2 (Horizontales) no cargan peso contra gravedad, solo inercia
            tau_m[:, i] = [0.0, 0.0, 9.81 * 0.5] # 0.5kg masa aprox
            
        return t, xi_m, q_m, tau_m

    def plot_graphs(self, t, xi, q, tau):
        """Muestra las 3 gráficas requeridas"""
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
        
        # Espacio de Trabajo
        ax1.plot(t, xi.T)
        ax1.set_title("Espacio de Trabajo (X,Y,Z)")
        ax1.legend(['x', 'y', 'z'])
        ax1.grid(True)

        # Espacio Articular
        ax2.plot(t, q.T)
        ax2.set_title("Espacio Articular (Juntas)")
        ax2.legend(['q1', 'q2', 'q3'])
        ax2.grid(True)

        # Torques
        ax3.plot(t, tau.T)
        ax3.set_title("Pares / Fuerzas")
        ax3.legend(['t1', 't2', 'F3'])
        ax3.grid(True)
        
        plt.show()

    def execute(self, q_m, t):
        self.get_logger().info("Ejecutando...")
        dt = t[1] - t[0]
        for i in range(q_m.shape[1]):
            msg = JointState()
            msg.name = self.joints
            msg.position = [float(x) for x in q_m[:, i]]
            self.pub.publish(msg)
            time.sleep(dt)

def main(args=None):
    rclpy.init(args=args)
    node = ScaraController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

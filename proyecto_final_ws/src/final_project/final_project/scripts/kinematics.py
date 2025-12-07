#!/usr/bin/env python3
from sympy import *
import numpy as np
import matplotlib.pyplot as plt

class RobotKinematics():
    def __init__(self):
        # Función por defecto para imprimir (se sustituye luego por el logger de ROS)
        self.print_fn = print
        # Longitudes de los eslabones del Robot RRR
        self.l1 = 0.3
        self.l2 = 0.3
        self.l3 = 0.3

    def redirect_print(self, new_print):
        """Permite redirigir los prints al logger de ROS"""
        self.print_fn = new_print

    def direct_kinematics(self):
        """Define las variables simbólicas necesarias"""
        self.theta_0_1, self.theta_1_2, self.theta_2_3 = symbols("t1, t2, t3")
        self.x_dot, self.y_dot, self.th_dot = symbols("x_dot, y_dot, th_dot")

    def trajectory_generator(self, q_in, xi_fn, duration=4):
        """Genera una trayectoria suave punto a punto"""
        self.print_fn(f"Generando trayectoria hacia {xi_fn}")
        
        # Parámetros de tiempo
        self.freq = 30
        self.dt = 1.0 / self.freq
        self.samples = int(self.freq * duration)
        self.t_m = np.linspace(0, duration, self.samples).reshape(1, -1)
        
        # Posición inicial cartesiana (Cinemática Directa RRR Vertical)
        q1, q2, q3 = q_in
        # Ecuaciones FK para RRR Planar Vertical
        x0 = self.l1*np.cos(q1) + self.l2*np.cos(q1+q2) + self.l3*np.cos(q1+q2+q3)
        z0 = self.l1*np.sin(q1) + self.l2*np.sin(q1+q2) + self.l3*np.sin(q1+q2+q3)
        
        # Interpolación polinómica (3er grado) para suavidad
        self.xi_m = np.zeros((3, self.samples))
        for i in range(self.samples):
            s = i / (self.samples - 1) # variable normalizada 0->1
            k = 3*(s**2) - 2*(s**3)    # polinomio
            
            # Interpolación lineal de las coordenadas cartesianas
            self.xi_m[0, i] = x0 + k * (xi_fn[0] - x0) # X
            self.xi_m[1, i] = z0 + k * (xi_fn[1] - z0) # Z
            self.xi_m[2, i] = 0 + k * (xi_fn[2] - 0)   # Theta global (No usado realmente en este ejemplo)

        self.q_in = q_in

    def inverse_kinematics(self):
        """Calcula los ángulos de las juntas para cada punto de la trayectoria"""
        self.q_m = np.zeros((3, self.samples))
        
        for i in range(self.samples):
            x = self.xi_m[0, i]
            z = self.xi_m[1, i]
            phi = self.xi_m[2, i] # Orientación global deseada
            
            # --- Cinemática Inversa Algebraica (RRR Planar) ---
            
            # 1. Posición de la muñeca (wrist)
            wx = x - self.l3 * np.cos(phi)
            wz = z - self.l3 * np.sin(phi)
            
            # 2. Teorema de Cosenos para q2 (Codo)
            r_sq = wx**2 + wz**2
            c2 = (r_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            # Protección numérica
            c2 = np.clip(c2, -1.0, 1.0)
            q2 = np.arccos(c2) # Codo abajo
            
            # 3. Geometría para q1 (Hombro)
            k1 = self.l1 + self.l2 * np.cos(q2)
            k2 = self.l2 * np.sin(q2)
            q1 = np.arctan2(wz, wx) - np.arctan2(k2, k1)
            
            # 4. Orientación para q3
            q3 = phi - q1 - q2
            
            # Guardar resultado
            self.q_m[:, i] = [q1, q2, q3]

    def ws_graph(self):
        """Gráfica del Espacio de Trabajo"""
        plt.figure()
        plt.title("Espacio Trabajo (Trayectoria Cartesiana)")
        plt.plot(self.t_m.T, self.xi_m.T)
        plt.legend(['x', 'z', 'th'])
        plt.grid(True)
        plt.show()

    def q_graph(self):
        """Gráfica del Espacio Articular"""
        plt.figure()
        plt.title("Espacio Articular (Juntas)")
        plt.plot(self.t_m.T, self.q_m.T)
        plt.legend(['q1', 'q2', 'q3'])
        plt.grid(True)
        plt.show()

#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

class RobotDynamics():
    def __init__(self):
        pass

    def define_kinematics(self, kin):
        """Recibe el objeto de cinemática para acceder a la trayectoria generada"""
        self.kin = kin

    def define_dynamics(self):
        """Define parámetros dinámicos básicos (masas y gravedad)"""
        # Masas aproximadas de los eslabones (kg)
        self.m = [1.0, 1.0, 1.0] 
        self.g = 9.81

    def lagrange_effort_generator(self):
        """
        Calcula los torques gravitacionales necesarios para la trayectoria.
        Modelo simplificado: Torque = Fuerza * Distancia_al_centro_de_masa
        """
        samples = self.kin.samples
        self.tau_m = np.zeros((3, samples))
        
        l1 = self.kin.l1
        l2 = self.kin.l2
        l3 = self.kin.l3
        
        for i in range(samples):
            # Obtener ángulos en el instante i
            q1 = self.kin.q_m[0, i]
            q2 = self.kin.q_m[1, i]
            q3 = self.kin.q_m[2, i]
            
            # Cálculo de pares (Simplificación de Lagrange para robot vertical)
            # Tau 3: Sostiene la masa 3
            t3 = self.m[2] * self.g * (l3/2) * np.cos(q1 + q2 + q3)
            
            # Tau 2: Sostiene masa 2 y la carga de masa 3
            t2 = t3 + self.m[1] * self.g * (l2/2) * np.cos(q1 + q2)
            
            # Tau 1: Sostiene masa 1 y la carga de 2 y 3
            t1 = t2 + self.m[0] * self.g * (l1/2) * np.cos(q1)
            
            self.tau_m[:, i] = [t1, t2, t3]

    def effort_graph(self):
        """Genera la gráfica de los pares calculados"""
        plt.figure()
        plt.title("Pares en las Juntas (Torques)")
        # Eje X es el tiempo
        t = self.kin.t_m.flatten()
        
        plt.plot(t, self.tau_m[0, :], label='Tau 1')
        plt.plot(t, self.tau_m[1, :], label='Tau 2')
        plt.plot(t, self.tau_m[2, :], label='Tau 3')
        
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Torque (Nm)")
        plt.legend()
        plt.grid(True)
        plt.show()

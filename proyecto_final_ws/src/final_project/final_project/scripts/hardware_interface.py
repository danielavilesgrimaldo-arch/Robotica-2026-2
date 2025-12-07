#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HardwareInterface(Node):
    def __init__(self):
        super().__init__("hardware_interface")
        
        # 1. Suscribirse a las órdenes de los controladores
        # (Escucha lo que mandan manipulator_controller o scara_controller)
        self.joint_hardware_objectives_subscriber = self.create_subscription(
            JointState, 
            "/joint_hardware_objectives", 
            self.hardware_obj_callback, 
            10
        )
        
        # 2. Publicador del estado actual del robot (Feedback para Rviz)
        self.joint_states_publisher = self.create_publisher(
            JointState, 
            "/joint_states", 
            10
        )
        
        self.current_joint_state = JointState()
        
        # --- LISTA UNIVERSAL DE ARTICULACIONES ---
        # Definimos los nombres de AMBOS robots.
        # RRR usa: shoulder_joint, arm_joint, forearm_joint
        # SCARA usa: shoulder_joint, forearm_joint, hand_joint
        # Al incluir todos, el nodo no fallará al arrancar con ninguno de los dos.
        self.current_joint_state.name = [
            "shoulder_joint", 
            "arm_joint", 
            "forearm_joint", 
            "hand_joint"
        ]
        
        # Inicializamos posiciones en 0.0 para las 4 posibles juntas
        self.current_joint_state.position = [0.0, 0.0, 0.0, 0.0]
        
        # Timer para simular el envío constante de datos (como un robot real a 10Hz)
        self.create_timer(0.1, self.joint_states_timer_callback)

    def hardware_obj_callback(self, msg:JointState):
        """
        Recibe la orden de movimiento y actualiza el estado interno.
        En una simulación simple, asumimos que el robot llega instantáneamente.
        """
        # Actualizamos solo las juntas que vienen en el mensaje
        # Esto permite que si el SCARA manda solo 3, solo actualice esas 3
        # y mantenga las otras en 0 o en su último valor.
        self.current_joint_state = msg

    def joint_states_timer_callback(self):
        """Publica el estado actual para que Rviz lo lea"""
        # Actualizamos la marca de tiempo
        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_publisher.publish(self.current_joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

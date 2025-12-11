#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ControlManager(Node):
    def __init__(self):
        super().__init__("control_manager")
        # Suscribirse al estado real (lo que dice el hardware)
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/joint_states",
            self.joint_state_callback, 
            10
        )
        # Suscribirse a los objetivos (lo que dice el controlador)
        self.joint_goals_subscriber = self.create_subscription(
            JointState, 
            "/joint_goals",
            self.joint_goal_callback, 
            10
        )
        # Publicar órdenes al hardware
        self.hardware_command_publisher = self.create_publisher(
            JointState, 
            "/joint_hardware_objectives", 
            10
        )

    def joint_state_callback(self, msg:JointState):
        pass

    def joint_goal_callback(self, msg:JointState):
        # Simplemente pasamos el mensaje del controlador al hardware
        self.hardware_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

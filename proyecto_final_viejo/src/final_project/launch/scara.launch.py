import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg = 'final_project'
    
    # 1. Definir rutas (Apuntando específicamente al SCARA)
    # Busca el modelo URDF del SCARA
    urdf = os.path.join(get_package_share_directory(pkg), 'urdf', 'scara_robot.urdf')
    # Busca la configuración visual que acabas de crear
    rviz_config = os.path.join(get_package_share_directory(pkg), 'config', 'scara_config.rviz')
    
    # 2. Procesar el robot con Xacro
    robot_desc = {'robot_description': Command(['xacro ', urdf])}

    return LaunchDescription([
        # A) Publicador del estado del robot (tf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_desc],
            output='screen'
        ),
        
        # B) Rviz2 con la configuración del SCARA precargada
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # C) Nodos Lógicos
        # Manager (Puente)
        Node(package=pkg, executable='custom_manager'),
        # Hardware Interface (Simulación Física Universal)
        Node(package=pkg, executable='hardware_interface'),
        # Controlador ESPECÍFICO para el SCARA
        Node(package=pkg, executable='scara_controller', output='screen')
    ])

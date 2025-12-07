import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg = 'final_project'
    
    # 1. Rutas a archivos
    # Busca el URDF en la carpeta 'urdf'
    urdf = os.path.join(get_package_share_directory(pkg), 'urdf', 'rrr_robot.urdf')
    # Busca la configuración de Rviz en la carpeta 'config'
    rviz_config = os.path.join(get_package_share_directory(pkg), 'config', 'rrr_config.rviz')
    
    # 2. Procesar el robot con Xacro
    robot_desc = {'robot_description': Command(['xacro ', urdf])}

    return LaunchDescription([
        # A) Publicador del estado del robot (tf)
        # Este nodo lee el URDF y publica las transformaciones
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_desc],
            output='screen'
        ),
        
        # B) Rviz2 con configuración precargada
        # Carga el archivo .rviz para que se vea el robot automáticamente
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # C) Nodos Lógicos (Scripts Python)
        # Manager: Puente de comunicación
        Node(package=pkg, executable='custom_manager'),
        # Hardware: Simulación física (debe ser el Universal que hicimos antes)
        Node(package=pkg, executable='hardware_interface'),
        # Controlador: Lógica específica del RRR (Cinemática, Dinámica, Gráficas)
        Node(package=pkg, executable='manipulator_controller', output='screen')
    ])

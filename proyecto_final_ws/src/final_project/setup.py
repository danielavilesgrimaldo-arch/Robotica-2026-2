from setuptools import setup
import os
from glob import glob

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- INSTALACIÓN DE RECURSOS ---
        # 1. Lanzadores
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 2. Modelos URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # 3. Configuraciones Rviz (¡Nuevo y necesario!)
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='tu@email.com',
    description='Proyecto Final',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registramos los 4 nodos ejecutables
            'custom_manager = final_project.scripts.custom_manager:main',
            'hardware_interface = final_project.scripts.hardware_interface:main',
            'manipulator_controller = final_project.scripts.manipulator_controller:main',
            'scara_controller = final_project.scripts.scara_controller:main',
        ],
    },
)

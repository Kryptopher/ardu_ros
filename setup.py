from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'missions', 'input_shaping'),
            glob('missions/input_shaping/*.csv')),
        (os.path.join('share', package_name, 'missions', 'edmdc'),
            glob('missions/edmdc/*.csv')),
        (os.path.join('share', package_name, 'experiments'),
            glob('experiments/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanjay Maharjan',
    maintainer_email='sanjay@lsu.edu',
    description='Research-grade drone mission platform',
    license='MIT',

    entry_points={
        'console_scripts': [
            'encoder_node     = drone_mission.nodes.encoder_node:main',
            'hardware_bridge  = drone_mission.nodes.hardware_bridge:main',
            'safety_monitor   = drone_mission.nodes.safety_monitor:main',
            'mission_manager  = drone_mission.nodes.mission_manager:main',
            'setpoint_publisher = drone_mission.nodes.setpoint_publisher:main',
            'logger_node      = drone_mission.nodes.logger_node:main',
        ],
    },
)

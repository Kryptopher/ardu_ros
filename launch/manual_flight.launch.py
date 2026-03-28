"""
manual_flight.launch.py
Logger only — for manual flights where you want data but no mission control.
Usage:
  ros2 launch drone_mission manual_flight.launch.py run_label:=manual_test
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('run_label',
            default_value='manual_flight',
            description='Label for log folder'),
        DeclareLaunchArgument('operator',
            default_value='unknown',
            description='Operator name'),
        DeclareLaunchArgument('notes',
            default_value='',
            description='Run notes'),
    ]

    encoder = Node(
        package='drone_mission',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        parameters=[{
            'publish_rate_hz': 200.0,
        }]
    )

    hardware_bridge = Node(
        package='drone_mission',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
    )

    logger = TimerAction(
        period=3.0,
        actions=[Node(
            package='drone_mission',
            executable='logger_node',
            name='logger_node',
            output='screen',
            parameters=[{
                'log_base_dir': '/home/pi/logs',
                'run_label':    LaunchConfiguration('run_label'),
                'operator':     LaunchConfiguration('operator'),
                'notes':        LaunchConfiguration('notes'),
                'autostart':    True,
            }]
        )]
    )

    return LaunchDescription(args + [
        encoder,
        hardware_bridge,
        logger,
    ])

"""
bench_test.launch.py
All nodes in test_mode — safe for bench testing without flying.
Safety aborts disabled, geofence disabled.
Usage:
  ros2 launch drone_mission bench_test.launch.py mission_file:=missions/input_shaping/pulse_5ms.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
os.environ['CYCLONEDDS_URI'] = f"file://{os.path.expanduser('~')}/cyclone_config.xml"
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

def generate_launch_description():

    args = [
        DeclareLaunchArgument('mission_file',
            default_value='/home/pi/ros2_px/src/drone_mission/missions/input_shaping/pulse_5ms.csv',
            description='Path to mission CSV'),
        DeclareLaunchArgument('shaper_type',
            default_value='none',
            description='Input shaper: none, ZV, ZVD'),
        DeclareLaunchArgument('rope_length',
            default_value='1.0',
            description='Rope length in metres'),
        DeclareLaunchArgument('run_label',
            default_value='bench_test',
            description='Label for log folder'),
    ]

    encoder = Node(
        package='drone_mission',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        parameters=[{'publish_rate_hz': 200.0}]
    )

    hardware_bridge = Node(
        package='drone_mission',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
    )

    safety_monitor = TimerAction(
        period=3.0,
        actions=[Node(
            package='drone_mission',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'test_mode': True,   # ← disabled
            }]
        )]
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
                'autostart':    True,
            }]
        )]
    )

    setpoint_publisher = TimerAction(
        period=4.0,
        actions=[Node(
            package='drone_mission',
            executable='setpoint_publisher',
            name='setpoint_publisher',
            output='screen',
        )]
    )

    mission_manager = TimerAction(
        period=5.0,
        actions=[Node(
            package='drone_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'mission_file':  LaunchConfiguration('mission_file'),
                'shaper_type':   LaunchConfiguration('shaper_type'),
                'rope_length':   LaunchConfiguration('rope_length'),
                'settle_vel_ms': 0.5,
                'settle_dur_s':  1.0,
            }]
        )]
    )

    return LaunchDescription(args + [
        encoder,
        hardware_bridge,
        safety_monitor,
        logger,
        setpoint_publisher,
        mission_manager,
    ])

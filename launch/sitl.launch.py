"""
sitl.launch.py
Full system launch for SITL testing on WSL2.
No encoder node (no GPIO). Auto-connects to SITL via TCP.
Usage:
  ros2 launch drone_mission sitl.launch.py
  ros2 launch drone_mission sitl.launch.py mission_file:=missions/input_shaping/pulse_5ms.csv shaper_type:=ZV rope_length:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import subprocess
os.environ['CYCLONEDDS_URI'] = f"file://{os.path.expanduser('~')}/cyclone_config.xml"
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
# Source workspace for custom messages
os.environ['AMENT_PREFIX_PATH'] = \
    f"/home/sanjay/ros2_px/install/drone_mission:" \
    f"/home/sanjay/ros2_px/install/drone_mission_msgs:" \
    f"{os.environ.get('AMENT_PREFIX_PATH', '')}"

def generate_launch_description():

    pkg_dir = os.path.expanduser(
        '~/ros2_px/src/drone_mission')

    args = [
        DeclareLaunchArgument('mission_file',
            default_value=os.path.join(pkg_dir,
                'missions/input_shaping/pulse_5ms.csv'),
            description='Path to mission CSV'),
        DeclareLaunchArgument('shaper_type',
            default_value='none',
            description='Input shaper: none, ZV, ZVD'),
        DeclareLaunchArgument('rope_length',
            default_value='1.0',
            description='Rope length in metres'),
        DeclareLaunchArgument('damping',
            default_value='0.02',
            description='Damping ratio'),
        DeclareLaunchArgument('run_label',
            default_value='sitl_test',
            description='Label for log folder'),
        DeclareLaunchArgument('operator',
            default_value='sanjay',
            description='Operator name'),
        DeclareLaunchArgument('notes',
            default_value='',
            description='Run notes'),
        DeclareLaunchArgument('settle_vel_ms',
            default_value='0.05',
            description='Settlement velocity threshold m/s'),
        DeclareLaunchArgument('settle_dur_s',
            default_value='1.0',
            description='Settlement duration seconds'),
        DeclareLaunchArgument('speedup',
            default_value='1.0',
            description='SITL speedup factor'),
    ]

    # 1. Hardware bridge — starts immediately
    hardware_bridge = Node(
        package='drone_mission',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
    )

    # 2. Safety monitor — test mode, no geofence
    safety_monitor = TimerAction(
        period=2.0,
        actions=[Node(
            package='drone_mission',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'test_mode':            True,
                'min_battery_voltage':  10.0,
                'warn_battery_voltage': 11.0,
                'max_altitude_m':       120.0,
                'warn_altitude_m':      100.0,
                'max_velocity_ms':      15.0,
                'comms_timeout_s':      5.0,
                'geofence_enabled':     False,
                'abort_on_mode_change': False,
            }]
        )]
    )

    # 3. Logger node
    logger = TimerAction(
        period=2.0,
        actions=[Node(
            package='drone_mission',
            executable='logger_node',
            name='logger_node',
            output='screen',
            parameters=[{
                'log_base_dir':  '/home/sanjay/logs',
                'run_label':     LaunchConfiguration('run_label'),
                'operator':      LaunchConfiguration('operator'),
                'shaper_type':   LaunchConfiguration('shaper_type'),
                'rope_length':   LaunchConfiguration('rope_length'),
                'notes':         LaunchConfiguration('notes'),
                'mission_file':  LaunchConfiguration('mission_file'),
                'log_flight':    True,
                'log_angles':    False,   # no encoder in SITL
                'log_events':    True,
                'autostart':     True,
            }]
        )]
    )

    # 4. Setpoint publisher
    setpoint_publisher = TimerAction(
        period=3.0,
        actions=[Node(
            package='drone_mission',
            executable='setpoint_publisher',
            name='setpoint_publisher',
            output='screen',
        )]
    )

    # 5. Mission manager — last
    mission_manager = TimerAction(
        period=4.0,
        actions=[Node(
            package='drone_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'mission_file':     LaunchConfiguration('mission_file'),
                'shaper_type':      LaunchConfiguration('shaper_type'),
                'rope_length':      LaunchConfiguration('rope_length'),
                'damping':          LaunchConfiguration('damping'),
                'settle_vel_ms':    LaunchConfiguration('settle_vel_ms'),
                'settle_dur_s':     LaunchConfiguration('settle_dur_s'),
                'settle_timeout_s': 30.0,
                'settle_proceed':   True,
                'auto_takeoff':     False,
                'arrival_radius_m': 0.5,
                'operator':         LaunchConfiguration('operator'),
                'run_label':        LaunchConfiguration('run_label'),
            }]
        )]
    )

    return LaunchDescription(args + [
        LogInfo(msg='=== SITL MISSION SYSTEM STARTING ==='),
        hardware_bridge,
        safety_monitor,
        logger,
        setpoint_publisher,
        mission_manager,
        LogInfo(msg='=== ALL NODES LAUNCHED ==='),
    ])

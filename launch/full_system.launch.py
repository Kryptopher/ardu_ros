"""
full_system.launch.py
Starts all nodes for a full mission flight.
Usage:
  ros2 launch drone_mission full_system.launch.py
  ros2 launch drone_mission full_system.launch.py mission_file:=missions/input_shaping/ZV_rope1m.csv shaper_type:=ZV rope_length:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
os.environ['CYCLONEDDS_URI'] = f"file://{os.path.expanduser('~')}/cyclone_config.xml"
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

def generate_launch_description():

    # ── Launch arguments ──────────────────────────────
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
        DeclareLaunchArgument('damping',
            default_value='0.02',
            description='Damping ratio'),
        DeclareLaunchArgument('run_label',
            default_value='unlabeled',
            description='Label for log folder'),
        DeclareLaunchArgument('operator',
            default_value='unknown',
            description='Operator name'),
        DeclareLaunchArgument('notes',
            default_value='',
            description='Run notes'),
        DeclareLaunchArgument('test_mode',
            default_value='false',
            description='Disable safety aborts for bench testing'),
        DeclareLaunchArgument('settle_vel_ms',
            default_value='0.3',
            description='Settlement velocity threshold m/s'),
        DeclareLaunchArgument('settle_dur_s',
            default_value='2.0',
            description='Settlement duration seconds'),
    ]

    # ── Nodes ─────────────────────────────────────────

    # 1. Encoder node — starts immediately
    encoder = Node(
        package='drone_mission',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        parameters=[{
            'publish_rate_hz': 200.0,
            'enc1_a': 6,
            'enc1_b': 13,
            'enc2_a': 19,
            'enc2_b': 26,
            'ppr':    1000,
        }]
    )

    # 2. Hardware bridge — starts immediately
    hardware_bridge = Node(
        package='drone_mission',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
    )

    # 3. Safety monitor — starts after 3s
    safety_monitor = TimerAction(
        period=3.0,
        actions=[Node(
            package='drone_mission',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'test_mode':            LaunchConfiguration('test_mode'),
                'min_battery_voltage':  21.0,
                'warn_battery_voltage': 22.2,
                'max_altitude_m':       120.0,
                'warn_altitude_m':      100.0,
                'max_velocity_ms':      12.0,
                'comms_timeout_s':      2.0,
                'geofence_enabled':     True,
                'abort_on_mode_change': True,
            }]
        )]
    )

    # 4. Logger node — starts after 3s
    logger = TimerAction(
        period=3.0,
        actions=[Node(
            package='drone_mission',
            executable='logger_node',
            name='logger_node',
            output='screen',
            parameters=[{
                'log_base_dir':  '/home/pi/logs',
                'run_label':     LaunchConfiguration('run_label'),
                'operator':      LaunchConfiguration('operator'),
                'shaper_type':   LaunchConfiguration('shaper_type'),
                'rope_length':   LaunchConfiguration('rope_length'),
                'notes':         LaunchConfiguration('notes'),
                'mission_file':  LaunchConfiguration('mission_file'),
                'log_flight':    True,
                'log_angles':    True,
                'log_events':    True,
                'autostart':     True,
            }]
        )]
    )

    # 5. Setpoint publisher — starts after 4s
    setpoint_publisher = TimerAction(
        period=4.0,
        actions=[Node(
            package='drone_mission',
            executable='setpoint_publisher',
            name='setpoint_publisher',
            output='screen',
        )]
    )

    # 6. Mission manager — starts after 5s (last)
    mission_manager = TimerAction(
        period=5.0,
        actions=[Node(
            package='drone_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'mission_file':    LaunchConfiguration('mission_file'),
                'shaper_type':     LaunchConfiguration('shaper_type'),
                'rope_length':     LaunchConfiguration('rope_length'),
                'damping':         LaunchConfiguration('damping'),
                'settle_vel_ms':   LaunchConfiguration('settle_vel_ms'),
                'settle_dur_s':    LaunchConfiguration('settle_dur_s'),
                'settle_timeout_s': 30.0,
                'settle_proceed':   True,
                'auto_takeoff':     False,
                'arrival_radius_m': 1.0,
                'operator':         LaunchConfiguration('operator'),
                'run_label':        LaunchConfiguration('run_label'),
            }]
        )]
    )

    return LaunchDescription(args + [
        LogInfo(msg='=== DRONE MISSION SYSTEM STARTING ==='),
        encoder,
        hardware_bridge,
        safety_monitor,
        logger,
        setpoint_publisher,
        mission_manager,
        LogInfo(msg='=== ALL NODES LAUNCHED ==='),
    ])

#!/usr/bin/env python3
"""
mission_manager.py — Mission state machine
Reads CSV, manages states, publishes commands to setpoint_publisher.
Home = position when ENTER pressed (or auto-takeoff completes).
All CSV coordinates are offsets from home.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import csv
import math
import time
import sys
import os
from datetime import datetime

from drone_mission_msgs.msg import (
    DroneState, MissionState, MissionEvent, PayloadAngles)
from std_msgs.msg import String
from mavros_msgs.srv import SetMode, CommandBool

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

# ── State machine states ───────────────────────────────
class State:
    PREFLIGHT        = 'PREFLIGHT'
    WAITING_ARM      = 'WAITING_ARM'
    WAITING_LOITER   = 'WAITING_LOITER'
    SNAPSHOT_ORIGIN  = 'SNAPSHOT_ORIGIN'
    SETTLING         = 'SETTLING'
    EXECUTING        = 'EXECUTING'
    WAITING_LAND     = 'WAITING_LAND'
    ABORTED          = 'ABORTED'
    DONE             = 'DONE'


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ── Parameters ────────────────────────────────
        self.declare_parameter('mission_file',
            '/home/pi/ros2_px/src/drone_mission/missions/input_shaping/pulse_5ms.csv')
        self.declare_parameter('shaper_type',        'none')
        self.declare_parameter('rope_length',         1.0)
        self.declare_parameter('damping',             0.02)
        self.declare_parameter('settle_vel_ms',       0.3)
        self.declare_parameter('settle_dur_s',        2.0)
        self.declare_parameter('settle_timeout_s',   30.0)
        self.declare_parameter('settle_proceed',      True)
        self.declare_parameter('auto_takeoff',        False)
        self.declare_parameter('takeoff_alt',         10.0)
        self.declare_parameter('arrival_radius_m',    1.0)
        self.declare_parameter('operator',            'unknown')
        self.declare_parameter('run_label',           'unlabeled')

        self.mission_file    = self.get_parameter('mission_file').value
        self.shaper_type     = self.get_parameter('shaper_type').value
        self.rope_length     = self.get_parameter('rope_length').value
        self.damping         = self.get_parameter('damping').value
        self.settle_vel      = self.get_parameter('settle_vel_ms').value
        self.settle_dur      = self.get_parameter('settle_dur_s').value
        self.settle_timeout  = self.get_parameter('settle_timeout_s').value
        self.settle_proceed  = self.get_parameter('settle_proceed').value
        self.auto_takeoff    = self.get_parameter('auto_takeoff').value
        self.takeoff_alt     = self.get_parameter('takeoff_alt').value
        self.arrival_radius  = self.get_parameter('arrival_radius_m').value

        # ── State machine ─────────────────────────────
        self.state           = State.PREFLIGHT
        self.prev_state      = None
        self.mission         = []
        self.cmd_index       = 0
        self.mission_start_t = None
        self.state_enter_t   = None
        self.settle_start_t  = None
        self.home            = None   # (x, y, z) NED at ENTER press

        # ── Drone state ───────────────────────────────
        self.drone           = None
        self.drone_last_t    = None

        # ── Settlement tracking ───────────────────────
        self._settle_ok_since = None

        # ── Publishers ────────────────────────────────
        self.state_pub = self.create_publisher(
            MissionState, '/mission/state', RELIABLE_QOS)
        self.event_pub = self.create_publisher(
            MissionEvent, '/mission/event', RELIABLE_QOS)
        self.cmd_pub   = self.create_publisher(
            String, '/mission/command', RELIABLE_QOS)
        self.active_pub = self.create_publisher(
            String, '/mission/active', RELIABLE_QOS)

        # ── Subscriptions ─────────────────────────────
        self.create_subscription(
            DroneState, '/drone/state',
            self._drone_cb, SENSOR_QOS)
        self.create_subscription(
            String, '/mission/abort',
            self._abort_cb, RELIABLE_QOS)

        # ── MAVROS services ───────────────────────────
        self.mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        self.arm_client  = self.create_client(
            CommandBool, '/mavros/cmd/arming')

        # ── Main timer at 20Hz ────────────────────────
        self.create_timer(0.05, self._tick)

        # ── Status timer at 2Hz ───────────────────────
        self.create_timer(0.5,  self._status_publish)

        # ── Load mission ──────────────────────────────
        self._load_mission()

        self.get_logger().info(
            f'Mission manager started\n'
            f'  file={self.mission_file}\n'
            f'  shaper={self.shaper_type}\n'
            f'  rope={self.rope_length}m\n'
            f'  cmds={len(self.mission)}')

    # ── Mission loading ───────────────────────────────
    def _load_mission(self):
        if not os.path.exists(self.mission_file):
            self.get_logger().error(
                f'Mission file not found: {self.mission_file}')
            return

        self.mission      = []
        self.manual_takeoff = True

        with open(self.mission_file) as f:
            rows = list(csv.DictReader(f))

        for i, row in enumerate(rows):
            seg_type = row['type'].strip()
            t_start  = float(row['t'])
            t_end    = float(rows[i+1]['t']) \
                       if i+1 < len(rows) else t_start + 5.0

            if seg_type == 'manual_takeoff':
                self.manual_takeoff = True
                continue
            if seg_type == 'auto_takeoff':
                self.manual_takeoff = False
                continue
            if seg_type == 'end':
                break
            if seg_type != 'cmd':
                continue

            # Get frame — default body for vel, ned for traj
            mode    = row.get('mode',    'vel').strip()
            profile = row.get('profile', 'step').strip()
            frame   = row.get('frame',   '').strip()
            if not frame:
                frame = 'ned' if mode == 'traj' else 'body'

            self.mission.append({
                't_start': t_start,
                't_end':   t_end,
                'mode':    mode,
                'profile': profile,
                'frame':   frame,
                'x':   float(row.get('x',  0)),
                'y':   float(row.get('y',  0)),
                'z':   float(row.get('z',  0)),
                'vx':  float(row.get('vx', 0)),
                'vy':  float(row.get('vy', 0)),
                'vz':  float(row.get('vz', 0)),
                'ax':  float(row.get('ax', 0)),
                'notes': row.get('notes', '').strip(),
            })

        self.get_logger().info(
            f'Loaded {len(self.mission)} commands')
        for seg in self.mission:
            self.get_logger().info(
                f"  t={seg['t_start']:.1f}s  "
                f"{seg['mode']:4s}  "
                f"{seg['profile']:6s}  "
                f"{seg['frame']:4s}  "
                f"vel=({seg['vx']:.2f},"
                f"{seg['vy']:.2f},"
                f"{seg['vz']:.2f})  "
                f"{seg['notes']}")

    # ── Callbacks ─────────────────────────────────────
    def _drone_cb(self, msg):
        self.drone        = msg
        self.drone_last_t = self.get_clock().now()

    def _abort_cb(self, msg):
        reason = msg.data
        self.get_logger().error(f'ABORT received: {reason}')
        self._pub_event('ABORT', reason)
        self._set_state(State.ABORTED)
        self._notify_active(False)

    # ── State machine tick ────────────────────────────
    def _tick(self):
        if self.drone is None:
            return

        s = self.state

        if s == State.PREFLIGHT:
            self._tick_preflight()
        elif s == State.WAITING_ARM:
            self._tick_waiting_arm()
        elif s == State.WAITING_LOITER:
            self._tick_waiting_loiter()
        elif s == State.SNAPSHOT_ORIGIN:
            self._tick_snapshot()
        elif s == State.SETTLING:
            self._tick_settling()
        elif s == State.EXECUTING:
            self._tick_executing()
        elif s == State.WAITING_LAND:
            self._tick_waiting_land()

    def _tick_preflight(self):
        if not self.mission:
            self.get_logger().warn('No mission loaded')
            return
        if self.drone is None:
            return
        # All good — move to waiting for arm
        self.get_logger().info('Preflight OK — waiting for arm')
        self._set_state(State.WAITING_ARM)

    def _tick_waiting_arm(self):
        if self.drone.armed:
            self.get_logger().info('Armed — waiting for LOITER/GUIDED')
            self._set_state(State.WAITING_LOITER)

    def _tick_waiting_loiter(self):
        mode = self.drone.mode
        if mode in ('LOITER', 'GUIDED', 'AUTO'):
            if self.manual_takeoff:
                if not hasattr(self, '_start_received'):
                    self._start_received = False
                if not hasattr(self, '_start_sub_created'):
                    self._start_sub_created = True
                    self.create_subscription(
                        String, '/mission/start',
                        self._start_cb, RELIABLE_QOS)
                    self.get_logger().info(
                        '\n>>> Send start signal:\n'
                        '    ros2 topic pub /mission/start '
                        'std_msgs/msg/String "{data: start}" --once\n')
                if not self._start_received:
                    return   # keep waiting, don't block
            self._set_state(State.SNAPSHOT_ORIGIN)

    def _start_cb(self, msg):
        self._start_received = True
        self.get_logger().info('Start signal received!')
   
    def _tick_snapshot(self):
        if self.drone is None:
            return
        # Save home position
        self.home = (
            self.drone.x,
            self.drone.y,
            self.drone.z)
        self.get_logger().info(
            f'Home snapshotted: '
            f'({self.home[0]:.2f}, '
            f'{self.home[1]:.2f}, '
            f'{self.home[2]:.2f}) NED')
        self._pub_event('HOME_SET',
            f'home=({self.home[0]:.2f},'
            f'{self.home[1]:.2f},'
            f'{self.home[2]:.2f})')

        # Notify logger to start
        self._notify_active(True)

        # Record mission start time
        self.mission_start_t = self.get_clock().now()
        self.cmd_index       = 0
        self._settle_ok_since = None

        self._pub_event('MISSION_START', f'cmds={len(self.mission)}')
        self.get_logger().info('--- MISSION START ---')
        self._set_state(State.SETTLING)

    def _tick_settling(self):
        if self.drone is None:
            return

        spd = math.sqrt(
            self.drone.vx**2 +
            self.drone.vy**2 +
            self.drone.vz**2)

        now = self.get_clock().now()

        if spd < self.settle_vel:
            if self._settle_ok_since is None:
                self._settle_ok_since = now
            else:
                dur = (now - self._settle_ok_since).nanoseconds / 1e9
                if dur >= self.settle_dur:
                    self.get_logger().info(
                        f'Settled — spd={spd:.2f}m/s '
                        f'for {dur:.1f}s')
                    self._pub_event('SETTLED',
                        f'spd={spd:.3f} t={self._mission_elapsed():.2f}')
                    self._settle_ok_since = None
                    self._set_state(State.EXECUTING)
        else:
            self._settle_ok_since = None

        # Settlement timeout
        if self.state_enter_t is not None:
            elapsed = (now - self.state_enter_t).nanoseconds / 1e9
            if elapsed > self.settle_timeout:
                if self.settle_proceed:
                    self.get_logger().warn(
                        f'Settlement timeout — proceeding anyway '
                        f'spd={spd:.2f}m/s')
                    self._pub_event('SETTLE_TIMEOUT',
                        f'spd={spd:.3f}')
                    self._set_state(State.EXECUTING)
                else:
                    self.get_logger().error(
                        'Settlement timeout — aborting')
                    self._abort_cb(
                        type('M', (), {'data': 'SETTLE_TIMEOUT'})())

    def _tick_executing(self):
        if not self.mission:
            self._mission_complete()
            return

        now_t = self._mission_elapsed()

        # Get current command
        if self.cmd_index >= len(self.mission):
            self._mission_complete()
            return

        cmd = self.mission[self.cmd_index]

        # Check if it's time for this command
        if now_t < cmd['t_start']:
            # Publish current command to setpoint_publisher
            self._publish_command(cmd, now_t)
            return

        # Time to advance to next command
        self.get_logger().info(
            f"t={now_t:.1f}s  "
            f"cmd {self.cmd_index+1}/{len(self.mission)}  "
            f"{cmd['mode']}  {cmd['profile']}  "
            f"{cmd['frame']}  "
            f"vel=({cmd['vx']:.2f},"
            f"{cmd['vy']:.2f},"
            f"{cmd['vz']:.2f})")

        self._pub_event('CMD_START',
            f"cmd={self.cmd_index+1}/{len(self.mission)} "
            f"{cmd['mode']} {cmd['profile']} {cmd['frame']}",
            now_t)

        self.cmd_index += 1

        # If this was the last command, complete
        if self.cmd_index >= len(self.mission):
            self._publish_command(cmd, now_t)
            return

        # Re-settle between commands if needed
        next_cmd = self.mission[self.cmd_index]
        if next_cmd['mode'] == 'vel':
            self._settle_ok_since = None
            self._set_state(State.SETTLING)

    def _publish_command(self, cmd, t):
        """Serialize command to JSON string for setpoint_publisher."""
        import json
        # Convert home-relative coords to absolute NED
        if self.home:
            hx, hy, hz = self.home
        else:
            hx = hy = hz = 0.0

        payload = {
            'mode':    cmd['mode'],
            'profile': cmd['profile'],
            'frame':   cmd['frame'],
            # Absolute NED target position
            'x':  hx + cmd['x'],
            'y':  hy + cmd['y'],
            'z':  hz + cmd['z'],
            # Velocity command (frame conversion in setpoint_publisher)
            'vx': cmd['vx'],
            'vy': cmd['vy'],
            'vz': cmd['vz'],
            'ax': cmd['ax'],
            # Shaper config
            'shaper':       self.shaper_type,
            'rope_length':  self.rope_length,
            'damping':      self.damping,
            # Timing
            't_start':  cmd['t_start'],
            't_end':    cmd['t_end'],
            't_mission': t,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.cmd_pub.publish(msg)

    def _mission_complete(self):
        self.get_logger().info('Mission complete — all commands fired')
        self._pub_event('MISSION_COMPLETE',
            f't={self._mission_elapsed():.2f}s')
        self._notify_active(False)
        self._set_state(State.WAITING_LAND)
        # Command LAND
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            self.mode_client.call_async(req)
            self.get_logger().info('Landing...')

    def _tick_waiting_land(self):
        if self.drone and not self.drone.armed:
            self.get_logger().info('Landed and disarmed — Done')
            self._pub_event('DONE', 'mission complete')
            self._set_state(State.DONE)
            # Publish DONE state explicitly
            msg = MissionState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.state = State.DONE
            msg.prev_state = State.WAITING_LAND
            msg.mission_time = self._mission_elapsed()
            msg.cmd_index = len(self.mission)
            msg.cmd_total = len(self.mission)
            self.state_pub.publish(msg)

    # ── Helpers ───────────────────────────────────────
    def _set_state(self, new_state):
        self.prev_state    = self.state
        self.state         = new_state
        self.state_enter_t = self.get_clock().now()
        self.get_logger().info(
            f'State: {self.prev_state} → {new_state}')

    def _mission_elapsed(self):
        if self.mission_start_t is None:
            return 0.0
        dt = self.get_clock().now() - self.mission_start_t
        return dt.nanoseconds / 1e9

    def _notify_active(self, active):
        msg = String()
        msg.data = str(active)
        self.active_pub.publish(msg)

    def _pub_event(self, event_type, description, t=None):
        msg             = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event_type  = event_type
        msg.description = description
        msg.mission_time = t if t is not None else self._mission_elapsed()
        msg.value       = 0.0
        self.event_pub.publish(msg)

    def _status_publish(self):
        msg              = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state        = self.state
        msg.prev_state   = self.prev_state or ''
        msg.mission_time = self._mission_elapsed()
        msg.cmd_index    = self.cmd_index
        msg.cmd_total    = len(self.mission)
        msg.settled      = self.state != State.SETTLING
        if self.drone:
            spd = math.sqrt(
                self.drone.vx**2 +
                self.drone.vy**2 +
                self.drone.vz**2)
            msg.vel_magnitude = spd
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

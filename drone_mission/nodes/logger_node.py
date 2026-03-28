#!/usr/bin/env python3
"""
logger_node.py — Unified data logger
Subscribes to /drone/state (50Hz) and /payload/angles (200Hz)
Writes timestamped run folder with all data + metadata
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from drone_mission_msgs.msg import DroneState, PayloadAngles, MissionState, MissionEvent
import csv
import os
import json
import shutil
from datetime import datetime

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        # ── Parameters ────────────────────────────────
        self.declare_parameter('log_base_dir',  '/home/pi/logs')
        self.declare_parameter('run_label',     'unlabeled')
        self.declare_parameter('mission_file',  '')
        self.declare_parameter('config_file',   '')
        self.declare_parameter('operator',      'unknown')
        self.declare_parameter('shaper_type',   'none')
        self.declare_parameter('rope_length',   0.0)
        self.declare_parameter('notes',         '')
        self.declare_parameter('log_flight',    True)
        self.declare_parameter('log_angles',    True)
        self.declare_parameter('log_events',    True)
        self.declare_parameter('autostart',     True)

        base_dir     = self.get_parameter('log_base_dir').value
        self.label   = self.get_parameter('run_label').value
        self.mission = self.get_parameter('mission_file').value
        self.config  = self.get_parameter('config_file').value
        self.operator= self.get_parameter('operator').value
        self.shaper  = self.get_parameter('shaper_type').value
        self.rope    = self.get_parameter('rope_length').value
        self.notes   = self.get_parameter('notes').value
        log_flight   = self.get_parameter('log_flight').value
        log_angles   = self.get_parameter('log_angles').value
        log_events   = self.get_parameter('log_events').value
        autostart    = self.get_parameter('autostart').value

        # ── Run folder ────────────────────────────────
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        folder_name    = f'{self.timestamp}_{self.label}'
        self.run_dir   = os.path.join(base_dir, folder_name)
        os.makedirs(self.run_dir, exist_ok=True)

        # ── State ─────────────────────────────────────
        self.logging_active  = False
        self.mission_start_t = None
        self.flight_rows     = 0
        self.angle_rows      = 0
        self.event_rows      = 0

        # File handles and writers
        self._flight_f  = None
        self._flight_w  = None
        self._angle_f   = None
        self._angle_w   = None
        self._event_f   = None
        self._event_w   = None

        # ── Open log files ────────────────────────────
        if log_flight:
            self._open_flight_log()
        if log_angles:
            self._open_angle_log()
        if log_events:
            self._open_event_log()

        # ── Subscriptions ─────────────────────────────
        self.create_subscription(
            DroneState, '/drone/state',
            self._drone_state_cb, SENSOR_QOS)

        self.create_subscription(
            PayloadAngles, '/payload/angles',
            self._angles_cb, SENSOR_QOS)

        self.create_subscription(
            MissionState, '/mission/state',
            self._mission_state_cb, RELIABLE_QOS)

        self.create_subscription(
            MissionEvent, '/mission/event',
            self._mission_event_cb, RELIABLE_QOS)

        # ── Diagnostics timer at 1Hz ──────────────────
        self.create_timer(1.0, self._diagnostics)

        # ── Auto-start logging ────────────────────────
        if autostart:
            self.start_logging()

        self.get_logger().info(
            f'Logger node started → {self.run_dir}')

    # ── File setup ────────────────────────────────────
    def _open_flight_log(self):
        path = os.path.join(self.run_dir, 'flight.csv')
        self._flight_f = open(path, 'w', newline='')
        self._flight_w = csv.writer(self._flight_f)
        self._flight_w.writerow([
            't',
            # Position
            'x','y','z',
            # Velocity NED
            'vx','vy','vz',
            # Velocity body
            'vx_b','vy_b','vz_b',
            # Attitude
            'roll','pitch','yaw',
            # Body rates
            'p','q','r',
            # Accel fused
            'ax','ay','az',
            # Accel raw
            'ax_raw','ay_raw','az_raw',
            # Mag
            'mag_x','mag_y','mag_z',
            # GPS
            'lat','lon','alt_gps','gps_fix','gps_sats','gps_pos_std',
            # Baro
            'alt_baro','climb_rate',
            # HUD
            'heading','groundspeed','airspeed','throttle',
            # Nav cmd
            'nav_roll_cmd','nav_pitch_cmd',
            # Wind
            'wind_x','wind_y','wind_z',
            # Motors
            'motor1','motor2','motor3','motor4',
            # Battery
            'batt_v','batt_a','batt_pct',
            # EKF
            'ekf_ok','pos_h_acc','pos_v_acc',
            # Mode
            'mode','armed','guided',
        ])
        self.get_logger().info(f'Flight log → {path}')

    def _open_angle_log(self):
        path = os.path.join(self.run_dir, 'angles.csv')
        self._angle_f = open(path, 'w', newline='')
        self._angle_w = csv.writer(self._angle_f)
        self._angle_w.writerow([
            't','pitch_deg','roll_deg',
            'pitch_count','roll_count','deg_per_count'])
        self.get_logger().info(f'Angle log → {path}')

    def _open_event_log(self):
        path = os.path.join(self.run_dir, 'events.csv')
        self._event_f = open(path, 'w', newline='')
        self._event_w = csv.writer(self._event_f)
        self._event_w.writerow([
            't_wall','t_mission','event_type','description','value'])
        self.get_logger().info(f'Event log → {path}')

    # ── Logging control ───────────────────────────────
    def start_logging(self):
        self.mission_start_t = self.get_clock().now()
        self.logging_active  = True
        self.flight_rows     = 0
        self.angle_rows      = 0
        self.event_rows      = 0
        self.get_logger().info('Logging STARTED')
        self._log_event('LOGGER_START', 'Logging started', 0.0)

    def stop_logging(self):
        self.logging_active = False
        self._log_event('LOGGER_STOP', 'Logging stopped', 0.0)
        self.get_logger().info(
            f'Logging STOPPED — '
            f'flight={self.flight_rows} rows  '
            f'angles={self.angle_rows} rows  '
            f'events={self.event_rows} rows')
        self._save_metadata()
        self._flush_and_close()

    def _mission_elapsed(self):
        if self.mission_start_t is None:
            return 0.0
        dt = self.get_clock().now() - self.mission_start_t
        return dt.nanoseconds / 1e9

    # ── Callbacks ─────────────────────────────────────
    def _drone_state_cb(self, msg):
        if not self.logging_active or self._flight_w is None:
            return
        t = self._mission_elapsed()
        self._flight_w.writerow([
            round(t, 4),
            round(msg.x,  4), round(msg.y,  4), round(msg.z,  4),
            round(msg.vx, 4), round(msg.vy, 4), round(msg.vz, 4),
            round(msg.vx_body, 4), round(msg.vy_body, 4), round(msg.vz_body, 4),
            round(msg.roll,  3), round(msg.pitch, 3), round(msg.yaw, 3),
            round(msg.p, 4), round(msg.q, 4), round(msg.r, 4),
            round(msg.ax, 4), round(msg.ay, 4), round(msg.az, 4),
            round(msg.ax_raw, 4), round(msg.ay_raw, 4), round(msg.az_raw, 4),
            round(msg.mag_x, 2), round(msg.mag_y, 2), round(msg.mag_z, 2),
            round(msg.lat, 8), round(msg.lon, 8), round(msg.alt_gps, 3),
            msg.gps_fix, msg.gps_sats, round(msg.gps_pos_std, 3),
            round(msg.alt_baro, 3), round(msg.climb_rate, 4),
            round(msg.heading, 1), round(msg.groundspeed, 4),
            round(msg.airspeed, 4), round(msg.throttle, 4),
            round(msg.nav_roll_cmd, 4), round(msg.nav_pitch_cmd, 4),
            round(msg.wind_x, 4), round(msg.wind_y, 4), round(msg.wind_z, 4),
            msg.motor1, msg.motor2, msg.motor3, msg.motor4,
            round(msg.battery_voltage, 3),
            round(msg.battery_current, 3),
            round(msg.battery_pct, 3),
            int(msg.ekf_ok),
            round(msg.pos_horiz_accuracy, 4),
            round(msg.pos_vert_accuracy,  4),
            msg.mode, int(msg.armed), int(msg.guided),
        ])
        self.flight_rows += 1

    def _angles_cb(self, msg):
        if not self.logging_active or self._angle_w is None:
            return
        t = self._mission_elapsed()
        self._angle_w.writerow([
            round(t, 5),
            round(msg.pitch_deg, 4),
            round(msg.roll_deg,  4),
            msg.pitch_count,
            msg.roll_count,
            round(msg.deg_per_count, 6),
        ])
        self.angle_rows += 1

    def _mission_state_cb(self, msg):
        if not self.logging_active:
            return
        if not hasattr(self, '_last_mission_state') or \
           self._last_mission_state != msg.state:
            self._last_mission_state = msg.state
            self._log_event(
                'MISSION_STATE',
                f'{msg.state} cmd={msg.cmd_index}/{msg.cmd_total}',
                msg.mission_time)
            # Stop logging when mission is done or aborted
            if msg.state in ('DONE', 'ABORTED'):
                self.get_logger().info(
                    f'Mission {msg.state} — stopping logger')
                self.stop_logging()

    def _mission_event_cb(self, msg):
        if not self.logging_active:
            return
        self._log_event(
            msg.event_type,
            msg.description,
            msg.mission_time)

    def _log_event(self, event_type, description, mission_time):
        if self._event_w is None:
            return
        t_wall = datetime.now().isoformat()
        self._event_w.writerow([
            t_wall,
            round(mission_time, 4),
            event_type,
            description,
            '',
        ])
        self.event_rows += 1

    # ── Metadata ──────────────────────────────────────
    def _save_metadata(self):
        meta = {
            'timestamp':      self.timestamp,
            'label':          self.label,
            'operator':       self.operator,
            'shaper_type':    self.shaper,
            'rope_length_m':  self.rope,
            'notes':          self.notes,
            'mission_file':   self.mission,
            'config_file':    self.config,
            'flight_rows':    self.flight_rows,
            'angle_rows':     self.angle_rows,
            'event_rows':     self.event_rows,
            'run_dir':        self.run_dir,
        }
        path = os.path.join(self.run_dir, 'metadata.json')
        with open(path, 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f'Metadata saved → {path}')

        # Copy mission and config files into run folder
        for src in [self.mission, self.config]:
            if src and os.path.exists(src):
                shutil.copy2(src, self.run_dir)
                self.get_logger().info(
                    f'Copied {os.path.basename(src)} → {self.run_dir}')

    def _flush_and_close(self):
        for f in [self._flight_f, self._angle_f, self._event_f]:
            if f:
                f.flush()
                f.close()
        self.get_logger().info('Log files closed')

    # ── Diagnostics ───────────────────────────────────
    def _diagnostics(self):
        if self.logging_active:
            self.get_logger().info(
                f'Logging — '
                f'flight={self.flight_rows}  '
                f'angles={self.angle_rows}  '
                f'events={self.event_rows}')

    def destroy_node(self):
        if self.logging_active:
            self.stop_logging()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LoggerNode()
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

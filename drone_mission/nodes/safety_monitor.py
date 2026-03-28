#!/usr/bin/env python3
"""
safety_monitor.py — Independent safety watchdog
Runs completely independently of mission_manager.
Two-zone geofence, battery, altitude, comms, velocity checks.
Publishes /safety/status and /mission/abort.
Directly commands RTL/LAND via MAVROS as backup.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
from datetime import datetime

from drone_mission_msgs.msg import DroneState, MissionEvent
from std_msgs.msg import String
from mavros_msgs.srv import SetMode

# ── Field polygons (lat, lon) ─────────────────────────
# Outer boundary — full airfield — hard RTL if breached
OUTER_POLYGON = [
    (30.379658, -91.221833),
    (30.378975, -91.216805),
    (30.382699, -91.214602),
    (30.383833, -91.216994),
]

# Inner boundary — clean landing field — warn if breached
INNER_POLYGON = [
    (30.382380, -91.217792),
    (30.381219, -91.219055),
    (30.381040, -91.218819),
    (30.382152, -91.217575),
]

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)


def point_in_polygon(lat, lon, polygon):
    """
    Ray casting algorithm for point-in-polygon.
    polygon: list of (lat, lon) tuples
    Returns True if point is inside polygon.
    """
    n      = len(polygon)
    inside = False
    x, y   = lon, lat
    px, py = polygon[-1][1], polygon[-1][0]
    for i in range(n):
        cx, cy = polygon[i][1], polygon[i][0]
        if ((cy > y) != (py > y)) and \
           (x < (px - cx) * (y - cy) / (py - cy) + cx):
            inside = not inside
        px, py = cx, cy
    return inside


def polygon_centroid(polygon):
    """Return centroid (lat, lon) of polygon."""
    lat = sum(p[0] for p in polygon) / len(polygon)
    lon = sum(p[1] for p in polygon) / len(polygon)
    return lat, lon


def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in metres between two GPS points."""
    R    = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a    = (math.sin(dphi / 2) ** 2 +
            math.cos(phi1) * math.cos(phi2) *
            math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # ── Parameters ────────────────────────────────
        self.declare_parameter('test_mode',            False)
        self.declare_parameter('min_battery_voltage',  21.0)
        self.declare_parameter('warn_battery_voltage', 22.2)
        self.declare_parameter('max_altitude_m',       120.0)
        self.declare_parameter('warn_altitude_m',      100.0)
        self.declare_parameter('max_velocity_ms',      12.0)
        self.declare_parameter('comms_timeout_s',      2.0)
        self.declare_parameter('check_rate_hz',        10.0)
        self.declare_parameter('abort_on_mode_change', True)
        self.declare_parameter('geofence_enabled',     True)

        self.test_mode       = self.get_parameter('test_mode').value
        self.min_batt        = self.get_parameter('min_battery_voltage').value
        self.warn_batt       = self.get_parameter('warn_battery_voltage').value
        self.max_alt         = self.get_parameter('max_altitude_m').value
        self.warn_alt        = self.get_parameter('warn_altitude_m').value
        self.max_vel         = self.get_parameter('max_velocity_ms').value
        self.comms_timeout   = self.get_parameter('comms_timeout_s').value
        self.abort_mode_chg  = self.get_parameter('abort_on_mode_change').value
        self.geo_enabled     = self.get_parameter('geofence_enabled').value
        check_rate           = self.get_parameter('check_rate_hz').value

        # ── State ─────────────────────────────────────
        self._drone          = None
        self._last_state_t   = None
        self._mission_active = False
        self._guided_mode    = None   # mode when mission started
        self._armed_last     = False
        self._abort_sent     = False

        # Warning flags (to avoid spamming)
        self._warn_inner     = False
        self._warn_alt       = False
        self._warn_batt      = False
        self._warn_vel       = False

        # ── Publishers ────────────────────────────────
        self.abort_pub = self.create_publisher(
            String, '/mission/abort', RELIABLE_QOS)

        self.status_pub = self.create_publisher(
            String, '/safety/status', RELIABLE_QOS)

        self.event_pub = self.create_publisher(
            MissionEvent, '/mission/event', RELIABLE_QOS)

        # ── Subscriptions ─────────────────────────────
        self.create_subscription(
            DroneState, '/drone/state',
            self._state_cb, SENSOR_QOS)

        self.create_subscription(
            String, '/mission/active',
            self._mission_active_cb, RELIABLE_QOS)

        # ── MAVROS service for direct RTL/LAND ────────
        self.mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # ── Check timer ───────────────────────────────
        self.create_timer(1.0 / check_rate, self._check)

        if self.test_mode:
            self.get_logger().warn(
                '*** TEST MODE — geofence and abort DISABLED ***')

        self.get_logger().info(
            f'Safety monitor started  '
            f'batt_min={self.min_batt}V  '
            f'alt_max={self.max_alt}m  '
            f'vel_max={self.max_vel}m/s  '
            f'geo={self.geo_enabled}  '
            f'test={self.test_mode}')

    # ── Callbacks ─────────────────────────────────────
    def _state_cb(self, msg):
        self._drone        = msg
        self._last_state_t = self.get_clock().now()

        # Track arm state — enable geofence when armed
        if msg.armed and not self._armed_last:
            self.get_logger().info('Drone ARMED — safety checks active')
            self._abort_sent = False
        if not msg.armed and self._armed_last:
            self.get_logger().info('Drone DISARMED — geofence suspended')
            self._reset_warnings()
        self._armed_last = msg.armed

        # Track guided mode at mission start
        if self._mission_active and self._guided_mode is None:
            self._guided_mode = msg.mode

    def _mission_active_cb(self, msg):
        active = msg.data.lower() == 'true'
        if active and not self._mission_active:
            self._mission_active = True
            self._guided_mode    = None
            self._abort_sent     = False
            self.get_logger().info('Mission ACTIVE — full safety monitoring')
        elif not active and self._mission_active:
            self._mission_active = False
            self._guided_mode    = None
            self.get_logger().info('Mission INACTIVE')

    # ── Main safety check ─────────────────────────────
    def _check(self):
        if self.test_mode:
            self._publish_status('TEST_MODE')
            return

        if self._drone is None:
            self._publish_status('NO_DATA')
            return

        d = self._drone

        # ── Comms timeout ─────────────────────────────
        if self._last_state_t is not None:
            dt = (self.get_clock().now() -
                  self._last_state_t).nanoseconds / 1e9
            if dt > self.comms_timeout:
                self._abort(f'COMMS_TIMEOUT {dt:.1f}s', direct_rtl=True)
                return

        # ── Only check geofence/flight when armed ─────
        if not d.armed:
            self._publish_status('DISARMED_OK')
            return

        # ── Battery ───────────────────────────────────
        if d.battery_voltage > 5.0:   # 0 = no data
            if d.battery_voltage <= self.min_batt:
                self._abort(
                    f'BATTERY_CRITICAL {d.battery_voltage:.2f}V',
                    direct_rtl=True)
                return
            elif d.battery_voltage <= self.warn_batt:
                if not self._warn_batt:
                    self._warn('BATTERY_LOW',
                               f'{d.battery_voltage:.2f}V < '
                               f'{self.warn_batt}V')
                    self._warn_batt = True
            else:
                self._warn_batt = False

        # ── Altitude ──────────────────────────────────
        alt = abs(d.alt_baro) if d.alt_baro != 0 else abs(d.z)
        if alt >= self.max_alt:
            self._abort(
                f'ALTITUDE_MAX {alt:.1f}m >= {self.max_alt}m',
                direct_rtl=True)
            return
        elif alt >= self.warn_alt:
            if not self._warn_alt:
                self._warn('ALTITUDE_HIGH',
                           f'{alt:.1f}m > {self.warn_alt}m')
                self._warn_alt = True
        else:
            self._warn_alt = False

        # ── Velocity ──────────────────────────────────
        spd = math.sqrt(d.vx**2 + d.vy**2 + d.vz**2)
        if spd > self.max_vel:
            if not self._warn_vel:
                self._warn('VELOCITY_HIGH',
                           f'{spd:.1f}m/s > {self.max_vel}m/s')
                self._warn_vel = True
        else:
            self._warn_vel = False

        # ── Mode change during mission ─────────────────
        if (self._mission_active and
                self.abort_mode_chg and
                self._guided_mode is not None and
                d.mode != self._guided_mode):
            self._abort(
                f'MODE_CHANGE {self._guided_mode}→{d.mode}',
                direct_rtl=False)
            return

        # ── Geofence ──────────────────────────────────
        if self.geo_enabled and d.gps_fix >= 0:
            self._check_geofence(d.lat, d.lon)

        self._publish_status('OK')

    def _check_geofence(self, lat, lon):
        # Skip if no valid GPS
        if lat == 0.0 and lon == 0.0:
            return

        in_outer = point_in_polygon(lat, lon, OUTER_POLYGON)
        in_inner = point_in_polygon(lat, lon, INNER_POLYGON)

        # Outside outer polygon — hard RTL
        if not in_outer:
            dist_to_inner = haversine_m(
                lat, lon,
                *polygon_centroid(INNER_POLYGON))
            self._abort(
                f'GEOFENCE_OUTER_BREACH '
                f'lat={lat:.6f} lon={lon:.6f} '
                f'dist_to_inner={dist_to_inner:.0f}m',
                direct_rtl=True)
            return

        # Outside inner polygon — warn
        if not in_inner:
            if not self._warn_inner:
                self._warn(
                    'GEOFENCE_INNER_BREACH',
                    f'Outside clean field — '
                    f'lat={lat:.6f} lon={lon:.6f}')
                self._warn_inner = True
        else:
            if self._warn_inner:
                self.get_logger().info(
                    'Geofence: back inside clean field')
            self._warn_inner = False

    # ── Abort and warn helpers ─────────────────────────
    def _abort(self, reason, direct_rtl=False):
        if self._abort_sent:
            return

        self.get_logger().error(f'SAFETY ABORT: {reason}')
        self._abort_sent = True

        # Publish abort to mission_manager
        msg = String()
        msg.data = reason
        self.abort_pub.publish(msg)

        # Log event
        self._pub_event('SAFETY_ABORT', reason)

        # Directly command RTL via MAVROS as backup
        if direct_rtl and self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'RTL'
            future = self.mode_client.call_async(req)
            self.get_logger().error(f'Direct RTL commanded — {reason}')
        elif not direct_rtl and self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'LOITER'
            future = self.mode_client.call_async(req)
            self.get_logger().warn(f'Direct LOITER commanded — {reason}')

    def _warn(self, warn_type, detail):
        self.get_logger().warn(f'SAFETY WARNING [{warn_type}]: {detail}')
        self._pub_event('SAFETY_WARN', f'{warn_type}: {detail}')
        self._publish_status(f'WARN_{warn_type}')

    def _pub_event(self, event_type, description):
        msg = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event_type   = event_type
        msg.description  = description
        msg.mission_time = 0.0
        msg.value        = 0.0
        self.event_pub.publish(msg)

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _reset_warnings(self):
        self._warn_inner = False
        self._warn_alt   = False
        self._warn_batt  = False
        self._warn_vel   = False
        self._abort_sent = False


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SafetyMonitor()
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

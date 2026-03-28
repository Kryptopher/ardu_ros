#!/usr/bin/env python3
"""
hardware_bridge.py — MAVROS abstraction layer
Subscribes to all MAVROS topics, publishes clean /drone/state at 50Hz
This is the ONLY node that touches MAVROS directly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

# MAVROS message types
from mavros_msgs.msg import State, RCIn, RCOut
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped

# Custom message
from drone_mission_msgs.msg import DroneState

# QoS for sensor topics
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)


def quat_to_rpy_deg(q):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    # Roll
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr, cosr))
    # Pitch
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # Yaw
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')

        # ── Raw data buckets ──────────────────────────
        self._odom         = None
        self._vel_body     = None
        self._imu          = None
        self._imu_raw      = None
        self._gps          = None
        self._gps_local    = None
        self._gps_sats     = None
        self._vfr          = None
        self._nav_ctrl     = None
        self._wind         = None
        self._rc_out       = None
        self._battery      = None
        self._state        = None
        self._mag          = None
        self._estimator    = None

        # ── Subscriptions ─────────────────────────────
        self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self._odom_cb, SENSOR_QOS)

        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_body',
            self._vel_body_cb, SENSOR_QOS)

        self.create_subscription(
            Imu, '/mavros/imu/data',
            self._imu_cb, SENSOR_QOS)

        self.create_subscription(
            Imu, '/mavros/imu/data_raw',
            self._imu_raw_cb, SENSOR_QOS)

        self.create_subscription(
            Imu, '/mavros/imu/mag',
            self._mag_cb, SENSOR_QOS)

        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._gps_cb, SENSOR_QOS)

        self.create_subscription(
            Odometry, '/mavros/global_position/local',
            self._gps_local_cb, SENSOR_QOS)

        self.create_subscription(
            BatteryState, '/mavros/battery',
            self._battery_cb, SENSOR_QOS)

        self.create_subscription(
            RCOut, '/mavros/rc/out',
            self._rcout_cb, SENSOR_QOS)

        self.create_subscription(
            State, '/mavros/state',
            self._state_cb, RELIABLE_QOS)

        # VfrHud
        try:
            from mavros_msgs.msg import VfrHud
            self.create_subscription(
                VfrHud, '/mavros/vfr_hud',
                self._vfr_cb, SENSOR_QOS)
        except Exception:
            self.get_logger().warn('VfrHud not available')

        # Nav controller
        try:
            from mavros_msgs.msg import NavControllerOutput
            self.create_subscription(
                NavControllerOutput, '/mavros/nav_controller_output/output',
                self._nav_ctrl_cb, SENSOR_QOS)
        except Exception:
            self.get_logger().warn('NavControllerOutput not available')

        # Wind estimation
        try:
            from geometry_msgs.msg import TwistWithCovarianceStamped
            self.create_subscription(
                TwistWithCovarianceStamped, '/mavros/wind_estimation',
                self._wind_cb, SENSOR_QOS)
        except Exception:
            self.get_logger().warn('Wind estimation not available')

        # Estimator status
        try:
            from mavros_msgs.msg import EstimatorStatus
            self.create_subscription(
                EstimatorStatus, '/mavros/estimator_status',
                self._estimator_cb, SENSOR_QOS)
        except Exception:
            self.get_logger().warn('EstimatorStatus not available')

        # ── Publisher ─────────────────────────────────
        self.pub = self.create_publisher(
            DroneState, '/drone/state', SENSOR_QOS)

        # ── Publish timer at 50Hz ─────────────────────
        self.create_timer(0.02, self._publish)

        # ── Diagnostics counter ───────────────────────
        self._pub_count = 0

        self.get_logger().info('Hardware bridge started — publishing /drone/state at 50Hz')

    # ── MAVROS callbacks ──────────────────────────────
    def _odom_cb(self, msg):
        self._odom = msg

    def _vel_body_cb(self, msg):
        self._vel_body = msg

    def _imu_cb(self, msg):
        self._imu = msg

    def _imu_raw_cb(self, msg):
        self._imu_raw = msg

    def _mag_cb(self, msg):
        self._mag = msg

    def _gps_cb(self, msg):
        self._gps = msg

    def _gps_local_cb(self, msg):
        self._gps_local = msg

    def _battery_cb(self, msg):
        self._battery = msg

    def _rcout_cb(self, msg):
        self._rc_out = msg

    def _state_cb(self, msg):
        self._state = msg

    def _vfr_cb(self, msg):
        self._vfr = msg

    def _nav_ctrl_cb(self, msg):
        self._nav_ctrl = msg

    def _wind_cb(self, msg):
        self._wind = msg

    def _estimator_cb(self, msg):
        self._estimator = msg

    # ── Main publish callback ─────────────────────────
    def _publish(self):
        msg = DroneState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # ── Position + velocity (NED) ─────────────────
        if self._odom:
            p = self._odom.pose.pose.position
            v = self._odom.twist.twist.linear
            msg.x  = p.x;  msg.y  = p.y;  msg.z  = p.z
            msg.vx = v.x;  msg.vy = v.y;  msg.vz = v.z

            # Attitude from odom quaternion
            q = self._odom.pose.pose.orientation
            msg.roll, msg.pitch, msg.yaw = quat_to_rpy_deg(q)

        # ── Body frame velocity ───────────────────────
        if self._vel_body:
            v = self._vel_body.twist.linear
            msg.vx_body = v.x
            msg.vy_body = v.y
            msg.vz_body = v.z

        # ── IMU fused (body rates + accel) ───────────
        if self._imu:
            av = self._imu.angular_velocity
            la = self._imu.linear_acceleration
            msg.p  = math.degrees(av.x)
            msg.q  = math.degrees(av.y)
            msg.r  = math.degrees(av.z)
            msg.ax = la.x
            msg.ay = la.y
            msg.az = la.z

        # ── Raw IMU accel ─────────────────────────────
        if self._imu_raw:
            la = self._imu_raw.linear_acceleration
            msg.ax_raw = la.x
            msg.ay_raw = la.y
            msg.az_raw = la.z

        # ── Magnetometer ──────────────────────────────
        if self._mag:
            mf = self._mag.magnetic_field
            msg.mag_x = mf.x
            msg.mag_y = mf.y
            msg.mag_z = mf.z

        # ── GPS ───────────────────────────────────────
        if self._gps:
            msg.lat     = self._gps.latitude
            msg.lon     = self._gps.longitude
            msg.alt_gps = self._gps.altitude
            msg.gps_fix = self._gps.status.status

        if self._gps_local:
            cov = self._gps_local.pose.covariance
            # cov[0] = variance in x, cov[7] = variance in y
            if cov[0] > 0:
                msg.gps_pos_std = math.sqrt((cov[0] + cov[7]) / 2.0)

        # ── VFR HUD ───────────────────────────────────
        if self._vfr:
            msg.heading     = float(self._vfr.heading)
            msg.groundspeed = self._vfr.groundspeed
            msg.airspeed    = self._vfr.airspeed
            msg.alt_baro    = self._vfr.altitude
            msg.climb_rate  = self._vfr.climb
            msg.throttle    = self._vfr.throttle

        # ── Nav controller ────────────────────────────
        if self._nav_ctrl:
            msg.nav_roll_cmd  = self._nav_ctrl.nav_roll
            msg.nav_pitch_cmd = self._nav_ctrl.nav_pitch

        # ── Wind ──────────────────────────────────────
        if self._wind:
            w = self._wind.twist.twist.linear
            msg.wind_x = w.x
            msg.wind_y = w.y
            msg.wind_z = w.z

        # ── RC outputs ────────────────────────────────
        if self._rc_out and len(self._rc_out.channels) >= 4:
            ch = self._rc_out.channels
            msg.motor1 = ch[0]
            msg.motor2 = ch[1]
            msg.motor3 = ch[2]
            msg.motor4 = ch[3]

        # ── Battery ───────────────────────────────────
        if self._battery:
            msg.battery_voltage = self._battery.voltage
            msg.battery_current = self._battery.current
            msg.battery_pct     = self._battery.percentage

        # ── Flight mode ───────────────────────────────
        if self._state:
            msg.mode   = self._state.mode
            msg.armed  = self._state.armed
            msg.guided = self._state.guided

        # ── EKF health ────────────────────────────────
        if self._estimator:
            msg.ekf_ok = bool(
                self._estimator.pos_horiz_abs and
                self._estimator.attitude)
            msg.pos_horiz_accuracy = getattr(
                self._estimator, 'pos_horiz_accuracy', 0.0)
            msg.pos_vert_accuracy  = getattr(
                self._estimator, 'pos_vert_accuracy', 0.0)
        else:
            # No estimator status — assume OK if we have odom
            msg.ekf_ok = self._odom is not None

        self.pub.publish(msg)

        # ── Diagnostics at 1Hz ────────────────────────
        self._pub_count += 1
        if self._pub_count % 50 == 0:
            self.get_logger().info(
                f'pos=({msg.x:.1f},{msg.y:.1f},{msg.z:.1f})  '
                f'rpy=({msg.roll:.1f},{msg.pitch:.1f},{msg.yaw:.1f})  '
                f'mode={msg.mode}  armed={msg.armed}  '
                f'batt={msg.battery_voltage:.1f}V  '
                f'ekf={msg.ekf_ok}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HardwareBridge()
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

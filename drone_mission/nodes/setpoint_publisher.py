#!/usr/bin/env python3
"""
setpoint_publisher.py — Control loop node
Receives commands from mission_manager via /mission/command
Handles: pos hold, vel step, vel trap, vel shaped (ZV/ZVD), trajectory
Publishes to /mavros/setpoint_raw/local at 50Hz
Frame conversion: body → NED using current yaw
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import json
import time

from mavros_msgs.msg import PositionTarget
from drone_mission_msgs.msg import DroneState
from std_msgs.msg import String

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

# ── ArduPilot type masks ───────────────────────────────
MASK_POS_ONLY = (
    PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
    PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
    PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW_RATE)

MASK_POS_VEL_ACC = PositionTarget.IGNORE_YAW_RATE


class SetpointPublisher(Node):
    def __init__(self):
        super().__init__('setpoint_publisher')

        # ── State ─────────────────────────────────────
        self.drone        = None
        self.current_cmd  = None
        self.cmd_start_t  = None
        self.shaped_prof  = None   # pre-computed shaper profile
        # Expected position (integrated from velocity commands)
        self.exp_x = self.exp_y = self.exp_z = 0.0

        # ── Publishers ────────────────────────────────
        self.raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local', 10)

        # ── Subscriptions ─────────────────────────────
        self.create_subscription(
            DroneState, '/drone/state',
            self._drone_cb, SENSOR_QOS)

        self.create_subscription(
            String, '/mission/command',
            self._cmd_cb, RELIABLE_QOS)

        # ── 50Hz control timer ────────────────────────
        self.create_timer(0.02, self._publish)

        self.get_logger().info(
            'Setpoint publisher started — 50Hz')

    # ── Callbacks ─────────────────────────────────────
    def _drone_cb(self, msg):
        self.drone = msg

    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Bad command JSON: {e}')
            return

        # Only recompute shaper if command changed
        if (self.current_cmd is None or
                cmd['mode'] != self.current_cmd.get('mode') or
                cmd['t_start'] != self.current_cmd.get('t_start')):

            self.current_cmd = cmd
            self.cmd_start_t = self.get_clock().now()

            if cmd['mode'] == 'vel':
                self._precompute_profile(cmd)
                # Snapshot current position for integration
                if self.drone:
                    self.exp_x = self.drone.x
                    self.exp_y = self.drone.y
                    self.exp_z = self.drone.z

            self.get_logger().info(
                f"New command: {cmd['mode']} "
                f"{cmd['profile']} "
                f"{cmd['frame']} "
                f"vel=({cmd['vx']:.2f},"
                f"{cmd['vy']:.2f},"
                f"{cmd['vz']:.2f})")

    # ── Profile precomputation ─────────────────────────
    def _precompute_profile(self, cmd):
        duration = cmd['t_end'] - cmd['t_start']
        vx = cmd['vx']; vy = cmd['vy']; vz = cmd['vz']
        ax = cmd['ax']
        profile  = cmd['profile']
        shaper   = cmd['shaper']

        # Build base velocity profile
        samples = self._build_profile(
            vx, vy, vz, ax, duration, profile)

        # Apply input shaper if needed
        if shaper in ('ZV', 'ZVD'):
            impulses = self._compute_impulses(
                cmd['rope_length'],
                cmd['damping'],
                shaper)
            samples = self._convolve(samples, impulses)

        self.shaped_prof = samples

    def _build_profile(self, vx, vy, vz, ax,
                       duration, profile, dt=0.02):
        samples = []
        t = 0.0
        if profile == 'step':
            while t <= duration + 1e-9:
                samples.append((t, vx, vy, vz))
                t += dt
        elif profile == 'trap':
            speed  = math.sqrt(vx**2 + vy**2 + vz**2)
            t_ramp = (speed / ax) if ax > 1e-6 else 0.0
            while t <= duration + 1e-9:
                if t < t_ramp:
                    s = t / t_ramp
                elif t > duration - t_ramp:
                    s = (duration - t) / t_ramp
                else:
                    s = 1.0
                s = max(0.0, min(1.0, s))
                samples.append((t, vx*s, vy*s, vz*s))
                t += dt
        else:
            while t <= duration + 1e-9:
                samples.append((t, vx, vy, vz))
                t += dt
        return samples

    def _compute_impulses(self, rope_length, damping, shaper):
        wn   = math.sqrt(9.81 / rope_length)
        zeta = damping
        wd   = wn * math.sqrt(1.0 - zeta**2)
        T_d  = math.pi / wd
        K    = math.exp(
            -zeta * math.pi / math.sqrt(1.0 - zeta**2))
        if shaper == 'ZV':
            A1 = 1.0 / (1.0 + K)
            A2 = K   / (1.0 + K)
            return [(A1, 0.0), (A2, T_d)]
        else:  # ZVD
            K2    = K * K
            denom = 1.0 + 2*K + K2
            return [(1.0/denom, 0.0),
                    (2*K/denom, T_d),
                    (K2/denom,  2*T_d)]

    def _convolve(self, samples, impulses, dt=0.02):
        t_end  = samples[-1][0] + impulses[-1][1]
        shaped = []
        t = 0.0
        while t <= t_end + 1e-9:
            svx = svy = svz = 0.0
            for (amp, t_imp) in impulses:
                v    = self._interp(samples, t - t_imp)
                svx += amp * v[0]
                svy += amp * v[1]
                svz += amp * v[2]
            shaped.append((t, svx, svy, svz))
            t += dt
        return shaped

    def _interp(self, samples, t):
        if t <= samples[0][0]:  return samples[0][1:]
        if t >= samples[-1][0]: return samples[-1][1:]
        for i in range(len(samples)-1):
            t0, t1 = samples[i][0], samples[i+1][0]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0)
                return tuple(
                    samples[i][j+1] +
                    a*(samples[i+1][j+1] - samples[i][j+1])
                    for j in range(3))
        return (0.0, 0.0, 0.0)

    # ── Frame conversion ──────────────────────────────
    def _body_to_ned(self, vx_b, vy_b):
        """Rotate body frame velocity to NED using current yaw."""
        if self.drone is None:
            return vx_b, vy_b
        yaw = math.radians(self.drone.yaw)
        vx_ned = vx_b * math.cos(yaw) - vy_b * math.sin(yaw)
        vy_ned = vx_b * math.sin(yaw) + vy_b * math.cos(yaw)
        return vx_ned, vy_ned

    # ── Main publish callback ─────────────────────────
    def _publish(self):
        if self.current_cmd is None or self.drone is None:
            return

        cmd  = self.current_cmd
        mode = cmd['mode']

        if mode == 'pos':
            self._publish_pos(cmd)
        elif mode == 'vel':
            self._publish_vel(cmd)
        elif mode == 'traj':
            self._publish_traj(cmd)

    def _publish_pos(self, cmd):
        """Position hold — send absolute NED position setpoint."""
        px = cmd['x']
        py = cmd['y']
        pz = cmd['z']

        # If body frame, convert offset to NED
        if cmd['frame'] == 'body' and self.drone:
            ox, oy = self._body_to_ned(
                cmd['x'] - (self.drone.x if self.drone else 0),
                cmd['y'] - (self.drone.y if self.drone else 0))
            # For hold (x=0,y=0,z=0) just hold current pos
            if cmd.get('vx', 0) == 0 and cmd.get('vy', 0) == 0:
                px = self.drone.x
                py = self.drone.y
                pz = cmd['z'] if cmd['z'] != 0 else self.drone.z

        self._pub_raw_pos(px, py, pz)

    def _publish_vel(self, cmd):
        """Velocity command with optional shaper."""
        if self.cmd_start_t is None:
            return

        elapsed = (self.get_clock().now() -
                   self.cmd_start_t).nanoseconds / 1e9
        dt      = 0.02

        if self.shaped_prof:
            v_now  = self._interp(self.shaped_prof, elapsed)
            v_prev = self._interp(
                self.shaped_prof, max(0.0, elapsed - dt))
        else:
            v_now  = (cmd['vx'], cmd['vy'], cmd['vz'])
            v_prev = v_now

        svx, svy, svz = v_now
        sax = (v_now[0] - v_prev[0]) / dt
        say = (v_now[1] - v_prev[1]) / dt
        saz = (v_now[2] - v_prev[2]) / dt

        # Frame conversion
        if cmd['frame'] == 'body':
            svx, svy = self._body_to_ned(svx, svy)
            sax, say = self._body_to_ned(sax, say)

        # Zero velocity → hold position
        if (abs(svx) < 0.001 and
                abs(svy) < 0.001 and
                abs(svz) < 0.001):
            if self.drone:
                self._pub_raw_pos(
                    self.drone.x,
                    self.drone.y,
                    self.drone.z)
            return

        # Integrate expected position
        self.exp_x += svx * dt
        self.exp_y += svy * dt
        self.exp_z += svz * dt

        self._pub_raw_pos_vel_acc(
            self.exp_x, self.exp_y, self.exp_z,
            svx, svy, svz,
            sax, say, saz)

    def _publish_traj(self, cmd):
        """
        Trajectory following — geometric or waypoint.
        vx=radius, vy=0, vz=period for circle/figure8
        """
        if self.cmd_start_t is None:
            return

        elapsed = (self.get_clock().now() -
                   self.cmd_start_t).nanoseconds / 1e9
        profile = cmd['profile']
        cx = cmd['x']; cy = cmd['y']; cz = cmd['z']
        radius = cmd['vx']
        period = cmd['vz'] if cmd['vz'] > 0 else 20.0
        omega  = 2.0 * math.pi / period

        if profile == 'circle':
            px = cx + radius * math.cos(omega * elapsed)
            py = cy + radius * math.sin(omega * elapsed)
            pz = cz
            # Feedforward velocity
            vx = -radius * omega * math.sin(omega * elapsed)
            vy =  radius * omega * math.cos(omega * elapsed)
            vz = 0.0

        elif profile == 'figure8':
            # Lemniscate of Bernoulli
            scale = radius
            denom = 1.0 + math.sin(omega * elapsed)**2
            px = cx + scale * math.cos(omega * elapsed) / denom
            py = cy + scale * math.sin(omega * elapsed) * \
                 math.cos(omega * elapsed) / denom
            pz = cz
            vx = 0.0; vy = 0.0; vz = 0.0  # FF todo

        elif profile == 'lemniscate':
            # Same as figure8 but scaled differently
            a  = radius
            t  = omega * elapsed
            px = cx + a * math.cos(t) / (1 + math.sin(t)**2)
            py = cy + a * math.sin(t) * math.cos(t) / \
                 (1 + math.sin(t)**2)
            pz = cz
            vx = 0.0; vy = 0.0; vz = 0.0

        else:
            # Hold center
            px = cx; py = cy; pz = cz
            vx = 0.0; vy = 0.0; vz = 0.0

        self._pub_raw_pos_vel_acc(
            px, py, pz, vx, vy, vz, 0, 0, 0)

    # ── MAVROS publish helpers ─────────────────────────
    def _pub_raw_pos(self, px, py, pz):
        msg = PositionTarget()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask        = MASK_POS_ONLY
        msg.position.x = px
        msg.position.y = py
        msg.position.z = pz
        self.raw_pub.publish(msg)

    def _pub_raw_pos_vel_acc(self, px, py, pz,
                              vx, vy, vz,
                              ax, ay, az):
        msg = PositionTarget()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask        = MASK_POS_VEL_ACC
        msg.position.x = px; msg.position.y = py
        msg.position.z = pz
        msg.velocity.x = vx; msg.velocity.y = vy
        msg.velocity.z = vz
        msg.acceleration_or_force.x = ax
        msg.acceleration_or_force.y = ay
        msg.acceleration_or_force.z = az
        self.raw_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SetpointPublisher()
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

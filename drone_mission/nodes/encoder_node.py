#!/usr/bin/env python3
"""
encoder_node.py — Dual quadrature encoder ROS2 node
Publishes /payload/angles at 200Hz
Pins: ENC1 (pitch) = GPIO6/13, ENC2 (roll) = GPIO19/26
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from drone_mission_msgs.msg import PayloadAngles
import pigpio
import time

# ── Pin assignments ───────────────────────────────────
ENC1_A = 6    # pitch A (white)
ENC1_B = 13   # pitch B (green)
ENC2_A = 19   # roll A
ENC2_B = 26   # roll B

# ── Encoder config ────────────────────────────────────
PPR            = 1000
COUNT_MODE     = 4
GEAR_RATIO     = 1.0
COUNTS_PER_REV = PPR * COUNT_MODE * GEAR_RATIO
DEG_PER_COUNT  = 360.0 / COUNTS_PER_REV
MIN_PULSE_US   = 300

# ── Quadrature decode table ───────────────────────────
QUAD_TABLE = {
    (0b00, 0b01): +1, (0b01, 0b11): +1,
    (0b11, 0b10): +1, (0b10, 0b00): +1,
    (0b00, 0b10): -1, (0b10, 0b11): -1,
    (0b11, 0b01): -1, (0b01, 0b00): -1,
}


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # ── Parameters (overridable from launch/yaml) ─
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('enc1_a', ENC1_A)
        self.declare_parameter('enc1_b', ENC1_B)
        self.declare_parameter('enc2_a', ENC2_A)
        self.declare_parameter('enc2_b', ENC2_B)
        self.declare_parameter('ppr', PPR)
        self.declare_parameter('count_mode', COUNT_MODE)
        self.declare_parameter('gear_ratio', GEAR_RATIO)
        self.declare_parameter('min_pulse_us', MIN_PULSE_US)
        self.declare_parameter('zero_on_start', True)

        rate        = self.get_parameter('publish_rate_hz').value
        self.pin_A1 = self.get_parameter('enc1_a').value
        self.pin_B1 = self.get_parameter('enc1_b').value
        self.pin_A2 = self.get_parameter('enc2_a').value
        self.pin_B2 = self.get_parameter('enc2_b').value
        ppr         = self.get_parameter('ppr').value
        count_mode  = self.get_parameter('count_mode').value
        gear_ratio  = self.get_parameter('gear_ratio').value
        min_pulse   = self.get_parameter('min_pulse_us').value

        counts_per_rev     = ppr * count_mode * gear_ratio
        self.deg_per_count = 360.0 / counts_per_rev

        # ── State ─────────────────────────────────────
        self.pin_level       = {self.pin_A1: 0, self.pin_B1: 0,
                                self.pin_A2: 0, self.pin_B2: 0}
        self.pitch_count     = 0
        self.roll_count      = 0
        self.last_pitch_state = 0b00
        self.last_roll_state  = 0b00
        self.error_count     = {'pitch': 0, 'roll': 0}
        self.publish_count   = 0

        # ── Publisher ─────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.pub = self.create_publisher(PayloadAngles, '/payload/angles', qos)

        # ── pigpio ────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                'pigpio not running — start with: sudo pigpiod')
            raise RuntimeError('pigpio not connected')

        for pin in [self.pin_A1, self.pin_B1, self.pin_A2, self.pin_B2]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pin_level[pin] = self.pi.read(pin)
            self.pi.set_glitch_filter(pin, min_pulse)

        # Read initial states
        self.last_pitch_state = (
            (self.pin_level[self.pin_A1] << 1) | self.pin_level[self.pin_B1])
        self.last_roll_state = (
            (self.pin_level[self.pin_A2] << 1) | self.pin_level[self.pin_B2])

        # Register callbacks
        self.cb1a = self.pi.callback(
            self.pin_A1, pigpio.EITHER_EDGE, self._pitch_cb)
        self.cb1b = self.pi.callback(
            self.pin_B1, pigpio.EITHER_EDGE, self._pitch_cb)
        self.cb2a = self.pi.callback(
            self.pin_A2, pigpio.EITHER_EDGE, self._roll_cb)
        self.cb2b = self.pi.callback(
            self.pin_B2, pigpio.EITHER_EDGE, self._roll_cb)

        # ── Publish timer ─────────────────────────────
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Encoder node started — '
            f'{rate:.0f}Hz  '
            f'deg/count={self.deg_per_count:.4f}  '
            f'pins pitch=({self.pin_A1},{self.pin_B1}) '
            f'roll=({self.pin_A2},{self.pin_B2})')

    # ── pigpio callbacks (interrupt-driven) ───────────
    def _pitch_cb(self, gpio, level, tick):
        self.pin_level[gpio] = level
        A = self.pin_level[self.pin_A1]
        B = self.pin_level[self.pin_B1]
        new_state = (A << 1) | B
        delta = QUAD_TABLE.get((self.last_pitch_state, new_state), 0)
        if delta == 0 and self.last_pitch_state != new_state:
            self.error_count['pitch'] += 1
        self.pitch_count += delta
        self.last_pitch_state = new_state

    def _roll_cb(self, gpio, level, tick):
        self.pin_level[gpio] = level
        A = self.pin_level[self.pin_A2]
        B = self.pin_level[self.pin_B2]
        new_state = (A << 1) | B
        delta = QUAD_TABLE.get((self.last_roll_state, new_state), 0)
        if delta == 0 and self.last_roll_state != new_state:
            self.error_count['roll'] += 1
        self.roll_count += delta
        self.last_roll_state = new_state

    # ── Publish timer callback ─────────────────────────
    def _publish(self):
        msg = PayloadAngles()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'payload'
        msg.pitch_deg       = self.pitch_count * self.deg_per_count
        msg.roll_deg        = self.roll_count  * self.deg_per_count
        msg.pitch_count     = self.pitch_count
        msg.roll_count      = self.roll_count
        msg.deg_per_count   = self.deg_per_count
        self.pub.publish(msg)

        self.publish_count += 1
        # Log at 1Hz
        if self.publish_count % 200 == 0:
            self.get_logger().info(
                f'pitch={msg.pitch_deg:+.2f}°  '
                f'roll={msg.roll_deg:+.2f}°  '
                f'errors pitch={self.error_count["pitch"]} '
                f'roll={self.error_count["roll"]}')

    def reset_counts(self):
        """Zero both encoders — call before mission start."""
        self.pitch_count = 0
        self.roll_count  = 0
        self.get_logger().info('Encoder counts reset to zero')

    def destroy_node(self):
        self.cb1a.cancel()
        self.cb1b.cancel()
        self.cb2a.cancel()
        self.cb2b.cancel()
        self.pi.stop()
        self.get_logger().info(
            f'Encoder node stopped — '
            f'final errors pitch={self.error_count["pitch"]} '
            f'roll={self.error_count["roll"]}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EncoderNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[encoder_node] Fatal: {e}')
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

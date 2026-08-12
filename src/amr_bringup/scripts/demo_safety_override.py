#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Prove the safety override actually overrides, and measure how fast.

The claim under test is specific: when an obstacle violates the speed-dependent
envelope, a low-level node discards whatever the navigation stack is asking for
and halts the robot. Watching a robot stop in Gazebo does not distinguish that
from nav2 simply planning around the obstacle.

So this script drives the robot with a *deliberately hostile* command stream -
a constant forward velocity published straight into `cmd_vel_nav`, ignoring
every obstacle - and records what actually reaches the base on `cmd_vel`. If
the override works, `cmd_vel` goes to zero while `cmd_vel_nav` is still
demanding full speed. That gap is the override, and it is measurable.

    ros2 run amr_bringup demo_safety_override.py --robot amr1 --speed 0.5

Reported at the end:

* whether `cmd_vel` was ever zeroed while the commanded velocity was non-zero;
* the measured obstacle distance and the envelope `d_safe = k*v^2 + d_min` at
  the moment it happened;
* reaction latency from the sensor's own timestamp, as recorded by the safety
  node itself.

Style: PEP 8, checked by ament_flake8.
"""

import argparse
import sys

import rclpy
from amr_msgs.msg import SafetyStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SafetyProbe(Node):
    """Publishes an unsafe command and watches what the base actually gets."""

    def __init__(self, robot, speed, rate):
        super().__init__('demo_safety_override')
        self.robot = robot
        self.speed = speed

        self.commanded = Twist()
        self.commanded.linear.x = speed

        self.publisher = self.create_publisher(Twist, f'/{robot}/cmd_vel_nav', 10)

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            Twist, f'/{robot}/cmd_vel', self._on_actual, sensor_qos)
        self.create_subscription(
            SafetyStatus, f'/{robot}/safety_status', self._on_status, sensor_qos)

        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.actual_linear = None
        self.override_seen = False
        self.override_record = None
        self.samples = 0
        self.halt_samples = 0
        self.worst_latency_us = 0
        self.latest_status = None

    def _publish(self):
        # Deliberately unconditional: this stream never yields to anything.
        self.publisher.publish(self.commanded)

    def _on_actual(self, message):
        self.actual_linear = message.linear.x
        self.samples += 1
        if abs(message.linear.x) < 1e-6 and self.speed > 1e-6:
            self.halt_samples += 1
            if not self.override_seen and self.latest_status is not None:
                self.override_seen = True
                self.override_record = self.latest_status

    def _on_status(self, message):
        self.latest_status = message
        self.worst_latency_us = max(self.worst_latency_us, message.reaction_latency_us)

    def report(self):
        print()
        print('=' * 72)
        print(f'Safety override probe: {self.robot}')
        print('=' * 72)
        print(f'  commanded on cmd_vel_nav : {self.speed:.2f} m/s, continuously')
        print(f'  cmd_vel samples observed : {self.samples}')
        print(f'  samples forced to zero   : {self.halt_samples}')

        if self.samples == 0:
            print()
            print('  NO DATA. Nothing was publishing cmd_vel. Is the safety override')
            print('  node running for this robot?')
            return 1

        if not self.override_seen:
            print()
            print('  The override never fired. Either nothing came within the envelope,')
            print('  or the override is not in the command path. Try driving toward a')
            print('  rack, or raise --speed to widen d_safe = k*v^2 + d_min.')
            return 1

        record = self.override_record
        reason = {
            SafetyStatus.REASON_NONE: 'none',
            SafetyStatus.REASON_OBSTACLE: 'obstacle',
            SafetyStatus.REASON_SENSOR_INVALID: 'sensor invalid',
            SafetyStatus.REASON_SENSOR_TIMEOUT: 'sensor timeout',
            SafetyStatus.REASON_MANUAL_OVERRIDE: 'manual override',
        }.get(record.reason, str(record.reason))

        print()
        print('  OVERRIDE CONFIRMED: cmd_vel was zeroed while cmd_vel_nav still')
        print('  demanded full speed. The navigation command was discarded, not obeyed.')
        print()
        print(f'    reason               : {reason}')
        print(f'    speed at trigger     : {record.current_speed:.3f} m/s')
        print(f'    nearest obstacle     : {record.min_obstacle_distance:.3f} m')
        print(f'    envelope d_safe      : {record.safe_distance:.3f} m')
        print(f'    margin violated by   : '
              f'{record.safe_distance - record.min_obstacle_distance:.3f} m')
        print(f'    worst reaction latency: {self.worst_latency_us / 1000.0:.1f} ms '
              f'(sensor timestamp to override publication)')
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', default='amr1', help='Robot namespace.')
    parser.add_argument(
        '--speed', type=float, default=0.5,
        help='Forward velocity to demand, m/s. Higher widens the envelope.')
    parser.add_argument('--rate', type=float, default=20.0, help='Publish rate, Hz.')
    parser.add_argument(
        '--duration', type=float, default=25.0, help='Seconds to run.')
    args = parser.parse_args(argv)

    rclpy.init()
    node = SafetyProbe(args.robot, args.speed, args.rate)
    node.get_logger().warn(
        f'driving {args.robot} at {args.speed} m/s with NO obstacle avoidance. '
        f'Only the safety override stands between it and the racks - which is '
        f'the point of the test.')
    try:
        start = node.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds * 1e-9 - start > args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave the robot stopped.
        node.publisher.publish(Twist())
        status = node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())

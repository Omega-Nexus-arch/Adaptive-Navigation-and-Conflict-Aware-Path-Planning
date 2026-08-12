#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Force the narrow-intersection conflict and record who yields.

Sends both robots through The Pinch - the 2.0 m doorway in the central
firewall, wide enough for one robot and not for two - from opposite sides at
the same moment. The yielding protocol has to resolve it.

    ros2 run amr_bringup demo_conflict.py

The script watches `/fleet/traffic_directives` and `/fleet/trajectories`
throughout and prints a timeline, so the outcome is evidence rather than
assertion:

* which robot was told to yield, and on what grounds;
* how long it was held;
* the minimum centre-to-centre separation actually achieved.

Exit status is non-zero if the wrong robot yielded, if neither did, or if the
two came closer than their combined footprints, which makes this usable as a
regression check on the protocol.

Style: PEP 8, checked by ament_flake8.
"""

import argparse
import math
import os
import subprocess
import sys

import rclpy
from amr_msgs.msg import PredictedTrajectory, TrafficDirective
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml

ACTION_NAMES = {
    TrafficDirective.ACTION_PROCEED: 'PROCEED',
    TrafficDirective.ACTION_SLOW: 'SLOW',
    TrafficDirective.ACTION_YIELD: 'YIELD',
    TrafficDirective.ACTION_HOLD: 'HOLD',
}


class ConflictWatcher(Node):
    """Records directives and inter-robot separation during the run."""

    def __init__(self, expected_yielder, expected_winner):
        super().__init__('demo_conflict')
        self.expected_yielder = expected_yielder
        self.expected_winner = expected_winner

        self.timeline = []
        self.last_action = {}
        self.yield_seconds = {}
        self.yield_started = {}
        self.positions = {}
        self.min_separation = float('inf')
        self.start = self.get_clock().now().nanoseconds * 1e-9

        self.create_subscription(
            TrafficDirective, '/fleet/traffic_directives', self._on_directive, 20)
        self.create_subscription(
            PredictedTrajectory, '/fleet/trajectories', self._on_trajectory, 20)

    def _elapsed(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.start

    def _on_directive(self, message):
        now = self._elapsed()
        previous = self.last_action.get(message.robot_id)
        if previous == message.action:
            return
        self.last_action[message.robot_id] = message.action

        name = ACTION_NAMES.get(message.action, str(message.action))
        self.timeline.append((now, message.robot_id, name, message.reason))
        self.get_logger().info(f'[{now:6.2f}s] {message.robot_id}: {name} - {message.reason}')

        if message.action == TrafficDirective.ACTION_YIELD:
            self.yield_started[message.robot_id] = now
        elif message.robot_id in self.yield_started:
            held = now - self.yield_started.pop(message.robot_id)
            self.yield_seconds[message.robot_id] = \
                self.yield_seconds.get(message.robot_id, 0.0) + held

    def _on_trajectory(self, message):
        if not message.points:
            return
        first = message.points[0].pose.position
        self.positions[message.robot_id] = (first.x, first.y)

        if len(self.positions) >= 2:
            names = sorted(self.positions)
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    a = self.positions[names[i]]
                    b = self.positions[names[j]]
                    self.min_separation = min(
                        self.min_separation, math.hypot(a[0] - b[0], a[1] - b[1]))

    def finalise(self):
        now = self._elapsed()
        for robot, started in list(self.yield_started.items()):
            self.yield_seconds[robot] = \
                self.yield_seconds.get(robot, 0.0) + (now - started)
            self.yield_started.pop(robot)

    def report(self, min_allowed_separation):
        self.finalise()
        print()
        print('=' * 72)
        print('The Pinch: conflict resolution')
        print('=' * 72)

        if not self.timeline:
            print('  No directives were received at all. Is traffic_control running?')
            return 1

        print('  Timeline:')
        for stamp, robot, action, reason in self.timeline:
            print(f'    [{stamp:6.2f}s] {robot:6s} {action:8s} {reason}')

        print()
        print('  Time held:')
        for robot in sorted(set(list(self.yield_seconds) + list(self.last_action))):
            print(f'    {robot:6s} yielded for {self.yield_seconds.get(robot, 0.0):5.2f} s')

        print()
        if math.isinf(self.min_separation):
            print('  Separation: never saw both robots. Are both broadcasting?')
            return 1
        print(f'  Minimum separation observed: {self.min_separation:.2f} m '
              f'(floor {min_allowed_separation:.2f} m)')

        problems = []
        if self.yield_seconds.get(self.expected_yielder, 0.0) <= 0.0:
            problems.append(
                f'{self.expected_yielder} never yielded; the protocol did not fire')
        if self.yield_seconds.get(self.expected_winner, 0.0) > 0.0:
            problems.append(
                f'{self.expected_winner} yielded, but it outranks '
                f'{self.expected_yielder} and should have kept going')
        if self.min_separation < min_allowed_separation:
            problems.append(
                f'the robots came within {self.min_separation:.2f} m, closer than '
                f'their combined footprints')

        print()
        if problems:
            for problem in problems:
                print(f'  FAIL: {problem}')
            return 1

        print(f'  PASS: {self.expected_yielder} gave way to {self.expected_winner}, '
              f'and they never came closer than {self.min_separation:.2f} m.')
        return 0


def send_goals(pairs):
    """Fire off send_goals.py without blocking on it."""
    command = ['ros2', 'run', 'amr_bringup', 'send_goals.py']
    for robot, target in pairs:
        command += ['--goal', f'{robot}={target}']
    return subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=90.0)
    parser.add_argument(
        '--expect-yielder', default='amr2',
        help='Robot that should give way (default: the light scout).')
    parser.add_argument(
        '--expect-winner', default='amr1',
        help='Robot that should keep going (default: the heavy mapper).')
    args = parser.parse_args(argv)

    # Combined footprints of the shipped pair, less a small tolerance for
    # localisation error.
    models_path = os.path.join(
        get_package_share_directory('amr_description'), 'config', 'robot_models.yaml')
    with open(models_path, 'r', encoding='utf-8') as handle:
        models = yaml.safe_load(handle)
    floor = (models['heavy_mapper']['footprint_radius'] +
             models['light_scout']['footprint_radius']) * 0.85

    rclpy.init()
    node = ConflictWatcher(args.expect_yielder, args.expect_winner)
    node.get_logger().info(
        'sending both robots through The Pinch from opposite sides; '
        'the doorway fits one at a time')

    # Crossing goals: each robot's target sits on the far side of the doorway.
    child = send_goals([('amr1', 'east_staging'), ('amr2', 'packing_bay_4')])

    try:
        start = node.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds * 1e-9 - start > args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        child.terminate()
        status = node.report(floor)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Send concurrent navigation goals to the fleet by waypoint name.

    # The brief's primary scenario: both robots, at the same time.
    ros2 run amr_bringup send_goals.py --goal amr1=heavy_storage \\
                                       --goal amr2=packing_bay_4

    # The ramp-only goal.
    ros2 run amr_bringup send_goals.py --goal amr1=mezzanine_storage

    # Anything not named: raw coordinates.
    ros2 run amr_bringup send_goals.py --goal amr2=12.0,3.5,0.0

Goals are dispatched together and awaited together, so "concurrent" means
concurrent rather than sequential-with-a-short-gap. Exit status is non-zero if
any robot fails, which makes this usable as a scripted acceptance check.

Style: PEP 8, checked by ament_flake8.
"""

import argparse
import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


def load_waypoints():
    """Read the named goals emitted by the world generator."""
    path = os.path.join(
        get_package_share_directory('amr_gazebo'), 'config', 'waypoints.yaml')
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)['waypoints']


def parse_goal(text, waypoints):
    """Parse ``robot=waypoint_name`` or ``robot=x,y,yaw``."""
    if '=' not in text:
        raise argparse.ArgumentTypeError(
            f"--goal needs the form robot=target, got '{text}'")
    robot, target = text.split('=', 1)
    robot = robot.strip()
    target = target.strip()

    if target in waypoints:
        entry = waypoints[target]
        return robot, float(entry['x']), float(entry['y']), float(entry['yaw']), target

    parts = [p for p in target.split(',') if p.strip()]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(
            f"'{target}' is neither a known waypoint nor x,y[,yaw]. "
            f"Known: {', '.join(sorted(waypoints))}")
    x = float(parts[0])
    y = float(parts[1])
    yaw = float(parts[2]) if len(parts) == 3 else 0.0
    return robot, x, y, yaw, f"({x}, {y})"


class GoalDispatcher(Node):
    """Sends one goal per robot and tracks all of them at once."""

    def __init__(self, goals):
        super().__init__('send_goals')
        self.goals = goals
        self.clients = {}
        self.results = {}
        self.pending = set()

        for robot, _x, _y, _yaw, _label in goals:
            self.clients[robot] = ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose')

    def dispatch(self, timeout=30.0):
        """Send every goal, then wait for all of them."""
        for robot, x, y, yaw, label in self.goals:
            client = self.clients[robot]
            self.get_logger().info(f'waiting for {robot} navigate_to_pose...')
            if not client.wait_for_server(timeout_sec=timeout):
                self.get_logger().error(
                    f'{robot}: no navigate_to_pose action server. Is nav2 up and '
                    f'has its lifecycle manager finished activating?')
                self.results[robot] = False
                continue

            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            self.get_logger().info(
                f'{robot} -> {label} at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
            self.pending.add(robot)
            future = client.send_goal_async(goal)
            future.add_done_callback(self._make_response_callback(robot))

    def _make_response_callback(self, robot):
        def callback(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(f'{robot}: goal rejected')
                self.results[robot] = False
                self.pending.discard(robot)
                return
            self.get_logger().info(f'{robot}: goal accepted')
            handle.get_result_async().add_done_callback(self._make_result_callback(robot))
        return callback

    def _make_result_callback(self, robot):
        def callback(future):
            status = future.result().status
            # 4 == STATUS_SUCCEEDED
            succeeded = status == 4
            self.results[robot] = succeeded
            level = self.get_logger().info if succeeded else self.get_logger().error
            level(f'{robot}: {"reached its goal" if succeeded else f"failed (status {status})"}')
            self.pending.discard(robot)
        return callback

    def done(self):
        return not self.pending


def main(argv=None):
    waypoints = load_waypoints()
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--goal', action='append', default=[], metavar='ROBOT=TARGET',
        help='Repeatable. TARGET is a waypoint name or x,y[,yaw].')
    parser.add_argument(
        '--list', action='store_true', help='List the named waypoints and exit.')
    parser.add_argument(
        '--timeout', type=float, default=300.0,
        help='Seconds to wait for all goals to finish (default: 300).')
    args = parser.parse_args(argv)

    if args.list:
        print('Named waypoints:')
        for name in sorted(waypoints):
            entry = waypoints[name]
            print(f"  {name:22s} ({entry['x']:7.2f}, {entry['y']:7.2f})  "
                  f"[{entry.get('level', 'ground')}]  {entry.get('description', '')}")
        return 0

    if not args.goal:
        parser.error('at least one --goal is required (or use --list)')

    goals = [parse_goal(text, waypoints) for text in args.goal]

    rclpy.init()
    node = GoalDispatcher(goals)
    try:
        node.dispatch()
        deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds * 1e-9 > deadline:
                node.get_logger().error(
                    f'timed out after {args.timeout:.0f} s; still waiting on: '
                    f'{", ".join(sorted(node.pending))}')
                break

        failures = [r for r, ok in node.results.items() if not ok]
        failures += sorted(node.pending)
        if failures:
            node.get_logger().error(f'FAILED: {", ".join(sorted(set(failures)))}')
            return 1
        node.get_logger().info('all goals reached')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

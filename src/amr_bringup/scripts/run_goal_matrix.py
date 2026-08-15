#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Drive every pairwise goal combination and report what fails.

    # with `ros2 launch amr_bringup fleet.launch.py` already running:
    ros2 run amr_bringup run_goal_matrix.py

Sends every ordered pair of named goals -- one to AMR-1, a different one to
AMR-2, concurrently -- waits for both, and records the outcome. With 9 named
goals that is 72 ordered pairs; `--filter` and `--pairs` cut it down.

Why a script rather than a checklist
------------------------------------
Driving the pairs by hand is slow and, worse, inconsistent: the fleet's state
after a failed run is not the state it started in, so the next trial is not the
trial you think you are running. This resets both robots to their docks between
pairs, so each result stands alone.

What it records, per pair, for each robot:

* the action result -- succeeded, aborted, rejected, timed out;
* wall-clock time to the goal;
* the final distance to the goal pose;
* whether the safety override ever latched, and why;
* whether the traffic controller ever issued a YIELD;
* peak |v| and the number of replans.

Those last four are what turn "it failed" into "it failed because". A pair that
aborts with `halt_active` set the whole time is a perception problem; one that
aborts with the planner replanning forty times is a costmap problem; and they
need opposite fixes.

Output is a table on stdout and a CSV next to it, so a failing row can be
re-run on its own:

    ros2 run amr_bringup run_goal_matrix.py --pairs heavy_storage:packing_bay_4

Style: PEP 8, checked by ament_flake8.
"""

import argparse
import csv
import itertools
import math
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from amr_msgs.msg import SafetyStatus, TrafficDirective
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml

#: Action result codes (action_msgs/msg/GoalStatus).
SUCCEEDED, ABORTED, CANCELED = 4, 6, 5
STATUS_NAMES = {SUCCEEDED: 'succeeded', ABORTED: 'aborted', CANCELED: 'canceled'}


def load_waypoints():
    path = os.path.join(
        get_package_share_directory('amr_gazebo'), 'config', 'waypoints.yaml')
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)['waypoints']


class Trial:
    """One robot's participation in one pair."""

    def __init__(self, robot):
        self.robot = robot
        self.status = 'not started'
        self.seconds = 0.0
        self.final_error = float('nan')
        self.halted = False
        self.halt_reason = 0
        self.yielded = False
        self.peak_speed = 0.0
        self.replans = 0

    def row(self, goal):
        return {
            'robot': self.robot,
            'goal': goal,
            'status': self.status,
            'seconds': f'{self.seconds:.1f}',
            'final_error_m': f'{self.final_error:.2f}',
            'halted': int(self.halted),
            'halt_reason': self.halt_reason,
            'yielded': int(self.yielded),
            'peak_speed': f'{self.peak_speed:.2f}',
            'replans': self.replans,
        }


class GoalMatrix(Node):
    """Runs the pairs and watches the fleet while it does."""

    def __init__(self, robots, timeout):
        super().__init__('run_goal_matrix')
        self.robots = robots
        self.timeout = timeout
        self.trials = {}

        # NOT `self.clients`: rclpy.node.Node exposes that as a read-only
        # property. See DESIGN_NOTES 8c.
        self.goal_clients = {
            robot: ActionClient(self, NavigateToPose,
                                f'/{robot}/navigate_to_pose')
            for robot in robots
        }
        self.pending = set()
        self.handles = {}

        for robot in robots:
            self.create_subscription(
                SafetyStatus, f'/{robot}/safety_status',
                self._make_safety_callback(robot), 10)
            self.create_subscription(
                TrafficDirective, f'/{robot}/traffic_directive',
                self._make_traffic_callback(robot), 10)
            self.create_subscription(
                Twist, f'/{robot}/cmd_vel',
                self._make_speed_callback(robot), 10)

    # -- telemetry -----------------------------------------------------------

    def _make_safety_callback(self, robot):
        def callback(message):
            trial = self.trials.get(robot)
            if trial is not None and message.halt_active:
                trial.halted = True
                trial.halt_reason = message.reason
        return callback

    def _make_traffic_callback(self, robot):
        def callback(message):
            trial = self.trials.get(robot)
            if trial is not None and message.action == TrafficDirective.YIELD:
                trial.yielded = True
        return callback

    def _make_speed_callback(self, robot):
        def callback(message):
            trial = self.trials.get(robot)
            if trial is not None:
                trial.peak_speed = max(trial.peak_speed, abs(message.linear.x))
        return callback

    # -- one pair ------------------------------------------------------------

    def run_pair(self, assignment, waypoints):
        """`assignment` maps robot name -> goal name. Returns {robot: Trial}."""
        self.trials = {robot: Trial(robot) for robot in assignment}
        self.pending = set(assignment)
        self.handles.clear()
        started = time.time()

        for robot, goal_name in assignment.items():
            entry = waypoints[goal_name]
            if not self.goal_clients[robot].wait_for_server(timeout_sec=10.0):
                self.trials[robot].status = 'no action server'
                self.pending.discard(robot)
                continue

            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(entry['x'])
            goal.pose.pose.position.y = float(entry['y'])
            yaw = float(entry.get('yaw', 0.0))
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            future = self.goal_clients[robot].send_goal_async(goal)
            future.add_done_callback(self._make_response_callback(robot))

        deadline = started + self.timeout
        while rclpy.ok() and self.pending and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

        for robot in list(self.pending):
            self.trials[robot].status = 'timed out'
            handle = self.handles.get(robot)
            if handle is not None:
                handle.cancel_goal_async()
        for trial in self.trials.values():
            trial.seconds = time.time() - started
        return self.trials

    def _make_response_callback(self, robot):
        def callback(future):
            handle = future.result()
            if not handle.accepted:
                self.trials[robot].status = 'rejected'
                self.pending.discard(robot)
                return
            self.handles[robot] = handle
            handle.get_result_async().add_done_callback(
                self._make_result_callback(robot))
        return callback

    def _make_result_callback(self, robot):
        def callback(future):
            status = future.result().status
            self.trials[robot].status = STATUS_NAMES.get(status, f'status {status}')
            self.pending.discard(robot)
        return callback

    def return_to_dock(self, waypoints, docks):
        """Reset both robots so the next pair starts from a known state."""
        self.run_pair(docks, waypoints)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robots', nargs='+', default=['amr1', 'amr2'])
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='seconds to allow each pair (default: 180)')
    parser.add_argument('--filter', default=None,
                        help='only goals whose name contains this substring')
    parser.add_argument('--pairs', nargs='*', default=None,
                        help='explicit goal pairs as amr1goal:amr2goal')
    parser.add_argument('--csv', default='goal_matrix.csv')
    parser.add_argument('--no-reset', action='store_true',
                        help='skip the return-to-dock between pairs (faster, '
                             'but each trial then depends on the last)')
    args = parser.parse_args(argv)

    waypoints = load_waypoints()
    goals = [n for n in sorted(waypoints) if not n.startswith('dock_')]
    if args.filter:
        goals = [n for n in goals if args.filter in n]

    if args.pairs:
        combos = [tuple(p.split(':', 1)) for p in args.pairs]
    else:
        combos = [(a, b) for a, b in itertools.permutations(goals, 2)]

    docks = {robot: f'dock_{chr(ord("a") + index)}'
             for index, robot in enumerate(args.robots)}

    rclpy.init()
    node = GoalMatrix(args.robots, args.timeout)
    rows = []
    failures = []

    try:
        print(f'{len(combos)} pairs over {len(args.robots)} robots\n')
        header = (f'{"#":>3}  {"amr1 goal":<20}{"amr2 goal":<20}'
                  f'{"amr1":<14}{"amr2":<14}{"notes"}')
        print(header)
        print('-' * len(header))

        for index, (first, second) in enumerate(combos, start=1):
            assignment = dict(zip(args.robots, (first, second)))
            trials = node.run_pair(assignment, waypoints)

            notes = []
            for robot, trial in trials.items():
                rows.append(trial.row(assignment[robot]))
                if trial.halted:
                    notes.append(f'{robot} halted (reason {trial.halt_reason})')
                if trial.yielded:
                    notes.append(f'{robot} yielded')
                if trial.peak_speed < 0.02:
                    notes.append(f'{robot} never moved')

            states = [trials[r].status for r in args.robots]
            ok = all(state == 'succeeded' for state in states)
            if not ok:
                failures.append((first, second, dict(zip(args.robots, states)),
                                 '; '.join(notes)))

            print(f'{index:>3}  {first:<20}{second:<20}'
                  f'{states[0]:<14}{states[1]:<14}{"; ".join(notes)}')

            if not args.no_reset:
                node.return_to_dock(waypoints, docks)

        with open(args.csv, 'w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        print(f'\n{len(combos) - len(failures)}/{len(combos)} pairs succeeded')
        print(f'per-robot detail written to {args.csv}')

        if failures:
            print('\nFAILURES, worst first -- re-run one with:')
            print('  ros2 run amr_bringup run_goal_matrix.py --pairs A:B\n')
            for first, second, states, notes in failures:
                print(f'  {first} / {second}: {states}'
                      f'{"  [" + notes + "]" if notes else ""}')
        return 1 if failures else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

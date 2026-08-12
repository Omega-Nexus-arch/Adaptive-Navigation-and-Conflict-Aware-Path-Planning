#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""A/B test the slope cost: does the planner really avoid the ramp?

The requirement has two halves and they pull in opposite directions - ramps
must be avoided when a flat route exists, and used when one does not. This
script asks the global planner for the same path three times and compares:

  A. Slope layer ON, doorway open   -> expect the flat route through The Pinch
  B. Slope layer ON, doorway shut   -> expect the Hump Bridge, because it is
                                       now the only crossing
  C. Slope layer OFF, doorway open  -> the control. Without slope cost the
                                       planner should be indifferent, and the
                                       shorter bridge route becomes attractive

A passes and B passes only if the cost is doing real work. C is what
distinguishes "the planner avoided the ramp" from "the flat route happened to
be shorter anyway".

    ros2 run amr_bringup demo_slope_planning.py

The doorway is closed by asking Gazebo to park a blocking box in it, so the
comparison is against the same planner, the same map, and the same goal.

Style: PEP 8, checked by ament_flake8.
"""

import argparse
import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.node import Node
import yaml

#: The Hump Bridge deck spans y in [2.0, 5.2] at x in [-1.6, 1.6].
BRIDGE_Y_MIN = 1.8
BRIDGE_Y_MAX = 5.4
#: The Pinch doorway spans y in [-1.0, 1.0] at x = 0.
DOORWAY_Y_MIN = -1.2
DOORWAY_Y_MAX = 1.2


def crossing_used(path):
    """Classify how a path crosses the central firewall at x = 0."""
    for i in range(len(path.poses) - 1):
        x0 = path.poses[i].pose.position.x
        x1 = path.poses[i + 1].pose.position.x
        if (x0 < 0.0) != (x1 < 0.0):
            y0 = path.poses[i].pose.position.y
            y1 = path.poses[i + 1].pose.position.y
            y = 0.5 * (y0 + y1)
            if DOORWAY_Y_MIN <= y <= DOORWAY_Y_MAX:
                return 'doorway', y
            if BRIDGE_Y_MIN <= y <= BRIDGE_Y_MAX:
                return 'bridge', y
            return f'other (y={y:.2f})', y
    return 'none', float('nan')


def path_length(path):
    total = 0.0
    for i in range(len(path.poses) - 1):
        a = path.poses[i].pose.position
        b = path.poses[i + 1].pose.position
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


class SlopeExperiment(Node):
    """Drives the three planning trials."""

    def __init__(self, robot):
        super().__init__('demo_slope_planning')
        self.robot = robot
        self.plan_client = self.create_client(
            GetPlan, f'/{robot}/planner_server/get_plan')
        self.clear_client = self.create_client(
            ClearEntireCostmap,
            f'/{robot}/global_costmap/clear_entirely_global_costmap')
        self.param_client = self.create_client(
            SetParameters, f'/{robot}/global_costmap/global_costmap/set_parameters')
        self.entity_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

    def _call(self, client, request, label, timeout=15.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'{label}: service unavailable')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def set_slope_enabled(self, enabled):
        parameter = Parameter()
        parameter.name = 'slope_layer.enabled'
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled)
        request = SetParameters.Request(parameters=[parameter])
        result = self._call(self.param_client, request, 'set slope_layer.enabled')
        return result is not None

    def move_blocker(self, x, y):
        """Park (or remove) the box that closes the doorway."""
        request = SetEntityState.Request()
        request.state.name = 'thirdparty_0'
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = 0.25
        request.state.pose.orientation.w = 1.0
        request.state.reference_frame = 'world'
        return self._call(self.entity_client, request, 'set_entity_state') is not None

    def clear_costmap(self):
        self._call(self.clear_client, ClearEntireCostmap.Request(), 'clear costmap')

    def plan(self, start_xy, goal_xy):
        request = GetPlan.Request()
        for target, (x, y) in (('start', start_xy), ('goal', goal_xy)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            setattr(request, target, pose)
        request.tolerance = 0.5
        result = self._call(self.plan_client, request, 'get_plan')
        return result.plan if result is not None else Path()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', default='amr1')
    args = parser.parse_args(argv)

    waypoints_path = os.path.join(
        get_package_share_directory('amr_gazebo'), 'config', 'waypoints.yaml')
    with open(waypoints_path, 'r', encoding='utf-8') as handle:
        waypoints = yaml.safe_load(handle)['waypoints']

    start = (waypoints['west_staging']['x'], waypoints['west_staging']['y'])
    goal = (waypoints['east_staging']['x'], waypoints['east_staging']['y'])

    rclpy.init()
    node = SlopeExperiment(args.robot)
    results = {}

    try:
        # --- A: slope on, doorway open ------------------------------------
        node.get_logger().info('A: slope cost ON, doorway open')
        node.set_slope_enabled(True)
        node.move_blocker(-14.0, -8.0)      # park the box out of the way
        node.clear_costmap()
        plan_a = node.plan(start, goal)
        results['A'] = (crossing_used(plan_a), path_length(plan_a))

        # --- B: slope on, doorway blocked ---------------------------------
        node.get_logger().info('B: slope cost ON, doorway blocked')
        node.move_blocker(0.0, 0.0)         # sit the box in the doorway
        node.clear_costmap()
        plan_b = node.plan(start, goal)
        results['B'] = (crossing_used(plan_b), path_length(plan_b))

        # --- C: slope off, doorway open (control) -------------------------
        node.get_logger().info('C: slope cost OFF, doorway open (control)')
        node.move_blocker(-14.0, -8.0)
        node.set_slope_enabled(False)
        node.clear_costmap()
        plan_c = node.plan(start, goal)
        results['C'] = (crossing_used(plan_c), path_length(plan_c))

        node.set_slope_enabled(True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    print('=' * 72)
    print('Slope-aware planning: flat doorway vs sloped bridge')
    print('=' * 72)
    labels = {
        'A': 'slope ON,  doorway open',
        'B': 'slope ON,  doorway blocked',
        'C': 'slope OFF, doorway open (control)',
    }
    for key in ('A', 'B', 'C'):
        (crossing, y), length = results[key]
        print(f'  {key}  {labels[key]:34s} -> {crossing:20s} '
              f'({length:5.1f} m)')

    problems = []
    if results['A'][0][0] != 'doorway':
        problems.append(
            'A: with a flat crossing available the planner should have taken it')
    if results['B'][0][0] != 'bridge':
        problems.append(
            'B: with the doorway shut the ramp is the only route and must be used')
    if results['C'][0][0] == 'doorway' and results['A'][0][0] == 'doorway':
        problems.append(
            'C: turning the slope cost off changed nothing, so A proves nothing '
            'about the cost - the flat route was simply shorter')

    print()
    if problems:
        for problem in problems:
            print(f'  FAIL: {problem}')
        return 1
    print('  PASS: ramps are avoided when an alternative exists and used when')
    print('        they are the only viable path, and the slope cost is what')
    print('        makes the difference.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""The two fleet-level nodes: traffic control and map fusion.

Both are singletons in the global namespace, unlike everything in
robot.launch.py. They are the only components with a view of the whole fleet,
and that is deliberate:

* **Traffic control** arbitrates conflicts. Yielding is a decision about a
  *pair* of robots; deciding it independently on each one invites the failure
  where both defer or neither does, depending on message timing.

* **Map fusion** produces the single unified occupancy grid the whole fleet
  plans against, and anchors each robot's private SLAM frame into the shared
  `map` frame.

Neither is safety-critical. Traffic control can only slow robots down, and
each robot's own safety override runs underneath regardless, so losing this
launch file costs throughput rather than collision avoidance.
"""

import os

from ament_index_python.packages import get_package_share_directory
from amr_bringup.fleet_loader import load_fleet
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fleet_nodes(context, *args, **kwargs):
    fleet_config = LaunchConfiguration('fleet_config').perform(context)

    # The merged grid's bounds come from the roster, not from numbers typed
    # here. nav2's global costmap is sized from the same block
    # (navigation.launch.py), so the fused map and the costmap it fills are the
    # same rectangle by construction rather than by two people remembering.
    extents = load_fleet(fleet_config)['map_extents']

    return [
        Node(
            package='amr_fleet_control',
            executable='traffic_control_node',
            name='traffic_control',
            output='screen',
            parameters=[{
                'fleet_config': fleet_config,
                'use_sim_time': True,
                # Conflict spheres in RViz. Worth the topic: "the robots
                # stopped" and "the robots stopped for this predicted
                # conflict, there, in 1.2 s" are different claims.
                'publish_markers': True,
            }],
        ),

        Node(
            package='amr_mapping',
            executable='map_fusion_node',
            name='map_fusion',
            output='screen',
            parameters=[{
                'fleet_config': fleet_config,
                'use_sim_time': True,
                'resolution': float(extents['resolution']),
                'x_min': float(extents['x_min']),
                'x_max': float(extents['x_max']),
                'y_min': float(extents['y_min']),
                'y_max': float(extents['y_max']),
                'publish_rate': 1.0,
            }],
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml')),
        OpaqueFunction(function=_fleet_nodes),
    ])

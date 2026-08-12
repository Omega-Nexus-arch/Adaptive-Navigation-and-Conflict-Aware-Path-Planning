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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    fleet_config = LaunchConfiguration('fleet_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml')),

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
                'resolution': 0.05,
                # Matches the generated world's extent with a small margin.
                'x_min': -24.0,
                'x_max': 24.0,
                'y_min': -17.0,
                'y_max': 17.0,
                'publish_rate': 1.0,
            }],
        ),
    ])

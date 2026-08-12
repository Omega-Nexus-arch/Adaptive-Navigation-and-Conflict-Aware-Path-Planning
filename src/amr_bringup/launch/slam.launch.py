# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Per-robot SLAM plus the selective-iteration map filter.

Expects to be included from inside the robot's namespace (robot.launch.py
pushes it), so every topic and frame here is relative.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _slam_nodes(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    bringup_share = get_package_share_directory('amr_bringup')

    # Each robot owns a private map frame; amr_mapping anchors it into the
    # shared one. Frames are prefixed to match the URDF.
    slam_params = RewrittenYaml(
        source_file=os.path.join(bringup_share, 'config', 'slam_toolbox.yaml'),
        root_key=robot_name,
        param_rewrites={
            'use_sim_time': 'True',
            'base_frame': f'{robot_name}/base_footprint',
            'odom_frame': f'{robot_name}/odom',
            'map_frame': f'{robot_name}/map',
        },
        convert_types=True,
    )

    return [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            # Keep SLAM's map topics private to this robot.
            #
            # THE SOURCE NAMES MUST CARRY A LEADING SLASH. slam_toolbox
            # constructs these publishers with *absolute* names -- literally
            # "/map" and "/map_metadata" -- so pushing a namespace does not
            # move them, and a remap rule written as the relative `map` matches
            # nothing at all. The rule has to name the topic exactly as the
            # node created it.
            #
            # The targets are relative, so under the pushed /<robot> namespace
            # they resolve to /<robot>/map and /<robot>/map_metadata.
            #
            # Left absolute, slam_toolbox publishes its own small, robot-private
            # grid onto the fleet's shared /map alongside map_fusion, and nav2's
            # static layer resizes the global costmap to whichever arrived last.
            # See DESIGN_NOTES 7g.
            remappings=[
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
            ],
        ),

        # The selective-iteration policy. Sits between SLAM and fusion: it
        # decides which parts of this robot's map are worth republishing,
        # prioritising unexplored boundaries and throttling well-travelled
        # aisles. Suppressed cells are emitted as unknown, which the fusion
        # node reads as "no new evidence" rather than "empty".
        Node(
            package='amr_mapping',
            executable='selective_mapping_node',
            name='selective_mapping',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'fleet_config': fleet_config,
                'use_sim_time': True,
                'frontier_radius': 1.0,
                'saturation_visits': 8,
                'saturated_period': 5.0,
                'explored_period': 1.0,
                'significant_change': 25,
                'traversal_radius': 1.5,
                'min_cells_to_publish': 12,
                'enabled': True,
            }],
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml')),
        OpaqueFunction(function=_slam_nodes),
    ])

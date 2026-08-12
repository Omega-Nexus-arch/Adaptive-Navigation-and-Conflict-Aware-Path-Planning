# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Everything belonging to ONE robot, in that robot's namespace.

This is the unit the fleet launch file repeats. It knows nothing about how
many robots exist, which is what lets `fleet.launch.py` scale from two to ten
by iterating a roster.

Composed of four sub-launches so each concern can be run, disabled or replaced
independently:

    robot.launch.py
      +-- (description + spawn)      URDF, robot_state_publisher, Gazebo spawn
      +-- (sensor validation)        amr_sensor_bsp
      +-- (fleet control)            smoother, safety override, trajectory
      +-- slam.launch.py             slam_toolbox + selective mapping
      +-- navigation.launch.py       nav2

Run one robot on its own for debugging:

    ros2 launch amr_bringup robot.launch.py robot_name:=amr1
"""

import os

from ament_index_python.packages import get_package_share_directory
from amr_bringup.fleet_loader import load_fleet
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _robot_nodes(context, *args, **kwargs):
    """Build the node set once the launch arguments have concrete values."""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    use_slam = LaunchConfiguration('slam').perform(context)
    use_navigation = LaunchConfiguration('navigation').perform(context)

    fleet = load_fleet(fleet_config)
    matches = [r for r in fleet['robots'] if r['name'] == robot_name]
    if not matches:
        raise RuntimeError(
            f"robot '{robot_name}' is not in {fleet_config}. "
            f"Roster: {', '.join(r['name'] for r in fleet['robots'])}")
    robot = matches[0]

    description_share = get_package_share_directory('amr_description')
    bringup_share = get_package_share_directory('amr_bringup')
    xacro_file = os.path.join(description_share, 'urdf', 'amr.urdf.xacro')
    model_library = os.path.join(description_share, 'config', 'robot_models.yaml')

    robot_description = Command([
        'xacro ', xacro_file,
        ' robot_name:=', robot_name,
        ' robot_model:=', robot['model'],
        ' model_config:=', model_library,
    ])

    common = {'use_sim_time': True}

    nodes = [
        # --- Description ------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # Links are prefixed inside the URDF itself, so frame_prefix
                # must stay empty or every frame would be doubly prefixed.
                'frame_prefix': '',
                **common,
            }],
        ),

        # --- Spawn into Gazebo at its roster pose ------------------------------
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', f'/{robot_name}/robot_description',
                '-entity', robot_name,
                '-robot_namespace', f'/{robot_name}',
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-Y', str(robot['yaw']),
                '-z', '0.05',
            ],
        ),

        # --- BSP sensor validation --------------------------------------------
        # Placed before everything that consumes sensor data. Nothing
        # downstream subscribes to a *_raw topic.
        Node(
            package='amr_sensor_bsp',
            executable='bsp_validation_node',
            name='bsp_validation',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'fleet_config': fleet_config,
                'ground_rejection_enabled': True,
                'check_image_intensity': True,
                **common,
            }],
        ),

        # --- Velocity smoothing (traffic scaling + accel/jerk limits) ---------
        Node(
            package='amr_fleet_control',
            executable='velocity_smoother_node',
            name='velocity_smoother',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'fleet_config': fleet_config,
                'control_rate': 50.0,
                **common,
            }],
        ),

        # --- Safety override: sole publisher of cmd_vel -----------------------
        Node(
            package='amr_fleet_control',
            executable='safety_override_node',
            name='safety_override',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'fleet_config': fleet_config,
                'control_rate': 50.0,
                **common,
            }],
        ),

        # --- Trajectory broadcast for conflict-aware planning -----------------
        Node(
            package='amr_fleet_control',
            executable='trajectory_broadcaster_node',
            name='trajectory_broadcaster',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'fleet_config': fleet_config,
                **common,
            }],
        ),
    ]

    includes = []
    if use_slam.lower() in ('true', '1', 'yes'):
        includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'slam.launch.py')),
                launch_arguments={
                    'robot_name': robot_name,
                    'fleet_config': fleet_config,
                }.items(),
            ))
    if use_navigation.lower() in ('true', '1', 'yes'):
        includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'navigation.launch.py')),
                launch_arguments={
                    'robot_name': robot_name,
                    'fleet_config': fleet_config,
                }.items(),
            ))

    # PushRosNamespace is what makes every robot's graph identical apart from
    # its prefix, so no node has to know its own name to find its topics.
    return [GroupAction([PushRosNamespace(robot_name)] + nodes + includes)]


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', description='Roster name, e.g. amr1.'),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml'),
            description='Fleet roster. Swap for fleet_ten_robots.yaml to scale up.'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Run slam_toolbox and the selective mapping filter.'),
        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='Run nav2 for this robot.'),
        OpaqueFunction(function=_robot_nodes),
    ])

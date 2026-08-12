# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Top-level entry point: the whole system, however many robots there are.

    ros2 launch amr_bringup fleet.launch.py

Scaling to ten robots is a config swap, not a code change:

    ros2 launch amr_bringup fleet.launch.py \\
        fleet_config:=<share>/amr_bringup/config/fleet_ten_robots.yaml

This file contains no robot names. It reads the roster and repeats
robot.launch.py once per entry, which is the mechanism behind the
"expand the fleet by changing a minimal number of configuration parameters"
requirement.

Structure:

    fleet.launch.py                 <- you are here
      +-- simulation.launch.py      Gazebo, world, dynamic obstacles
      +-- robot.launch.py  x N      per-robot stack (from the roster)
      +-- fleet_control.launch.py   traffic controller, map fusion
      +-- (rviz)
"""

import os

from ament_index_python.packages import get_package_share_directory
from amr_bringup.fleet_loader import load_fleet
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fleet(context, *args, **kwargs):
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    use_simulation = LaunchConfiguration('simulation').perform(context).lower()
    use_rviz = LaunchConfiguration('rviz').perform(context).lower()
    use_slam = LaunchConfiguration('slam').perform(context)
    use_navigation = LaunchConfiguration('navigation').perform(context)
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

    bringup_share = get_package_share_directory('amr_bringup')
    fleet = load_fleet(fleet_config)

    actions = [
        LogInfo(msg=[
            f"Launching {len(fleet['robots'])} robot(s) from {fleet_config}: ",
            ', '.join(
                f"{r['name']}({r['model']}, priority {r['yield_priority']})"
                for r in fleet['robots']),
        ]),
    ]

    if use_simulation in ('true', '1', 'yes'):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'simulation.launch.py'))))

    # Start the global singleton BEFORE robot stacks. map_fusion publishes the
    # static map -> <robot>/map anchors required by slam_toolbox and nav2; if it
    # comes up after the robots, global costmaps wait on a frame that cannot
    # exist yet and lifecycle startup becomes timing-dependent.
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'fleet_control.launch.py')),
            launch_arguments={'fleet_config': fleet_config}.items(),
        ))

    # Robots are staggered rather than started together. Spawning several
    # models into Gazebo simultaneously reliably wedges the spawn service, and
    # bringing up N nav2 stacks at once on one machine trips lifecycle bonds.
    for index, robot in enumerate(fleet['robots']):
        actions.append(
            TimerAction(
                period=spawn_delay * index,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_share, 'launch', 'robot.launch.py')),
                        launch_arguments={
                            'robot_name': robot['name'],
                            'fleet_config': fleet_config,
                            'slam': use_slam,
                            'navigation': use_navigation,
                            # Global services are already above; never duplicate them.
                            'fleet_services': 'false',
                        }.items(),
                    ),
                ],
            ))

    if use_rviz in ('true', '1', 'yes'):
        actions.append(
            TimerAction(
                period=spawn_delay * len(fleet['robots']) + 3.0,
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        arguments=[
                            '-d', os.path.join(bringup_share, 'rviz', 'fleet.rviz')],
                        parameters=[{'use_sim_time': True}],
                    ),
                ],
            ))

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml'),
            description='Fleet roster. Point at fleet_ten_robots.yaml to scale up.'),
        DeclareLaunchArgument(
            'simulation', default_value='true',
            description='Start Gazebo. False when driving real hardware.'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Run SLAM and selective mapping on every robot.'),
        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='Run nav2 on every robot.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz with the fleet configuration.'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='6.0',
            description='Seconds between robot bringups. Raise on a slow machine '
                        'if nav2 lifecycle bonds time out during startup.'),
        OpaqueFunction(function=_fleet),
    ])

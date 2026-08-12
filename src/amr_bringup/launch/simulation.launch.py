# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Gazebo, the warehouse world, and the dynamic-obstacle traffic.

One of five composable launch files. This one owns *the world* and nothing
else - no robots, no navigation - so it can be run alone while tuning the
layout, and so a hardware bringup can simply not include it.

    ros2 launch amr_bringup simulation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_share = get_package_share_directory('gazebo_ros')
    amr_gazebo_share = get_package_share_directory('amr_gazebo')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    dynamic_obstacles = LaunchConfiguration('dynamic_obstacles')
    obstacle_seed = LaunchConfiguration('obstacle_seed')

    arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                amr_gazebo_share, 'worlds', 'warehouse_multilevel.world'),
            description='SDF world to load. Regenerate with '
                        '`ros2 run amr_gazebo generate_world.py`.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run the Gazebo client. Set false for headless CI.'),
        DeclareLaunchArgument(
            'dynamic_obstacles', default_value='true',
            description='Drive the pedestrians and third-party robots. Turn off '
                        'to isolate a planner problem from a traffic problem.'),
        DeclareLaunchArgument(
            'obstacle_seed', default_value='20260811',
            description='RNG seed for obstacle motion. Fixed by default so an '
                        'interesting run can be reproduced exactly.'),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            # Required for the gazebo_ros plugins the robots and obstacles use.
            'init': 'true',
            'factory': 'true',
            'force_system': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    obstacle_driver = Node(
        package='amr_gazebo',
        executable='dynamic_obstacle_driver.py',
        name='dynamic_obstacle_driver',
        output='screen',
        condition=IfCondition(dynamic_obstacles),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('amr_gazebo'), 'config', 'dynamic_obstacles.yaml']),
            {'use_sim_time': True, 'random_seed': obstacle_seed},
        ],
    )

    return LaunchDescription(arguments + [gzserver, gzclient, obstacle_driver])

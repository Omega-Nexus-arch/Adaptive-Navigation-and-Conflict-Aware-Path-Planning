# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""nav2 for one robot, parameterised from the model library.

The per-robot values in nav2_params.yaml are placeholders; every one of them
is rewritten here from amr_description/config/robot_models.yaml. That is what
keeps the planner, the motion smoother and the Gazebo plugin agreeing about
what the robot can physically do.

Note the output wiring: nav2's controller publishes to `cmd_vel_nav`, not
`cmd_vel`. The command then passes through the velocity smoother and the
safety override before it reaches the base. nav2 never writes `cmd_vel`
directly, which is what makes the safety override authoritative rather than
advisory.
"""

import os

from ament_index_python.packages import get_package_share_directory
from amr_bringup.fleet_loader import load_fleet, map_substitutions, nav2_substitutions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

#: nav2 lifecycle nodes, in the order the manager should bring them up.
NAV2_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]


def _navigation_nodes(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)

    bringup_share = get_package_share_directory('amr_bringup')
    gazebo_share = get_package_share_directory('amr_gazebo')
    elevation_map = os.path.join(gazebo_share, 'maps', 'warehouse_elevation.yaml')

    fleet = load_fleet(fleet_config)
    matches = [r for r in fleet['robots'] if r['name'] == robot_name]
    if not matches:
        raise RuntimeError(f"robot '{robot_name}' is not in {fleet_config}")

    substitutions = nav2_substitutions(matches[0], fleet['global_frame'], elevation_map)

    # Two-stage substitution; see the header of nav2_params.yaml for why.
    #
    # Stage 1 - frames, textually. Each `<ns>` keeps whatever value its own line
    # carries, so the global costmap stays anchored to `map` while the rolling
    # local costmap gets `<robot>/odom`. RewrittenYaml cannot express this: it
    # matches on key name, so one `global_frame` rewrite would hit both.
    textual = {'<ns>': robot_name}
    textual.update(map_substitutions(fleet['map_extents']))

    template = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    with open(template, 'r', encoding='utf-8') as handle:
        expanded = handle.read()
    for placeholder, value in textual.items():
        expanded = expanded.replace(placeholder, value)

    # A surviving `<...>` means a placeholder was added to the YAML and not to
    # the substitution map. That would reach nav2 as an unparseable value and
    # surface as a generic parameter error, so name it here instead.
    for line_number, line in enumerate(expanded.splitlines(), start=1):
        stripped = line.split('#', 1)[0]
        if '<' in stripped and '>' in stripped:
            raise RuntimeError(
                f'nav2_params.yaml line {line_number} still contains an '
                f'unsubstituted placeholder after expansion: {line.strip()!r}')

    # Written next to the other build artefacts rather than /tmp so the exact
    # parameters a robot was launched with can be inspected after the fact.
    scratch = os.path.join(
        os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'amr_bringup')
    os.makedirs(scratch, exist_ok=True)
    expanded_path = os.path.join(scratch, f'nav2_params_{robot_name}.yaml')
    with open(expanded_path, 'w', encoding='utf-8') as handle:
        handle.write(expanded)

    # Stage 2 - the values that genuinely are uniform wherever they appear.
    configured_params = RewrittenYaml(
        source_file=expanded_path,
        root_key=robot_name,
        param_rewrites=substitutions,
        convert_types=True,
    )

    nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            # The chain: nav2 -> cmd_vel_nav -> smoother -> cmd_vel_smoothed
            #            -> safety override -> cmd_vel -> base.
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': NAV2_NODES,
                # Generous: on a machine running two full nav2 stacks plus
                # Gazebo, a 5 s default trips during startup and takes the
                # whole fleet down with it.
                'bond_timeout': 20.0,
                'attempt_respawn_reconnection': True,
            }],
        ),
    ]
    return nodes


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml')),
        OpaqueFunction(function=_navigation_nodes),
    ])

# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Everything belonging to one robot, in that robot's namespace.

The startup order here is intentional and is part of the system's correctness:

1. ``fleet_control.launch.py`` (when run standalone) publishes the shared
   ``map`` frame and the map -> <robot>/map anchors.
2. robot_state_publisher publishes the static body TF and Gazebo spawns the
   entity.
3. After ``startup_delay`` seconds, Gazebo's diff-drive plugin has published
   <robot>/odom -> <robot>/base_footprint, so SLAM and nav2 can activate
   without racing a TF frame that does not exist yet.

``fleet.launch.py`` starts fleet-level services once before repeating this file
and passes ``fleet_services:=false`` to each instance.
"""

import os

from ament_index_python.packages import get_package_share_directory
from amr_bringup.fleet_loader import load_fleet
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(value):
    """Interpret a ROS launch boolean consistently."""
    return value.strip().lower() in ('true', '1', 'yes')


def namespaced(robot_name, actions):
    """Wrap ``actions`` so every node inside lands under ``/<robot_name>``.

    Use this for EVERY block of per-robot actions, including deferred ones.

    ``PushRosNamespace`` is *scoped*: a ``GroupAction`` applies it while
    visiting its children and pops it immediately afterwards. ``TimerAction``
    does not run its children during that visit -- it defers them to a later
    tick, by which point the namespace has already been popped. Nesting a timer
    inside a namespaced group therefore launches the timed nodes in the **root**
    namespace, silently.

    That single mistake produced three unrelated-looking failures:

    * ``slam_toolbox`` started as ``/slam_toolbox`` instead of
      ``/<robot>/slam_toolbox``, so its ``root_key: <robot>`` parameters never
      matched. It fell back to defaults and published ``map -> odom`` while the
      robot's own frames were ``<robot>/odom`` and ``<robot>/base_footprint``,
      leaving two unconnected TF trees.
    * nav2's servers started unnamespaced for the same reason, so
      ``No critics defined for FollowPath`` aborted the lifecycle bringup.
    * the local costmap silently loaded nav2's default plugin list instead of
      the configured one, so the conflict-aware fleet layer was absent.

    Every deferred block must therefore carry its own push. This helper is the
    only place that wrapping happens, and ``test_launch_namespacing.py`` walks
    the generated launch description to prove nothing escapes it.
    """
    return GroupAction([PushRosNamespace(robot_name)] + list(actions))


def _robot_nodes(context, *args, **kwargs):
    """Build the robot action group once launch arguments are concrete."""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    use_slam = _enabled(LaunchConfiguration('slam').perform(context))
    use_navigation = _enabled(LaunchConfiguration('navigation').perform(context))
    launch_fleet_services = _enabled(
        LaunchConfiguration('fleet_services').perform(context))
    startup_delay = float(LaunchConfiguration('startup_delay').perform(context))

    fleet = load_fleet(fleet_config)
    matches = [robot for robot in fleet['robots'] if robot['name'] == robot_name]
    if not matches:
        raise RuntimeError(
            f"robot '{robot_name}' is not in {fleet_config}. "
            f"Roster: {', '.join(robot['name'] for robot in fleet['robots'])}")
    robot = matches[0]

    description_share = get_package_share_directory('amr_description')
    bringup_share = get_package_share_directory('amr_bringup')
    xacro_file = os.path.join(description_share, 'urdf', 'amr.urdf.xacro')
    model_library = os.path.join(description_share, 'config', 'robot_models.yaml')

    # Command() output is XML, not YAML. ParameterValue prevents launch from
    # attempting to YAML-parse the URDF before robot_state_publisher sees it.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' robot_name:=', robot_name,
            ' robot_model:=', robot['model'],
            ' model_config:=', model_library,
        ]),
        value_type=str,
    )
    common = {'use_sim_time': True}

    # The arguments are absolute where they need to be, but this node itself
    # stays in the robot group so a fleet gets /amr1/spawn_entity and
    # /amr2/spawn_entity rather than two nodes fighting for /spawn_entity.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', f'/{robot_name}/robot_description',
            '-entity', robot_name,
            '-robot_namespace', f'/{robot_name}',
            '-x', str(robot['x']), '-y', str(robot['y']),
            '-Y', str(robot['yaw']), '-z', '0.05',
        ],
    )

    namespaced_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # URDF links are already named amrN/<link>.
                'frame_prefix': '',
                **common,
            }],
        ),
        Node(
            package='amr_sensor_bsp', executable='bsp_validation_node',
            name='bsp_validation', output='screen',
            parameters=[{
                'robot_name': robot_name, 'fleet_config': fleet_config,
                'ground_rejection_enabled': True, 'check_image_intensity': True,
                **common,
            }],
        ),
        Node(
            package='amr_fleet_control', executable='velocity_smoother_node',
            name='velocity_smoother', output='screen',
            parameters=[{
                'robot_name': robot_name, 'fleet_config': fleet_config,
                'control_rate': 50.0, **common,
            }],
        ),
        Node(
            package='amr_fleet_control', executable='safety_override_node',
            name='safety_override', output='screen',
            parameters=[{
                'robot_name': robot_name, 'fleet_config': fleet_config,
                'control_rate': 50.0, **common,
            }],
        ),
        Node(
            package='amr_fleet_control', executable='trajectory_broadcaster_node',
            name='trajectory_broadcaster', output='screen',
            parameters=[{'robot_name': robot_name, 'fleet_config': fleet_config, **common}],
        ),
    ]

    delayed_stack = []
    if use_slam:
        delayed_stack.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'slam.launch.py')),
            launch_arguments={'robot_name': robot_name, 'fleet_config': fleet_config}.items()))
    if use_navigation:
        delayed_stack.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'navigation.launch.py')),
            launch_arguments={'robot_name': robot_name, 'fleet_config': fleet_config}.items()))

    actions = []
    if launch_fleet_services:
        # Deliberately NOT namespaced: map fusion and traffic control are fleet
        # singletons living in the root namespace. Standalone robot debugging
        # still needs the shared map frame; fleet.launch.py sets this false and
        # starts them once for the whole fleet.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'fleet_control.launch.py')),
            launch_arguments={'fleet_config': fleet_config}.items()))

    # Block 1: everything that can start immediately.
    actions.append(namespaced(robot_name, [spawn_entity] + namespaced_nodes))

    # Block 2: SLAM and nav2, deferred until Gazebo has spawned the robot and
    # the diff-drive plugin is publishing odom -> base_footprint. Starting them
    # as siblings of the spawn races that plugin and produces "frame does not
    # exist" lifecycle failures.
    #
    # The TimerAction sits OUTSIDE block 1 and its children get their own
    # namespace push. Nesting the timer inside block 1's group would leave these
    # nodes in the root namespace -- see namespaced() for what that breaks.
    if delayed_stack:
        actions.append(TimerAction(
            period=startup_delay,
            actions=[
                namespaced(robot_name, [
                    LogInfo(msg=[
                        f'[{robot_name}] Gazebo/TF settle delay complete; starting ',
                        'SLAM and navigation.']),
                    *delayed_stack,
                ]),
            ],
        ))

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', description='Roster name, e.g. amr1.'),
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(bringup_share, 'config', 'fleet.yaml'),
            description='Fleet roster. Swap for fleet_ten_robots.yaml to scale up.'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument(
            'fleet_services', default_value='true',
            description='Start global map fusion/traffic services. fleet.launch.py sets false.'),
        DeclareLaunchArgument(
            'startup_delay', default_value='3.0',
            description='Seconds for Gazebo spawn and odometry TF before SLAM/nav2 start.'),
        OpaqueFunction(function=_robot_nodes),
    ])

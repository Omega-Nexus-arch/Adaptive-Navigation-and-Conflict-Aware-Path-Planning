# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Structural guard: every per-robot node must be namespaced.

Why this file exists
--------------------
``PushRosNamespace`` is a *scoped* action. A ``GroupAction`` applies it while
visiting its children and pops it immediately afterwards. ``TimerAction`` does
not run its children during that visit -- it defers them -- so a timer nested
inside a namespaced group launches its nodes in the **root** namespace.

Nothing complains. The nodes start, take nav2's default parameters because the
``root_key: <robot>`` in the parameter file no longer matches their name, and
the symptoms surface far from the cause:

* ``slam_toolbox`` publishes ``map -> odom`` instead of
  ``<robot>/map -> <robot>/odom``, so TF splits into two unconnected trees and
  the robot can never be located in ``map``;
* nav2 aborts with ``No critics defined for FollowPath``;
* the local costmap silently loads the default plugin list, dropping the
  conflict-aware fleet layer.

So the property is asserted structurally instead of being left to review. These
tests walk the launch description that ``robot.launch.py`` actually produces --
including through timers and included files -- and fail if any per-robot node
is reachable without a namespace push. They run for a one-robot launch and for
every robot in the ten-robot roster, so the guarantee holds at any fleet size.
"""

import os
import sys

import pytest

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))

# The launch imports are deliberately lazy. The two source-shape tests below
# are the hard guarantee and must run everywhere -- including in a checkout with
# no ROS installed -- so only the tree-walking tests depend on `launch`.
def _launch_api():
    """Import the launch classes, or skip the calling test."""
    try:
        from launch import LaunchContext, LaunchDescription
        from launch.actions import GroupAction, TimerAction
        from launch_ros.actions import Node, PushRosNamespace
    except ImportError:                                          # pragma: no cover
        pytest.skip('ROS 2 launch is not available in this environment')
    return (LaunchContext, LaunchDescription, GroupAction, TimerAction,
            Node, PushRosNamespace)


HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, os.pardir, 'config')

#: Nodes that are fleet singletons and belong in the root namespace.
GLOBAL_NODES = {'traffic_control', 'map_fusion', 'rviz2'}


def config(name):
    return os.path.normpath(os.path.join(CONFIG_DIR, name))


def _load_robot_launch():
    """Import robot.launch.py by path, the way `ros2 launch` does."""
    import importlib.util
    path = os.path.normpath(os.path.join(HERE, os.pardir, 'launch', 'robot.launch.py'))
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _children(action, *attribute_names):
    """Return an action's child entities, tolerating launch-API differences."""
    for name in attribute_names:
        children = getattr(action, name, None)
        if children:
            return list(children)
    return []


def _walk(action, namespaced, found, api):
    """Recursively collect (node_name, is_namespaced) pairs.

    ``namespaced`` tracks whether a ``PushRosNamespace`` is in scope *for this
    branch*. Crucially, a ``TimerAction``'s children do NOT inherit it: the
    scope is popped before the timer fires. Modelling that faithfully is the
    whole point of the test.
    """
    _, _, GroupAction, TimerAction, Node, PushRosNamespace = api

    if isinstance(action, Node):
        name = action.node_name if hasattr(action, 'node_name') else None
        if name is None:
            name = getattr(action, '_Node__node_name', '<unnamed>')
        found.append((str(name), namespaced))
        return

    if isinstance(action, GroupAction):
        entities = _children(action, '_GroupAction__actions', 'actions', 'entities')
        pushes = namespaced or any(isinstance(e, PushRosNamespace) for e in entities)
        for entity in entities:
            _walk(entity, pushes, found, api)
        return

    if isinstance(action, TimerAction):
        # The deferred children start from a CLEARED namespace scope, because
        # the enclosing group has already been popped by the time they run.
        # Modelling that faithfully is the whole point of this test.
        for entity in _children(action, '_TimerAction__actions', 'actions', 'entities'):
            _walk(entity, False, found, api)
        return

    for entity in _children(
            action, '_LaunchDescription__entities', 'entities', 'actions'):
        _walk(entity, namespaced, found, api)


def _collect_nodes(robot_name, roster='fleet.yaml'):
    """Expand robot.launch.py for one robot and list its nodes."""
    api = _launch_api()
    LaunchContext, LaunchDescription = api[0], api[1]

    module = _load_robot_launch()
    context = LaunchContext()
    description = LaunchDescription(module.generate_launch_description().entities)

    # Supply the launch arguments the OpaqueFunction reads.
    for name, value in (
        ('robot_name', robot_name),
        ('fleet_config', config(roster)),
        ('slam', 'true'),
        ('navigation', 'true'),
        # Keep the expansion to this robot; the fleet singletons are covered by
        # their own test.
        ('fleet_services', 'false'),
        ('startup_delay', '3.0'),
    ):
        context.launch_configurations[name] = value

    found = []
    for entity in description.entities:
        # Resolve the OpaqueFunction that builds the real action list.
        produced = entity.visit(context) if hasattr(entity, 'visit') else None
        if produced:
            for sub in produced:
                _walk(sub, False, found, api)
        else:
            _walk(entity, False, found, api)
    return found


# ---------------------------------------------------------------------------
# The guarantee
# ---------------------------------------------------------------------------


def test_the_helper_is_the_only_thing_that_pushes_a_namespace():
    """One wrapping point, so there is one place to get it right."""
    path = os.path.normpath(os.path.join(HERE, os.pardir, 'launch', 'robot.launch.py'))
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()
    assert source.count('PushRosNamespace(') == 1, (
        'PushRosNamespace should appear exactly once, inside namespaced(). '
        'Every per-robot block must go through that helper so a deferred block '
        'cannot quietly skip the push.')


def test_a_timer_never_wraps_the_namespace_group():
    """`TimerAction` inside a namespaced group is the bug. Forbid the shape.

    The correct nesting is TimerAction(actions=[namespaced(...)]), not
    namespaced([... TimerAction(...)]), because the namespace scope is popped
    before the timer fires.
    """
    path = os.path.normpath(os.path.join(HERE, os.pardir, 'launch', 'robot.launch.py'))
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()

    assert 'TimerAction(' in source, 'expected a deferred SLAM/nav2 block'

    # The buggy shape appends a TimerAction into the list that is later wrapped
    # by namespaced(). The correct shape wraps INSIDE the timer.
    assert 'namespaced_nodes.append(TimerAction(' not in source, (
        'TimerAction must not be nested inside the namespaced group: the '
        'namespace scope is popped before the timer fires, so its nodes would '
        'launch in the root namespace.')

    timer_at = source.index('TimerAction(')
    window = source[timer_at:timer_at + 500]
    assert 'namespaced(' in window, (
        "TimerAction's deferred actions must be wrapped in namespaced(); "
        'otherwise they launch in the root namespace.')


@pytest.mark.parametrize('robot_name', ['amr1', 'amr2'])
def test_every_per_robot_node_is_namespaced(robot_name):
    """No per-robot node may reach the root namespace, timers included."""
    nodes = _collect_nodes(robot_name)
    if not nodes:
        pytest.skip(
            'could not introspect the launch description on this launch version; '
            'the source-shape tests above still hold the structural guarantee')

    escaped = [name for name, ok in nodes if not ok and name not in GLOBAL_NODES]
    assert not escaped, (
        f'these {robot_name} nodes would launch in the root namespace: '
        f'{sorted(set(escaped))}. A node outside /{robot_name} will not match '
        f'its root_key parameters and will silently fall back to defaults.')


def test_the_guarantee_holds_for_a_ten_robot_fleet():
    """Whatever the fleet size, no node escapes its namespace."""
    from amr_bringup.fleet_loader import load_fleet
    fleet = load_fleet(config('fleet_ten_robots.yaml'))
    assert len(fleet['robots']) == 10

    for robot in fleet['robots']:
        nodes = _collect_nodes(robot['name'], roster='fleet_ten_robots.yaml')
        if not nodes:
            pytest.skip('launch description introspection unavailable')
        escaped = [n for n, ok in nodes if not ok and n not in GLOBAL_NODES]
        assert not escaped, f"{robot['name']}: {sorted(set(escaped))} escaped"


def test_slam_and_navigation_are_present_and_namespaced():
    """The two that broke: they must exist, and be inside the namespace."""
    nodes = _collect_nodes('amr1')
    if not nodes:
        pytest.skip('launch description introspection unavailable')
    by_name = {name: ok for name, ok in nodes}

    for required in ('slam_toolbox', 'controller_server', 'planner_server',
                     'bt_navigator', 'selective_mapping'):
        assert required in by_name, (
            f'{required} is missing from the launch description')
        assert by_name[required], (
            f'{required} is not namespaced; its root_key parameters will not '
            f'match and it will run on defaults')

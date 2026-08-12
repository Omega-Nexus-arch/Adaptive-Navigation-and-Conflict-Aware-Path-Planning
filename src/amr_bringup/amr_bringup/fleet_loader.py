# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Read the fleet roster and model library from launch files.

The C++ side has ``amr_core::FleetConfig`` for this. Launch is Python, and
launch has to know the roster *before* any node starts in order to decide how
many nodes to create at all. This module is that reader.

It deliberately mirrors the C++ loader's validation rather than trusting the
file: duplicate names and duplicate priorities are rejected here too, so a bad
roster fails at launch description construction with a readable message
instead of producing a half-started fleet whose nodes then throw one by one.

Style: PEP 8, checked by ament_flake8.
"""

import os

import yaml


class FleetConfigError(RuntimeError):
    """Raised for a malformed roster or model library."""


def _unwrap(document):
    """Accept both a bare mapping and the ``fleet:``/``ros__parameters:`` wrappers."""
    node = document.get('fleet', document)
    if isinstance(node, dict) and 'ros__parameters' in node:
        node = node['ros__parameters']
    return node


def load_model_library(path):
    """Load ``robot_models.yaml`` and return {model_name: properties}."""
    if not os.path.isfile(path):
        raise FleetConfigError(f"model library not found: {path}")
    with open(path, 'r', encoding='utf-8') as handle:
        models = yaml.safe_load(handle)
    if not isinstance(models, dict) or not models:
        raise FleetConfigError(f"model library is empty or malformed: {path}")
    return models


def load_fleet(path):
    """Load and validate a fleet roster.

    Returns a dict with keys ``robots``, ``policy``, ``global_frame`` and
    ``models``, where each robot entry has its model block resolved under
    ``model_properties``.
    """
    if not os.path.isfile(path):
        raise FleetConfigError(f"fleet config not found: {path}")
    with open(path, 'r', encoding='utf-8') as handle:
        fleet = _unwrap(yaml.safe_load(handle))

    robots = fleet.get('robots')
    if not robots:
        raise FleetConfigError(f"{path}: needs a non-empty `robots:` list")

    library_path = fleet.get('model_library')
    if not library_path:
        raise FleetConfigError(f"{path}: needs a `model_library:` path")
    if not os.path.isabs(library_path):
        library_path = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(path)), library_path))
    models = load_model_library(library_path)

    seen_names = set()
    seen_priorities = set()
    resolved = []

    for index, robot in enumerate(robots):
        where = f"{path}: robots[{index}]"
        name = robot.get('name')
        model = robot.get('model')
        if not name:
            raise FleetConfigError(f"{where}: missing 'name'")
        if not model:
            raise FleetConfigError(f"{where}: missing 'model'")
        if name in seen_names:
            # The name is used verbatim as a ROS namespace, so a duplicate
            # would silently cross-wire two robots' cmd_vel.
            raise FleetConfigError(f"{where}: duplicate robot name '{name}'")
        if '/' in name:
            raise FleetConfigError(
                f"{where}: robot name '{name}' must not contain '/'")
        if model not in models:
            raise FleetConfigError(
                f"{where}: model '{model}' is not defined in {library_path}. "
                f"Available: {', '.join(sorted(models))}")

        priority = robot.get('yield_priority', models[model].get('yield_priority', 0))
        if priority in seen_priorities:
            raise FleetConfigError(
                f"{where}: yield_priority {priority} is already used. Priorities "
                f"must be unique or the yielding protocol is non-deterministic.")

        seen_names.add(name)
        seen_priorities.add(priority)

        entry = dict(robot)
        entry['yield_priority'] = priority
        entry['model_properties'] = models[model]
        entry.setdefault('x', 0.0)
        entry.setdefault('y', 0.0)
        entry.setdefault('yaw', 0.0)
        resolved.append(entry)

    return {
        'robots': resolved,
        'policy': fleet.get('policy', {}),
        'global_frame': fleet.get('global_frame', 'map'),
        'models': models,
        'model_library_path': library_path,
    }


def nav2_substitutions(robot, global_frame, elevation_map_path):
    """Build the per-robot nav2 parameter overrides.

    Every value here is derived from the model library, so tuning a robot's
    physics in one YAML file automatically retunes its planner. Before this
    existed the same numbers were duplicated into a per-robot nav2 file and
    drifted the first time anyone touched them.
    """
    model = robot['model_properties']
    name = robot['name']
    prefix = f"{name}/"

    max_vel_x = float(model['max_vel_x'])
    max_vel_theta = float(model['max_vel_theta'])
    footprint_radius = float(model['footprint_radius'])
    inscribed = float(model.get('inscribed_radius', footprint_radius * 0.6))

    # The controller is given the model's *acceleration* ceiling directly. The
    # jerk limit is not expressed here because nav2's controllers have no
    # concept of jerk; that is precisely why the custom velocity smoother sits
    # downstream of the controller rather than being configured into it.
    return {
        'use_sim_time': 'True',
        'global_frame': global_frame,
        'robot_base_frame': f"{prefix}base_footprint",
        'odom_topic': f"/{name}/odom",
        'robot_radius': str(footprint_radius),
        'inflation_radius': str(round(footprint_radius + 0.20, 3)),
        'inscribed_radius': str(inscribed),
        'max_vel_x': str(max_vel_x),
        'min_vel_x': str(float(model.get('min_vel_x', -0.4))),
        'max_vel_theta': str(max_vel_theta),
        'max_speed_xy': str(max_vel_x),
        'min_speed_xy': str(float(model.get('min_vel_x', -0.4))),
        'acc_lim_x': str(float(model['max_accel_x'])),
        'acc_lim_theta': str(float(model['max_accel_theta'])),
        'decel_lim_x': str(-float(model['max_decel_x'])),
        'decel_lim_theta': str(-float(model['max_accel_theta'])),
        'robot_name': name,
        'elevation_map': elevation_map_path,
        # Climbing ability differs per model, so the same ramp can be costly
        # for one robot and lethal for another.
        'max_traversable_angle_degrees': str(
            float(model.get('max_traversable_angle_degrees', 16.0))),
    }

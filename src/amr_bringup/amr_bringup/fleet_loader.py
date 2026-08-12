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


def _find_in_source_tree(anchor_file, package, relative):
    """Walk up from ``anchor_file`` looking for a sibling ``<package>/<relative>``.

    Lets the config load straight from a source checkout with nothing sourced --
    which is what makes the loader usable from a unit test, and what lets someone
    reading the repository run it without building first.
    """
    directory = os.path.dirname(os.path.abspath(anchor_file))
    while True:
        candidate = os.path.join(directory, package, relative)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(directory)
        if parent == directory:
            return None
        directory = parent


def resolve_package_uri(uri, anchor_file=None):
    """Resolve ``package://<pkg>/<relative>`` to an absolute path.

    Two lookups, in order:

    1. **The ament index** (``AMENT_PREFIX_PATH``) -- the install space. This is
       what runs when the workspace is sourced, i.e. in production.
    2. **The surrounding source tree** -- walk up from ``anchor_file`` looking
       for a sibling package directory.

    Both are needed. Without (1) the config cannot work once installed; without
    (2) it cannot be loaded from a checkout that has not been built, which would
    make the unit tests depend on a sourced workspace.

    Returns None when the scheme does not match, so callers fall through to the
    absolute/relative forms.
    """
    prefix = 'package://'
    if not uri.startswith(prefix):
        return None
    remainder = uri[len(prefix):]
    if '/' not in remainder:
        raise FleetConfigError(
            f"malformed package URI '{uri}': expected package://<pkg>/<path>")
    package, relative = remainder.split('/', 1)

    # 1. Install space, via the ament index.
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(get_package_share_directory(package), relative)
        if os.path.isfile(candidate):
            return candidate
    except Exception:                                        # noqa: BLE001
        pass
    for entry in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if not entry:
            continue
        candidate = os.path.join(entry, 'share', package, relative)
        if os.path.isfile(candidate):
            return candidate

    # 2. Source tree.
    if anchor_file is not None:
        candidate = _find_in_source_tree(anchor_file, package, relative)
        if candidate is not None:
            return candidate

    raise FleetConfigError(
        f"cannot resolve '{uri}'. Looked in AMENT_PREFIX_PATH (is {package} built "
        f"and the workspace sourced?) and in the source tree above "
        f"{anchor_file or '<no anchor given>'}.")


def resolve_library_path(anchor_file, path):
    """Resolve a model-library reference to an absolute path.

    Three accepted forms, in precedence order:

    1. ``package://amr_description/config/robot_models.yaml`` -- resolved via
       the ament index. **This is the form the shipped configs use**, because it
       is the only one that works in both the source tree and the install space.
    2. an absolute path -- used verbatim.
    3. a path relative to the referring file -- convenient for test fixtures
       that sit beside their model library.

    Form 3 is what ``fleet.yaml`` originally shipped with, and it was wrong: a
    relative hop out of ``install/amr_bringup/share/amr_bringup/config/`` lands
    in ``install/amr_bringup/share/``, not in ``amr_description``'s install
    prefix. It worked from the source tree and broke the moment the workspace
    was installed.
    """
    resolved = resolve_package_uri(path, anchor_file)
    if resolved is not None:
        return resolved
    if os.path.isabs(path):
        return path
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(anchor_file)), path))


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
    library_path = resolve_library_path(path, library_path)
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

    # NOTE ON WHAT IS *NOT* HERE.
    #
    # No frame names. RewrittenYaml rewrites by key name, replacing every
    # occurrence in the file with one value -- which is fine for a radius and
    # wrong for a frame. `global_frame` is `map` in the global costmap and
    # `<ns>/odom` in the rolling local costmap; a blanket rewrite would anchor
    # the local costmap to the map frame and make it lurch on every SLAM
    # correction. `local_frame` in behavior_server has no counterpart elsewhere
    # and would simply never be rewritten, leaving an unprefixed `odom` that
    # does not resolve for a namespaced robot.
    #
    # Frames therefore use `<ns>` placeholders substituted textually in
    # navigation.launch.py, where each occurrence keeps its own value.
    #
    # The controller is given the model's *acceleration* ceiling directly. The
    # jerk limit is not expressed here because nav2's controllers have no
    # concept of jerk; that is precisely why the custom velocity smoother sits
    # downstream of the controller rather than being configured into it.
    del global_frame, prefix   # Frames go through <ns>, not through here.
    return {
        'use_sim_time': 'True',
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

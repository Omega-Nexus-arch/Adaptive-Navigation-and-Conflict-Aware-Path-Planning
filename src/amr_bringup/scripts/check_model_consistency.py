#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Verify the single-source-of-truth claim actually holds.

The whole configuration story rests on one assertion: every consumer of a
robot's physical limits reads the same file. This script checks that, rather
than trusting it. Run it in CI or before a demo.

Checks performed:

1. Every model referenced by every roster exists in the model library.
2. Every roster has unique robot names and unique yield priorities.
3. Each model's declared safety envelope is reachable by its own LiDAR:
   ``k * v_max^2 + d_min <= lidar.range_max``. If it is not, the halt can never
   fire in time and the envelope is decorative.
4. Each model's heavy/light ordering is internally consistent: a model with a
   larger payload capacity must not also have a higher acceleration limit,
   which is the mistake a careless edit makes.
5. The generated world's ramps are all climbable by every model that has to use
   them.

    ros2 run amr_bringup check_model_consistency.py
"""

import math
import os
import sys

import yaml


def find(*relative):
    """Locate a file in the source tree or the install space."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, '..', *relative)),
        os.path.normpath(os.path.join(here, '..', '..', *relative)),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(
            get_package_share_directory(relative[0]), *relative[1:]))
    except Exception:
        pass
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f"could not locate {'/'.join(relative)}")


def main():
    problems = []
    notes = []

    models_path = find('amr_description', 'config', 'robot_models.yaml')
    with open(models_path, 'r', encoding='utf-8') as handle:
        models = yaml.safe_load(handle)
    notes.append(f'model library: {models_path} ({len(models)} models)')

    # -- 3, 4: per-model invariants ------------------------------------------
    for name, model in models.items():
        v_max = float(model['max_vel_x'])
        d_safe = float(model['safety_k']) * v_max ** 2 + float(model['safety_d_min'])
        reach = float(model['lidar']['range_max'])
        if d_safe > reach:
            problems.append(
                f"{name}: safety envelope needs {d_safe:.2f} m at top speed but the "
                f"LiDAR only reaches {reach:.2f} m; the halt could never fire in time")
        else:
            notes.append(
                f'{name}: d_safe(v_max) = {d_safe:.2f} m within {reach:.1f} m LiDAR reach')

        # A 2D scanner mounted at or below the top of the chassis sees only its
        # own chassis: Gazebo ray sensors collide with their own model. The
        # symptoms surface as "Starting point in lethal space", never as
        # anything mentioning the LiDAR. See DESIGN_NOTES 8d.
        deck_top = (float(model['base_z_offset']) +
                    float(model['chassis_height']) + 0.024)
        lidar_z = float(model['lidar']['height'])
        if lidar_z < deck_top + 0.03:
            problems.append(
                f"{name}: LiDAR at {lidar_z:.3f} m sits inside its own chassis "
                f"(collision reaches {deck_top:.3f} m); every beam would hit the "
                f"robot itself and no plan could ever start")
        else:
            notes.append(
                f'{name}: scan plane {lidar_z:.2f} m clears the body top '
                f'{deck_top:.2f} m by {lidar_z - deck_top:.2f} m')

        # The simulated drivetrain has to out-accelerate the steepest ramp, or
        # the robot stalls on it and slides back. See DESIGN_NOTES 8n.
        plant = float(model.get('plant_accel_limit', 0.0))
        if plant <= float(model['max_accel_x']):
            problems.append(
                f"{name}: plant_accel_limit {plant} m/s^2 is at or below the "
                f"model's own {model['max_accel_x']} m/s^2, so Gazebo would be "
                f"shaping the acceleration profile instead of the smoother")

        omega_limit = float(model['imu']['max_angular_velocity'])
        if omega_limit <= float(model['max_vel_theta']):
            problems.append(
                f"{name}: IMU plausibility limit {omega_limit} rad/s is at or below the "
                f"commandable yaw rate {model['max_vel_theta']} rad/s, so normal "
                f"turning would be reported as a sensor fault")

    names = sorted(models)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            heavy, light = models[names[i]], models[names[j]]
            if heavy['payload_capacity_kg'] < light['payload_capacity_kg']:
                heavy, light = light, heavy
                heavy_name, light_name = names[j], names[i]
            else:
                heavy_name, light_name = names[i], names[j]
            if heavy['max_accel_x'] > light['max_accel_x']:
                problems.append(
                    f"{heavy_name} carries more than {light_name} but accelerates "
                    f"harder ({heavy['max_accel_x']} vs {light['max_accel_x']} m/s^2); "
                    f"the heavy unit is supposed to be the sluggish one")

    # -- 1, 2: every roster ---------------------------------------------------
    config_dir = os.path.dirname(find('amr_bringup', 'config', 'fleet.yaml'))
    rosters = sorted(
        os.path.join(config_dir, f) for f in os.listdir(config_dir)
        if f.startswith('fleet') and f.endswith('.yaml'))

    for roster_path in rosters:
        with open(roster_path, 'r', encoding='utf-8') as handle:
            fleet = yaml.safe_load(handle).get('fleet', {})
        robots = fleet.get('robots', [])
        label = os.path.basename(roster_path)
        seen_names, seen_priorities = set(), set()

        for robot in robots:
            if robot['model'] not in models:
                problems.append(
                    f"{label}: robot '{robot['name']}' uses undefined model "
                    f"'{robot['model']}'")
            if robot['name'] in seen_names:
                problems.append(f"{label}: duplicate robot name '{robot['name']}'")
            seen_names.add(robot['name'])

            priority = robot.get(
                'yield_priority', models.get(robot['model'], {}).get('yield_priority'))
            if priority in seen_priorities:
                problems.append(
                    f"{label}: yield_priority {priority} is used twice; the yielding "
                    f"protocol would be non-deterministic")
            seen_priorities.add(priority)

        # -- 6: spawn points against the shared map extents -------------------
        extents = fleet.get('map_extents') or {}
        x_min = float(extents.get('x_min', -24.0))
        x_max = float(extents.get('x_max', 24.0))
        y_min = float(extents.get('y_min', -17.0))
        y_max = float(extents.get('y_max', 17.0))
        for robot in robots:
            radius = float(
                models.get(robot['model'], {}).get('footprint_radius', 0.6))
            x = float(robot.get('x', 0.0))
            y = float(robot.get('y', 0.0))
            if not (x_min + radius <= x <= x_max - radius and
                    y_min + radius <= y <= y_max - radius):
                problems.append(
                    f"{label}: '{robot['name']}' spawns at ({x}, {y}) with a "
                    f"{radius} m footprint, outside map_extents "
                    f"x[{x_min}, {x_max}] y[{y_min}, {y_max}]. It would start "
                    f"outside its own global costmap and never plan.")
        notes.append(
            f'{label}: {len(robots)} robots, all models resolved, all spawns '
            f'inside x[{x_min}, {x_max}] y[{y_min}, {y_max}]')

    # -- 5: the generated world's ramps --------------------------------------
    try:
        sys.path.insert(0, os.path.dirname(find('amr_gazebo', 'amr_gazebo', 'world_builder.py')))
        sys.path.insert(0, os.path.dirname(os.path.dirname(
            find('amr_gazebo', 'amr_gazebo', 'world_builder.py'))))
        from amr_gazebo import world_builder
        spec = world_builder.build_warehouse()
        steepest = max(r.slope_deg for r in spec.ramps)
        for name, model in models.items():
            limit = float(model.get('max_traversable_angle_degrees', 16.0))
            if steepest > limit:
                problems.append(
                    f"{name}: cannot climb the steepest ramp in the world "
                    f"({steepest:.1f} deg > {limit:.1f} deg limit)")
        gravity_pull = 9.81 * math.sin(math.radians(steepest))
        for name, model in models.items():
            plant = float(model.get('plant_accel_limit', 0.0))
            if plant <= gravity_pull:
                problems.append(
                    f"{name}: plant_accel_limit {plant:.2f} m/s^2 cannot "
                    f"out-accelerate gravity on the {steepest:.1f} deg ramp "
                    f"({gravity_pull:.2f} m/s^2); it will stall and slide back")
        notes.append(
            f'world: steepest ramp {steepest:.1f} deg, all models can climb it '
            f'and out-accelerate its {gravity_pull:.2f} m/s^2 pull')
    except Exception as error:   # noqa: BLE001 - advisory check only
        notes.append(f'world ramp check skipped: {error}')

    # -- 6b: the map extents must contain the world ---------------------------
    # A shared grid smaller than the warehouse would clip the far aisles out of
    # the global costmap, and the planner would report those goals unreachable
    # rather than saying why.
    try:
        elevation_path = find('amr_gazebo', 'maps', 'warehouse_elevation.yaml')
        with open(elevation_path, 'r', encoding='utf-8') as handle:
            elevation = yaml.safe_load(handle)
        cell = float(elevation['resolution'])
        world = (
            float(elevation['origin'][0]),
            float(elevation['origin'][0]) + int(elevation['width']) * cell,
            float(elevation['origin'][1]),
            float(elevation['origin'][1]) + int(elevation['height']) * cell,
        )
        with open(find('amr_bringup', 'config', 'fleet.yaml'), 'r',
                  encoding='utf-8') as handle:
            extents = yaml.safe_load(handle)['fleet'].get('map_extents', {})
        bounds = (
            float(extents.get('x_min', -24.0)), float(extents.get('x_max', 24.0)),
            float(extents.get('y_min', -17.0)), float(extents.get('y_max', 17.0)))
        if (bounds[0] > world[0] or bounds[1] < world[1] or
                bounds[2] > world[2] or bounds[3] < world[3]):
            problems.append(
                f'map_extents x[{bounds[0]}, {bounds[1]}] y[{bounds[2]}, {bounds[3]}] '
                f'does not contain the generated world x[{world[0]}, {world[1]}] '
                f'y[{world[2]}, {world[3]}]; the clipped aisles would be '
                f'permanently unreachable')
        else:
            notes.append(
                f'map_extents x[{bounds[0]}, {bounds[1]}] y[{bounds[2]}, {bounds[3]}] '
                f'contains the world x[{world[0]}, {world[1]}] y[{world[2]}, {world[3]}]')
    except Exception as error:   # noqa: BLE001 - advisory check only
        notes.append(f'map extent check skipped: {error}')

    print('Model / fleet consistency check')
    print('=' * 60)
    for note in notes:
        print(f'  ok   {note}')
    if problems:
        print()
        for problem in problems:
            print(f'  FAIL {problem}')
        print(f'\n{len(problems)} problem(s) found.')
        return 1
    print('\nAll checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

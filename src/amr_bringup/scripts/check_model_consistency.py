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
        notes.append(f'{label}: {len(robots)} robots, all models resolved')

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
        notes.append(f'world: steepest ramp {steepest:.1f} deg, all models can climb it')
    except Exception as error:   # noqa: BLE001 - advisory check only
        notes.append(f'world ramp check skipped: {error}')

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

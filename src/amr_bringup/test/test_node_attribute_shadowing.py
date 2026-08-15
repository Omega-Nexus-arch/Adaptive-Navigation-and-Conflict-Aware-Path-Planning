# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""No node may assign to an attribute its base class already owns.

Why this file exists
--------------------
`rclpy.node.Node` exposes several read-only properties -- `clients`,
`publishers`, `subscriptions`, `services`, `timers`, `guards`, `waitables`,
`context`, `handle`. They are natural names for a dictionary of things a node
holds, which is exactly the problem: `self.clients = {}` reads perfectly and
raises

    AttributeError: can't set attribute 'clients'

the moment the constructor runs. Nothing catches it earlier, because Python
only resolves the property at assignment time. A demo script that had never
been executed end to end therefore shipped broken, and the failure surfaced as
"the robots don't move" during a scripted acceptance run.

This is the same shape as the C++ `Run()` collision recorded in DESIGN_NOTES
7b, in a different language: a subclass silently colliding with a base-class
member. That one cost a build; this one cost a demo. Both are cheap to prevent
and expensive to diagnose, so both are now asserted.

The check is source-level and needs no ROS, so it runs in a bare checkout. When
`rclpy` *is* importable, one extra test re-derives the reserved list from the
real class, so the hardcoded copy cannot silently go stale.
"""

import ast
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir))

#: Read-only properties of `rclpy.node.Node` (Humble). Verified against the
#: real class by `test_the_reserved_list_matches_rclpy` when ROS is present.
RESERVED = {
    'clients',
    'context',
    'default_callback_group',
    'guards',
    'handle',
    'publishers',
    'services',
    'subscriptions',
    'timers',
    'waitables',
}


def python_sources():
    """Every Python file in the workspace, as (relative path, text)."""
    for root, directories, files in os.walk(SRC):
        directories[:] = [d for d in directories
                          if d not in ('__pycache__', 'build', 'install', 'log')]
        for name in sorted(files):
            if not name.endswith('.py'):
                continue
            path = os.path.join(root, name)
            with open(path, 'r', encoding='utf-8') as handle:
                yield os.path.relpath(path, SRC), handle.read()


def node_classes(tree):
    """Classes that inherit from `Node` (directly or as `rclpy.node.Node`)."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        for base in node.bases:
            name = base.id if isinstance(base, ast.Name) else (
                base.attr if isinstance(base, ast.Attribute) else None)
            if name == 'Node':
                yield node
                break


def assigned_attributes(class_node):
    """`self.<name>` targets assigned anywhere in the class body."""
    for node in ast.walk(class_node):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets = [node.target]
        for target in targets:
            if (isinstance(target, ast.Attribute) and
                    isinstance(target.value, ast.Name) and
                    target.value.id == 'self'):
                yield target.attr, node.lineno


def test_no_node_shadows_a_read_only_property_of_its_base_class():
    checked = 0
    for path, source in python_sources():
        tree = ast.parse(source, filename=path)
        for class_node in node_classes(tree):
            checked += 1
            for attribute, line in assigned_attributes(class_node):
                assert attribute not in RESERVED, (
                    f'{path}:{line}: {class_node.name} assigns to '
                    f'`self.{attribute}`, which rclpy.node.Node exposes as a '
                    f'read-only property. This raises AttributeError the first '
                    f'time the constructor runs. Rename it -- e.g. '
                    f'`{attribute}_by_name` or `goal_{attribute}`.')
    assert checked >= 4, f'expected several Node subclasses, found {checked}'


def test_the_reserved_list_matches_rclpy():
    """Keeps the hardcoded list honest wherever ROS is actually installed."""
    try:
        from rclpy.node import Node
    except ImportError:                                          # pragma: no cover
        pytest.skip('rclpy is not available in this environment')

    actual = {
        name for name in dir(Node)
        if not name.startswith('_') and
        isinstance(getattr(Node, name, None), property) and
        getattr(Node, name).fset is None
    }
    missing = actual - RESERVED
    assert not missing, (
        f'rclpy.node.Node has read-only properties this test does not guard: '
        f'{sorted(missing)}. Add them to RESERVED.')


def test_every_demo_script_parses_and_defines_a_main():
    """A script nobody ran is a script nobody tested. This is the floor.

    It would not have caught the `clients` collision on its own -- that needs a
    constructor to run -- which is precisely why the check above is source-level
    rather than import-level.
    """
    scripts = os.path.join(SRC, 'amr_bringup', 'scripts')
    entries = [f for f in sorted(os.listdir(scripts)) if f.endswith('.py')]
    assert entries, 'expected the demo scripts to exist'

    for name in entries:
        with open(os.path.join(scripts, name), 'r', encoding='utf-8') as handle:
            tree = ast.parse(handle.read(), filename=name)
        functions = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
        assert 'main' in functions, f'{name} has no main()'


def test_no_script_selects_a_logger_severity_through_a_variable():
    """`level = logger.info if ok else logger.error` is a latent crash.

    rclpy caches severity against the *caller location*. Both branches resolve
    to one source line, so the first failure after a success raises
    `ValueError: Logger severity cannot be changed between calls` -- from
    inside an action result callback, which kills the executor and takes the
    script down while goals are still in flight. It is invisible until a goal
    actually fails, which is why it survived until now.
    """
    import glob
    import os
    import re

    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.normpath(os.path.join(here, os.pardir, os.pardir))

    pattern = re.compile(
        r'=\s*(?:self\.)?get_logger\(\)\.\w+\s+if\b|'
        r'=\s*(?:self\.)?get_logger\(\)\.\w+\s*$')
    offenders = []
    for path in glob.glob(os.path.join(src, '**', '*.py'), recursive=True):
        if os.sep + 'test' + os.sep in path:
            continue
        with open(path, 'r', encoding='utf-8') as handle:
            for number, line in enumerate(handle, 1):
                if pattern.search(line.split('#', 1)[0]):
                    offenders.append(f'{os.path.basename(path)}:{number}')

    assert not offenders, (
        'a logger method is being bound to a variable and called from one '
        f'line with two severities: {offenders}. Call the logger directly on '
        f'each branch instead.'
    )

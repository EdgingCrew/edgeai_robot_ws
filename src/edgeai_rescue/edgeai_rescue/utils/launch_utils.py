"""Scenario selection shared by all launch files.

A scenario (edgeai_robot_description/scenarios/<name>.yaml) lists the robots and names the world,
map and target files that belong together. The active one comes from the EDGEAI_SCENARIO
environment variable (a scenario name or a path to a scenario file), default `midterm`, so the
separately started launch files (simulation, Nav2, controllers, coordinator) always agree.
"""
import os

from ament_index_python import get_package_share_directory
import yaml

DEFAULT_SCENARIO = 'midterm'


def scenario_file(name=None):
    name = name or os.environ.get('EDGEAI_SCENARIO') or DEFAULT_SCENARIO
    if name.endswith('.yaml'):
        return os.path.abspath(name)
    return os.path.join(get_package_share_directory('edgeai_robot_description'), 'scenarios', f'{name}.yaml')


def load_scenario(name=None):
    """{'name', 'file', 'robots', 'world' (file name in worlds/), 'map' and 'targets' (absolute paths)}."""
    path = scenario_file(name)
    if not os.path.exists(path):
        available = sorted(f[:-5] for f in os.listdir(os.path.dirname(path)) if f.endswith('.yaml'))
        raise FileNotFoundError(f'scenario file {path} not found; available: {", ".join(available)}')
    with open(path) as f:
        data = yaml.safe_load(f)
    description = get_package_share_directory('edgeai_robot_description')
    return {
        'name': os.path.splitext(os.path.basename(path))[0],
        'file': path,
        'robots': data['robots'],
        'world': data['world'],
        'map': os.path.join(description, 'maps', data['map']),
        'targets': os.path.join(get_package_share_directory('edgeai_rescue'), 'config', data['targets']),
    }


def load_robots():
    return load_scenario()['robots']

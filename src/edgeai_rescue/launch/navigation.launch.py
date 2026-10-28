import math
import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from edgeai_rescue.utils.launch_utils import load_robots
from ament_index_python.packages import get_package_share_directory


# The packaged edgeai_arena map was recorded from robot1's spawn pose, so its
# map frame sits at world (-1.5, -1.5) with no rotation: map = world + (1.5, 1.5).
MAP_ORIGIN_IN_WORLD = (-1.5, -1.5)


def params_with_initial_pose(params_file, robot):
    """Copy of params_file with AMCL starting at the robot's spawn pose (map frame)."""
    with open(params_file) as f:
        params = yaml.safe_load(f)
    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': robot['x'] - MAP_ORIGIN_IN_WORLD[0],
        'y': robot['y'] - MAP_ORIGIN_IN_WORLD[1],
        'z': 0.0,
        'yaw': robot['yaw'],
    }
    out = tempfile.NamedTemporaryFile(
        'w', prefix=f'nav2_{robot["name"]}_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(params, out)
    return out.name


def generate_launch_description():

    pkg_share = get_package_share_directory('edgeai_rescue')
    pkg_description = get_package_share_directory('edgeai_robot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    ROBOTS = load_robots()

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_description, 'maps', 'edgeai_arena.yaml'),
        description='Map YAML shared by all robots'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Open one Nav2 RViz window per robot'
    )

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    actions = [map_arg, rviz_arg]
    for robot in ROBOTS:
        name = robot['name']

        # map_server + AMCL + navigation servers in /<name>, using /<name>/tf
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': name,
                'use_namespace': 'true',
                'map': LaunchConfiguration('map'),
                'params_file': params_with_initial_pose(params_file, robot),
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items()
        ))

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
            ),
            launch_arguments={
                'namespace': name,
                'use_namespace': 'true',
                'rviz_config': os.path.join(
                    pkg_nav2_bringup, 'rviz', 'nav2_namespaced_view.rviz'),
                'use_sim_time': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))
        ))

    return LaunchDescription(actions)

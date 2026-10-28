import math
import os
import tempfile

import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


ROBOTS = [
    {'name': 'robot1', 'x': -1.5, 'y': -1.5, 'yaw': 0.0},
    {'name': 'robot2', 'x': 1.5, 'y': 1.5, 'yaw': math.pi},
]


def generate_launch_description():

    pkg_share = get_package_share_directory('edgeai_rescue')
    pkg_description = get_package_share_directory('edgeai_robot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
 
    node = Node(
        package='edgeai_rescue',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'rescue.yaml'), {
            'target_path_file': os.path.join(pkg_share, 'config', 'targets_example.yaml'),
            'period': 0.5,
            'robot_names': [r['name'] for r in ROBOTS],
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([node])

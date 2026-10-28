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

from edgeai_rescue.utils.launch_utils import load_robots


def generate_launch_description():

    pkg_share = get_package_share_directory('edgeai_rescue')
    pkg_description = get_package_share_directory('edgeai_robot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    ROBOTS = load_robots()

    description = []
    for robot in ROBOTS:
        name = robot['name']
        description.append(Node(
            package='edgeai_rescue',
            executable='robot_controller_node',
            name=f'{name}_controller',
            namespace=name,
            output='screen',
            parameters=[{
                'global_frame': 'map',
                'base_frame': 'base_link',
                'period': 0.5,
                'use_sim_time': True,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ))



    return LaunchDescription(description)

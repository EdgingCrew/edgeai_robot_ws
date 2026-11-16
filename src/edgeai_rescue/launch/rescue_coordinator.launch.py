import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from edgeai_rescue.utils.launch_utils import load_scenario


def generate_launch_description():
    pkg_share = get_package_share_directory('edgeai_rescue')
    scenario = load_scenario()  # EDGEAI_SCENARIO, default midterm

    args = [
        DeclareLaunchArgument('targets', default_value=scenario['targets'],
                              description=f'Target list (map frame), scenario {scenario["name"]}'),
        DeclareLaunchArgument('map', default_value=scenario['map'],
                              description='Map YAML used for travel times (same map as Nav2)'),
        DeclareLaunchArgument('scheduler', default_value='cpsat', description='cpsat | greedy'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='Start once all robot poses are known instead of waiting for /start_rescue'),
    ]

    node = Node(
        package='edgeai_rescue',
        executable='rescue_coordinator',
        name='rescue_coordinator',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'rescue_coordinator.yaml'), {
            'robot_names': [r['name'] for r in scenario['robots']],
            'targets_file': LaunchConfiguration('targets'),
            'map_yaml': LaunchConfiguration('map'),
            'scheduler': LaunchConfiguration('scheduler'),
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': True,
        }],
    )
    return LaunchDescription([*args, node])

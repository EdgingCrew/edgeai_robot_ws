import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import PushRosNamespace, SetRemap

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('edgeai_robot_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace to run SLAM for'
    )

    # SLAM toolbox in the robot namespace, using that robot's TF tree.
    # Publishes /<name>/map and map -> odom on /<name>/tf.
    slam = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_slam_toolbox,
                    'launch',
                    'online_async_launch.py'
                )
            ),
            launch_arguments={
                'slam_params_file': os.path.join(
                    pkg_share,
                    'config',
                    'slam_toolbox.yaml'
                ),
                'use_sim_time': 'true',
            }.items()
        ),
    ])

    return LaunchDescription([
        namespace_arg,
        slam,
    ])

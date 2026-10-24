import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('edgeai_robot_description')

    # Topics in this config are relative, so they resolve inside the namespace
    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'realistic_robot.rviz'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace to visualize'
    )

    # RViz for one robot, reading that robot's /<name>/tf tree
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['-d', rviz_config],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    return LaunchDescription([
        namespace_arg,
        rviz,
    ])

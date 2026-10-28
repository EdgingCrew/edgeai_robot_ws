import math
import os
from os import path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro
import yaml

from edgeai_rescue.utils.launch_utils import load_robots



# Nodes in a robot namespace use that robot's TF topics
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def robot_nodes(robot, xacro_file, rviz_launch_file):

    name = robot['name']

    # Convert Xacro to robot_description with namespaced Gazebo topics
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'namespace': name}
    )

    # Publish robot TF on /<name>/tf and /<name>/tf_static
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name,
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
        }],
        remappings=TF_REMAPPINGS
    )

    # Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-topic', f'/{name}/robot_description',
            '-x', str(robot['x']),
            '-y', str(robot['y']),
            '-z', '0.01',
            '-Y', str(robot['yaw']),
        ],
        output='screen'
    )

    # ROS 2 <-> Gazebo bridge for this robot's topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{name}_bridge',
        output='screen',
        arguments=[
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # LiDAR
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # RGB-D camera
            f'/{name}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ]
    )

    # Static TF for Gazebo RGB-D camera sensor frame
    # (Gazebo names it <model>/<link>/<sensor>)
    camera_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_sensor_tf',
        namespace=name,
        output='screen',
        remappings=TF_REMAPPINGS,
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id',
            f'{name}/base_footprint/rgbd_camera',
        ]
    )

    # One RViz window per robot
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        launch_arguments={'namespace': name}.items(),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [
        robot_state_publisher,
        spawn_robot,
        bridge,
        camera_sensor_tf,
        rviz,
    ]


def generate_launch_description():

    # Package paths
    package_name = 'edgeai_robot_description'

    pkg_share = get_package_share_directory(package_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # New realistic robot model
    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'edgeai_robot_real.urdf.xacro'
    )

    # Reuse the existing Gazebo world
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'empty.sdf'
    )

    rviz_launch_file = os.path.join(
        pkg_share,
        'launch',
        'rviz.launch.py'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open one RViz window per robot'
    )

    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Topics shared by all robots
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
    )
    path = os.path.join(get_package_share_directory('edgeai_robot_description'), 'config', 'robots.yaml')
    with open(path) as f:
        ROBOTS = yaml.safe_load(f)['robots']
        
    robots = [
        node
        for robot in ROBOTS
        for node in robot_nodes(robot, xacro_file, rviz_launch_file)
    ]

    return LaunchDescription([
        rviz_arg,
        gazebo,
        bridge,
        *robots,
    ])

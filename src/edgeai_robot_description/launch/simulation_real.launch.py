import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # Package paths
    package_name = 'edgeai_robot_description'

    pkg_share = get_package_share_directory(package_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without its GUI (server only)'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Start SLAM toolbox online async mapping'
    )

    slam_params_file = os.path.join(
        pkg_share,
        'config',
        'slam_toolbox.yaml'
    )

    nav_arg = DeclareLaunchArgument(
        'nav',
        default_value='true',
        description='Start Nav2 navigation servers (navigate_to_pose)'
    )

    nav2_params_file = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )

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

    # Convert Xacro to robot_description
    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

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
            'gz_args': [
                '-r ',
                PythonExpression([
                    "'-s ' if '",
                    LaunchConfiguration('headless'),
                    "'.lower() == 'true' else ''"
                ]),
                world_file
            ]
        }.items()
    )

    # Publish robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn the new robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'edgeai_robot_real',
            '-topic', 'robot_description',
            '-x', '-1.5',
            '-y', '-1.5',
            '-z', '0.01'
        ],
        output='screen'
    )

    # ROS 2 <-> Gazebo bridge for DiffDrive
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # LiDAR
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # RGB-D camera
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

        ]
    )

    # Static TF for Gazebo RGB-D camera sensor frame
    camera_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id',
            'edgeai_robot_real/base_footprint/rgbd_camera',
        ]
    )

    # SLAM toolbox mapping on simulation time
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_slam_toolbox,
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Nav2 navigation on simulation time; SLAM provides map -> odom
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2_bringup,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': RewrittenYaml(
                source_file=nav2_params_file,
                param_rewrites={
                    'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(
                        pkg_share, 'config', 'navigate_to_pose_edgeai.xml'),
                },
                convert_types=True,
            ),
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav'))
    )

    return LaunchDescription([
        headless_arg,
        slam_arg,
        nav_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        camera_sensor_tf,
        slam,
        nav2,
    ])
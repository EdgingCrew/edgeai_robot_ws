"""Two EdgeAI robots in the rescue arena (worlds/rescue_arena.sdf), each with its
own SLAM toolbox and Nav2.

Every robot lives in its own namespace (/robot1, /robot2) with its own TF tree
(/robotN/tf). Maps (/robotN/map) are merged by multirobot_map_merge and the
robots explore together with explore_lite (m-explore-jazzy workspace), see
README.md "Two robots: map merging and exploration".

use_merged_map:=true (default) points each robot's global costmap at the merged
map of map_merge (/robotN/merged_map, frame world), so a robot can plan to
places only the other robot has seen. It needs map_merge running with
config/map_merge_params.yaml: the costmap waits for world -> map from it.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import LifecycleNode, Node, PushROSNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_common.launch import ReplaceString, RewrittenYaml

from ament_index_python.packages import get_package_share_directory

import xacro


# Spawn poses in the start area, facing into the arena. They are also map_merge's
# init_pose_* (config/map_merge_params.yaml): keep both in sync.
ROBOTS = [
    {'name': 'robot1', 'x': 0.5, 'y': 0.5, 'yaw': 1.5708},
    {'name': 'robot2', 'x': 1.1, 'y': 0.5, 'yaw': 1.5708},
]


def robot_actions(robot, pkg_share, pkg_nav2_bringup, use_merged_map, use_rviz):
    name = robot['name']
    # Gazebo model name, also the prefix of the camera's point cloud frame
    model_name = f'edgeai_robot_{name}'
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    robot_description = xacro.process_file(
        os.path.join(pkg_share, 'urdf', 'edgeai_robot_real.urdf.xacro'),
        mappings={'namespace': name},
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        remappings=remappings,
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', model_name,
            '-topic', 'robot_description',
            '-x', str(robot['x']),
            '-y', str(robot['y']),
            '-z', '0.01',
            '-Y', str(robot['yaw']),
        ],
        output='screen',
    )

    # Same topics as simulation_real.launch.py, under /<name>. No /clock here:
    # one clock bridge for the whole world, see generate_launch_description().
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{name}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
    )

    camera_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_sensor_tf',
        output='screen',
        arguments=[
            '--frame-id', 'camera_link',
            '--child-frame-id', f'{model_name}/base_footprint/rgbd_camera',
        ],
        remappings=remappings,
    )

    scan_filter = Node(
        package='edgeai_robot_description',
        executable='scan_robot_filter',
        name='scan_robot_filter',
        output='screen',
        parameters=[{
            'other_robots': [r['name'] for r in ROBOTS if r['name'] != name],
            'use_sim_time': True,
        }],
        remappings=remappings,
    )

    # slam_toolbox is a lifecycle node: configure and activate it on startup
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            RewrittenYaml(
                source_file=os.path.join(pkg_share, 'config', 'slam_toolbox.yaml'),
                root_key=name,
                param_rewrites={},
                convert_types=True,
            ),
            {'use_sim_time': True, 'use_lifecycle_manager': False},
        ],
        # scan_filtered: without the other robots, so they leave no ghosts in the map
        remappings=remappings + [('/scan', 'scan_filtered'), ('/map', 'map')],
    )
    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # config/nav2_params.yaml with this robot's topics. navigation_launch.py puts
    # the parameters under the namespace itself.
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(
                pkg_share, 'config', 'navigate_to_pose_edgeai.xml'),
            'bt_navigator.ros__parameters.odom_topic': f'/{name}/odom',
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic':
                f'/{name}/scan',
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic':
                f'/{name}/scan',
            # Merged map of map_merge (frame world) or the robot's own SLAM map
            'global_costmap.global_costmap.ros__parameters.global_frame': PythonExpression(
                ["'world' if '", use_merged_map, "'.lower() == 'true' else 'map'"]
            ),
            'global_costmap.global_costmap.ros__parameters.static_layer.map_topic':
                PythonExpression([
                    f"'/{name}/merged_map' if '", use_merged_map,
                    f"'.lower() == 'true' else '/{name}/map'",
                ]),
        },
        convert_types=True,
    )
    # One process for all Nav2 servers: one per server exhausts CycloneDDS
    # participant indices with two robots (see README.md). The container needs the
    # whole parameter file because the costmaps run inside it.
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[
            RewrittenYaml(
                source_file=nav2_params,
                root_key=name,
                param_rewrites={},
                convert_types=True,
            ),
            {'use_sim_time': True},
        ],
        remappings=remappings,
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': name,
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    # One RViz per robot: an RViz shows one TF tree, and each robot has its own
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', ReplaceString(
            source_file=os.path.join(pkg_share, 'rviz', 'multi_robot.rviz'),
            replacements={'<robot_namespace>': f'/{name}'},
        )],
        parameters=[{'use_sim_time': True}],
        remappings=remappings,
    )

    return GroupAction([
        PushROSNamespace(name),
        robot_state_publisher,
        spawn_robot,
        bridge,
        camera_sensor_tf,
        scan_filter,
        slam,
        slam_configure,
        slam_activate,
        nav2_container,
        nav2,
        rviz,
    ])


def generate_launch_description():
    pkg_share = get_package_share_directory('edgeai_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without its GUI (server only)'
    )
    use_merged_map_arg = DeclareLaunchArgument(
        'use_merged_map',
        default_value='true',
        description='Plan on map_merge\'s merged map (needs map_merge running) '
                    'instead of each robot\'s own SLAM map'
    )
    use_merged_map = LaunchConfiguration('use_merged_map')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open one RViz per robot (rviz/multi_robot.rviz, frame world: '
                    'needs map_merge running)'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r ',
                PythonExpression([
                    "'-s ' if '",
                    LaunchConfiguration('headless'),
                    "'.lower() == 'true' else ''"
                ]),
                os.path.join(pkg_share, 'worlds', 'rescue_arena.sdf'),
            ]
        }.items()
    )

    # One /clock for the world: a bridge per robot would publish it twice,
    # and out-of-order messages step time backwards.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robots = [
        robot_actions(robot, pkg_share, pkg_nav2_bringup, use_merged_map,
                      LaunchConfiguration('rviz'))
        for robot in ROBOTS
    ]

    return LaunchDescription([
        headless_arg,
        use_merged_map_arg,
        rviz_arg,
        gazebo,
        clock_bridge,
        *robots,
    ])


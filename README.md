# EdgeAI Robot Workspace

A ROS 2 workspace for a realistic differential-drive robot in Gazebo, with RViz visualization, LiDAR, an RGB-D camera, and SLAM toolbox mapping.

## Environment

- Ubuntu 24.04 (x86-64).
- Gazebo Harmonic.
- ROS 2 Jazzy managed by **Pixi**, using the RoboStack Jazzy and conda-forge channels in `pixi.toml`.
- Pixi installed and available on `PATH`; a graphical desktop for Gazebo and RViz.

The manifest and lockfile define the workspace dependencies. Use the Pixi environment for ROS commands and builds; the instructions below do not require sourcing `/opt/ros/jazzy/setup.bash`.

## Install and build

```bash
git clone https://github.com/dongjw-hit/edgeai_robot_ws.git
cd edgeai_robot_ws
pixi install
pixi run colcon build --symlink-install
```

Run these commands from the repository root. Rebuild after changing package resources or adding configuration files.

## Start the simulation

From the repository root:

```bash
./start_sim.sh
```

The portable realistic startup has been tested successfully. AMCL localization remains a local experiment; its launch and parameter files are not included in this repository. Installed AMCL dependencies do **not** mean that localization or autonomous navigation is complete.

## Robots

The robots are listed in `ROBOTS` in the `robots.yaml` config file inside `edgeai_robot_description`.

Each robot gets:

- its own topic namespace, for example `/robot2/cmd_vel` and `/robot2/scan`;
- its own TF tree on `/<name>/tf` and `/<name>/tf_static`, with unprefixed frame names (`odom`, `base_footprint`, `lidar_link`, ...). This follows the Nav2 multi-robot convention.

There is no global `/tf` and no frame shared between robots. Any node that needs a robot's transforms must run in that robot's namespace and remap the TF topics, as `launch/rviz.launch.py` and `launch/slam.launch.py` do:

```python
Node(..., namespace='robot1',
     remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])
```

## ROS commands in another terminal

In each additional terminal, enter the cloned repository and activate its environment:

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source install/setup.bash
```

Replace `/path/to/edgeai_robot_ws` with your clone location. The following commands assume this setup and a running simulation.

### Main topics

Per-robot topics are under the robot's namespace; `<name>` is `robot1` or `robot2`.

| Topic | ROS message type | Purpose |
| --- | --- | --- |
| `/<name>/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `/<name>/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive odometry |
| `/<name>/robot_description` | `std_msgs/msg/String` | Robot URDF |
| `/<name>/joint_states` | `sensor_msgs/msg/JointState` | Simulated joint states |
| `/<name>/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan |
| `/<name>/camera/image` | `sensor_msgs/msg/Image` | RGB image |
| `/<name>/camera/depth_image` | `sensor_msgs/msg/Image` | Depth image |
| `/<name>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration |
| `/<name>/camera/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point cloud |
| `/<name>/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms, including `odom` → `base_footprint` |
| `/<name>/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed robot and sensor transforms |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/<name>/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy map when SLAM toolbox is running for that robot |

The saved RViz configuration uses `odom` as its fixed frame. Its topic names are relative, so each RViz window shows the robot whose namespace it runs in. To inspect available topics:

```bash
ros2 topic list
ros2 topic echo /robot1/odom --once
```

### Move and stop

In clear space, publish a slow forward command to one robot:

```bash
ros2 topic pub --rate 10 /robot1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stop the publisher with **Ctrl+C**, then explicitly send a zero velocity command:

```bash
ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Stopping the publisher alone should not be treated as a robot stop command.

## Mapping with SLAM toolbox

Keep the simulation running. In another prepared terminal, start SLAM for one robot:

```bash
ros2 launch edgeai_robot_description slam.launch.py namespace:=robot1
```

This runs SLAM toolbox as `/robot1/slam_toolbox` with simulation time. It reads `/robot1/scan`, publishes `/robot1/map`, and publishes `map` → `odom` on `/robot1/tf`. The configuration in `config/slam_toolbox.yaml` uses relative names (`scan`, `base_footprint`, `odom`, `map`), so the same file works in any robot namespace, with mapping mode enabled and a map resolution of 0.05 m. Simulation time comes from the bridged `/clock` topic. The simulation startup script does not start SLAM automatically.

In that robot's RViz window, set **Fixed Frame** to `map` after SLAM starts and enable the **Map** display. Drive slowly through free space to collect scans and revisit previously observed areas. Avoid sustained contact with walls or obstacles.

### Save the occupancy map

While SLAM is running and `/robot1/map` is available, use another prepared terminal at the repository root:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/my_map" -t /robot1/map \
  --ros-args -p use_sim_time:=true
```

This writes `maps/my_map.yaml` and its associated map image. Choose a new basename for each map you want to preserve. These files are occupancy-map exports, not a serialized SLAM toolbox pose graph for resuming a mapping session.

The package also contains `src/edgeai_robot_description/maps/edgeai_arena.yaml` and its map image. Saving `maps/my_map` does not overwrite the packaged map.

## Known limitation: odometry drift during collisions

In the reported movement tests, normal straight driving stayed synchronized between Gazebo and RViz, and an explicit stop command stopped the robot. When the robot contacted a wall or obstacle, its physical motion in Gazebo stopped while odometry continued changing and the robot in RViz continued moving.

The current `/<name>/odom` source is the Gazebo DiffDrive plugin. Treat this odometry as an estimate rather than collision-corrected ground truth: wheel motion under contact or slip can diverge from actual displacement. The observed collision drift remains unresolved. Stop the robot when it becomes blocked, avoid driving into obstacles during mapping, and do not interpret continued RViz movement as physical motion in Gazebo. SLAM is not a guarantee that this behavior is corrected.

## Repository layout

```text
pixi.toml / pixi.lock                 Pixi environment and dependency lock
start_sim.sh                         Simulation + RViz entry point
src/edgeai_robot_description/
  launch/simulation_real.launch.py   Realistic simulation, robot list and bridges
  launch/rviz.launch.py              RViz for one robot namespace
  launch/slam.launch.py              SLAM toolbox for one robot namespace
  urdf/edgeai_robot_real.urdf.xacro   Current robot model
  worlds/empty.sdf                   Simulation world
  rviz/realistic_robot.rviz           RViz configuration
  config/slam_toolbox.yaml           Mapping parameters
  maps/                             Packaged map resources
src/edgeai_rescue_interfaces/        ExecuteRescue action, EstimateCosts service, RobotStatus message
src/edgeai_rescue/
  edgeai_rescue/planner_node.py      Centralized greedy task planner
  edgeai_rescue/robot_controller.py  Per-robot task execution (zone + dwell)
  edgeai_rescue/targets.py           Target list loading and rescue-zone geometry
  launch/                            Contains the needed launch files
  config/                            Nav2, rescue and example target parameters
```

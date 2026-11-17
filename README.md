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

The script resolves the workspace relative to its own location, activates Pixi, sources `install/setup.bash`, launches `simulation_real.launch.py`, and opens RViz with `realistic_robot.rviz`. It requires a completed build and does not depend on a fixed home-directory path. Press **Ctrl+C** in its terminal to stop the launched session.

The launch file loads `edgeai_robot_real.urdf.xacro`, starts the world in `worlds/empty.sdf`, spawns `edgeai_robot_real`, and starts the ROS–Gazebo bridge and robot transforms. The old simple model and its launch script have been removed; `start_sim.sh` is the current entry point.

The portable realistic startup has been tested successfully. AMCL localization remains a local experiment; its launch and parameter files are not included in this repository. Installed AMCL dependencies do **not** mean that localization or autonomous navigation is complete.

## ROS commands in another terminal

In each additional terminal, enter the cloned repository and activate its environment:

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source install/setup.bash
```

Replace `/path/to/edgeai_robot_ws` with your clone location. The following commands assume this setup and a running simulation.

### Main topics

| Topic | ROS message type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms, including `odom` → `base_footprint` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed robot and sensor transforms |
| `/joint_states` | `sensor_msgs/msg/JointState` | Simulated joint states |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan |
| `/camera/image` | `sensor_msgs/msg/Image` | RGB image |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Depth image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point cloud |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy map when SLAM toolbox is running |

The saved RViz configuration uses `odom` as its fixed frame. To inspect available topics:

```bash
ros2 topic list
ros2 topic echo /odom --once
```

### Move and stop

In clear space, publish a slow forward command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stop the publisher with **Ctrl+C**, then explicitly send a zero velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Stopping the publisher alone should not be treated as a robot stop command.

## Mapping with SLAM toolbox

Keep the simulation running. In another prepared terminal at the repository root:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/src/edgeai_robot_description/config/slam_toolbox.yaml" \
  use_sim_time:=true
```

The repository configuration uses `/scan`, `base_footprint`, `odom`, and `map`, with mapping mode enabled and a map resolution of 0.05 m. Simulation time comes from the bridged `/clock` topic. The simulation startup script does not start SLAM automatically.

In RViz, set **Fixed Frame** to `map` after SLAM starts and add a **Map** display for `/map`. Drive slowly through free space to collect scans and revisit previously observed areas. Avoid sustained contact with walls or obstacles.

### Save the occupancy map

While SLAM is running and `/map` is available, use another prepared terminal at the repository root:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/my_map" \
  --ros-args -p use_sim_time:=true
```

This writes `maps/my_map.yaml` and its associated map image. Choose a new basename for each map you want to preserve. These files are occupancy-map exports, not a serialized SLAM toolbox pose graph for resuming a mapping session.

The package also contains `src/edgeai_robot_description/maps/edgeai_arena.yaml` and its map image. Saving `maps/my_map` does not overwrite the packaged map.

## Two robots: map merging and exploration

`launch/multi_robot_simulation.launch.py` starts the same world with two robots, `robot1` at (-1.5, -1.5) and `robot2` at (1.5, 1.5). Each robot runs in its own namespace (`/robot1/scan`, `/robot1/cmd_vel`, ..., its own TF tree on `/robot1/tf`) with its own SLAM toolbox and Nav2. Map merging (`multirobot_map_merge`) and frontier exploration (`explore_lite`) come from the [m-explore-jazzy](https://github.com/EdgingCrew/m-explore-jazzy) workspace and run in its Pixi environment; both environments use CycloneDDS on domain 0, so the nodes see each other.

By default (`use_merged_map:=true`) each robot's Nav2 global costmap uses the merged map from map_merge, so a robot can plan into areas only the other robot has seen, and the robots explore the merged map together, spreading out to different frontiers. Start map_merge before or right after the simulation: the global costmaps wait for the `world -> map` transform it publishes, and until then print `Timed out waiting for transform from base_footprint to world ... frame does not exist`. With `use_merged_map:=false` each robot plans on its own SLAM map.

The launch file opens one RViz per robot (`rviz/multi_robot.rviz`, disable with `rviz:=false`): RViz shows one TF tree, and each robot has its own. Both use the fixed frame `world`, so they show the robots only once map_merge runs: the merged map, that robot's model, scan, global costmap, plan and explore_lite frontiers. The **2D Goal Pose** tool sends a goal to that robot.

Terminal 1, simulation (this repository):

```bash
pixi shell
source install/setup.bash
ros2 launch edgeai_robot_description multi_robot_simulation.launch.py
```

Terminal 2, map merging (m-explore-jazzy repository, `pixi shell`), with the robots' start poses from `config/map_merge_params.yaml`:

```bash
ros2 launch multirobot_map_merge map_merge.launch.py \
  params_file:=/path/to/edgeai_robot_ws/src/edgeai_robot_description/config/map_merge_params.yaml
```

Terminals 3 and 4, exploration (m-explore-jazzy repository, `pixi shell`):

```bash
ros2 launch explore_lite explore.launch.py namespace:=robot1 \
  params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/params_multirobot.yaml
ros2 launch explore_lite explore.launch.py namespace:=robot2 \
  params_file:=$(ros2 pkg prefix explore_lite)/share/explore_lite/config/params_multirobot.yaml
```

The merged map is `/map` in frame `world` (robot1's map frame); `rviz2 -d map_merge/launch/map_merge.rviz` from m-explore-jazzy shows it. Each robot also gets `/robotN/merged_map`, the merged map with its own map on top, which its costmap and explore_lite use.

If you change the spawn poses in the launch file (`ROBOTS`), change `init_pose_*` in `config/map_merge_params.yaml` to match: they are robot2's start pose relative to robot1's.

Notes:
- The launch file runs each robot's Nav2 servers in one process (`use_composition`). With one process per server, two robots exceed CycloneDDS's participant limit when it cannot use multicast discovery (e.g. on Wi-Fi): nodes die with "Failed to find a free participant index".
- Only one `/clock` is bridged for the world; a bridge per robot would publish it twice and step simulation time backwards.
- Two RGB-D cameras and two GPU lidars are heavy: the simulation ran at about 0.3x real time in a test. Use `headless:=true` to skip the Gazebo GUI.

## Known limitation: odometry drift during collisions

In the reported movement tests, normal straight driving stayed synchronized between Gazebo and RViz, and an explicit stop command stopped the robot. When the robot contacted a wall or obstacle, its physical motion in Gazebo stopped while `/odom` continued changing and the robot in RViz continued moving.

The current `/odom` source is the Gazebo DiffDrive plugin. Treat this odometry as an estimate rather than collision-corrected ground truth: wheel motion under contact or slip can diverge from actual displacement. The observed collision drift remains unresolved. Stop the robot when it becomes blocked, avoid driving into obstacles during mapping, and do not interpret continued RViz movement as physical motion in Gazebo. SLAM is not a guarantee that this behavior is corrected.

## Repository layout

```text
pixi.toml / pixi.lock                 Pixi environment and dependency lock
start_sim.sh                         Simulation + RViz entry point
src/edgeai_robot_description/
  launch/simulation_real.launch.py   Realistic simulation and bridges
  urdf/edgeai_robot_real.urdf.xacro   Current robot model
  worlds/empty.sdf                   Simulation world
  rviz/realistic_robot.rviz           RViz configuration
  config/slam_toolbox.yaml           Mapping parameters
  maps/                             Packaged map resources
```

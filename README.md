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

By default this loads the **Midterm Check arena** (`worlds/midterm_arena.sdf`, Figure 1 of the midterm task description): a 5 m × 5 m arena with walls W1/W1b/W2, boxes O2/O3 and two ArUco stands (S1 = ID 1, C1 = ID 2, DICT_4X4_50). The world frame equals the PDF frame (inner area x, y ∈ [0, 5] m), and both robots start in the lower-left start area. Useful launch arguments for `simulation_real.launch.py` (pass them to `./start_sim.sh`):

- `headless:=true` runs the Gazebo server only (sensors still render offscreen);
- `rviz:=false` skips the per-robot RViz windows.

### Scenarios

A scenario (`src/edgeai_robot_description/scenarios/<name>.yaml`) bundles everything that must match between the simulation, Nav2, the robot controllers and the coordinator: the robots and their spawn poses, the world, the map and the target list. Every launch file reads the active one from `EDGEAI_SCENARIO` (default `midterm`), so set it once for the whole stack:

```bash
EDGEAI_SCENARIO=rooms tmuxp load tmux_session.yaml        # or export it in every terminal
ros2 run edgeai_rescue preview_scenario rooms             # offline: first plan + picture (rooms_plan.png)
```

If a tmux server is already running, the new session does not inherit the variable; run `tmux set-environment -g EDGEAI_SCENARIO rooms` first (or `tmux kill-server`).

| Scenario | Arena | Robots | Targets | What it tests |
|---|---|---|---|---|
| `midterm` | Midterm Check, 5 × 5 m | 2 | S1, C1 | the midterm task |
| `midterm_plus` | Midterm Check, 5 × 5 m | 2 | 4 normal + 2 coop | more choices in the real arena |
| `rooms` | hall + 3 rooms, 8 × 6 m, 1.2 m doors | 3 | 6 normal + 2 coop, rule S4 → C1 | doors, a pass-through room, an ordering rule |
| `warehouse` | shelf rows, 8 × 8 m, 1.2 m aisles | 4 | 7 normal + 3 coop | 4 robots, CP-SAT at its time limit |
| `bottleneck` | two halls + one corridor, 8 × 5 m | 3 | 5 normal + 1 coop | a parking spot inside the only corridor (blocking) |

World and map are generated from the scenario's `layout` (walls as segments, obstacle boxes) and the targets' `marker` ids, so map frame = world frame. After editing a layout or a target position:

```bash
pixi run python src/edgeai_robot_description/scripts/make_world.py [scenario ...]   # default: all
pixi run python src/edgeai_robot_description/scripts/make_aruco_stands.py --ids 1-12 # stand models, if a new id is needed
```

`test/test_scenarios.py` fails when a generated file is out of date, and checks every scenario: robot starts and parking spots are clear of the inflated obstacles and of each other, every target is reachable and the first CP-SAT plan rescues everything. The old arena (`worlds/empty.sdf`, SLAM map `edgeai_arena`) needs `MAP_ORIGIN_IN_WORLD = (-1.5, -1.5)` in `navigation.launch.py` and has no scenario file.

## Task 2: rescue coordinator (CP-SAT)

Start everything with `tmuxp load tmux_session.yaml`, or by hand in separate terminals:

```bash
./start_sim.sh
ros2 launch edgeai_rescue navigation.launch.py rviz:=true      # one Nav2 stack per robot
ros2 launch edgeai_rescue robot_controller.launch.py            # per-robot execute_rescue servers
ros2 launch edgeai_rescue rescue_coordinator.launch.py          # targets:=... scheduler:=cpsat|greedy autostart:=true
ros2 service call /start_rescue std_srvs/srv/Empty              # start the run (unless autostart:=true)
```

How it works:

1. The coordinator computes travel times with Dijkstra on the map (`scheduler/travel.py`).
2. It plans who rescues which target, in which order and when, with OR-Tools CP-SAT (`scheduler/cpsat_scheduler.py`). The objective is the earliest finish of the last rescue. A greedy plan is used as the warm start and as the fallback.
3. It sends each robot to a parking point in front of the target's marker, through that robot's controller and Nav2.
4. It times the dwell itself, at 10 Hz in sim time:
   - a rescue counts after `dwell_*` seconds continuously within the target radius;
   - a cooperative target needs both robots inside at the same time;
   - leaving the radius resets the timer.
5. It replans when a visit fails, a robot runs late, a robot finishes early, or the ordering rules change.
6. It switches to "rescue as many as possible" when the remaining time is too short for all targets.
7. It plans around robots parked in corridors and doorways. At startup it finds the parking spots (and later the robot positions) that cut other legs, i.e. legs that become impossible or `block_detour` metres longer while a robot stands there (`TravelMatrix.blocking`). CP-SAT then keeps every robot driving such a leg away from the spot while it is occupied: from the parked robot's arrival until its dwell ends + `block_margin`, or for good after its last visit (`scenarios/bottleneck` shows the effect).
8. If Nav2 stops within its goal tolerance but outside the rescue radius, each resend aims a few cm deeper towards the target (`resend_shift`, `max_goal_shift`) instead of repeating the same goal.

Inputs:

- Targets: `config/targets_<scenario>.yaml` (the active scenario's), map frame. It gives the stand position, the marker direction, the rescue radius and the optional `precedence` ordering rules. The parking-point rule is in `targets.zone_points`.
- Ordering rules at runtime (Task 2B / LLM output):
  `ros2 topic pub --once /rescue_rules std_msgs/msg/String '{data: "{\"precedence\": [[\"S1\", \"C1\"]]}"}'`
  The rules are validated first: the ids must exist and there must be no cycles. Invalid rules are logged and ignored.
- Parameters: `config/rescue_coordinator.yaml`. Calibrate `nominal_speed` from the logged legs (compare `goal_sent` and `arrived` times).

Logs: every run writes `logs/task2_<timestamp>.jsonl`. It records plans, goals, arrivals, waits, dwell start and reset, rescues with planned versus actual times, failures and replans, plus robot-to-target distances at 1 Hz. To summarize a run:

```bash
ros2 run edgeai_rescue summarize_run [logs/task2_<timestamp>.jsonl]   # default: newest log in ./logs
```

Tests (plain Python, no ROS needed): `pixi run python -m pytest src/edgeai_rescue`.

The previous greedy `planner_node` is still available (`ros2 launch edgeai_rescue planner.launch.py`, using the old example targets). The tmux session starts the coordinator instead.

## Robots

The robots are listed in the active scenario file (`edgeai_robot_description/scenarios/<name>.yaml`, see [Scenarios](#scenarios)); all launch files read them through `edgeai_rescue.utils.launch_utils.load_robots()`.

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

The package also contains `src/edgeai_robot_description/maps/midterm_arena.yaml` (generated from the world) and `edgeai_arena.yaml` (SLAM-recorded in the old `empty.sdf` world). Saving `maps/my_map` does not overwrite the packaged maps.

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
  scenarios/                         Robots + world + map + targets per test scenario (EDGEAI_SCENARIO)
  worlds/                            Generated scenario worlds (midterm_arena.sdf = default); empty.sdf = old arena
  models/aruco_stand_<id>/           Target stands with ArUco DICT_4X4_50 markers 1-12
  scripts/make_world.py              Scenario layout -> world file + occupancy map
  scripts/sdf_to_map.py              World file -> occupancy map
  scripts/make_aruco_stands.py       Stand models and marker textures
  rviz/realistic_robot.rviz           RViz configuration
  config/slam_toolbox.yaml           Mapping parameters
  maps/                             Packaged map resources
src/edgeai_rescue_interfaces/        ExecuteRescue action, EstimateCosts service, RobotStatus message
src/edgeai_rescue/
  edgeai_rescue/rescue_coordinator.py  ROS node: CP-SAT rescue coordinator (Task 2)
  edgeai_rescue/mission.py           ROS-free coordinator logic: dispatch, dwell, coop sync, replanning
  edgeai_rescue/scheduler/           CP-SAT scheduler + greedy fallback, travel-time matrix (Dijkstra)
  edgeai_rescue/run_log.py           JSONL run log and summary
  edgeai_rescue/map_utils.py         Map loading, inflation, A*
  edgeai_rescue/planner_node.py      Previous greedy task planner (not started by tmux)
  edgeai_rescue/robot_controller.py  Per-robot navigation (+ stuck watchdog), publishes pose2d
  edgeai_rescue/targets.py           Target list loading, parking points, ordering-rule validation
  scripts/summarize_run              Summary of a coordinator run log
  scripts/preview_scenario           Offline first plan of a scenario, printed and drawn
  test/                              pytest unit tests (scheduler, travel, targets, mission, scenarios)
  launch/                            Contains the needed launch files
  config/                            Nav2, coordinator, target lists
```

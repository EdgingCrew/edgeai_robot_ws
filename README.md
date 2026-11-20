# EdgeAI Robot Workspace: Part 2 rescue coordinator

Multi-robot rescue in Gazebo (ROS 2 Jazzy via Pixi). A central coordinator plans who rescues which target, in which order and when, with OR-Tools CP-SAT. Each robot drives with its own Nav2 stack, and the coordinator checks the dwell rules (10 s inside the radius; both robots at the same time for cooperative targets) and replans when something goes wrong.

## Build

```bash
pixi install
pixi run colcon build --symlink-install
```

In every new terminal: `pixi shell && source install/setup.bash`

## Run
Everything is run by the `tmux_session.yaml` file, which opens the Gazebo simulation, Nav2 controllers, and all other needed nodes.
Run with
```bash
EDGEAI_SCENARIO=midterm tmuxp load tmux_session.yaml  # sim, Nav2, controllers, coordinator
ros2 service call /start_rescue std_srvs/srv/Empty    # start, once the robots are localized
ros2 run edgeai_rescue summarize_run                  # summary of the newest logs/task2_*.jsonl
```
If you forget to kill the tmux session, the map variable will not work properly.
If that happens, run
```bash
tmux kill-server
```

Ordering rules at runtime (validated: known ids, no cycles):

```bash
ros2 topic pub --once /rescue_rules std_msgs/msg/String '{data: "{\"precedence\": [[\"S1\", \"C1\"]]}"}'
```

## Scenarios

`EDGEAI_SCENARIO` picks the robots, world, map and targets for every launch file (default `midterm`):

| Scenario | Robots | Targets | Tests |
|---|---|---|---|
| `midterm` | 2 | S1, C1 | the midterm arena |
| `midterm_plus` | 2 | 4 normal + 2 coop | more targets in the midterm arena |
| `rooms` | 3 | 6 normal + 2 coop, rule S4 before C1 | doors, ordering rule |
| `warehouse` | 4 | 7 normal + 3 coop | 4 robots, solver time limit |
| `bottleneck` | 3 | 5 normal + 1 coop | a parking spot inside the only corridor |

```bash
ros2 run edgeai_rescue preview_scenario rooms   # first plan without the simulator -> rooms_plan.png
pixi run python src/edgeai_robot_description/scripts/make_world.py   # regenerate worlds/maps after editing scenarios/
```

## Tests

```bash
pixi run python -m pytest src/edgeai_rescue
```

## Where things are

- `src/edgeai_rescue/config/rescue_coordinator.yaml`: coordinator parameters (speed, dwell, solver limits, blocking).
- `src/edgeai_rescue/config/targets_<scenario>.yaml`: targets and `precedence` rules.
- `src/edgeai_rescue/config/nav2_params.yaml`, `navigate_to_pose_rescue.xml`: Nav2 setup per robot and its behaviour tree (short recoveries).
- `src/edgeai_robot_description/scenarios/<scenario>.yaml`: robots, layout and file names per scenario.
- `src/edgeai_rescue/edgeai_rescue/`:
  - `rescue_coordinator.py`: the coordinator node;
  - `mission.py`: dwell checks, replanning;
  - `scheduler/`: CP-SAT model, travel times;
  - `robot_overlay.py`: draws the robots into each other's costmaps, since the lidars scan above the robot bodies.

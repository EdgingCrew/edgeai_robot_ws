# ArUco marker detection demo

A small, self-contained building block for **Task 1** (exploration + ArUco target list).
It lets a simulated robot **see an ArUco marker, compute where the marker is in a fixed world
frame, and keep a target list** (ID, type, x, y, yaw) that does not change when the robot moves.

Everything here is plain Python + ROS 2 (`rclpy`, `tf2_ros`) + OpenCV. Nothing in the KTH
workspace is modified (no changes to `pixi.toml`, launch files or the world file).

---

## What it does (in 5 lines)

1. `keyboard_teleop.py` drives the simulated robot from the terminal (W/A/S/D or arrow keys) by publishing `Twist` on `/cmd_vel`, and always sends zero velocity on exit.
2. `make_marker.py` and `aruco_0.sdf` create an ArUco marker image (`DICT_4X4_50`, id 0) and a 0.15 m textured plate that we spawn into the running Gazebo world.
3. `aruco_detector.py` subscribes to `/camera/image` and `/camera/camera_info`, finds markers with `cv2.aruco`, and runs `solvePnP` (known marker size + camera intrinsics) to get the 3D pose of each marker in the camera frame.
4. It uses tf2 (at the image timestamp) to transform that pose into a fixed frame (`odom` or `map`), giving x, y and a yaw. Observations are filtered by distance and merged per marker ID, so the result stays stable while the robot drives around.
5. A 1 Hz timer prints the target list and publishes arrows + ID labels on `/aruco_targets` for RViz. Ctrl+C saves the list to a JSON file.

## How it works

```
/camera/image ──► detect marker corners (pixels)
/camera/camera_info ─► intrinsics K          │
                                             ▼
                      solvePnP(marker size, corners, K)
                                             │   pose in camera_optical_frame
                                             ▼
                  tf2 lookup at image timestamp (camera_optical_frame → odom/map)
                                             │   pose in a FIXED frame
                                             ▼
        keep observations closer than max_distance, average per marker ID
        (mean for x, y;  circular mean for yaw, so +179° and -179° do not cancel)
                                             │
                                             ▼
              target list: id, type, x, y, yaw   →  log, RViz markers, JSON file
```

Key idea: a pixel position only gives a *direction*. The known marker size (0.15 m) gives the
*distance*. tf2 then moves the result from the moving camera frame into a fixed frame, which is
why the marker coordinates stay the same while the distance to the marker changes.

Frame conventions:
- `camera_optical_frame` uses the OpenCV convention: x right, y down, z forward. The image
  `frame_id` in Gazebo is a different (body-style) frame, so we always use `camera_optical_frame`.
- The marker's yaw is the direction the marker **faces** (its normal). For the default
  placement in this demo (plate at x=0.2, facing the robot) the expected yaw is about ±180°.

## Files

| File | Purpose |
|---|---|
| `keyboard_teleop.py` | Keyboard teleoperation (W/A/S/D or arrows, space/X = stop, Q = quit) |
| `make_marker.py` | Generates a marker PNG with `cv2.aruco` |
| `aruco_0.png` | Generated marker image for ID 0 (`DICT_4X4_50`) |
| `aruco_0.sdf` | Gazebo model: thin plate with the marker as texture (0.15 m marker, 0.225 m plate) |
| `aruco_detector.py` | The detector node (the main part) |

## Prerequisites

- The KTH simulation from this repository builds and runs (Pixi environment, Gazebo Harmonic, RViz).
- `cv2` with the `aruco` module is available in the Pixi environment. Check with:
  `python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"` (needs OpenCV 4.7+).
- Every terminal below must have the workspace environment active, for example:
  ```bash
  cd ~/edgeai_robot_ws
  pixi shell
  source install/setup.bash
  ```
  (If you already set up auto-activation in `~/.bashrc`, new terminals have this already.)

## Quick start

All commands run from the `aruco_demo` folder unless stated otherwise.

**Terminal 1 – start the simulation** (as described in the main README of this repo):

```bash
cd ~/edgeai_robot_ws
./start_sim.sh
```

**Terminal 2 – put the marker into the running world:**

```bash
cd ~/edgeai_robot_ws/aruco_demo

# The SDF contains an absolute path to the PNG. Point it at YOUR home directory (once):
sed -i "s|/home/ubuntu_ros2|$HOME|g" aruco_0.sdf

# (Optional) regenerate the image: python3 make_marker.py --id 0 --out .

ros2 run ros_gz_sim create -world empty -file aruco_0.sdf -name aruco_0 -x 0.2 -y -1.5 -z 0.11
```

The robot starts at (-1.5, -1.5) looking along +x, so the plate appears about 1.7 m in front of it.
To remove it again (to respawn):

```bash
gz service -s /world/empty/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "aruco_0" type: MODEL'
```

**Terminal 3 – drive the robot:**

```bash
python3 keyboard_teleop.py
```

Do not drive into walls: the simulated odometry keeps drifting during collisions.

**Terminal 4 – run the detector** (stop with Ctrl+C, which also saves the target list):

```bash
python3 aruco_detector.py --ros-args -p use_sim_time:=true -p target_frame:=odom
```

`use_sim_time:=true` is required, otherwise the tf lookup at the image timestamp fails.
Use `target_frame:=map` instead once SLAM Toolbox is running.

**RViz** (the one started by the simulation):

1. *Add* → *By topic* → `/aruco_targets` → *MarkerArray* (Fixed Frame = `odom`, or `map` with SLAM).
2. *Add* → *By topic* → `/camera/image` → *Image*; under *Topic*, set **Reliability Policy = Best Effort**
   (sensor topics are Best Effort; with Reliable you only see "No Image").

## What you should see

- Terminal 4 prints once per second, for example:
  `TARGET id=0 x=+1.70 y=+0.02 yaw=-178 deg (n=1200)`
- In RViz a green arrow at about (1.7, 0.0) pointing toward the robot, labelled `ID 0`.
- Drive around: the distance changes but `x`, `y` stay within a few cm. Yaw is noisier (about ±10°, see limitations).

## Parameters of `aruco_detector.py`

Set with `--ros-args -p name:=value`.

| Parameter | Default | Meaning |
|---|---|---|
| `image_topic` | `/camera/image` | RGB image (rgb8) |
| `camera_info_topic` | `/camera/camera_info` | Intrinsics (read once) |
| `dictionary` | `DICT_4X4_50` | `cv2.aruco` dictionary name (the rulebook has not fixed this yet) |
| `marker_size` | `0.15` | Marker side in meters (must match the real marker) |
| `target_frame` | `map` | Fixed frame for the result (`odom` works without SLAM) |
| `optical_frame` | `camera_optical_frame` | Frame of the camera in OpenCV convention |
| `max_distance` | `3.0` | Ignore observations farther away than this (m) |
| `output_file` | `aruco_targets.json` | Saved on Ctrl+C (in the folder where you started the node) |

Output file format:

```json
[
  {"id": 0, "type": "unknown", "x": 1.70, "y": 0.02, "yaw": -3.10, "n": 1200}
]
```

`yaw` is in radians. `type` is a placeholder until the rulebook defines the marker types.

## Known limitations

- Yaw from a single small marker seen head-on is noisy (a few degrees up to about 10° at 1.5 m). Position is much more accurate.
- Targets are stored in the chosen fixed frame at observation time. With `map`, **loop closure in SLAM Toolbox can shift the map**, so stored positions can become stale. Not handled yet.
- The average weighs all observations equally (old and new, near and far).
- Occasional warnings `Lookup would require extrapolation into the future` mean one image was newer than the latest tf data and was skipped. Harmless.
- Dictionary, marker size, marker types and yaw definition are placeholders until the rulebook is final.
- Tested with a single marker (ID 0) so far.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| RViz Image shows "No Image" | Reliability Policy must be *Best Effort*; restart RViz/sim if it still fails. Check `ros2 topic hz /camera/image`. |
| No `TARGET` lines | The marker must be visible in the camera image first. Check that `target_frame` exists (`odom` without SLAM). |
| No arrow in RViz | *Namespaces* under MarkerArray is empty → nothing received; check that the detector is running (`ros2 topic hz /aruco_targets` should show about 1 Hz). |
| `odom ... does not exist` at start | tf not received yet, goes away after a second. |
| Marker plate is white/black only in Gazebo | Check the PNG path in `aruco_0.sdf` (see `sed` step above). |

## Next steps

- Run together with SLAM Toolbox in the `map` frame and handle loop-closure shifts.
- Test several markers (different IDs, positions and orientations).
- Feed the targets into autonomous exploration (Task 1) and the mission layer (Task 2).

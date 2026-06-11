# Mirte Master Lab Cleanup Robot

![Status](https://img.shields.io/badge/status-in%20progress-yellow)

# Quickstart

This quickstart covers installation and common launch patterns for all packages in the [`Mirte_Lab_Clean`](https://github.com/matt-rbt/Mirte_Lab_Clean) repository.

---

## Installation

### Desktop (simulation + robot)

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/matt-rbt/Mirte_Lab_Clean

# 2. Import external dependencies
cd ~/ros2_ws
vcs import src/ < src/mirte_lc/sources.repos

# 3. Update nested submodules
cd src/mirte-ros-packages && git submodule update --init --recursive && cd ../..

# 4. Install rosdeps and build
rosdep install -y --from-paths src/ --ignore-src --rosdistro humble
colcon build --symlink-install
```

### Robot only (no Gazebo)

The Gazebo packages are large and unnecessary on the physical robot. Use a sparse checkout to exclude them:

```bash
cd ~/mirte_ros_ws/src && mkdir mirte_lc && cd mirte_lc
git init
git remote add origin https://github.com/matt-rbt/Mirte_Lab_Clean
git sparse-checkout init --no-cone
# Edit .git/info/sparse-checkout:
#   /*
#   !mirte_lc_gazebo/
git read-tree -mu HEAD
git pull origin main
```

---

## Packages

| Package | Description |
|---|---|
| [`mirte_lc_labclean`](mirte_lc_labclean/README.md) | Behaviour tree and task orchestration |
| [`mirte_lc_nav2`](mirte_lc_nav2/README.md) | Coverage action server and path planners |
| [`mirte_lc_moveit_cpp`](mirte_lc_moveit_cpp/README.md) | MoveIt action server for arm and gripper |
| [`mirte_lc_vision`](mirte_lc_vision/README.md) | YOLO gripper camera detector |
| [`mirte_lc_perception`](mirte_lc_perception/README.md) | Depth point cloud object locator |
| [`mirte_lc_msgs`](mirte_lc_msgs/README.md) | Custom ROS 2 messages, services, and actions |
| [`mirte_lc_moveit`](mirte_lc_moveit/README.md) | MoveIt configuration for the Mirte arm |
| [`mirte_lc_gazebo`](mirte_lc_gazebo/README.md) | Gazebo simulation worlds and launch files |
| [`mirte_navigation`](mirte_navigation/README.md) | Nav2 configuration and pre-built maps |

---

## Running on the real robot

Start the full stack with a single launch file:

```bash
ros2 launch mirte_lc_labclean labclean_bringup.launch.py use_sim_time:=false
```

This brings up the behaviour tree, Nav2, MoveIt, the coverage navigator, and both perception nodes together.

Individual nodes can also be started separately:

```bash
# Behaviour tree
ros2 run mirte_lc_labclean labclean_tree

# Coverage action server
ros2 run mirte_lc_nav2 labclean_manager

# MoveIt action server
ros2 run mirte_lc_moveit_cpp moveit_action_server

# Depth object locator
ros2 run mirte_lc_perception object_locator

# YOLO gripper detector
ros2 run mirte_lc_vision yolo_detector

# Interactive arm test CLI
ros2 run mirte_lc_labclean test_node
```

---

## Running in simulation

### Launch Gazebo

```bash
ros2 launch mirte_lc_gazebo gazebo.launch.py
```

Gazebo loads the lab world from `mirte_lc_gazebo/worlds/`. The default world is the BK lab environment. Pass `world:=<name>` to select a different world file from that directory.

### Launch Nav2 with a pre-built map

```bash
ros2 launch mirte_navigation navigation.launch.py use_sim_time:=true
```

Maps are stored in `mirte_navigation/maps/`. Pass `map:=<path_to_yaml>` to use a specific map file.

### Full simulation stack

```bash
# Terminal 1 — Gazebo
ros2 launch mirte_lc_gazebo gazebo.launch.py

# Terminal 2 — Nav2 + MoveIt + behaviour tree
ros2 launch mirte_lc_labclean labclean_bringup.launch.py use_sim_time:=true
```

---

## Configuration

### `mirte_lc_labclean/config/`

| File | Purpose |
|---|---|
| `labclean.rviz` | RViz layout with LED markers, costmaps, point clouds, and arm displays |
| `octomap.yaml` | OctoMap resolution and update parameters for the 3-D obstacle layer |

### `mirte_navigation/maps/`

Pre-built occupancy grid maps (`.pgm` + `.yaml`) of the lab environment used by Nav2 for localisation and global planning. To build a new map:

```bash
ros2 launch mirte_navigation slam.launch.py use_sim_time:=true
```

Drive the robot around the environment, then save:

```bash
ros2 run nav2_map_server map_saver_cli -f mirte_navigation/maps/<map_name>
```

### `mirte_lc_gazebo/worlds/`

Gazebo world files (`.world` or `.sdf`) describing the simulated lab environment, including walls, furniture, and randomly placed lab objects. To add objects to a world, edit the corresponding file or place additional model SDF files in `mirte_lc_gazebo/models/`.

### `mirte_lc_moveit/config/`

MoveIt SRDF, joint limits, kinematics solver configuration, and planning pipeline parameters for the `mirte_arm` and `mirte_gripper` planning groups. Adjust `kinematics.yaml` to change the IK solver or tolerance, and `joint_limits.yaml` to tune velocity and acceleration limits.

---

## Visualisation

Open RViz with the pre-configured layout:

```bash
rviz2 -d mirte_lc_labclean/config/labclean.rviz
```

Open the py-trees tree viewer:

```bash
py-trees-tree-viewer --no-sandbox
```

Watch blackboard variables live:

```bash
py-trees-blackboard-watcher --list
py-trees-blackboard-watcher /battery.percentage
```

Render the behaviour tree as a graph (requires Graphviz):

```bash
py-trees-render -b mirte_lc_labclean.labclean_tree.create_root
```

An alternative Foxglove layout is available at `mirte_foxglove_config.json` in the repository root.

# Examples

Common task-level examples for the Mirte Lab Clean application.

---

## Send an arm command from the terminal

Use the interactive test CLI to move the arm or gripper without writing code.

```bash
ros2 run mirte_lc_labclean test_node
```

```
> arm_name vigilant        # move arm to the vigilant standby pose
> arm_name standby         # deploy arm for object approach
> gripper_name open        # open gripper
> gripper_name close       # close gripper
> arm_pose 0.085 0.0 0.47  # move wrist to a Cartesian position
> wrist_joint 1.57         # rotate wrist 90°
> gripper_joint 0.4        # set gripper to a specific joint value
> q                        # quit
```

See [`mirte_lc_labclean`](mirte_lc_labclean/README.md) for the full command reference.

---

## Run a coverage plan manually

Send a `NavigateCoverage` goal directly from the command line to test a specific planner without the behaviour tree:

```bash
ros2 action send_goal /labclean_navigator/coverage mirte_lc_msgs/action/NavigateCoverage \
  "{planner_type: 'SkeletonPlanner', verbose: true}"
```

Available planners: `SkeletonPlanner`, `SpanningTreePlanner`, `BousPlanner`, `CVTPlanner`.

Pause, resume, or stop a running plan:

```bash
# Pause
ros2 service call /labclean_navigator/set_state mirte_lc_msgs/srv/ServeCoverageStatus \
  "{command: 0}"

# Resume
ros2 service call /labclean_navigator/set_state mirte_lc_msgs/srv/ServeCoverageStatus \
  "{command: 1}"

# Stop
ros2 service call /labclean_navigator/set_state mirte_lc_msgs/srv/ServeCoverageStatus \
  "{command: 2}"
```

---

## Query the planar object detector

Trigger YOLO inference on the gripper camera and inspect the result:

```bash
ros2 service call /perception/planar/get_detected_objects \
  mirte_lc_msgs/srv/GetDetectedObjects "{}"
```

The response contains a `DetectedObjectArray` with normalised bounding box poses and labels (`"target"` or `"trash"`).

Watch the annotated camera stream in RViz or with `rqt_image_view`:

```bash
ros2 run rqt_image_view rqt_image_view /gripper_camera/image_annotated
```

---

## Inspect detected objects from the depth camera

Echo bounding box detections as they arrive:

```bash
ros2 topic echo /perception/depth/detected_objects
```

View the intermediate point clouds in RViz to debug the processing pipeline:

| Topic | Stage |
|---|---|
| `/obtained_pointcloud` | Raw input (camera frame) |
| `/sculpted_pointcloud` | After NaN removal |
| `/downsampled_pointcloud` | After voxel downsampling |
| `/plane_segmented_pointcloud` | After RANSAC plane removal |
| `/exclusive_pointcloud` | After costmap filtering (map frame) |
| `/object_bounding_boxes` | Final oriented bounding box markers |

---

## Use a pre-built map

Point Nav2 at an existing map instead of running SLAM:

```bash
ros2 launch mirte_navigation navigation.launch.py \
  use_sim_time:=false \
  map:=$(ros2 pkg prefix mirte_navigation)/share/mirte_navigation/maps/lab_map.yaml
```

Maps live in `mirte_navigation/maps/`. Each map consists of a `.pgm` image and a `.yaml` metadata file describing resolution and origin.

---

## Build and save a new map

Run SLAM Toolbox in online mapping mode:

```bash
ros2 launch mirte_navigation slam.launch.py use_sim_time:=false
```

Drive the robot around the environment manually or with Nav2 exploration. When the map looks complete, save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_lab_map
```

Copy the resulting `my_lab_map.pgm` and `my_lab_map.yaml` into `mirte_navigation/maps/` and update the map path in your launch arguments.

---

## Launch a specific Gazebo world

```bash
ros2 launch mirte_lc_gazebo gazebo.launch.py world:=lab_cluttered
```

World files are in `mirte_lc_gazebo/worlds/`. To spawn additional objects into a running simulation:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -file mirte_lc_gazebo/models/lab_object/model.sdf \
  -entity lab_object_1 \
  -x 1.0 -y 0.5 -z 0.0
```

---

## Watch the behaviour tree live

```bash
# Start the tree viewer in a separate terminal while the tree is running
py-trees-tree-viewer --no-sandbox
```

The viewer updates in real time and shows which branches are RUNNING, SUCCESS, or FAILURE. Useful for debugging why a task is not executing.

Watch a specific blackboard variable:

```bash
py-trees-blackboard-watcher /cloud_objects_detected
py-trees-blackboard-watcher /battery_low_warning
py-trees-blackboard-watcher /explore_status
```

---

## Render the behaviour tree as a diagram

```bash
py-trees-render -b mirte_lc_labclean.labclean_tree.create_root
# Opens a .dot / .svg file showing the full tree structure
```

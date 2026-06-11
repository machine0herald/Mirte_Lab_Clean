# Labclean package

Autonomous lab-cleaning application for the Mirte robot, built on [py_trees_ros](https://github.com/splintered-reality/py_trees_ros). The package provides a behaviour tree that coordinates exploration, object detection, arm manipulation, and coverage navigation to clean a lab environment without human intervention.

---

## Run

Start the full bringup stack (behaviour tree + Nav2 + MoveIt + perception pipeline):

```bash
ros2 launch mirte_lc_labclean labclean_bringup.launch.py use_sim_time:=false
```

Individual nodes can also be run directly:

```bash
ros2 run mirte_lc_labclean labclean_tree
ros2 run mirte_lc_labclean labclean_manager
ros2 run mirte_lc_labclean test_node
```

---

## Nodes

### `labclean_tree`

The main py_trees_ros behaviour tree. Ticks at 500 ms and drives the full task loop.

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `/explore/status` | `explore_lite_msgs/ExploreStatus` | Frontier exploration state |
| `/dashboard/cancel` | `std_msgs/Empty` | Cancel button event |
| `/dashboard/start` | `std_msgs/Empty` | Start button event |
| `/io/power/power_watcher` | battery msg | Battery state for low-battery detection |
| `/perception/depth/detected_objects` | `mirte_lc_msgs/DetectedObjectArray` | 3-D (cloud) detections from depth perception pipeline |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/io/leds/leds/set_color` (service) | `mirte_msgs/SetNeopixel` | LED strip colour |
| `/labclean_led_markers` | `visualization_msgs/Marker` | LED colour visualisation in RViz |

**Action clients**

| Action server | Type | Used by |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | `NavigateToPosition` behaviour |
| `/arm_controller/move_to_position` | `mirte_lc_msgs/MoveToPosition` | `MoveArm` behaviour |
| `/move_to_position` | `mirte_lc_msgs/MoveToPosition` | `PickObject` behaviour |
| `/labclean_navigator/coverage` | `mirte_lc_msgs/NavigateCoverage` | `CoverageTask` behaviour |

**Service clients**

| Service | Type | Used by |
|---|---|---|
| `/labclean_navigator/set_state` | `mirte_lc_msgs/ServeCoverageStatus` | `SetCoverageStatus` behaviour |
| `/perception/planar/get_detected_objects` | `mirte_lc_msgs/GetDetectedObjects` | `GetPlanarObjects` behaviour |

**Blackboard variables**

| Key | Type | Description |
|---|---|---|
| `cloud_objects_detected` | `list[DetectedObject]` | Latest detections from depth camera |
| `num_cloud_objects_detected` | `int` | Count of depth-camera detections |
| `planar_objects_detected` | `list[DetectedObject]` | Detections from planar perception service |
| `planar_objects_detected_bool` | `bool` | Set to `True` when planar objects are available |
| `explore_status` | `str` | Latest exploration state string |
| `cancel_button` | event | Set when cancel is pressed |
| `start_button` | event | Set when start is pressed |
| `battery_low_warning` | `bool` | Set by battery monitor when charge is below 25 % |

---

## Behaviour tree structure

![Behavior Tree](assets/labcleantree.png)

---

## Behaviours

### `FlashLedStrip`

Sends a colour command to the Neopixel LED strip via the `/io/leds/leds/set_color` service and publishes a flat `Marker` cube above the map origin for RViz visualisation. Always returns `SUCCESS` after one tick.

```python
behaviours.FlashLedStrip(name="Flash Red", colour=[1.0, 0.0, 0.0])
```

> **Note:** The hardware LED channel mapping is swapped relative to the message field names (`r`→blue, `g`→red, `b`→green). The behaviour compensates for this internally.

### `SetCoverageStatus`

Calls `/labclean_navigator/set_state` to pause, resume, or stop the coverage navigator. Returns `RUNNING` until the service responds, then `SUCCESS` or `FAILURE`.

```python
behaviours.SetCoverageStatus(name="Pause Coverage", requested_status="pause")
# requested_status: "pause" | "resume" | "stop"
```

### `NavigateToPosition`

Navigates the robot to a goal pose using Nav2's `BasicNavigator`. The target can be a fixed map coordinate or the closest object read from the blackboard.

```python
# Fixed target
behaviours.NavigateToPosition(name="Dock Action", target_position=[0.0, 0.0])

# Closest object from blackboard
behaviours.NavigateToPosition(name="Approach", blackboard_key="cloud_objects_detected")
```

Returns `SUCCESS` when the robot is within 0.5 m of the goal, `FAILURE` if navigation fails or if no progress is made for 15 s, and `RUNNING` otherwise.

### `MoveArm`

Sends a goal to `/move_to_position`. Accepts a named pose, a blackboard pose, or an explicit `(x, y, z)` position.

```python
behaviours.MoveArm(name="Deploy Arm", predefined_pose="standby")
behaviours.MoveArm(name="Place", predefined_pose="place_right")
```

### `PickObject`

Executes a seven-step pick-and-place sequence against the `/move_to_position` action server:

| Step | Action |
|---|---|
| `approach` | Move wrist above closest detected object |
| `open` | Open gripper |
| `dive` | Lower wrist to object |
| `grip` | Close gripper |
| `place` | Move to `place_left` (target) or `place_right` (non-target) |
| `let_go` | Open gripper |
| `standby` | Return arm to vigilant pose |

The closest object is selected by Euclidean distance from `base_link`. Returns `FAILURE` if any step is rejected or if no objects are available.

### `CoverageTask`

Sends a coverage goal with `planner_type = SKELETON` by default to `/labclean_navigator/coverage` and monitors the action until completion.

### `GetPlanarObjects`

Calls `/perception/planar/get_detected_objects` and writes results to the blackboard. Returns `FAILURE` if no objects are detected (used with `Retry` to poll up to 5×).

---

## Useful CLI commands

```bash
# Render the behaviour tree structure as a dot graph
py-trees-render -b mirte_lc_labclean.labclean_tree.create_root

# Watch all blackboard variables live
py-trees-blackboard-watcher --list

# Watch a specific blackboard variable
py-trees-blackboard-watcher /battery.percentage

# Open the graphical tree viewer
py-trees-tree-viewer --no-sandbox
```

---

## Configuration

| File | Purpose |
|---|---|
| `config/labclean.rviz` | RViz layout including LED marker, costmaps, and arm displays |
| `config/octomap.yaml` | OctoMap parameters for the 3-D obstacle layer |

---

## Dependencies

- `py_trees` / `py_trees_ros`
- `nav2_simple_commander`, `nav2_msgs`
- `mirte_msgs`, `mirte_lc_msgs`
- `explore_lite_msgs`
- `tf2_ros`
- `visualization_msgs`, `geometry_msgs`, `action_msgs`

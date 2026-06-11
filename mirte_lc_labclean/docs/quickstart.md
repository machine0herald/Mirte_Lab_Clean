# Quickstart for mirte_lc_labclean

This quickstart explains common usage for the [`mirte_lc_labclean`](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_labclean) package.

## Run

Start the full LabClean bringup stack, which launches the behaviour tree, Nav2, MoveIt, the coverage navigator, and both perception nodes:

```bash
ros2 launch mirte_lc_labclean labclean_bringup.launch.py use_sim_time:=false
```

For simulation, set `use_sim_time:=true`. Individual nodes can also be started separately:

```bash
ros2 run mirte_lc_labclean labclean_tree
ros2 run mirte_lc_labclean labclean_manager
ros2 run mirte_lc_labclean test_node
```

## Node information

### labclean_tree

py_trees_ros behaviour tree. Ticks at 500 ms and drives the full task loop: LED indication → topic subscriptions → exploration guard → object detection and handling → coverage navigation.

**Key topics and interfaces**

| Interface | Type | Direction |
|---|---|---|
| `/explore/status` | `explore_lite_msgs/ExploreStatus` | Subscribed |
| `/perception/depth/detected_objects` | `mirte_lc_msgs/DetectedObjectArray` | Subscribed |
| `/dashboard/cancel` | `std_msgs/Empty` | Subscribed |
| `/dashboard/start` | `std_msgs/Empty` | Subscribed |
| `/io/power/power_watcher` | battery | Subscribed |
| `/io/leds/leds/set_color` | `mirte_msgs/SetNeopixel` | Service client |
| `/labclean_led_markers` | `visualization_msgs/Marker` | Published |

### labclean_manager

Alias for the coverage action server. See [`mirte_lc_nav2`](../mirte_lc_nav2/README.md).

### test_node

Interactive CLI for manually commanding the arm and gripper. See [`examples`](mirte_lc_labclean_examples.md).

## Configuration

| File | Purpose |
|---|---|
| `config/labclean.rviz` | RViz layout — LED markers, costmaps, point clouds, arm displays |
| `config/octomap.yaml` | OctoMap resolution and update rate for the 3-D obstacle layer |

## Useful CLI commands

```bash
# Render the tree structure as a dot graph
py-trees-render -b mirte_lc_labclean.labclean_tree.create_root

# Watch all blackboard variables live
py-trees-blackboard-watcher --list

# Watch a specific variable
py-trees-blackboard-watcher /battery.percentage

# Live tree viewer (while tree is running)
py-trees-tree-viewer --no-sandbox
```
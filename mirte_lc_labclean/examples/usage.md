# Examples for mirte_lc_labclean

## Inspect the behaviour tree live

While `labclean_tree` is running, open the graphical viewer in a second terminal:

```bash
py-trees-tree-viewer --no-sandbox
```

The viewer updates in real time and shows which branches are RUNNING, SUCCESS, or FAILURE. Useful for understanding why a task is blocked.

## Watch blackboard variables

```bash
# List all registered keys
py-trees-blackboard-watcher --list

# Tail specific variables
py-trees-blackboard-watcher /cloud_objects_detected
py-trees-blackboard-watcher /explore_status
py-trees-blackboard-watcher /battery_low_warning
```

## Render the tree as a diagram

```bash
py-trees-render -b mirte_lc_labclean.labclean_tree.create_root
# Produces a .dot file and opens it as an SVG if Graphviz is installed
```

## Manually trigger the arm test CLI

```bash
ros2 run mirte_lc_labclean test_node
```

```
> arm_name vigilant          # move arm to standby pose
> arm_name standby           # deploy arm for approach
> gripper_name open          # open gripper
> gripper_name close         # close gripper
> arm_pose 0.085 0.0 0.47    # Cartesian wrist position
> wrist_joint 1.57           # rotate wrist 90°
> gripper_joint 0.4          # direct gripper joint value
> help                       # print command reference
> q                          # quit
```

## Simulate a start/cancel button press

The behaviour tree listens to dashboard button topics. Publish to them manually:

```bash
# Start
ros2 topic pub --once /dashboard/start std_msgs/msg/Empty "{}"

# Cancel
ros2 topic pub --once /dashboard/cancel std_msgs/msg/Empty "{}"
```

## Check LED strip output in RViz

The `FlashLedStrip` behaviour publishes a `Marker` on `/labclean_led_markers`. Add a **Marker** display in RViz pointed at that topic to see the current LED colour as a flat cube above the map. Colours map to states:

| Colour | State |
|---|---|
| Orange | Idle / startup |
| Green | Object detected, handling in progress |
| Red | Battery low, docking |
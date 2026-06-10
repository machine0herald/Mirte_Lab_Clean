# Quickstart for mirte_lc_labclean

This quickstart explains common usage for the `mirte_lc_labclean` package.

## Run (example)
Start the LabClean bringup stack, which launches the behaviour tree, Nav2 stack,
MoveIt support, and perception pipeline:

```bash
ros2 launch mirte_lc_labclean labclean_bringup.launch.py use_sim_time:=false
```

You can also run the core nodes directly:

```bash
ros2 run mirte_lc_labclean labclean_tree
ros2 run mirte_lc_labclean test_node
```

## Node information
- `labclean_tree`: primary behaviour tree node for LabClean
- `labclean_manager`: manager node that transitions exploration to coverage
- `test_node`: simple range sensor test controller

## Configuration
Use the labclean package configuration files to tune launch behavior and
sensor topics. For example, `config/labclean.rviz` and `config/octomap.yaml`.

# Quickstart for mirte_lc_gazebo

This quickstart explains common usage for the `mirte_lc_gazebo` package.

## Run (example)
Launch the Gazebo world and LabClean integration stack:

```bash
ros2 launch mirte_lc_gazebo gazebo_mirte_lc.launch.py
```

The launch file starts:
- Gazebo simulation with `floor_with_cubes_2.world`
- `labclean_tree` from `mirte_lc_labclean`
- the LabClean bringup stack via `mirte_lc_labclean`

## Node information
- `gazebo`: Gazebo simulator process
- `labclean_tree`: behaviour tree node launched as part of labclean bringup

## Configuration
The Gazebo launch uses the world file under
`worlds/floor_with_cubes_2/floor_with_cubes_2.world`.

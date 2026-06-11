# Gazebo package

## Run (example)

Launch the Gazebo world and LabClean integration stack:

```bash
ros2 launch mirte_lc_gazebo gazebo_mirte_lc.launch.py
```

## Launch Files

The launch file starts:

- Gazebo simulation with one of the worlds in the `./worlds` folder
- `labclean_tree` from [`mirte_lc_labclean`](https://github.com/matt-rbt/Mirte_Lab_Clean/blob/main/mirte_lc_labclean/mirte_lc_labclean/labclean_tree.py)
- the LabClean bringup stack via [`mirte_lc_labclean`](https://github.com/matt-rbt/Mirte_Lab_Clean/blob/main/mirte_lc_labclean/launch/labclean_bringup.launch.py)

## Worlds

The package contains a world folder with three testing worlds for lab cleanup

- [floor](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_gazebo/worlds/floor), an empty lab cleanup floor.
- [floor_with_cubes](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_gazebo/worlds/floor_with_cubes), a lab with green cubes as portable objects.
- [floor_with_cubes_2](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_gazebo/worlds/floor_with_cubes_2), a larger lab with pink cubes and more obstacles.
- [floor_with_electronics](), a lab with electronics as portable objects.


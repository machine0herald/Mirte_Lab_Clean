# Quickstart for mirte_lc_nav2

This quickstart explains common usage for the `mirte_lc_nav2` package.

## Run (example)
Launch the LabClean Nav2 stack with the coverage and exploration nodes:

```bash
ros2 launch mirte_lc_nav2 mirte_lc_nav2.launch.py use_sim_time:=false
```

This launch will start:
- `labclean_navigator` (package: `mirte_lc_nav2`)
- `planner_server`, `controller_server`, `bt_navigator`, `behavior_server` (Nav2)
- `explore_node` (package: `explore_lite`)

## Node information
- `labclean_navigator`: coverage action server for LabClean navigation
- `explore_node`: exploration planner for map discovery

## Configuration
Use the package config files in `config/`, including:
- `nav2_coverage_params.yaml`
- `fbe_params.yaml`

Common launch arguments:
- `use_sim_time` (bool): whether to use simulation clock

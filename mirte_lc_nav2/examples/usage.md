# Examples for mirte_lc_nav2

## Send a coverage goal from the command line

```bash
# Skeleton planner (default, recommended)
ros2 action send_goal /labclean_navigator/coverage \
  mirte_lc_msgs/action/NavigateCoverage \
  "{planner_type: 'SkeletonPlanner', verbose: true}"

# Spanning tree planner
ros2 action send_goal /labclean_navigator/coverage \
  mirte_lc_msgs/action/NavigateCoverage \
  "{planner_type: 'SpanningTreePlanner', verbose: true}"

# CVT planner
ros2 action send_goal /labclean_navigator/coverage \
  mirte_lc_msgs/action/NavigateCoverage \
  "{planner_type: 'CVTPlanner', verbose: false}"
```

## Pause, resume, and stop a running plan

```bash
# Pause — robot stops at its current position; remaining path is saved
ros2 service call /labclean_navigator/set_state \
  mirte_lc_msgs/srv/ServeCoverageStatus "{command: 0}"

# Resume — robot continues from the saved position
ros2 service call /labclean_navigator/set_state \
  mirte_lc_msgs/srv/ServeCoverageStatus "{command: 1}"

# Stop — aborts the goal entirely
ros2 service call /labclean_navigator/set_state \
  mirte_lc_msgs/srv/ServeCoverageStatus "{command: 2}"
```

The response includes `succeeded` (bool) and `remaining_poses` (int32, Nav2 feedback value at the time of the request).

## Watch coverage feedback

```bash
ros2 action send_goal --feedback /labclean_navigator/coverage \
  mirte_lc_msgs/action/NavigateCoverage \
  "{planner_type: 'SkeletonPlanner', verbose: true}"
```

Feedback fields: `completion_percentage`, `distance_remaining`, `current_segment`, `total_segments`.

## Run a planner standalone (without ROS, e.g. Jupyter)

The planner classes work without a ROS node for offline testing. Pass `node=None` to suppress all ROS logging and publishing:

```python
import numpy as np
from mirte_lc_nav2.navigators import SkeletonPath

# Create a planner with no ROS node
planner = SkeletonPath(node=None, resolution=0.1)

# plan() requires a nav_msgs/OccupancyGrid message object and a start position
# In a notebook, load a saved map and construct the message manually, then:
planner.plan(map_msg, start=np.array([0.0, 0.0]))

print(f"Generated {len(planner.paths)} path segments")
for i, seg in enumerate(planner.paths):
    print(f"  Segment {i}: {len(seg)} waypoints")
```

## Visualise planned paths in RViz

While the coverage server is running, add the following displays in RViz:

| Display type | Topic | Description |
|---|---|---|
| MarkerArray | `/systematic_navigator/planned_path` | Planned waypoint cubes |
| PolygonStamped | `/systematic_navigator/map_contours` | Extracted boundary contours |
| MarkerArray | `/systematic_navigator/decomposed_map` | Decomposition cells |
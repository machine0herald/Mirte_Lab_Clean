# Quickstart for mirte_lc_nav2

This quickstart explains common usage for the [`mirte_lc_nav2`](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_nav2) package.

## Run

```bash
ros2 run mirte_lc_nav2 labclean_manager
```

The node connects to Nav2's `goThroughPoses` action server and subscribes to `/global_costmap/costmap`. It is ready to accept goals once Nav2 is running.

## Node information

### labclean_action_server

**Action servers**

| Action | Type | Description |
|---|---|---|
| `/labclean_navigator/coverage` | `mirte_lc_msgs/NavigateCoverage` | Execute a full coverage plan |

**Services**

| Service | Type | Description |
|---|---|---|
| `/labclean_navigator/set_state` | `mirte_lc_msgs/ServeCoverageStatus` | Pause (`0`), resume (`1`), or stop (`2`) ongoing coverage |

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Costmap used for path planning |

**Feedback published during coverage**

| Field | Description |
|---|---|
| `completion_percentage` | Percentage of segments completed |
| `current_segment` | Index of the segment being executed |
| `total_segments` | Total number of segments in the plan |

## Available planners

| Planner string | Class | Algorithm |
|---|---|---|
| `SkeletonPlanner` | `SkeletonPath` | Medial axis skeleton → graph traversal |
| `SpanningTreePlanner` | `SpanningTreePath` | DFS spanning tree → contour circumnavigation |
| `CVTPlanner` | `CVTPath` | Centroidal Voronoi Tessellation + TSP |

## Configuration

The planner is selected per goal via the `planner_type` field of the `NavigateCoverage` action. No static configuration file is required. Planner parameters (resolution, scale, seed count) are set at construction time in `navigators.py` and can be adjusted there.
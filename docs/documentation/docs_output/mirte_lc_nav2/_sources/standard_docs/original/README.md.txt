# Coverage Navigation package

Coverage navigation package for the LabClean application. Provides the action server that executes coverage plans, a set of pluggable path planners, and shared utilities for map processing and ROS path conversion.

---

## Run

```bash
ros2 run mirte_lc_nav2 labclean_navigator
```

---

## Nodes

### `labclean_action_server`

Accepts `NavigateCoverage` goals, delegates path planning to the requested planner, and drives the robot through coverage segments using Nav2's `goThroughPoses`. Supports pause, resume, and stop at runtime via a service interface.

**Action servers**

| Action | Type | Description |
|---|---|---|
| `/labclean_navigator/coverage` | `mirte_lc_msgs/NavigateCoverage` | Execute a full coverage plan |

**Services**

| Service | Type | Description |
|---|---|---|
| `/labclean_navigator/set_state` | `mirte_lc_msgs/ServeCoverageStatus` | Pause, resume, or stop ongoing coverage |

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Live costmap used for coverage planning |

---

## Coverage execution flow

```
Receive NavigateCoverage goal
│
├── Look up robot position via TF (map → base_link)
├── Instantiate requested planner
├── planner.plan(costmap, start)
│     ├── update_map()     — threshold, contour extraction, world↔pixel conversion
│     ├── generate_path()  — planner-specific algorithm
│     └── sanitize_paths() — clamp waypoints to costmap bounds
│
├── Sort segments longest-first
│
└── For each segment:
      ├── goThroughPoses(segment)
      ├── Poll feedback loop
      │     ├── cancel requested  → canceled()
      │     ├── stop requested    → abort()
      │     ├── pause requested   → cancelTask()
      │     │     ├── save_path() — reinsert remaining segment at queue front
      │     │     └── spin until resume → goThroughPoses(remaining)
      │     └── isTaskComplete()  → advance to next segment
      └── publish_feedback (completion %, current/total segments)
```

### Pause / resume behaviour

On pause the server calls `cancelTask()` immediately, then calls `save_path()` to compute the remaining portion of the current segment before re-queuing it. On resume, `goThroughPoses` is re-issued on that saved segment. The `remaining_poses` field in the service response reflects the last feedback value from Nav2 at the time of the pause.

`save_path()` finds the resume index by computing the closest waypoint to the current robot pose (TF lookup). If that lookup fails it falls back to using `remaining_poses` to slice the tail of the segment.

---

## Planners

All planners inherit from `SystematicNavigator` and implement `generate_path(start)`. The `PLANNERS` registry maps name strings to classes.

```python
from mirte_lc_nav2.navigators import PLANNERS
# PLANNERS = {
#   "SkeletonPlanner":     SkeletonPath,
#   "SpanningTreePlanner": SpanningTreePath,
#   "StraightLinePlanner": StraightLinePath,
#   "CVTPlanner":          CVTPath,
# }
```

### `SkeletonPlanner` (default)

Computes the medial axis skeleton of the free space, converts it to a graph, and traverses between leaf nodes using shortest graph paths.

<!-- | Step | Detail |
|---|---|
| Skeletonization | `skimage.morphology.skeletonize` on the binary costmap |
| Graph | KDTree-connected waypoint graph; disconnected components bridged |
| Traversal | Greedy nearest-leaf-first over graph shortest paths |
| Multi-group | Runs once per polygon group; start of next group is end of previous | -->
![skeletonplanner image](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_nav2/docs/skeleton.png)

### `SpanningTreePlanner`

Downsamples the costmap, builds a DFS spanning tree over free cells, and circumnavigates contours around the tree.

<!-- | Step | Detail |
|---|---|
| Downsampling | `cv2.resize` with `scale` factor (default `0.06`) |
| Tree | `nx.dfs_tree` per connected component, composed into a directed graph, then edge-subdivided |
| Contours | Rectangular regions around tree nodes → `cv2.findContours` |
| Path | Contour pixels resampled from nearest point to start, then wrapped | -->

![skeletonplanner image](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_nav2/docs/tree.png)


### `CVTPlanner`

Samples coverage waypoints using Centroidal Voronoi Tessellation (Lloyd's algorithm), then solves a nearest-neighbour TSP over the centroids.

| Parameter | Default | Description |
|---|---|---|
| `n_seeds` | `30` | Number of Voronoi cells |
| `n_iterations` | `20` | Lloyd iterations |

### `StraightLinePath`

Generates a single diagonal straight-line trajectory from the start pose. Useful for testing and calibration only.

---

## `SystematicNavigator` — shared base class

All planners share this base. Key shared methods:

| Method | Description |
|---|---|
| `plan(map_msg, start)` | Entry point: calls `update_map` → `generate_path` → `sanitize_paths` |
| `update_map(map_msg)` | Thresholds occupancy grid, extracts and groups polygon contours, stores `self.polymap` |
| `sanitize_paths()` | Clamps all waypoints to valid costmap bounds; removes duplicate points introduced by clamping |
| `set_waypoints(waypoints)` | Builds a KDTree-connected `nx.Graph`; bridges disconnected components |
| `find_leaf_nodes(graph)` | Returns nodes with degree 1 |
| `find_nearest_node(graph, pos, nodes)` | Nearest node by Euclidean distance |
| `world_to_pixel_poly(polygon)` | Map-frame coordinates → costmap pixel indices |
| `pixel_to_world_poly(polygon)` | Costmap pixel indices → map-frame coordinates |

All planners also follow this pattern for retrieving a map
![skeletonplanner image](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_nav2/docs/navigator.png)

**Published topics** (when a `node` is provided)

| Topic | Type | Description |
|---|---|---|
| `/systematic_navigator/map_contours` | `geometry_msgs/PolygonStamped` | Extracted map boundary contours |
| `/systematic_navigator/decomposed_map` | `visualization_msgs/MarkerArray` | Decomposition cells (if applicable) |
| `/systematic_navigator/planned_path` | `visualization_msgs/MarkerArray` | Planned waypoint path |

---

## Utilities

### `to_ros_path(points, frame_id, spacing)`

Converts a list of `(x, y)` points into a `nav_msgs/Path` by removing duplicates (< 1 mm apart) and uniformly resampling to the requested spacing.

```python
from mirte_lc_nav2.utils import to_ros_path
path = to_ros_path(segment, frame_id="map", spacing=0.1)
```

| Argument | Default | Description |
|---|---|---|
| `points` | — | Sequence of `(x, y)` tuples |
| `frame_id` | `"map"` | TF frame for output poses |
| `spacing` | `0.1` m | Target distance between output poses |

### `log(node, msg_type, msg)`

Thin wrapper around the ROS logger. Falls back to `print()` when `node` is `None` (e.g. running planners in Jupyter).

```python
from mirte_lc_nav2.utils import log, LogType
log(node, LogType.INFO, "planning complete")
log(None, LogType.WARN, "running without ROS node")
```

| `LogType` | ROS level |
|---|---|
| `INFO` | `get_logger().info` |
| `WARN` | `get_logger().warn` |
| `ERR` | `get_logger().error` |
| `DEBUG` | `get_logger().debug` |

---

## Dependencies

- `rclpy`, `tf2_ros`
- `nav2_simple_commander` (`BasicNavigator`, `PyCostmap2D`)
- `mirte_lc_msgs`
- `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`
- `numpy`, `opencv-python` (`cv2`)
- `networkx`
- `scikit-image` (`skimage.morphology`)
- `scipy` (`KDTree`, `Voronoi`)
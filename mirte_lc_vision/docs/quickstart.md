# Quickstart for mirte_lc_vision

This quickstart explains common usage for the [`mirte_lc_vision`](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_vision) package.

## Run

```bash
# 3D depth-based object locator
ros2 run mirte_lc_vision object_locator

# 2D YOLO gripper camera classifier
ros2 run mirte_lc_vision yolo_detector
```

Both nodes are typically started together as part of the bringup launch. They can also run independently.

## Prerequisites

**object_locator**
- Depth camera publishing on `/camera/depth/points` (BEST_EFFORT QoS)
- Nav2 providing `/global_costmap/costmap`
- TF chain: `map` → `base_link` → camera optical frame

**yolo_detector**
- Gripper camera publishing on `/camera/color/image_raw`
- YOLO weights at `mirte_lc_vision/models/ColourdetectionYOLO26n.pt`

## Node information

### object_locator

Processes depth point clouds, removes ground planes via iterative RANSAC, transforms surviving points into the map frame, clusters with DBSCAN, and publishes oriented bounding boxes.

Startup is deferred by 5 s to allow TF to become available.

**Subscribed topics**

| Topic | Type | QoS |
|---|---|---|
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 10 |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | default |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/perception/depth/detected_objects` | `mirte_lc_msgs/DetectedObjectArray` | Detected objects in the map frame |
| `/object_bounding_boxes` | `visualization_msgs/MarkerArray` | OBB markers for RViz (10 s lifetime, green) |
| `/obtained_pointcloud` | `sensor_msgs/PointCloud2` | Raw input (camera frame) |
| `/sculpted_pointcloud` | `sensor_msgs/PointCloud2` | After NaN removal |
| `/downsampled_pointcloud` | `sensor_msgs/PointCloud2` | After downsampling |
| `/plane_segmented_pointcloud` | `sensor_msgs/PointCloud2` | After RANSAC plane removal |
| `/exclusive_pointcloud` | `sensor_msgs/PointCloud2` | After costmap filtering (map frame) |

**Key parameters (set in source)**

| Parameter | Value | Description |
|---|---|---|
| Startup delay | 5 s | Wait before registering subscriptions |
| Processing interval | 2 s | How often the queue is drained |
| DBSCAN `eps` | 0.03 m | Cluster neighbourhood radius |
| DBSCAN `min_samples` | 20 | Minimum points per cluster |
| RANSAC `distance_threshold` | 0.003 m | Plane inlier distance |
| RANSAC plane mask | 0.008 m | Points within this distance of plane are removed |
| Costmap occupancy threshold | 5 | Cost at or above which a cell is treated as occupied |

---

### yolo_detector

Runs YOLO inference on the gripper camera feed on demand. Detection is triggered by a service call rather than continuously streamed, which suits the behaviour tree's polling pattern.

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gripper camera feed (BGR8) |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/gripper_camera/image_annotated` | `sensor_msgs/Image` | YOLO-annotated frame, published on each service call |

**Services**

| Service | Type | Description |
|---|---|---|
| `/perception/planar/get_detected_objects` | `mirte_lc_msgs/GetDetectedObjects` | Run inference on the latest frame and return detections |

**Model**

| Property | Value |
|---|---|
| Weights | `mirte_lc_vision/models/ColourdetectionYOLO26n.pt` |
| Confidence threshold | 0.3 |
| Input size | 640 px |
| Inference device | CUDA if available, otherwise CPU |
| Target classes | `green`, `red`, `purple` → label `"target"` |
| All other classes | → label `"trash"` |
| Coordinate system | Normalised image coordinates (`xywhn`) — **not** map frame |

## Configuration

Both nodes are configured by editing constants directly in their source files. After any change, rebuild with:

```bash
colcon build --packages-select mirte_lc_vision --symlink-install
```
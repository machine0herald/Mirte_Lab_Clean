# Perception package

Perception package for the LabClean application. Provides two independent nodes: a depth point cloud processor that detects and localises objects in the map frame, and a YOLO-based gripper camera classifier that identifies object classes for sorting decisions.

---

## Run

```bash
# Depth-based object locator
ros2 run mirte_lc_perception object_locator

# YOLO gripper camera detector
ros2 run mirte_lc_vision classifier_2d
```

---

## Nodes

### `object_locator`

Processes depth point clouds from `/camera/depth/points`, removes ground and planar surfaces via iterative RANSAC, transforms surviving points into the map frame, clusters them with DBSCAN, and publishes oriented bounding boxes as both RViz markers and `DetectedObjectArray` messages.

Startup is deferred by 5 s to allow TF to become available before subscriptions are registered.

**Subscribed topics**

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 10 | Depth point cloud from the camera |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | default | Costmap for rejecting detections inside walls |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/perception/depth/detected_objects` | `mirte_lc_msgs/DetectedObjectArray` | Detected objects in the map frame |
| `/object_bounding_boxes` | `visualization_msgs/MarkerArray` | OBB markers for RViz (10 s lifetime, green) |
| `/obtained_pointcloud` | `sensor_msgs/PointCloud2` | Raw input cloud (camera frame) |
| `/sculpted_pointcloud` | `sensor_msgs/PointCloud2` | After NaN removal (camera frame) |
| `/downsampled_pointcloud` | `sensor_msgs/PointCloud2` | After voxel downsampling (camera frame) |
| `/plane_segmented_pointcloud` | `sensor_msgs/PointCloud2` | After plane removal (camera frame) |
| `/exclusive_pointcloud` | `sensor_msgs/PointCloud2` | After costmap occupancy filtering (map frame) |

**Processing pipeline**

```
Incoming PointCloud2 (queued, maxlen=1)
│
├── Wait until TF available (base_link ← camera frame, map ← base_link)
│
├── Read XYZ points, drop NaNs
├── Publish /obtained_pointcloud
│
├── Remove non-finite points
├── Publish /sculpted_pointcloud  (+ /downsampled_pointcloud)
│
├── Iterative RANSAC plane removal
│     ├── segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=2000)
│     ├── Remove inliers from working cloud
│     ├── Mask HQ cloud points within 0.008 m of the plane
│     └── Repeat until < 100 points remain or < 500 inliers found
│
├── Publish /plane_segmented_pointcloud (camera frame)
│
├── TF transform: camera frame → base_link → map
│
├── Reject points inside occupied costmap cells (threshold ≥ 5)
├── Publish /exclusive_pointcloud (map frame)
│
├── DBSCAN clustering (eps=0.03, min_samples=20)
│
└── For each cluster:
      ├── Compute oriented bounding box (Open3D OBB)
      ├── Reject if OBB center is in occupied costmap cell
      ├── Extract yaw from OBB rotation matrix
      ├── Append DetectedObject (map frame pose + OBB size)
      └── Append RViz CUBE marker
```

**DBSCAN parameters**

| Parameter | Value |
|---|---|
| `eps` | 0.03 m |
| `min_samples` | 20 |

**Plane segmentation parameters**

| Parameter | Value |
|---|---|
| `distance_threshold` | 0.003 m |
| `ransac_n` | 3 |
| `num_iterations` | 2000 |
| Inlier plane mask distance | 0.008 m |
| Minimum inliers to continue | 500 |
| Minimum cloud size to continue | 100 |

> **Note:** Object orientation is flattened to yaw-only before publishing — roll and pitch from the OBB rotation matrix are discarded.

---

### `classifier_2d`

Runs YOLO inference on the gripper camera feed on demand. Detection results are returned via a service call rather than continuously streamed, which suits the behaviour tree's `GetPlanarObjects` polling pattern.

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gripper camera feed |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/gripper_camera/image_annotated` | `sensor_msgs/Image` | YOLO-annotated frame (published on each service call) |

**Services**

| Service | Type | Description |
|---|---|---|
| `/perception/planar/get_detected_objects` | `mirte_lc_msgs/GetDetectedObjects` | Run inference on the latest frame and return detections |

**Model**

| Parameter | Value |
|---|---|
| Weights | `mirte_lc_vision/models/ColourdetectionYOLO26n.pt` |
| Default confidence | 0.3 |
| Input size | 640 px |
| Inference device | CUDA if available, otherwise CPU |

**Label mapping**

Detections are classified into two labels used by the behaviour tree's `PickObject` for sort routing:

| Model class | Published label |
|---|---|
| `green`, `red`, `purple` | `"target"` |
| any other class | `"trash"` |

**Detection coordinate system**

Pose and size fields in the returned `DetectedObject` messages use normalised image coordinates (`xywhn` from YOLO boxes). `z` is always `0.0`. These are **not** in the map frame — they are pixel-space fractions intended for use by the arm controller to centre the gripper over an object.

---

## Dependencies

- `rclpy`, `tf2_ros`, `cv_bridge`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`
- `mirte_lc_msgs`
- `nav2_simple_commander` (`PyCostmap2D`)
- `numpy`, `open3d`
- `scikit-learn` (`DBSCAN`)
- `scipy` (`Rotation`)
- `opencv-python` (`cv2`)
- `torch`, `ultralytics` (YOLO node only)
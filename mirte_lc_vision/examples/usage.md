# Examples for mirte_lc_vision

## object_locator

### Echo detected objects

```bash
ros2 topic echo /perception/depth/detected_objects
```

Each object in the array has a map-frame `pose` (with yaw-only orientation), an OBB `size`, a `confidence` score, and a `label`.

### View the point cloud pipeline in RViz

Add the following **PointCloud2** displays to step through the processing stages:

| Topic | Stage |
|---|---|
| `/obtained_pointcloud` | Raw input — confirms camera is publishing |
| `/sculpted_pointcloud` | After NaN removal |
| `/plane_segmented_pointcloud` | After iterative RANSAC plane removal |
| `/exclusive_pointcloud` | After costmap wall filtering (map frame) |

Add a **MarkerArray** display on `/object_bounding_boxes` to see the final green OBB markers.

### Check detection rate

```bash
ros2 topic hz /perception/depth/detected_objects
```

Expected: ~0.5 Hz (one processing cycle every 2 s).

### Check how many points survive plane segmentation

```bash
ros2 topic echo /plane_segmented_pointcloud --field width
```

`width` is the surviving point count. If consistently 0 the RANSAC thresholds may need tuning for your scene.

### Tune DBSCAN clustering

Edit `mirte_lc_vision/mirte_lc_vision/object_locator2.py`:

```python
clustering = DBSCAN(
    eps=0.03,        # increase to merge nearby clusters, decrease to split them
    min_samples=20   # increase to reject noise, decrease to detect small objects
).fit(points_map)
```

### Tune RANSAC plane removal

In the same file, adjust the iterative plane segmentation call:

```python
plane_model, inliers = pcd_LQ.segment_plane(
    distance_threshold=0.003,   # tighter = only removes flat surfaces
    ransac_n=3,
    num_iterations=2000
)
```

And the mask distance:

```python
plane_mask = dist < 0.008   # increase to remove more points near the plane
```

### Debug costmap occupancy filtering

Objects near walls may be incorrectly rejected if the costmap inflation radius is large. The rejection threshold is `5` (any cell with cost ≥ 5 discards the detection). To raise it, edit `object_locator2.py`:

```python
def is_occupied(self, x, y, threshold=5):   # increase threshold to be less aggressive
```

---

## yolo_detector

### Call the detection service manually

```bash
ros2 service call /perception/planar/get_detected_objects \
  mirte_lc_msgs/srv/GetDetectedObjects "{}"
```

The response is a `DetectedObjectArray`. Pose coordinates are normalised image fractions (`xywhn`) — not map-frame metres. `z` is always `0.0`.

### View the annotated camera stream

```bash
ros2 run rqt_image_view rqt_image_view /gripper_camera/image_annotated
```

The annotated frame is only published when the service is called. Trigger a call first if the topic appears empty.

### Check the raw camera feed

```bash
ros2 topic hz /camera/color/image_raw
```

If this shows 0 Hz the camera is not publishing and the detector will return an empty array.

### Change confidence threshold

Edit `mirte_lc_vision/mirte_lc_vision/classifier_2d.py` (or the YOLO node file):

```python
self.detector = Yolo26Cam(
    targets=["green", "red", "purple"],
    conf=0.3,   # lower = more detections; higher = fewer false positives
)
```

### Add or remove target classes

```python
self.detector = Yolo26Cam(
    targets=["green", "red", "purple", "blue"],   # "blue" now also labelled "target"
    conf=0.3,
)
```

### Use a different model

```python
self.detector = Yolo26Cam(
    targets=["green", "red"],
    model_path="/path/to/your_model.pt",
    conf=0.4,
    imgsz=320,
)
```

Rebuild after any source change:

```bash
colcon build --packages-select mirte_lc_vision --symlink-install
```
# ROS 2 Messages package

ROS 2 message, service, and action definitions for the LabClean application.

---

## Services

### `ServeCoverageStatus.srv`

Pause, resume, or stop the coverage navigator.

**Request**

| Field | Type | Constants | Description |
|---|---|---|---|
| `command` | `uint8` | `PAUSE=0` `RESUME=1` `STOP=2` | Command to send to the coverage navigator |

**Response**

| Field | Type | Description |
|---|---|---|
| `succeeded` | `bool` | Whether the command was applied successfully |
| `remaining_poses` | `int32` | Number of waypoints remaining in the coverage plan |

### `GetDetectedObjects.srv`

Call the 2D classification and detection service.

**Request**

| Field | Description |
|---|---|
| empty | - |

**Response**

| Field | Type | Description |
|---|---|---|
| `detected_objects` | `DetectedObjectArray.msg` | detected object array message. |

---

## Messages

### `DetectedObjectArray.msg`

| Field | Type | Description |
|---|---|---|
| `objects` | `DetectedObject[]` | Array of detected objects |
| `length` | `int32` | Number of objects in the array |

### `DetectedObject.msg`

| Field | Type | Description |
|---|---|---|
| `pose` | `geometry_msgs/Pose` | Object position and orientation in the map frame |
| `size` | `geometry_msgs/Vector3` | Bounding box dimensions (metres) |
| `confidence` | `float32` | Detection confidence score `[0.0, 1.0]` |
| `label` | `string` | Object class label (e.g. `"target"`) |

---

## Actions

### `NavigateCoverage.action`

Execute a full coverage navigation plan over the mapped area.

**Goal**

| Field | Type | Constants | Description |
|---|---|---|---|
| `planner_type` | `string` | `BOUSTROPHEDON="BousPlanner"` `SPANNINGTREE="SpanningTreePlanner"` `SKELETON="SkeletonPlanner"` | Coverage planner to use |
| `verbose` | `bool` | | Enable verbose logging in the coverage navigator |

**Result**

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | Whether coverage completed successfully |
| `message` | `string` | Human-readable status or error description |

**Feedback**

| Field | Type | Description |
|---|---|---|
| `completion_percentage` | `float32` | Percentage of coverage area visited `[0.0, 100.0]` |
| `distance_remaining` | `float32` | Estimated path distance remaining (metres) |
| `current_segment` | `int32` | Index of the segment currently being executed |
| `total_segments` | `int32` | Total number of segments in the coverage plan |

---

### `MoveToPosition.action`

Move the MIRTE arm or gripper to a target position. Goal fields are evaluated in priority order — set only one per goal (see [`mirte_lc_moveit_cpp`](../mirte_lc_moveit_cpp/README.md) for execution details).

**Goal**

| Field | Type | Description |
|---|---|---|
| `mirte_arm_target_pose` | `geometry_msgs/Pose` | Cartesian target pose for the arm (IK solved to `wrist` link). Active when `position.x != 0.0`. |
| `mirte_arm_named_target` | `string` | Named MoveIt target for the arm (e.g. `"vigilant"`, `"standby"`). Takes priority over pose. |
| `mirte_gripper_named_target` | `string` | Named MoveIt target for the gripper (e.g. `"open"`, `"close"`). Ignored if value is `"none"`. |
| `mirte_gripper_joint_target` | `float64` | Direct gripper joint position (rad / m). Active when `!= 0.0`. |
| `mirte_wrist_joint_target` | `float64` | Direct wrist joint position (rad). Active when `!= 0.0`. |

**Result**

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | Whether the motion completed successfully |

**Feedback**

| Field | Type | Description |
|---|---|---|
| `state` | `string` | Current execution state string from the action server |

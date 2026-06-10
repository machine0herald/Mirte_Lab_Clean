# Examples for mirte_lc_moveit_cpp

## Send a named arm target

```bash
ros2 action send_goal /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_arm_named_target: 'vigilant', mirte_gripper_named_target: 'none'}"
```

Common named targets for the arm:

| Name | Description |
|---|---|
| `vigilant` | Upright standby pose (startup default) |
| `standby` | Deployed, ready for approach |
| `place_left` | Place position for target objects |
| `place_right` | Place position for trash objects |

## Send a named gripper target

```bash
ros2 action send_goal /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_arm_named_target: '', mirte_gripper_named_target: 'open'}"

ros2 action send_goal /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_arm_named_target: '', mirte_gripper_named_target: 'close'}"
```

## Send a Cartesian arm pose

```bash
ros2 action send_goal /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_arm_target_pose: {position: {x: 0.085, y: 0.0, z: 0.47}, orientation: {x: 0.7, y: 0.0, z: 0.7, w: 0.0}}, mirte_gripper_named_target: 'none'}"
```

The IK solver targets the `wrist` link. If exact IK fails the server retries with approximate IK automatically.

## Set wrist joint directly

```bash
ros2 action send_goal /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_wrist_joint_target: 1.57, mirte_gripper_named_target: 'none'}"
```

## Set gripper joint directly

```bash
ros2 action send_goal /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_arm_named_target: '', mirte_gripper_joint_target: 0.4, mirte_gripper_named_target: 'none'}"
```

## Run the single-pose example executable

```bash
ros2 run mirte_lc_moveit_cpp moveit_example
```

Moves the arm to `(0.085, 0.0, 0.47)` using `setApproximateJointValueTarget`, then logs the resulting wrist pose. No action server required — good for a quick MoveIt smoke test.

## Watch action server feedback

```bash
ros2 action send_goal --feedback /move_to_position \
  mirte_lc_msgs/action/MoveToPosition \
  "{mirte_arm_named_target: 'standby', mirte_gripper_named_target: 'none'}"
```

Feedback field: `state` (string) — current execution state reported by the server.
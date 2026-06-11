# Quickstart for mirte_lc_moveit_cpp

This quickstart explains common usage for the [`mirte_lc_moveit_cpp`](https://github.com/matt-rbt/Mirte_Lab_Clean/tree/main/mirte_lc_moveit_cpp) package.

## Run

### Action server

```bash
ros2 run mirte_lc_moveit_cpp moveit_action_server
```

On startup the node moves the arm to the `vigilant` named pose before accepting goals. MoveIt and `ros2_control` must already be running.

### As a composable node

```bash
ros2 component load /ComponentManager \
  mirte_lc_moveit_cpp \
  mirte_lc_moveit_cpp::MirteLCMoveItActionServer
```

### MoveIt example executable

```bash
ros2 run mirte_lc_moveit_cpp moveit_example
```

Plans and executes a single hard-coded pose `(0.085, 0.0, 0.47)` for the `mirte_arm` group. Useful for verifying the MoveIt stack is functional.

## Node information

### mirte_lc_moveit_action_server

**Action servers**

| Action | Type | Description |
|---|---|---|
| `/move_to_position` | `mirte_lc_msgs/MoveToPosition` | Execute arm and gripper motion goals |

**MoveIt planning groups**

| Group | Used for |
|---|---|
| `mirte_arm` | Named targets, Cartesian pose goals, wrist joint control |
| `mirte_gripper` | Named targets, gripper joint control |

## Goal field priority

Fields are evaluated in this order; the first non-default field wins:

| Priority | Field | Active when |
|---|---|---|
| 1 | `mirte_arm_named_target` | non-empty string |
| 2 | `mirte_arm_target_pose` | `position.x != 0.0` |
| 3 | `mirte_wrist_joint_target` | `!= 0.0` |
| 4 | `mirte_gripper_named_target` | `!= "none"` |
| 5 | `mirte_gripper_joint_target` | `!= 0.0` |

Set only one field per goal. See the [README](README.md) for the full Cartesian execution pipeline.
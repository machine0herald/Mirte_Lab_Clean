# MoveIt Package (Python)

Interactive command-line client for the `MoveToPosition` action server. Lets you drive the Mirte arm, gripper, and wrist directly from a terminal without writing any code — useful for tuning named poses, verifying hardware, and debugging the manipulation pipeline.

---

## Run

```bash
ros2 run mirte_lc_labclean test_node
```

The node connects to the `move_to_position` action server and drops into an interactive prompt.

---

## Node information

### `move_to_position_action_client`

**Action clients**

| Action server | Type | Description |
|---|---|---|
| `/move_to_position` | `mirte_lc_msgs/MoveToPosition` | Target motion commands for arm, gripper, and wrist |

**Feedback / result logging**

| Event | Log message |
|---|---|
| Goal accepted | `Goal accepted` |
| Goal rejected | `Goal rejected` |
| Result received | `Success: <bool>` |
| Feedback tick | `Feedback: <state string>` |

---

## Interactive commands

Type commands at the `>` prompt. Each command blocks until the action server returns a result before accepting the next input.

### Arm

| Command | Arguments | Example | Description |
|---|---|---|---|
| `arm_pose` | `x y z` | `arm_pose 0.2 0.1 0.3` | Move the arm to a Cartesian position. Orientation is fixed at quaternion `(0.7, 0.0, 0.7, 0.0)`. |
| `arm_name` | `TARGET` | `arm_name home` | Move the arm to a named MoveIt target. |

### Gripper

| Command | Arguments | Example | Description |
|---|---|---|---|
| `gripper_name` | `TARGET` | `gripper_name open` | Set the gripper to a named state. |
| `gripper_joint` | `VALUE` | `gripper_joint 0.4` | Set the gripper joint to a specific value (radians / metres depending on joint type). |

### Wrist

| Command | Arguments | Example | Description |
|---|---|---|---|
| `wrist_joint` | `VALUE` | `wrist_joint 1.57` | Set the wrist joint to a specific value (radians). |

### General

| Command | Description |
|---|---|
| `help` | Print the command reference. |
| `q` | Quit and shut down the node. |

---

## Goal field mapping

Each command maps to one field of `MoveToPosition.Goal`. Fields not set by a command are left at their default (empty string / zero).

| Command | Goal field set |
|---|---|
| `arm_pose` | `mirte_arm_target_pose` |
| `arm_name` | `mirte_arm_named_target` |
| `gripper_name` | `mirte_gripper_named_target` |
| `gripper_joint` | `mirte_gripper_joint_target` |
| `wrist_joint` | `mirte_wrist_joint_target` |

---

## Dependencies

- `rclpy`
- `mirte_lc_msgs`
- `geometry_msgs`
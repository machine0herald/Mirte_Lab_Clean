# mirte_lc_moveit_cpp

MoveIt action server for the MIRTE arm and gripper. Exposes the `/move_to_position` action and handles named targets, Cartesian pose goals, and gripper commands via `MoveGroupInterface`. Also includes a standalone example executable for quick motion testing.

---

## Run

### Action server (main executable)

```bash
ros2 run mirte_lc_moveit_cpp moveit_action_server
```

On startup the node moves the arm to the `vigilant` named pose before accepting goals.

### As a composable node

The server is registered as a ROS 2 component and can be loaded into a component container:

```bash
ros2 component load /ComponentManager mirte_lc_moveit_cpp mirte_lc_moveit_cpp::MirteLCMoveItActionServer
```

### MoveIt example

```bash
ros2 run mirte_lc_moveit_cpp moveit_example
```

Plans and executes a single hard-coded pose `(0.085, 0.0, 0.47)` for the `mirte_arm` group, then logs the resulting wrist pose. Useful for verifying the MoveIt stack is functional.

---

## Node information

### `mirte_lc_moveit_action_server`

**Action servers**

| Action | Type | Description |
|---|---|---|
| `/move_to_position` | `mirte_lc_msgs/MoveToPosition` | Execute arm and gripper motion goals |

**MoveIt planning groups**

| Group | Used for |
|---|---|
| `mirte_arm` | Arm named targets, Cartesian pose goals, wrist joint control |
| `mirte_gripper` | Gripper named targets, gripper joint control |

---

## Goal field execution logic

Each field of `MoveToPosition.Goal` is evaluated in priority order. The first non-default field wins; subsequent fields are ignored for that goal.

| Priority | Field | Condition | Behaviour |
|---|---|---|---|
| 1 | `mirte_arm_named_target` | non-empty string | Sets named target on `mirte_arm` and calls `move()` |
| 2 | `mirte_arm_target_pose` | `position.x != 0.0` | Runs full IK → planning → TOTG → execution pipeline (see below) |
| 3 | `mirte_wrist_joint_target` | `!= 0.0` | Copies current joint values, overrides joint index 3, calls `move()` |
| 4 | `mirte_gripper_named_target` | `!= "none"` | Sets named target on `mirte_gripper` and calls `move()` |
| 5 | `mirte_gripper_joint_target` | `!= 0.0` | Copies current gripper joint values, overrides index 0, calls `move()` |

> **Note:** `mirte_wrist_joint_target` and `mirte_gripper_named_target` / `mirte_gripper_joint_target` are not mutually exclusive — if priorities 3 and 4/5 are both set, the wrist move executes first but the gripper result will not be returned (the goal has already been succeeded by the gripper branch). Set only one field per goal to avoid this.

---

## Cartesian pose execution pipeline

When `mirte_arm_target_pose` is used the server runs the following steps. The goal is aborted if any step fails.

1. **Fetch current state** — `getCurrentState()`, enforce bounds, set as start state.
2. **IK — exact** — `setFromIK` with `return_approximate_solution = false`, tip link `wrist`.
3. **IK — approximate fallback** — retried with `return_approximate_solution = true` if exact IK fails.
4. **Bounds check** — `satisfiesBounds()` logged (does not abort on violation).
5. **Planning** — `setPositionTarget` on `wrist`, then `plan()`.
6. **Tolerance relaxation fallback** — if planning fails, position tolerance is widened to `0.001` m and joint tolerance to `0.01` rad, then replanned.
7. **Time parameterisation** — `TimeOptimalTrajectoryGeneration` applied to the plan.
8. **Execution** — `execute(plan)`.
9. **Wrist-level compensation** — after execution, joint 3 is set to `-π − joint[1] − joint[2]` to keep the wrist level.

---

## Tolerances

| Tolerance | Initial value | Fallback value |
|---|---|---|
| Goal position | 0.001 m | 0.001 m (unchanged) |
| Goal joint | 0.001 rad | 0.01 rad |

---

## Logging

The node logs the following at `INFO` level for every goal:

- All four goal fields on receipt (`handle_goal`)
- Pose reference frame and end-effector link
- Planning frame and all three tolerances
- Current and target joint values (labelled `[CURRENT]` / `[TARGET]`)
- IK outcome (exact and approximate)
- Bounds validity and collision state
- Trajectory point count and per-point timestamps
- Time parameterisation result
- Final wrist pose after execution

Cancel requests are logged at `WARN`.

---

## Dependencies

- `rclcpp`, `rclcpp_action`, `rclcpp_components`
- `mirte_lc_msgs`
- `moveit_ros_planning_interface` (`MoveGroupInterface`, `RobotModelLoader`, `RobotState`)
- `moveit_ros_planning` (`PlanningScene`)
- `moveit_core` (`TimeOptimalTrajectoryGeneration`, `RobotTrajectory`)
- `geometry_msgs`
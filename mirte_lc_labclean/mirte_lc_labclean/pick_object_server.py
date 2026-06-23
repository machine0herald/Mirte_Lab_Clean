"""Action server that executes the multi-step pick sequence for a single DetectedObject."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

from mirte_lc_msgs.action import MoveToPosition, PickObject
from mirte_lc_msgs.msg import DetectedObject
from rclpy.action import ActionClient


class PickObjectServer(Node):

    def __init__(self):
        super().__init__("pick_object_server")
        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arm_client = ActionClient(
            self, MoveToPosition, "/move_to_position",
            callback_group=self.cb_group,
        )

        self._action_server = ActionServer(
            self,
            PickObject,
            "/labclean/pick_object",
            execute_callback=self.execute_cb,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )
        self.get_logger().info("PickObjectServer ready")

    # ------------------------------------------------------------------
    # Arm helpers
    # ------------------------------------------------------------------

    async def _send_arm_goal(self, goal_msg: MoveToPosition.Goal, step: str) -> bool:
        """Send a MoveToPosition goal and await the result. Returns True on success."""
        send_future = self.arm_client.send_goal_async(goal_msg)
        await send_future
        gh = send_future.result()

        if not gh.accepted:
            self.get_logger().warn(f"[PickObjectServer] step '{step}' rejected by arm server")
            return False

        result_future = gh.get_result_async()
        await result_future
        result = result_future.result()

        if not result.result.success:
            self.get_logger().warn(f"[PickObjectServer] step '{step}' failed")
            return False

        return True

    def _lookup_wrist(self) -> tuple[float, float, float] | None:
        """Return wrist position in base_link, or None on TF failure."""
        try:
            t = self.tf_buffer.lookup_transform("base_link", "wrist", rclpy.time.Time())
            self.get_logger().info(f"\033[36m found transform (x = {t.transform.translation.x}, \
                                                            y = {t.transform.translation.y}, \
                                                            z = {t.transform.translation.z}) \033[0m"
                                                            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
        except Exception as e:
            self.get_logger().warn(f"[PickObjectServer] TF wrist lookup failed: {e}")
            return None

    # ------------------------------------------------------------------
    # Steps
    # ------------------------------------------------------------------

    async def _step_open(self, obj: DetectedObject) -> bool:
        goal = MoveToPosition.Goal()
        goal.mirte_gripper_named_target = "open"
        return await self._send_arm_goal(goal, "open")

    async def _step_slight(self, obj: DetectedObject) -> bool:
        wrist = self._lookup_wrist()
        if wrist is None:
            return False
        wx, wy, wz = wrist
        pose = Pose()
        pose.position.x = wx + obj.pose.position.y* 0.05
        pose.position.y = wy + obj.pose.position.x * 0.1
        pose.position.z = wz

        pose.orientation.x = 0.7
        pose.orientation.y = 0.0
        pose.orientation.z = 0.7
        pose.orientation.w = 0.0
        self.get_logger().info(
            f"[slight] target: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}"
        )
        goal = MoveToPosition.Goal()
        goal.mirte_arm_target_pose = pose
        return await self._send_arm_goal(goal, "slight")

    async def _step_dive(self, obj: DetectedObject) -> bool:
        wrist = self._lookup_wrist()
        if wrist is None:
            return False
        wx, wy, _ = wrist
        pose = Pose()
        pose.position.x = wx
        pose.position.y = wy
        pose.position.z = 0.3
        pose.orientation.x = 0.7
        pose.orientation.y = 0.0
        pose.orientation.z = 0.7
        pose.orientation.w = 0.0
        self.get_logger().info(
            f"[dive] target: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}"
        )
        goal = MoveToPosition.Goal()
        goal.mirte_arm_target_pose = pose
        return await self._send_arm_goal(goal, "dive")

    async def _step_grip(self, obj: DetectedObject) -> bool:
        goal = MoveToPosition.Goal()
        goal.mirte_gripper_named_target = "close"
        return await self._send_arm_goal(goal, "grip")

    async def _step_place(self, obj: DetectedObject) -> bool:
        goal = MoveToPosition.Goal()
        goal.mirte_arm_named_target = (
            "place_left" if obj.label == "target" else "place_right"
        )
        return await self._send_arm_goal(goal, "place")

    async def _step_let_go(self, obj: DetectedObject) -> bool:
        goal = MoveToPosition.Goal()
        goal.mirte_gripper_named_target = "open"
        return await self._send_arm_goal(goal, "let_go")

    async def _step_standby(self, obj: DetectedObject) -> bool:
        goal = MoveToPosition.Goal()
        goal.mirte_arm_named_target = "vigilant"
        return await self._send_arm_goal(goal, "standby")

    # ------------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------------

    async def execute_cb(self, goal_handle):
        obj: DetectedObject = goal_handle.request.object

        self.get_logger().info(
            f"[PickObjectServer] received goal: label='{obj.label}' "
            f"pose=({obj.pose.position.x:.3f}, {obj.pose.position.y:.3f}, {obj.pose.position.z:.3f})"
        )

        steps = [
            ("open",    self._step_open),
            ("slight",  self._step_slight),
            ("dive",    self._step_dive),
            ("grip",    self._step_grip),
            ("place",   self._step_place),
            ("let_go",  self._step_let_go),
            ("standby", self._step_standby),
        ]

        feedback_msg = PickObject.Feedback()

        for i, (name, fn) in enumerate(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return PickObject.Result(success=False, message="Cancelled")

            feedback_msg.current_step = name
            feedback_msg.step_index = i
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"\033[36m [PickObjectServer] step {i}/{len(steps)-1}: {name} \033[0m")

            if not await fn(obj):
                goal_handle.abort()
                return PickObject.Result(success=False, message=f"Step '{name}' failed")

        goal_handle.succeed()
        return PickObject.Result(success=True, message="Pick complete")


def main():
    rclpy.init()
    node = PickObjectServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
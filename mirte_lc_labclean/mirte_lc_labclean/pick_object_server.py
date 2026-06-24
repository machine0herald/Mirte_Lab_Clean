"""Action server that executes the multi-step pick sequence for a single DetectedObject."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

from mirte_lc_msgs.action import MoveToPosition, PickObject
from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
from threading import Lock


class PickObjectServer(Node):

    def __init__(self):
        super().__init__("pick_object_server")
        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.jtc_publisher = self.create_publisher(
            JointTrajectory,
            'mirte_master_arm_controller/joint_trajectory',
            10
        )

        self.arm_client = ActionClient(
            self, MoveToPosition, "/move_to_position",
            callback_group=self.cb_group,
        )

        self.detected_objects = []
        self.detection_lock = Lock()

        self.detection_subscriber = self.create_subscription(
            DetectedObjectArray,
            "/perception/planar/detected_objects",
            self.detection_callback,
            10,
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

    def get_object_by_label(self, label):
        with self.detection_lock:
            for obj in self.detected_objects:
                if obj.label == label:
                    return obj

        return None

    def detection_callback(self, msg):
        self.detected_objects = msg.objects

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
        x_img = obj.pose.position.x - 0.5
        y_img = -(obj.pose.position.y - 0.5)

        pose.position.x = wx + y_img * 0.1
        pose.position.y = wy + x_img * 0.2
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
        ############################################################
        # margin = 0.1
        # gain = 2.0
        # max_iters = 1000
        # target = [-0.3, 0.0]

        # for _ in range(max_iters):
        #     current_obj = self.get_object_by_label(obj.label)

        #     if current_obj is None:
        #         continue

        #     x_img = (current_obj.pose.position.x - 0.5)
        #     y_img = -(current_obj.pose.position.y - 0.5)

        #     if abs(target[1] - x_img) < margin and abs(target[0] - y_img) < margin:
        #         self.get_logger().info("Object centered")
        #         return True

        #     msg = JointTrajectory()
        #     msg.joint_names = [
        #         "elbow_joint",
        #         "shoulder_lift_joint",
        #         "shoulder_pan_joint",
        #         "wrist_joint",
        #     ]

        #     point = JointTrajectoryPoint()

        #     point.velocities = [
        #         -gain * (target[0] - y_img),
        #         gain *(target[0] - y_img),
        #         gain * (target[1] - x_img),
        #         gain *(target[0] - y_img),
        #     ]

        #     point.time_from_start.sec = 1

        #     msg.points.append(point)

        #     self.jtc_publisher.publish(msg)
        #     self.get_logger().info(
        #         f"Publishishing joint speeds: {point.velocities}"
        #     )
        #     self.get_logger().info(
        #         f"Centering: x={x_img:.3f} y={y_img:.3f}"
        #     )

        #     time.sleep(0.1)

        # self.get_logger().warn("Failed to center object")
        # return False

    async def _step_dive(self, obj: DetectedObject) -> bool:
        wrist = self._lookup_wrist()
        if wrist is None:
            return False
        wx, wy, _ = wrist
        pose = Pose()
        pose.position.x = wx
        pose.position.y = wy
        pose.position.z = 0.15
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
        requested_label = goal_handle.request.object.label

        self.get_logger().info(
            f"Looking for object label '{requested_label}'"
        )

        obj = self.get_object_by_label(requested_label)

        if obj is None:
            goal_handle.abort()
            return PickObject.Result(
                success=False,
                message=f"No object found with label {requested_label}"
            )

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
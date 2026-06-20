"""ROS2 behaviours for the Mirte LabClean application.

This module defines a set of py_trees behaviours used in the LabClean
behaviour tree. Behaviours include LED control, coverage navigation, object
retrieval, and plan execution using ROS2 action servers and services.
"""

import math
import time

from mirte_msgs.msg import NeopixelColor
from mirte_msgs.srv import SetNeopixel

from mirte_lc_msgs.action import MoveToPosition, NavigateCoverage
from mirte_lc_msgs.srv import ServeCoverageStatus
from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
import py_trees
import py_trees_ros
import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus

from mirte_lc_msgs.srv import GetDetectedObjects
from tf2_ros import Buffer, TransformListener
import numpy as np


# ===========================================================================
# FlashLedStrip
# ===========================================================================

class FlashLedStrip(py_trees.behaviour.Behaviour):
    """Flash the LED strip with a colour command.

    Args:
        name (str): Name of the behaviour.
        colour (list[float]): RGB colour values in range [0.0, 1.0].
    """

    def __init__(self, name: str, colour: list = [0.0, 0.0, 1.0]):
        super(FlashLedStrip, self).__init__(name=name)
        self.name = name
        self.colour = colour

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e

        self.node.color = "FlashBlue"
        self.neopixel_client = self.node.create_client(
            SetNeopixel, "/io/leds/leds/set_color"
        )
        self.led_marker_publisher = self.node.create_publisher(
            Marker, '/labclean_led_markers', 10
        )
        self.feedback_message = "Neopixel service client created"

    def initialise(self):
        if self.node.color == self.name:
            return

        else:
            self.node.color = self.name
            self.logger.info(
                "%s.initialise(), sending led request" % self.__class__.__name__
            )
            request = SetNeopixel.Request()
            request.color = NeopixelColor()
            # Robot LED strip has swapped channels: msg.g=red, msg.b=green, msg.r=blue
            request.color.g = int(self.colour[0] * 255)
            request.color.b = int(self.colour[1] * 255)
            request.color.r = int(self.colour[2] * 255)
            self.future = self.neopixel_client.call_async(request)
            self._publish_led_marker(self.colour)
            self.feedback_message = "Sent LED request"

    def _publish_led_marker(self, colour):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.9
        marker.scale.y = 1.9
        marker.scale.z = 0.03
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.4
        marker.color.r = colour[0]
        marker.color.g = colour[1]
        marker.color.b = colour[2]
        marker.color.a = 1.0
        self.led_marker_publisher.publish(marker)

    def update(self):
        # self.feedback_message = "LED updated"
        return py_trees.common.Status.SUCCESS


# ===========================================================================
# SetCoverageStatus
# ===========================================================================

class SetCoverageStatus(py_trees.behaviour.Behaviour):
    """Send a pause/resume/stop command to the coverage navigation server.

    Args:
        name (str): Name of the behaviour.
        requested_status (str): One of ``pause``, ``resume``, or ``stop``.
    """

    status_commands = {
        "pause":  ServeCoverageStatus.Request.PAUSE,
        "resume": ServeCoverageStatus.Request.RESUME,
        "stop":   ServeCoverageStatus.Request.STOP,
    }

    def __init__(self, name: str, requested_status: str):
        super(SetCoverageStatus, self).__init__(name=name)
        self.requested_status = requested_status

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e
        self.client = self.node.create_client(
            ServeCoverageStatus, "/labclean_navigator/set_state"
        )
        self.future = None
        self.feedback_message = "Coverage status client created"

    def initialise(self):
        request = ServeCoverageStatus.Request()
        request.command = self.status_commands[self.requested_status]
        self.future = self.client.call_async(request)
        self.feedback_message = f"Sent {self.requested_status} request"

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE
        if not self.future.done():
            return py_trees.common.Status.RUNNING
        try:
            response = self.future.result()
            self.feedback_message = (
                f"Coverage {self.requested_status} ok, "
                f"remaining={response.remaining_poses}"
            )
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Service call failed: {e}"
            return py_trees.common.Status.FAILURE


# ===========================================================================
# NavigateToPosition
# ===========================================================================

class NavigateToPosition(py_trees.behaviour.Behaviour):
    """Navigate the robot to a target position.

    Sends a Nav2 goal and monitors progress. Handles TF not ready, Nav2 not
    ready, feedback dropout, and genuine stuck situations gracefully.

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str, optional): Blackboard key containing a list of
            DetectedObject messages. The closest object is used as the target.
        target_position (list[float] | tuple[float, float], optional): Fixed
            XY position in the map frame. Used when blackboard_key is None.
        goal_tolerance (float): Distance in metres at which to consider the
            goal reached and cancel early. Default 0.5m.
        stuck_timeout (float): Seconds without progress before cancelling.
            Default 20.0s.
        nav2_timeout (float): Seconds to wait for Nav2 to become active.
            Default 5.0s.
        standoff (float): Metres to stop short of a detected object so the
            robot doesn't drive into it. Default 0.4m.
    """

    def __init__(
        self,
        name: str,
        blackboard_key: str = None,
        target_position=None,
        goal_tolerance: float = 0.003,
        stuck_timeout: float = 20.0,
        nav2_timeout: float = 5.0,
        standoff: float = 0.4,
    ):
        super(NavigateToPosition, self).__init__(name=name)
        self.blackboard_key = blackboard_key
        self.target_position = target_position
        self.goal_tolerance = goal_tolerance
        self.stuck_timeout = stuck_timeout
        self.nav2_timeout = nav2_timeout
        self.standoff = standoff

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e

        self.blackboard = self.attach_blackboard_client(name=self.name)
        if self.blackboard_key is not None:
            self.blackboard.register_key(
                key=self.blackboard_key,
                access=py_trees.common.Access.READ,
            )

        # Single Navigator instance — created once, reused across all ticks
        self.Navigator = BasicNavigator()

        self._distance_remaining = float('inf')
        self._last_feedback_time = 0.0

        self.node.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._feedback_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def _feedback_callback(self, msg):
        self._distance_remaining = msg.feedback.distance_remaining
        self._last_feedback_time = time.monotonic()

    def initialise(self):
        self._best_distance = math.inf
        self._last_progress_time = time.monotonic()
        self._target_set = False
        self._distance_remaining = float('inf')
        self._last_feedback_time = time.monotonic()

        # Cancel any lingering goal
        if not self.Navigator.isTaskComplete():
            self.node.get_logger().warn(
                f"[{self.name}] Previous nav goal still active, cancelling"
            )
            self.Navigator.cancelTask()

        # Wait briefly for Nav2 to be idle
        deadline = time.monotonic() + self.nav2_timeout
        while time.monotonic() < deadline:
            if self.Navigator.isTaskComplete():
                break
            time.sleep(0.1)
        else:
            self.node.get_logger().warn(
                f"[{self.name}] Nav2 not ready after {self.nav2_timeout}s, skipping"
            )
            return

        # TF lookup
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"[{self.name}] TF not ready, will retry: {e}"
            )
            return

        rx = transform.transform.translation.x
        ry = transform.transform.translation.y

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"

        if self.blackboard_key is not None:
            objects = self.blackboard.get(self.blackboard_key)
            if not objects:
                self.feedback_message = "No objects on blackboard"
                return

            closest = min(
                objects,
                key=lambda obj: (obj.pose.position.x - rx) ** 2
                              + (obj.pose.position.y - ry) ** 2,
            )
            obj_x = closest.pose.position.x
            obj_y = closest.pose.position.y

            dist_to_obj = math.hypot(obj_x - rx, obj_y - ry)
            if dist_to_obj > self.standoff:
                scale = (dist_to_obj - self.standoff) / dist_to_obj
                pose_msg.pose.position.x = rx + (obj_x - rx) * scale
                pose_msg.pose.position.y = ry + (obj_y - ry) * scale
            else:
                self.feedback_message = "Already within standoff distance"
                self._close_enough = True
                self._target_set = True
                return

        elif self.target_position is not None:
            pose_msg.pose.position.x = float(self.target_position[0])
            pose_msg.pose.position.y = float(self.target_position[1])

        else:
            self.feedback_message = "No target specified"
            return

        # Set orientation toward goal
        dx = pose_msg.pose.position.x - rx
        dy = pose_msg.pose.position.y - ry
        yaw = math.atan2(dy, dx)
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.Navigator.goToPose(pose_msg)
        self._target_set = True
        self.feedback_message = (
            f"Navigating to ({pose_msg.pose.position.x:.2f}, "
            f"{pose_msg.pose.position.y:.2f})"
        )
        self.node.get_logger().info(f"[{self.name}] {self.feedback_message}")

    def update(self):
        if not self._target_set:
            return py_trees.common.Status.FAILURE

        current_distance = self._distance_remaining
        self.node.get_logger().info(f"distance remaining: {current_distance}")

        # Close enough
        if 0.0 <= current_distance < self.goal_tolerance:
            self.Navigator.cancelTask()
            self.feedback_message = f"Reached goal ({current_distance:.2f}m)"
            self.node.get_logger().info(f"[{self.name}] {self.feedback_message}")
            return py_trees.common.Status.SUCCESS

        # Progress tracking
        if current_distance < self._best_distance - 0.05:
            self._best_distance = current_distance
            self._last_progress_time = time.monotonic()

        elapsed = time.monotonic() - self._last_progress_time
        if elapsed > self.stuck_timeout:
            self.node.get_logger().warn(
                f"[{self.name}] Stuck for {elapsed:.1f}s, cancelling"
            )
            self.Navigator.cancelTask()
            return py_trees.common.Status.FAILURE

        # # Nav2 completed on its own
        # if self.Navigator.isTaskComplete():
        #     result = self.Navigator.getResult()
        #     if result == TaskResult.SUCCEEDED:
        #         self.feedback_message = "Navigation succeeded"
        #         return py_trees.common.Status.SUCCESS
        #     else:
        #         self.feedback_message = f"Navigation failed: {result}"
        #         self.node.get_logger().warn(f"[{self.name}] {self.feedback_message}")
        #         return py_trees.common.Status.FAILURE

        self.feedback_message = (
            f"distance={current_distance:.2f}m, "
            f"no progress for {elapsed:.1f}s"
        )
        return py_trees.common.Status.RUNNING


# ===========================================================================
# MoveArm
# ===========================================================================

class MoveArm(py_trees.behaviour.Behaviour):
    """Move the robot arm to a named or explicit target pose.

    Returns SUCCESS only when the action server confirms the goal succeeded.

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str, optional): Blackboard key holding a target pose.
        target_position (tuple[float, float, float], optional): Explicit XYZ.
        predefined_pose (str, optional): Name of a predefined arm pose.
    """

    def __init__(
        self,
        name: str,
        blackboard_key: str = None,
        target_position=None,
        predefined_pose: str = None,
    ):
        super(MoveArm, self).__init__(name=name)
        self.blackboard_key = blackboard_key
        self.target_position = target_position
        self.predefined_pose = predefined_pose

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e

        self.blackboard = self.attach_blackboard_client(name=self.name)
        if self.blackboard_key is not None:
            self.blackboard.register_key(
                key=self.blackboard_key,
                access=py_trees.common.Access.READ,
            )

        self.arm_action_client = ActionClient(
            self.node, MoveToPosition, "/move_to_position"
        )
        self.arm_future = None
        self.goal_handle = None
        self.result_future = None

    def initialise(self):
        self.arm_future = None
        self.goal_handle = None
        self.result_future = None

        goal_msg = MoveToPosition.Goal()

        if self.predefined_pose is not None:
            goal_msg.mirte_arm_named_target = self.predefined_pose

        elif self.blackboard_key is not None:
            goal_msg.mirte_arm_target_pose = self.blackboard.get(self.blackboard_key)

        elif self.target_position is not None:
            pose_msg = Pose()
            pose_msg.position.x = self.target_position[0]
            pose_msg.position.y = self.target_position[1]
            pose_msg.position.z = self.target_position[2]
            goal_msg.mirte_arm_target_pose = pose_msg

        else:
            self.feedback_message = "No target specified"
            return

        self.arm_future = self.arm_action_client.send_goal_async(goal_msg)
        self.feedback_message = "Goal sent, waiting for acceptance"

    def update(self):
        if self.arm_future is None:
            self.feedback_message = "No goal was sent"
            return py_trees.common.Status.FAILURE

        # Stage 1: goal acceptance
        if not self.arm_future.done():
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            self.goal_handle = self.arm_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = "Goal rejected by action server"
                self.node.get_logger().warn(f"[{self.name}] Arm goal rejected")
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            self.feedback_message = "Goal accepted, arm moving"

        # Stage 2: wait for result
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result()
        if result.result.success:
            self.feedback_message = "Arm move succeeded"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Arm move failed (success=False)"
            self.node.get_logger().warn(f"[{self.name}] {self.feedback_message}")
            return py_trees.common.Status.FAILURE


# ===========================================================================
# PickObject
# ===========================================================================

class PickObject(py_trees.behaviour.Behaviour):
    """Pick the closest detected object using a multi-step arm sequence.

    Steps: approach → open → dive → grip → place → let_go → standby

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str): Blackboard key holding a list of DetectedObject.
    """

    STEPS = [
        # "look", 
        # "approach", 
        "open", 
        "dive", 
        "grip", 
        "place", 
        "let_go", 
        "standby"
        ]

    def __init__(self, name: str, blackboard_key: str = "planar_objects_detected_array"):
        super(PickObject, self).__init__(name=name)
        self.blackboard_key = blackboard_key

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=self.blackboard_key,
            access=py_trees.common.Access.READ,
        )

        self.pick_action_client = ActionClient(
            self.node, MoveToPosition, "/move_to_position"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Non-blocking server wait
        if not self.pick_action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().warn(
                f"[{self.name}] /move_to_position server not available at setup"
            )

    def initialise(self):
        self.step = 0
        self.current_future = None
        self.goal_handle = None
        self.result_future = None
        self.object = None

        objects = self.blackboard.get(self.blackboard_key)
        if not objects:
            self.feedback_message = "No objects on blackboard"
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            self.object = min(
                objects,
                key=lambda o: (o.pose.position.x - rx) ** 2
                            + (o.pose.position.y - ry) ** 2,
            )
        except Exception as e:
            self.node.get_logger().warn(f"[{self.name}] TF lookup failed: {e}")
            self.object = objects[0]

        self._send_step()

    def _build_goal(self):
        goal_msg = MoveToPosition.Goal()
        obj = self.object
        step = self.STEPS[self.step]
        if step == "look":
            goal_msg.mirte_arm_named_target = "standby"

        if step == "approach":
            try:
                t = self.tf_buffer.lookup_transform(
                    "base_link", "wrist", rclpy.time.Time()
                )
                rx = t.transform.translation.x
                ry = t.transform.translation.y
                rz = t.transform.translation.z
            except Exception:
                rx, ry, rz = 0.0, 0.0, 0.3

            k_p = 1.0 / 10000.0
            error = 200.0 - obj.pose.position.y
            pose = Pose()
            pose.position.x = rx + obj.pose.position.x * k_p
            pose.position.y = ry + error * k_p        # fixed k_pf typo
            pose.position.z = rz
            goal_msg.mirte_arm_target_pose = pose

        elif step == "open":
            goal_msg.mirte_gripper_named_target = "open"

        elif step == "dive":
            try:
                t = self.tf_buffer.lookup_transform(
                    "wrist", "base_link", rclpy.time.Time()
                )
                rx = 0.085
                ry = 0.0
                rz = 0.47
                self.node.get_logger().info(f"[{self.name}]: transforms found")
            except Exception:
                rx, ry, rz = 0.0, 0.0, 0.03

            pose = Pose()
            pose.position.x = rx
            pose.position.y = ry
            pose.position.z = rz - 0.15
            goal_msg.mirte_arm_target_pose = pose

        elif step == "grip":
            goal_msg.mirte_gripper_named_target = "close"

        elif step == "place":
            # Use obj.label if available, otherwise default to place_right
            label = getattr(obj, 'label', None)
            goal_msg.mirte_arm_named_target = (
                "place_left" if label == "target" else "place_right"
            )

        elif step == "let_go":
            goal_msg.mirte_gripper_named_target = "open"

        elif step == "standby":
            goal_msg.mirte_arm_named_target = "vigilant"

        return goal_msg

    def _send_step(self):
        goal_msg = self._build_goal()
        self.current_future = self.pick_action_client.send_goal_async(goal_msg)
        self.goal_handle = None
        self.result_future = None
        self.feedback_message = f"Sending step: {self.STEPS[self.step]}"
        self.node.get_logger().info(
            f"[{self.name}] {self.feedback_message}"
        )

    def update(self):
        if self.object is None:
            return py_trees.common.Status.FAILURE

        # Stage 1: goal acceptance
        if not self.current_future.done():
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            self.goal_handle = self.current_future.result()
            if not self.goal_handle.accepted:
                self.node.get_logger().warn(
                    f"[{self.name}] Step {self.STEPS[self.step]} rejected"
                )
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()

        # Stage 2: wait for result
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result()
        if not result.result.success:
            self.node.get_logger().warn(
                f"[{self.name}] Step {self.STEPS[self.step]} failed"
            )
            return py_trees.common.Status.FAILURE

        # Advance to next step
        self.step += 1
        if self.step >= len(self.STEPS):
            self.feedback_message = "Pick sequence complete"
            return py_trees.common.Status.SUCCESS

        self._send_step()
        return py_trees.common.Status.RUNNING


# ===========================================================================
# CoverageTask
# ===========================================================================

class CoverageTask(py_trees.behaviour.Behaviour):
    """Send a coverage navigation goal and monitor until complete.

    Args:
        name (str): Name of the behaviour.
        planner (str): Planner type, e.g. ``skeleton``.
    """

    def __init__(self, name: str, planner: str):
        super(CoverageTask, self).__init__(name=name)
        self.planner = planner

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e

        self.coverage_action_client = ActionClient(
            self.node, NavigateCoverage, "/labclean_navigator/coverage"
        )
        self.goal_handle = None
        self.result_future = None
        self.coverage_future = None

    def initialise(self):
        self.logger.info("{}.initialise()".format(self.qualified_name))
        self.goal_handle = None
        self.result_future = None

        if not self.coverage_action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().warn(
                f"[{self.name}] Coverage action server not available"
            )
            self.coverage_future = None
            return

        goal_msg = NavigateCoverage.Goal()
        goal_msg.planner_type = NavigateCoverage.Goal.SKELETON
        goal_msg.verbose = True
        self.coverage_future = self.coverage_action_client.send_goal_async(goal_msg)
        self.feedback_message = "Coverage goal sent"

    def update(self):
        if self.coverage_future is None:
            return py_trees.common.Status.FAILURE

        if not self.coverage_future.done():
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            self.goal_handle = self.coverage_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = "Coverage goal rejected"
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()

        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "Coverage complete"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"Coverage failed: {result.status}"
        return py_trees.common.Status.FAILURE


# ===========================================================================
# GetPlanarObjects
# ===========================================================================

class GetPlanarObjects(py_trees.behaviour.Behaviour):
    """Query the planar object detection service and write to the blackboard.

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str): Blackboard key to write detected objects into.
    """

    def __init__(self, name: str, blackboard_key: str = "planar_objects_detected_array"):
        super(GetPlanarObjects, self).__init__(name=name)
        self.blackboard_key = blackboard_key

    def setup(self, **kwargs):
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            ) from e

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=self.blackboard_key,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="planar_objects_detected_bool",
            access=py_trees.common.Access.WRITE,
        )
        self.client = self.node.create_client(
            GetDetectedObjects, "/perception/planar/get_detected_objects"
        )
        self.future = None

    def initialise(self):
        self.logger.info("{}.initialise()".format(self.qualified_name))
        self.future = self.client.call_async(GetDetectedObjects.Request())
        self.feedback_message = "Waiting for detection response"

    def update(self):
        if self.future is None:
            self.feedback_message = "Service unavailable"
            return py_trees.common.Status.FAILURE

        if not self.future.done():
            return py_trees.common.Status.RUNNING

        response = self.future.result()
        objects = response.detected_objects.objects

        if len(objects) == 0:
            self.feedback_message = "No objects detected"
            return py_trees.common.Status.FAILURE

        self.blackboard.set(self.blackboard_key, objects)
        self.blackboard.set("planar_objects_detected_bool", True)
        self.feedback_message = f"Detected {len(objects)} objects"
        return py_trees.common.Status.SUCCESS
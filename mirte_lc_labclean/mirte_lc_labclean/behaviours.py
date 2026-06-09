"""ROS2 behaviours for the Mirte LabClean application.

This module defines a set of py_trees behaviours used in the LabClean
behaviour tree. Behaviours include LED control, coverage navigation, object
retrieval, and plan execution using ROS2 action servers and services.
"""

from mirte_msgs.msg import NeopixelColor
from mirte_msgs.srv import SetNeopixel

from mirte_lc_msgs.action import MoveToPosition, NavigateCoverage
from mirte_lc_msgs.srv import ServeCoverageStatus
from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
from rclpy.action import ActionClient
import std_msgs.msg as std_msgs

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus

from mirte_lc_msgs.srv import GetDetectedObjects
from tf2_ros import Buffer, TransformListener
import numpy as np

class FlashLedStrip(py_trees.behaviour.Behaviour):
    """Flash the LabClean LED strip with a colour command.

    This behaviour publishes a Neopixel colour request and shows an LED marker
    in the simulated map. It remains in the running state and does not
    complete successfully until interrupted or cancelled.

    Args:
        name (str): Name of the behaviour.
        colour (list[float], optional): RGB colour values in range [0.0, 1.0].
            Defaults to [0, 0, 1.0].
    """

    def __init__(
        self,
        name: str,
        colour: list = [0, 0, 1.0],
    ):
        super(FlashLedStrip, self).__init__(name=name)
        self.colour = colour

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.neopixel_client = self.node.create_client(
            SetNeopixel, "/io/leds/leds/set_color"
        )
        self.feedback_message = "Neopixel service client created"
        
        self.led_marker_publisher = self.node.create_publisher(Marker, '/labclean_led_markers', 10)

    def initialise(self):
        """Send the LED colour request and publish a marker."""
        self.logger.info(
            "%s.initialise(), sending led request" % self.__class__.__name__
        )
        request = SetNeopixel.Request()

        # Robot and message definitions differ
        # msg       led strip
        # red       blue
        # green     red      
        # blue      green
        request.color = NeopixelColor()
        request.color.g = int(self.colour[0] * 255)
        request.color.b = int(self.colour[1] * 255)
        request.color.r = int(self.colour[2] * 255)

        self.future = self.neopixel_client.call_async(request)
        self.publish_led_marker(self.colour)
        self.feedback_message = "Sent LED request"
        
    def publish_led_marker(self, colour):
        """Publish a visualization marker describing the LED colour.

        Args:
            colour (list[float]): The RGB colour values to display in RViz.
        """
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
        self.feedback_message = f"LED updated"
        return py_trees.common.Status.SUCCESS


class SetCoverageStatus(py_trees.behaviour.Behaviour):
    """Update the coverage navigation status using a service call.

    The behaviour sends a pause, resume, or stop command to the coverage
    navigation server. It monitors the asynchronous service response and
    reports success when the request is complete.

    Args:
        name (str): Name of the behaviour.
        requested_status (str): One of ``pause``, ``resume``, or ``stop``.
    """

    status_commands = {
        "pause": ServeCoverageStatus.Request.PAUSE,
        "resume": ServeCoverageStatus.Request.RESUME,
        "stop": ServeCoverageStatus.Request.STOP,
    }

    def __init__(
        self,
        name: str,
        requested_status: str,
    ):
        super(SetCoverageStatus, self).__init__(name=name)
        self.requested_status = requested_status

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.coverage_status_client = self.node.create_client(
            ServeCoverageStatus, "/labclean_navigator/set_state"
        )

        self.feedback_message = "Coverage Status service client created"

    def initialise(self):
        """Send the requested coverage status command."""
        self.logger.info(
            f"%s.initialise(), sending {self.requested_status} request"
            % self.__class__.__name__
        )
        request = ServeCoverageStatus.Request()

        # Adjust field names to match your srv definition
        try:
            request.command = self.status_commands[self.requested_status]
        except KeyError as e:
            error_message = f"Requested status command \
                {self.requested_status} does not exist, \
                possible commands are {self.status_commands}"
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.future = self.coverage_status_client.call_async(request)

        self.feedback_message = f"Sent {self.requested_status} request"

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self.future.done():

            try:
                response = self.future.result()
                self.feedback_message = f"status updated, status: {response.succeeded} \
                    waypoints left: {response.remaining_poses}"
                return py_trees.common.Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"Service call failed: {e}"
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class NavigateToPosition(py_trees.behaviour.Behaviour):
    """Navigate the robot to a target position.

    This behaviour sends a navigation goal to the ROS2 navigation stack and
    monitors the distance remaining until the target is reached.

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str, optional): Blackboard key containing detected objects.
        target_position (list[float] | tuple[float, float], optional): Fixed
            XY position in the map frame.
    """

    def __init__(self, name: str, blackboard_key: str=None, target_position=None):
        super(NavigateToPosition, self).__init__(name=name)
        self.target_position = target_position
        self.blackboard_key = blackboard_key

    def setup(self, **kwargs):
        """Create ROS2 subscriptions and blackboard bindings for navigation.

        Args:
            **kwargs: Dictionary containing runtime context from the behaviour
                tree. Expects a ROS2 node under the key ``node``.

        Raises:
            KeyError: If ``node`` is not provided in ``kwargs``.
        """
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.blackboard = self.attach_blackboard_client(name=self.name)
        if self.blackboard_key is not None:
            self.blackboard.register_key(
                key=self.blackboard_key,
                access=py_trees.common.Access.READ
            )

        self._distance_remaining = float('inf')
        
        self.node.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._feedback_callback,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def _feedback_callback(self, msg):
        self._distance_remaining = msg.feedback.distance_remaining


    def initialise(self):
        self.Navigator = BasicNavigator()
        self._close_enough = False
        transform = self.tf_buffer.lookup_transform(
            "map",
            "base_link",
            rclpy.time.Time()
        )
        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        
        if self.blackboard_key is not None:
            objects = self.blackboard.get(self.blackboard_key)
            
            if not objects:
                self.feedback_message = "No objects on blackboard"
                return
            
            closest = min(
                objects,
                key=lambda obj: (obj.pose.position.x - rx)**2 + (obj.pose.position.y - ry)**2
            )
            
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = closest.pose.position.x
            pose_msg.pose.position.y = closest.pose.position.y
            self.Navigator.goToPose(pose_msg)

        elif self.target_position is not None:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = float(self.target_position[0])
            pose_msg.pose.position.y = float(self.target_position[1])
            self.Navigator.goToPose(pose_msg)

    def update(self):
        if self._close_enough:
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(str(self._distance_remaining))

        if 0.0 < self._distance_remaining < 0.5:
            self._close_enough = True
            self.Navigator.cancelTask()
            return py_trees.common.Status.SUCCESS

        navigation_result = self.Navigator.getResult()
        if navigation_result == TaskResult.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif navigation_result in [TaskResult.FAILED, TaskResult.CANCELED]:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class MoveArm(py_trees.behaviour.Behaviour):
    """Move the robot arm to a named or explicit target pose.

    The behaviour can use a predefined pose name, a blackboard key, or an explicit
    target position to build an arm movement action goal.

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str, optional): Blackboard key holding a target pose.
        target_position (tuple[float, float, float], optional): Explicit arm pose
            position.
        predefined_pose (str, optional): Name of a predefined arm pose.
    """

    def __init__(self, name: str, blackboard_key: str=None, target_position=None, predefined_pose: str=None):
        super(MoveArm, self).__init__(name=name)
        self.target_position = target_position
        self.blackboard_key = blackboard_key
        self.predefined_pose = predefined_pose

    def setup(self, **kwargs):
        """Prepare the arm action client and attach a blackboard client.

        Args:
            **kwargs: Dictionary containing runtime context from the behaviour
                tree. Expects a ROS2 node under the key ``node``.
        """
        self.logger.info("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.blackboard = self.attach_blackboard_client(name=self.name)
        if self.blackboard_key is not None:
            self.blackboard.register_key(
                key=self.blackboard_key,
                access=py_trees.common.Access.READ
            )

        
        self.arm_action_client = ActionClient(self.node, MoveToPosition, "/arm_controller/move_to_position")
        self.goal_handle = None
        
    
    def initialise(self):
        """Build and send a target goal to the arm action server."""
        self.goal_handle = None
        self.result_future = None
        
        goal_msg = MoveToPosition.Goal()

        if self.predefined_pose is not None:
            goal_msg.mirte_arm_named_target = self.predefined_pose

        elif self.blackboard_key is not None:
            goal_msg.target_pose = self.blackboard.get(self.blackboard_key)
        
        elif self.target_position is not None:
            # Create a Pose message from the target_position tuple
            pose_msg = Pose()
            pose_msg.position.x = self.target_position[0]
            pose_msg.position.y = self.target_position[1]
            pose_msg.position.z = self.target_position[2]

            goal_msg = MoveToPosition.Goal()
            goal_msg.target_pose = pose_msg

        self.arm_future = self.arm_action_client.send_goal_async(goal_msg)
    
    def update(self):
        rclpy.spin_once(self.node, timeout_sec=5)
        return py_trees.common.Status.SUCCESS


class PickObject(py_trees.behaviour.Behaviour):
    """Pick an object from the workspace using a multi-step manipulation sequence.

    The behaviour chooses the closest detected object and executes a series of
    manipulation steps, including approach, gripper control, and placement.
    """

    STEPS = [
        "approach", "open", "dive", "grip", "place", "let_go", "standby"
    ]

    def __init__(self, name: str, blackboard_key: str = "planar_objects_detected_array"):
        """Create the pick object behaviour.

        Args:
            name (str): Name of the behaviour.
            blackboard_key (str): Blackboard key from which to read detected objects.
        """
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
            access=py_trees.common.Access.READ
        )

        self.pick_action_client = ActionClient(
            self.node, MoveToPosition, "/move_to_position"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.pick_action_client.wait_for_server()

    def initialise(self):
        self.step = 0
        self.current_future = None
        self.goal_handle = None
        self.result_future = None

        objects = self.blackboard.get(self.blackboard_key)
        if not objects:
            self.object = None
            return

        # pick the closest object
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            self.object = min(
                objects,
                key=lambda o: (o.pose.position.x - rx)**2 + (o.pose.position.y - ry)**2
            )
        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")
            self.object = objects[0]

        self._send_step()

    def _build_goal(self):
        goal_msg = MoveToPosition.Goal()
        obj = self.object
        step = self.STEPS[self.step]

        if step == "approach":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "wrist", rclpy.time.Time()
                )
                rx = transform.transform.translation.x
                ry = transform.transform.translation.y
                rz = transform.transform.translation.z
            except Exception:
                rx, ry, rz = 0.0, 0.0, 0.3

            k_p = 1 / 10000
            target_pose_y = 200
            error = target_pose_y - obj.pose.position.y

            pose = Pose()
            pose.position.x = rx + obj.pose.position.x * k_p
            pose.position.y = ry + error * k_p
            pose.position.z = rz
            goal_msg.target_pose = pose

        elif step == "open":
            goal_msg.mirte_gripper_named_target = "open"

        elif step == "dive":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "wrist", rclpy.time.Time()
                )
                rx = transform.transform.translation.x
                ry = transform.transform.translation.y
                rz = transform.transform.translation.z
            except Exception:
                rx, ry, rz = 0.0, 0.0, 0.3

            k_p = 1 / 10000
            target_pose_y = 200
            error = target_pose_y - obj.pose.position.y

            pose = Pose()
            pose.position.x = rx + obj.pose.position.x * k_p
            pose.position.y = ry + error * k_p
            pose.position.z = rz - 0.1
            goal_msg.target_pose = pose

        elif step == "grip":
            goal_msg.mirte_gripper_named_target = "close"

        elif step == "place":
            goal_msg.mirte_arm_named_target = (
                "place_left" if obj.label == "target" else "place_right"
            )

        elif step == "let_go":
            goal_msg.mirte_gripper_named_target = "open"

        elif step == "standby":
            goal_msg.mirte_arm_named_target = "standby"

        return goal_msg

    def _send_step(self):
        goal_msg = self._build_goal()
        self.current_future = self.pick_action_client.send_goal_async(goal_msg)
        self.goal_handle = None
        self.result_future = None
        self.feedback_message = f"Sending step: {self.STEPS[self.step]}"

    def update(self):
        if self.object is None:
            return py_trees.common.Status.FAILURE

        # Stage 1: wait for goal acceptance
        if not self.current_future.done():
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            self.goal_handle = self.current_future.result()
            if not self.goal_handle.accepted:
                self.node.get_logger().warn(
                    f"Step {self.STEPS[self.step]} rejected"
                )
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()

        # Stage 2: wait for result
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().warn(
                f"Step {self.STEPS[self.step]} failed with status {result.status}"
            )
            return py_trees.common.Status.FAILURE

        # Step succeeded — move to next
        self.step += 1
        if self.step >= len(self.STEPS):
            return py_trees.common.Status.SUCCESS

        self._send_step()
        return py_trees.common.Status.RUNNING


class CoverageTask(py_trees.behaviour.Behaviour):
    """Start a coverage navigation action for lab cleaning.

    The behaviour sends a coverage goal and monitors the action server until
    coverage completes or fails.

    Args:
        name (str): Name of the behaviour.
        planner (str): Planner type for coverage navigation, such as ``skeleton``.
    """

    def __init__(self, name: str, planner: str):
        super(CoverageTask, self).__init__(name=name)
        self.planner = planner

    def setup(self, **kwargs):
        """Create the coverage action client and capture the ROS2 node.

        Args:
            **kwargs: Dictionary containing runtime context from the behaviour
                tree. Expects a ROS2 node under the key ``node``.

        Raises:
            KeyError: If ``node`` is not provided in ``kwargs``.
        """
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e
        self.logger.info("{}.setup()".format(self.qualified_name))
        self.coverage_action_client = ActionClient(self.node, NavigateCoverage, "/labclean_navigator/coverage")
        self.goal_handle = None

    def initialise(self):
        """Send the coverage navigation action goal."""
        self.logger.info("{}.initialise()".format(self.qualified_name))
        goal_msg = NavigateCoverage.Goal()
        goal_msg.planner_type = NavigateCoverage.Goal.SKELETON
        goal_msg.verbose = True
        
        self.coverage_action_client.wait_for_server()
        self.coverage_future = self.coverage_action_client.send_goal_async(goal_msg)
        self.feedback_message = "Sent request"

    def update(self):
        """Check the coverage action status and return the behaviour result.

        Returns:
            py_trees.common.Status: SUCCESS when coverage completes,
                FAILURE if the action fails, or RUNNING while still active.
        """
        if self.coverage_future is None:
            return py_trees.common.Status.FAILURE

        # Stage 1: wait for server to accept the goal
        if not self.coverage_future.done():
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            self.goal_handle = self.coverage_future.result()
            if not self.goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            # Stage 2: now request the actual result
            self.result_future = self.goal_handle.get_result_async()

        # Stage 2: wait for the action to complete
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# class BoundingBoxes2BB(py_trees.behaviour.Behaviour):
#     def __init__(self, name: str):
#         super(BoundingBoxes2BB, self).__init__(name=name)
    
#     def setup(self, **kwargs):
#         """
#         Setup the publisher which will stream commands to the mock robot.

#         Args:
#             **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

#         Raises:
#             :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
#         """
#         try:
#             self.node = kwargs["node"]
#         except KeyError as e:
#             error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
#                 self.qualified_name
#             )
#             raise KeyError(error_message) from e
#         self.bounding_box_sub = self.node.create_subscription(
#             DetectedObjectArray, '/perception/depth/detected_objects', self.sub_callback, 10)
#         self.bounding_boxes = []
#         self.navigator = BasicNavigator
        
#     def sub_callback(self, msg):
#         boxes = msg.objects
#         for box in boxes:
#             for existing_box in self.bounding_boxes:
#                 if np.linalg.norm(np.array(box.pose) - np.array(existing_box.pose)) > 0.1:
#                     exists = True
#                 else:
#                     exists = False
#             if exists:
#                 self.bounding_boxes.append(box)
#                 self.bounding_boxes = sorted(self.bounding_boxes, key=)

class GetPlanarObjects(py_trees.behaviour.Behaviour):
    """Query the planar object detection service and store results on the blackboard.

    The behaviour requests detected objects from the perception service and writes
    them to the specified blackboard key.

    Args:
        name (str): Name of the behaviour.
        blackboard_key (str): Blackboard key to write detected objects into.
    """

    def __init__(self, name: str, blackboard_key: str = "planar_objects_detected_array"):
        """Create the planar object detection behaviour.

        Args:
            name (str): Name of the behaviour.
            blackboard_key (str): Blackboard key for detected objects storage.
        """
        super(GetPlanarObjects, self).__init__(name=name)
        self.blackboard_key = blackboard_key

    def setup(self, **kwargs):
        """Register the service client and prepare the blackboard write keys.

        Args:
            **kwargs: Dictionary containing runtime context from the behaviour
                tree. Expects a ROS2 node under the key ``node``.
        """
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
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="planar_objects_detected_bool",
            access=py_trees.common.Access.WRITE
        )

        self.client = self.node.create_client(
            GetDetectedObjects,
            "/perception/planar/get_detected_objects"
        )
        self.future = None

    def initialise(self):
        self.logger.info("{}.initialise()".format(self.qualified_name))
        self.future = None

        rclpy.spin_once(self.node, timeout_sec=5)
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
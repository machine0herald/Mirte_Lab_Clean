from mirte_msgs.msg import NeopixelColor
from mirte_msgs.srv import SetNeopixel

from mirte_lc_msgs.action import MoveToPosition, NavigateCoverage
from mirte_lc_msgs.srv import ServeCoverageStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
from rclpy.action import ActionClient
import std_msgs.msg as std_msgs

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


class FlashLedStrip(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """

    def __init__(
        self,
        name: str,
        colour: list = [0, 0, 255],
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
        self.logger.debug("{}.setup()".format(self.qualified_name))
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
        
        self.led_marker_publisher = self.create_publisher(Marker, '/labclean_led_markers', 10)

    def initialize(self):
        """ """
        self.logger.debug(
            "%s.initialise(), sending led request" % self.__class__.__name__
        )
        request = SetNeopixel.Request()

        # Adjust field names to match your srv definition
        request.color = NeopixelColor()
        request.color.r = self.colour[0]
        request.color.g = self.colour[1]
        request.color.b = self.colour[2]

        self.future = self.neopixel_client.call_async(request)
        self.publish_led_marker(self.colour)
        self.feedback_message = "Sent LED request"
        
    def publish_led_marker(self, colour):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.9
        marker.scale.y = 0.03
        marker.scale.z = 1.9
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.8
        marker.pose.position.z = 0.8
        
        marker.color.r = colour[0]
        marker.color.g = colour[1]
        marker.color.b = colour[2]
        marker.color.a = 1.0
        
        self.led_marker_publisher.publish(marker)

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self.future.done():

            try:
                response = self.future.result()
                self.feedback_message = f"LED updated, status: {response.status}"
                return py_trees.common.Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"Service call failed: {e}"
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class SetCoverageStatus(py_trees.behaviour.Behaviour):
    """
    Tells the coverage server to pause or resume navigating the coverage path
        - pause: Sends service request with pause which makes the node around
        the server store the remaining waypoints and cancel the action

        - resume: sends service request with resume which tells the coverage
        server node to resume the coverage navigation where it left off from
        the saved remaining waypoints
    Args:
        py_trees
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
        self.logger.debug("{}.setup()".format(self.qualified_name))
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

    def initialize(self):
        """ """
        self.logger.debug(
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
                    waypoints left: {len(response.remaining_poses)}"
                return py_trees.common.Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"Service call failed: {e}"
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class NavigateToPosition(py_trees.behaviour.Behaviour):
    """
    This behaviour sends a goal to the navigation stack to move the robot to a specified position.
    It returns RUNNING while the robot is moving, SUCCESS when it reaches the goal, and FAILURE if it fails to reach the goal.

    Args:
        name: name of the behaviour
        predefined_pose: predefined pose to move the arm to
        target_position: a tuple (x, y) representing the target position in the map frame
    """

    def __init__(self, name: str, blackboard_key: str=None, target_position=None):
        super(NavigateToPosition, self).__init__(name=name)
        self.target_position = target_position
        self.blackboard_key = blackboard_key

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.Navigator = BasicNavigator()

    def initialize(self):
        """
        Send the navigation goal to the robot.
        """            
        if self.blackboard_key is not None:
            self.target_pose_msg = self.blackboard.get(self.blackboard_key)
            self.Navigator.goToPose(self.target_pose_msg)
        elif self.target_position is not None:
            pose_msg = Pose()
            pose_msg.position.x = self.target_position[0]
            pose_msg.position.y = self.target_position[1]
            self.Navigator.goToPose(self.pose_msg)

    def update(self):
        """
        Check the status of the navigation action and return the appropriate status for the behaviour.

        Returns:
            :attr:`~py_trees.common.Status.SUCCESS` if the robot reaches the target position, :attr:`~py_trees.common.Status.FAILURE` if it fails, and :attr:`~py_trees.common.Status.RUNNING` while it's still moving.
        """
        navigation_result = self.Navigator.getResult()

        if navigation_result == TaskResult.SUCCEEDED:
            return py_trees.common.Status.SUCCESS

        elif navigation_result in [TaskResult.FAILED, TaskResult.CANCELED]:
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING


class MoveArm(py_trees.behaviour.Behaviour):
    """
    This behaviour sends a goal to the arm manipulation stack to move the robot's arm to a specified position.
    It returns RUNNING while the arm is moving, SUCCESS when it reaches the goal, and FAILURE if it fails to reach the goal.

    Args:
        name: name of the behaviour
        target_position: a tuple (x, y, z) representing the target position for the arm in the robot's coordinate frame
    """

    def __init__(self, name: str, blackboard_key: str=None, target_position=None, predefined_pose: str=None):
        super(MoveArm, self).__init__(name=name)
        self.target_position = target_position
        self.blackboard_key = blackboard_key
        self.predefined_pose = predefined_pose

    def setup(self, **kwargs):
        """
        Setup the action client for arm manipulation.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.arm_action_client = ActionClient(self.node, MoveToPosition, "/arm_controller/move_to_position")
    
    def initialise(self):
        """
        Send the arm movement goal to the robot.
        """
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
        """
        Check the status of the arm movement action and return the appropriate status for the behaviour.

        Returns:
            :attr:`~py_trees.common.Status.SUCCESS` if the arm reaches the target position, :attr:`~py_trees.common.Status.FAILURE` if it fails, and :attr:`~py_trees.common.Status.RUNNING` while it's still moving.
        """
        if self.arm_future is None:
            return py_trees.common.Status.FAILURE

        if self.arm_future.done():
            result = self.arm_future.result()
            if result.status == rcl_msgs.GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


# class PickObject(py_trees.behaviour.Behaviour):
#     """
#     This behaviour sends a goal to the arm manipulation stack to pick up an object at a specified position.
#     It returns RUNNING while the arm is moving, SUCCESS when it successfully picks up the object, and FAILURE if it fails.

#     Args:
#         name: name of the behaviour
#         object_position: a tuple (x, y, z) representing the position of the object in the robot's coordinate frame
#     """

#     def __init__(self, name: str, blackboard_key: str=None, object_position: tuple=None):
#         super(PickObject, self).__init__(name=name)
#         self.blackboard_key = blackboard_key
#         self.object_position = object_position

#     def setup(self, **kwargs):
#         """
#         Setup the publisher which will stream commands to the mock robot.

#         Args:
#             **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

#         Raises:
#             :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
#         """
#         self.logger.debug("{}.setup()".format(self.qualified_name))
#         try:
#             self.node = kwargs["node"]
#         except KeyError as e:
#             error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
#                 self.qualified_name
#             )
#             raise KeyError(error_message) from e
#         self.logger.debug("{}.setup()".format(self.qualified_name))

#         self.pick_action_client = ActionClient(self.node, MoveToPosition, "/arm_controller/pick_object")
    
#     def initialise(self):
#         """
#         Send the pick object goal to the robot.
#         """
#         if self.blackboard_key is not None:
#             goal_msg = MoveToPosition.Goal()
#             goal_msg.target_pose = self.blackboard.get(self.blackboard_key)
#             self.pick_future = self.pick_action_client.send_goal_async(goal_msg)
#         elif self.object_position is not None:
#             # Create a Pose message from the object_position tuple
#             pose_msg = Pose()
#             pose_msg.position.x = self.object_position[0]
#             pose_msg.position.y = self.object_position[1]
#             pose_msg.position.z = self.object_position[2]

#             goal_msg = MoveToPosition.Goal()
#             goal_msg.target_pose = pose_msg
#             self.pick_future = self.pick_action_client.send_goal_async(goal_msg)
    
#     def update(self):
#         """
#         Check the status of the pick object action and return the appropriate status for the behaviour.

#         Returns:
#             :attr:`~py_trees.common.Status.SUCCESS` if the arm successfully picks up the object, :attr:`~py_trees.common.Status.FAILURE` if it fails, and :attr:`~py_trees.common.Status.RUNNING` while it's still moving.
#         """
#         if self.pick_future is None:
#             return py_trees.common.Status.FAILURE

#         if self.pick_future.done():
#             result = self.pick_future.result()
#             if result.status == rcl_msgs.GoalStatus.STATUS_SUCCEEDED:
#                 return py_trees.common.Status.SUCCESS
#             else:
#                 return py_trees.common.Status.FAILURE

#         return py_trees.common.Status.RUNNING


class CoverageTask(py_trees.behaviour.Behaviour):
    """
    This behaviour sends a goal to the coverage navigation stack to start covering a specified area.
    It returns RUNNING while the robot is covering, SUCCESS when it finishes covering the area, and FAILURE if it fails.

    Args:
        name: name of the behaviour
        area: a list of tuples [(x1, y1), (x2, y2), ...] representing the vertices of the area to cover in the map frame
    """

    def __init__(self, name: str, planner: str):
        super(CoverageTask, self).__init__(name=name)
        self.planner = planner

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e
        self.logger.debug("{}.setup()".format(self.qualified_name))
        self.coverage_action_client = ActionClient(self.node, NavigateCoverage, "/coverage_controller/move_to_position")

    def initialise(self):
        """
        Send the coverage navigation goal to the robot.
        """
        goal_msg = NavigateCoverage.Goal()
        goal_msg.planner_type = self.planner
        self.coverage_future = self.coverage_action_client.send_goal_async(goal_msg)

    def update(self):
        """
        Check the status of the coverage navigation action and return the appropriate status for the behaviour.

        Returns:
            :attr:`~py_trees.common.Status.SUCCESS` if the robot finishes covering the area, :attr:`~py_trees.common.Status.FAILURE` if it fails, and :attr:`~py_trees.common.Status.RUNNING` while it's still covering.
        """
        if self.coverage_future is None:
            return py_trees.common.Status.FAILURE

        if self.coverage_future.done():
            result = self.coverage_future.result()
            if result.status == rcl_msgs.GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
from mirte_msgs.msg import NeopixelColor
from mirte_msgs.srv import SetNeopixel

from mirte_lc_msgs.srv import ServeCoverageStatus

import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import std_msgs.msg as std_msgs


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
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.neopixel_client = self.node.create_client(
            SetNeopixel, "/io/leds/leds/set_color"
        )
        self.feedback_message = "Neopixel service client created"

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

        self.future = self.client.call_async(request)

        self.feedback_message = "Sent LED request"

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
        "pause": ServeCoverageStatus.PAUSE,
        "resume": ServeCoverageStatus.RESUME,
        "stop": ServeCoverageStatus.STOP,
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

        self.future = self.client.call_async(request)

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

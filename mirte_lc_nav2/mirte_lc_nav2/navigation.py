"""
ros2 run mirte_lc_nav2 labclean_navigator
ros2 lifecycle set labclean_navigator configure
"""

from typing import Optional
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Goals
from nav2_msgs.action import NavigateThroughPoses

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionClient

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

import mirte_lc_nav2.navigators as nv
import mirte_lc_nav2.utils as ut

import numpy as np
import math


def get_path_planner(name):
    path_planners = {
        "bous": nv.BousPath,
        "spiral": nv.SpiralPath,
        "straightline": nv.StraightLinePath,
    }
    if name in path_planners:
        return path_planners[name]
    raise ValueError(f"Unknown path planner: {name}")


class LabCleanNavigator(Node):
    """
    Our lifecycle talker node.\\
    TODO: \\
        Read parameters before every configure, 
        so it is possible to use different planners upon lifecycle restarts 
    """

    def __init__(
        self,
        node_name: str,
    ) -> None:
        super().__init__(node_name)

        # Set Planner Parameters
        self._planner_name = self.declare_parameter("planner_type", "bous")
        self._verbose = self.declare_parameter("verbose", True)

        # ROS2 Interfaces
        self._posegoal_publisher = None
        self._path_publisher = None
        self.costmap_sub = None
        self.path = None
        self.map = None
        self.counter = True

    def path_publisher(self) -> None:
        """
        Publish a new path message when enabled.
        """
        if self.counter:
            if self.map is None:
                self.get_logger().warn("Map not received yet")
                return
            self.planner.update_map(self.map)

            self.path = ut.to_ros_path(self.planner.path)

            if self.path is None:
                return  # nothing to send yet

            self.get_logger().info("New path generated, updating waypoints.")

            goal = NavigateThroughPoses.Goal()
            goal.poses = self.path.poses
            for pose in goal.poses:
                pose.header.frame_id = "map"

            if self._verbose:
                self.get_logger().info(
                    f"Path publisher is active. Sending path goal request"
                )

            self._path_publisher.wait_for_server()

            # Cancel if a previous goal is still executing
            if hasattr(self, "goal_handle") and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()

            self._send_goal_future = self._path_publisher.send_goal_async(
                goal, feedback_callback=self.feedback_callback
            )

            self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self._result_future = self.goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)
        self.path = None
        self.counter = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if self._verbose:
            self.get_logger().info(f"Distance remaining: {feedback.distance_remaining}")

    def result_callback(self, future):
        result = future.result().result
        if self._verbose:
            self.get_logger().info(f"Path execution result: {result}")
            self.get_logger().info("Path execution complete")
            self.trigger_shutdown()

    def costmap_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.map = np.array(msg.data, dtype=np.int16).reshape((height, width))
        self.path_publisher()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
            in "unconfigured" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        try:
            self.planner = get_path_planner(self._planner_name.value)(self)
            self.get_logger().info("on_configure() is called.")

            # Configure Navigators
            match self.planner.navigator_type:
                case "systematic":
                    self.costmap_sub = self.create_subscription(
                        OccupancyGrid,
                        "/global_costmap/costmap",
                        self.costmap_callback,
                        10,
                    )
                    self._path_publisher = ActionClient(
                        self, NavigateThroughPoses, "/navigate_through_poses"
                    )
                case "reactive":
                    self._goal_publisher = self.create_lifecycle_publisher(
                        PoseStamped, "/goal_pose", 10
                    )
                    self.goal_timer = self.create_timer(1.0, self.goal_publisher)
                case _:
                    self.get_logger().error(
                        f"Unknown navigator type: {self.planner.navigator_type}"
                    )

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE

    def startup(self, node_name="bt_navigator"):
        # Waits for the node within the tester namespace to become active
        print(f"Waiting for {node_name} to become active..")
        node_service = f"{node_name}/get_state"
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            print(f"Getting {node_name} state...")
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f"Result of get_state: {state}")
            time.sleep(2)
        return

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the
        # inactive and enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks,
        # and we don't need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")
        super().on_activate(state)
        # self.path_publisher()

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_deactivate() is called.")
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")

        # Destroy subscription
        if hasattr(self, "costmap_sub") and self.costmap_sub is not None:
            self.destroy_subscription(self.costmap_sub)
            self.costmap_sub = None

        # Destroy action client
        if hasattr(self, "_path_publisher") and self._path_publisher is not None:
            self._path_publisher.destroy()
            self._path_publisher = None

        # Destroy goal publisher (reactive mode)
        if hasattr(self, "_goal_publisher") and self._goal_publisher is not None:
            self.destroy_publisher(self._goal_publisher)
            self._goal_publisher = None

        # Destroy timer (reactive mode)
        if hasattr(self, "goal_timer") and self.goal_timer is not None:
            self.destroy_timer(self.goal_timer)
            self.goal_timer = None

        # Reset state
        self.map = None
        self.path = None
        self.previous_path = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.

        :return: The state machine either invokes a transition to the "finalized" state or stays
            in the current state depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        if self._timer is not None:
            self.destroy_timer(self._timer)
        if self._pub is not None:
            self.destroy_publisher(self._pub)

        self.get_logger().info("on_shutdown() is called.")
        return TransitionCallbackReturn.SUCCESS

    def trigger_shutdown(self):
        client = self.create_client(ChangeState, f"/{self.get_name()}/change_state")

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Lifecycle change_state service not available")
            return

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DESTROY

        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info("Shutdown transition requested")
        )


def main() -> None:
    try:
        rclpy.init()

        executor = SingleThreadedExecutor()
        lc_node = LabCleanNavigator("labclean_navigator")
        executor.add_node(lc_node)

        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

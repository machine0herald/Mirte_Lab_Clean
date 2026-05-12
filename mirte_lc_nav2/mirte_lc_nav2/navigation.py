"""
ros2 run mirte_lc_nav2 labclean_navigator
ros2 lifecycle set labclean_navigator configure
"""
import numpy as np
np.float = float
from typing import Optional
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Goals
from nav2_msgs.action import FollowPath, NavigateThroughPoses

from tf2_ros import Buffer, TransformListener

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

# import navigators as nv
# import utils as ut

import numpy as np
import math


def lookup_planner(name):
    planners = nv.PLANNERS
    try:
        return planners[name]
    except Exception as e:
        print(
            f"{e}, Desired planner '{name}' does not exist, \
            recognized types are: {planners.keys()}, \
            Defaulting to Boustrophedon Planner"
         )
        return planners['BousPlanner']


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
        self._planner_name = self.declare_parameter("planner_type", "SkeletonPlanner")
        self._verbose = self.declare_parameter("verbose", True)

        # ROS2 Interfaces
        self._posegoal_publisher = None
        self._path_publisher = None
        self.costmap_sub = None
        self.queue = []
        self.map = None
        self.counter = True
        self.executing = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ------------- Navigate Through Poses ----------------
    def execute(self):
        if len(self.queue) == 0:
            self.get_logger().warn("Queue is empyty")
            return

        if self.executing:
            self.get_logger().info("Already executing a goal.")
            return

        self.path_publisher()

    def path_publisher(self) -> None:
        """
        Publish a new path message when enabled.
        """

        path = ut.to_ros_path(self.queue[0])

        self.get_logger().info("New path generated, updating waypoints.")

        # goal = FollowPath.Goal()
        # goal.path = path
        goal = NavigateThroughPoses.Goal()
        goal.poses = path.poses

        if self._verbose:
            self.get_logger().info(
                f"Path publisher is active. Sending path goal request: {goal}"
            )

        self._path_publisher.wait_for_server()

        self.executing = True
        self._send_goal_future = self._path_publisher.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.goal_handle = goal_handle

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

        self.get_logger().warn("Goal accepted, popping from queue")

    def feedback_callback(self, feedback_msg):
        if self.executing:
            feedback = feedback_msg.feedback
            self.distance_remaining = feedback.distance_remaining
            if self._verbose:
                self.get_logger().info(f"Distance Remaining {feedback.distance_remaining}")

            # Adjusted logic to ensure proper goal handling
            if self.distance_remaining < 0.1:
                self.get_logger().info("Goal reached successfully.")
                self.goal_handle.cancel_goal_async()

                # Manually advance queue only if there are remaining goals
                if len(self.queue) > 0:
                    self.queue.pop(0)

                self.executing = False

                # Start next path only if queue is not empty
                if len(self.queue) > 0:
                    self.execute()
        else:
            return

    def result_callback(self, future):
        result = future.result().result
        if self._verbose:
            self.get_logger().info(f"Path execution result: {result}")
            self.get_logger().info("Path execution complete")
        if self.executing:
            if len(self.queue) > 0:
                self.queue.pop(0)
            self.executing = False

            # Start next path only if queue is not empty
            if len(self.queue) > 0:
                self.execute()

    # ------------- Helper Functions ----------------
    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return np.array([x, y])

        except Exception as e:
            self.get_logger().warn(f"Could not get robot pose: {e}")
            return None

    def costmap_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.map = np.array(msg.data, dtype=np.int16).reshape((height, width))
        if self.counter:
            if self.map is None:
                self.get_logger().warn("Map not received yet")
                return
            
            start = self.get_robot_position()
            
            if start is None:
                return

            self.planner.plan(self.map, start)
            self.queue.extend(self.planner.paths)
            self.counter = None
        self.execute()

    # ------------- Lifecycle states ----------------
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
            self.planner = lookup_planner(self._planner_name.value)(self)
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
                    # self._path_publisher = ActionClient(
                    #     self, FollowPath, "/follow_path"
                    # )                    
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

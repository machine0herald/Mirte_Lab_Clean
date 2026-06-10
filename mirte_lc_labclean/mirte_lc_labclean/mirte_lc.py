"""LabClean manager node for ROS2.

This module contains a simple ROS2 node that listens for exploration status
messages and starts coverage navigation when exploration is complete.
"""

import rclpy
import numpy as np
from rclpy.node import Node, Timer
from rclpy.action import (
    ActionServer,
    ActionClient,
    CancelResponse,
    GoalResponse,
)

from std_msgs.msg import String, Bool
from explore_lite_msgs.msg import ExploreStatus
from mirte_lc_msgs.action import NavigateCoverage


class LabcleanManager(Node):
    """Manage the transition from exploration to lab coverage navigation.

    The manager subscribes to exploration status updates and sends a
    NavigateCoverage action goal when exploration completes.
    """

    def __init__(self):
        """Initialize the Labclean manager node.

        The node creates a subscription to exploration status and an action
        client for coverage navigation.
        """
        super().__init__("labclean_manager")
        self.explored = False

        self.exploration_sub = self.create_subscription(
            ExploreStatus, "/explore/status", self.exploration_callback, 10
        )

        self.navigation_client = ActionClient(
            self, NavigateCoverage, "/labclean_navigator/coverage"
        )

        # Publishers to control the Lifecycle of navigation and exploration nodes
        self.exploration_controller = self.create_publisher(Bool, "explore/resume", 10)
        self.navigation_controller = self.create_publisher(
            Bool, "/labclean_navigator/active", 10
        )

    def exploration_callback(self, msg):
        """Handle exploration status updates.

        Args:
            msg (:class:`explore_lite_msgs.msg.ExploreStatus`): Exploration status message.
        """
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            self.get_logger().info(
                "Exploration completed, starting labclean navigation"
            )
            if not self.explored:
                self.explored = True
                self.send_coverage_goal()

        elif msg.status == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self.get_logger().info("Exploration active")

    def send_coverage_goal(self):
        """Send a coverage navigation goal.

        The goal uses the SKELETON planner and enables verbose feedback.
        """
        goal_msg = NavigateCoverage.Goal()
        goal_msg.planner_type = NavigateCoverage.Goal.SKELETON
        goal_msg.verbose = True

        self.navigation_client.wait_for_server()

        self.navigation_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback,
        ).add_done_callback(self.goal_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Log progress updates from the coverage navigation action.

        Args:
            feedback_msg: The feedback message from the action server.
        """
        feedback = feedback_msg.feedback
        progress = feedback.completion_percentage
        self.get_logger().info(f"Labclean navigation progress: {progress:.2f}%")

    def goal_response_callback(self, future):
        """Handle the goal response from the coverage navigation action server.

        Args:
            future: The future returned by `send_goal_async`.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Labclean navigation goal rejected")
            return

        self.get_logger().info("Labclean navigation goal accepted")


def main(args=None):
    """Start the Labclean manager ROS2 node.

    Args:
        args (list, optional): Command line arguments passed to `rclpy.init`.
    """
    rclpy.init(args=args)
    node = LabcleanManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LabcleanManager")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LabcleanManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LabcleanManager")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

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

    def __init__(self):
        super().__init__('labclean_manager')
        self.explored = False

        self.exploration_sub = self.create_subscription(
            ExploreStatus,
            '/explore/status',
            self.exploration_callback,
            10
        )
        
        self.navigation_client = ActionClient(
            self, NavigateCoverage, '/labclean_navigator/coverage'
            )
        
        # Publishers to control the Lifecycle of navigation and exploration nodes        
        self.exploration_controller = self.create_publisher(Bool, 'explore/resume', 10)
        self.navigation_controller = self.create_publisher(Bool, '/labclean_navigator/active', 10)
        
    def exploration_callback(self, msg):
        if msg.status == ExploreStatus.RETURNED_TO_ORIGIN:
            self.get_logger().info('Exploration completed, starting labclean navigation')
            if not self.explored:
                self.explored = True
                self.send_coverage_goal()

        elif msg.status == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self.get_logger().info('Exploration active')
    
    def send_coverage_goal(self):
        goal_msg = NavigateCoverage.Goal()
        goal_msg.planner_type = NavigateCoverage.Goal.BOUSTROPHEDON
        goal_msg.verbose = True
        
        self.navigation_client.wait_for_server()
        self.navigation_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback,
        ).add_done_callback(self.goal_response_callback)
        
    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        idx = feedback.current_segment
        total_segments = feedback.total_segments
        
        if total_segments > 0:
            progress = float(idx + 1) / float(total_segments) * 100.0
            self.get_logger().info(f'Labclean navigation progress: {progress:.2f}%')
            
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Labclean navigation goal rejected')
            return
        
        self.get_logger().info('Labclean navigation goal accepted')


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
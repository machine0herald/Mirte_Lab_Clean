import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    ActionClient,
    CancelResponse,
    GoalResponse,
)

from nav_msgs.msg import OccupancyGrid

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import Bool
from mirte_lc_msgs.srv import ServeCoverageStatus
from mirte_lc_msgs.action import NavigateCoverage
import mirte_lc_nav2.navigators as nv
import mirte_lc_nav2.utils as ut

import time

class LabCleanActionServer(Node):

    def __init__(self):
        super().__init__("labclean_action_server")
        self.pause_requested = False
        self.stop_requested = False
        self.remaining_poses = -1
        self.map = None
        self.nav_goal_handle = None
        self.current_segment = None
        self.goal_paths = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.set_status_server = self.create_service(
            ServeCoverageStatus, "/labclean_navigator/set_state", self.status_callback
        )

        self.server = ActionServer(
            self,
            NavigateCoverage,
            "/labclean_navigator/coverage",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_callback,
            10,
        )

    ###########
    # Helpers #
    ###########

    def get_robot_position(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return np.array([x, y])

        except Exception as e:
            self.get_logger().error(str(e))
            return None

    def lookup_planner(self, planner_name):

        planners = nv.PLANNERS

        if planner_name in planners:
            return planners[planner_name]

        self.get_logger().warn(f"Unknown planner '{planner_name}', using BousPlanner")

        return planners["BousPlanner"]
    
    ####################
    # Costmap Callback #
    ####################

    def costmap_callback(self, msg):
        self.map_msg = msg

    ####################################
    # Server Goal and Cancel Callbacks #
    ####################################

    def goal_callback(self, goal_request):
        self.get_logger().info("Received cleaning goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    ###########################
    # Server Execution Callback #
    ###########################

    def execute_callback(self, goal_handle):
        self.segment_idx = 0
        self.current_goal_handle = goal_handle
        planner_type = goal_handle.request.planner_type
        verbose = goal_handle.request.verbose
        self.verbose = verbose

        result = NavigateCoverage.Result()

        start = self.get_robot_position()

        if start is None:
            result.success = False
            result.message = "Could not determine robot pose"
            goal_handle.abort()
            return result

        planner_cls = self.lookup_planner(planner_type)
        self.planner = planner_cls(self)

        self.get_logger().info(f"Planning with {planner_type}")

        self.planner.plan(self.map_msg, start)
        self.goal_paths = self.planner.paths.copy()

        if self.planner.paths is None or len(self.planner.paths) == 0:
            result.success = False
            result.message = "Planner produced no paths"
            goal_handle.abort()
            return result

        self.goal_paths = self.planner.paths
        total_segments = len(self.goal_paths)

        self.get_logger().info('''
                        ##########################
                        # Starting Coverage Task #
                        ##########################
                        ''')
        # Execute paths sequentially
        while len(self.goal_paths) > 0:

            segment = self.goal_paths[0]
            self.goal_paths.pop(0)
            self.segment_idx += 1
            idx = self.segment_idx
            self.current_segment = segment


            path = ut.to_ros_path(segment)
            self.planner.goThroughPoses(path)

            self.get_logger().info(f"Executing segment {idx+1}/{total_segments}")
            
            self.get_logger().info('''
                ################################
                # Sending Navigation Goal Task #
                ################################
                                ''')

            while True:
                self.nav_feedback = self.planner.getFeedback()
                
                # Extract remaining poses from feedback if available
                if self.nav_feedback and hasattr(self.nav_feedback, 'number_of_recoveries'):
                    # Update remaining poses from nav2 feedback
                    if hasattr(self.nav_feedback, 'navigation_path'):
                        self.remaining_poses = len(self.nav_feedback.navigation_path.poses)
                
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Cleaning canceled"
                    return result

                if self.stop_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = "Cleaning stopped"
                    return result

                # Handle pause - stay in pause loop without breaking
                if self.pause_requested:
                    self.get_logger().info("Pausing navigation...")
                    self.get_logger().info(f"Pause in segment {idx}")
                    self.planner.cancelTask()
                    self.save_path()
                    
                    # Wait for resume signal - allow the node to process callbacks
                    while self.pause_requested:
                        # self.get_logger().info('Paused')
                        # Process incoming callbacks (service calls, etc.) while paused
                        rclpy.spin_once(self, timeout_sec=0.5)
                    
                    # Resume: restart navigation on current segment
                    self.get_logger().info(f"Resuming segment {idx}")
                    path = ut.to_ros_path(self.current_segment)
                    self.planner.goThroughPoses(path)
                    continue  # Continue checking feedback for resumed navigation

                if self.planner.isTaskComplete():
                    self.get_logger().info('------ TASK COMPLETE --------')
                    break
            
            # Publish feedback
            feedback = NavigateCoverage.Feedback()

            feedback.current_segment = idx + 1
            feedback.total_segments = total_segments

            feedback.completion_percentage = (
                float(idx + 1) / float(total_segments)
            ) * 100.0

            goal_handle.publish_feedback(feedback)

            if verbose:
                self.get_logger().info(f"Completed segment {idx+1}")

        # Finished
        goal_handle.succeed()

        result.success = True
        result.message = "Cleaning completed"

        return result

    #########################
    # Status Manager Server #
    #########################

    def status_callback(self, request, response):

        requested_status = request.command

        if requested_status == request.PAUSE:
            self.pause_requested = True
            self.get_logger().info('''
                                    ###########################
                                    # Pausing Navigation Task #
                                    ###########################
                                    ''')
            # Don't cancel task here - let execute_callback handle it
            # This preserves the ability to continue from where we paused

            response.succeeded = True

        elif requested_status == request.RESUME:
            self.pause_requested = False
            self.get_logger().info('''
                    ############################
                    # Resuming Navigation Task #
                    ############################
                    ''')
            response.succeeded = True

        elif requested_status == request.STOP:
            self.stop_requested = True
            response.succeeded = True

        else:
            response.succeeded = False

        response.remaining_poses = self.remaining_poses

        return response

    def save_path(self):
        """
            saves remaining coverage path
        """

        if self.remaining_poses <= 0:
            self.get_logger().warn(
                f"No remaining pose information available (remaining_poses={self.remaining_poses}). "
                "Will resume from current segment start."
            )
            return

        if self.current_segment is None:
            self.get_logger().warn("No current segment to save")
            return
        path = self.current_segment.copy()

        # Try to compute remaining path from current robot pose for accuracy
        remaining_path = []
        robot_pos = self.get_robot_position()

        if robot_pos is not None:
            try:
                arr = np.array(path)
                dists = np.linalg.norm(arr - robot_pos, axis=1)
                idx = int(np.argmin(dists))

                # Resume from the next pose after the closest one
                if idx < len(path) - 1:
                    remaining_path = path[idx + 1 :]
                else:
                    remaining_path = []

            except Exception as e:
                self.get_logger().warn(
                    f"Could not compute remaining path from robot pose: {e}."
                )

        # Fallback: use remaining_poses if we couldn't compute from robot pose
        if (remaining_path is None or len(remaining_path) == 0) and self.remaining_poses > 0:
            number_remaining = min(self.remaining_poses, len(path))
            remaining_path = path[-number_remaining:]

        if not remaining_path:
            self.get_logger().warn(
                f"No remaining pose information available after computation (remaining_poses={self.remaining_poses}). Will resume from current segment start."
            )
            return

        self.get_logger().info(
            f"Saving remaining path: {len(remaining_path)} poses remaining out of {len(path)} total"
        )

        # Insert the partial segment back into the queue to resume from there
        self.goal_paths.insert(0, remaining_path)


def main():
    rclpy.init()
    node = LabCleanActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

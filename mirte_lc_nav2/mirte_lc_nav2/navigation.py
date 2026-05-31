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
from nav2_msgs.action import NavigateThroughPoses
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

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_callback,
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            "/navigate_through_poses",
        )

        self.server = ActionServer(
            self,
            NavigateCoverage,
            "/labclean_navigator/coverage",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    ###############################
    # Costmap Subscriber Callback #
    ###############################

    def costmap_callback(self, msg):
        width = msg.info.width
        height = msg.info.height

        self.map = np.array(msg.data, dtype=np.int16).reshape((height, width))

    def goal_callback(self, goal_request):
        self.get_logger().info("Received cleaning goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

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

    #######################
    # Coverage Navigation #
    #######################

    async def execute_callback(self, goal_handle):
        self.segment_idx = 0
        self.current_goal_handle = goal_handle
        planner_type = goal_handle.request.planner_type
        verbose = goal_handle.request.verbose
        self.verbose = verbose

        result = NavigateCoverage.Result()

        # wait for map
        while self.map is None:
            self.get_logger().info("Waiting for costmap...")
            time.sleep(1.0)

        start = self.get_robot_position()

        if start is None:
            result.success = False
            result.message = "Could not determine robot pose"
            goal_handle.abort()
            return result

        planner_cls = self.lookup_planner(planner_type)

        planner = planner_cls(self)

        self.get_logger().info(f"Planning with {planner_type}")

        planner.plan(self.map, start)
        self.goal_paths = planner.paths.copy()
        if planner.paths is None or len(planner.paths) == 0:
            result.success = False
            result.message = "Planner produced no paths"
            goal_handle.abort()
            return result

        total_segments = len(planner.paths)

        self.get_logger().info("waiting on navigatethroughposes server")
        self.nav_client.wait_for_server()

        self.goal_paths = planner.paths

        # Execute paths sequentially
        # for idx, segment in enumerate(self.goal_paths):
        while len(self.goal_paths) > 0:
            segment = self.goal_paths.pop(0)
            self.segment_idx += 1
            idx = self.segment_idx

            self.current_segment = segment

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result.success = False
                result.message = "Cleaning canceled"

                return result

            path = ut.to_ros_path(segment)

            nav_goal = NavigateThroughPoses.Goal()
            nav_goal.poses = path.poses

            self.get_logger().info(f"Executing segment {idx+1}/{total_segments}")

            send_goal_future = self.nav_client.send_goal_async(
                nav_goal, feedback_callback=self.nav_feedback_callback
            )

            self.nav_goal_handle = await send_goal_future

            if self.pause_requested:
                self.get_logger().info("Pausing navigation...")
                self.get_logger().info(f"Pause in segment {idx}")
                
                while self.pause_requested:
                    self.get_logger().info('Paused')
                    time.sleep(0.1)

            else:
                if self.stop_requested:
                    await self.nav_goal_handle.cancel_all_goals_async()
                    goal_handle.abort()

                    result.success = False
                    result.message = "Cleaning stopped"

                    return result

                if not self.nav_goal_handle.accepted:
                    result.success = False
                    result.message = "Nav2 rejected goal"

                    goal_handle.abort()
                    return result

                result_future = self.nav_goal_handle.get_result_async()

                # Wait for completion
                nav_result = await result_future

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

    def nav_feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.distance_remaining = feedback.distance_remaining
        self.remaining_poses = feedback.number_of_poses_remaining

        # if self.verbose:
        #     self.get_logger().info(
        #         f"Distance Remaining: {self.distance_remaining} | Poses Remaining: {self.remaining_poses}"
        #     )

    #########################
    # Status Manager Server #
    #########################

    def status_callback(self, request, response):

        requested_status = request.command

        match requested_status:

            case request.PAUSE:
                self.pause_requested = True

                if self.nav_goal_handle is not None:
                    self.get_logger().info('Cancelling goal in pause')
                    self.nav_goal_handle.cancel_goal_async()
                    
                self.save_path()

                response.succeeded = True

            case request.RESUME:
                self.pause_requested = False
                response.succeeded = True

            case request.STOP:
                self.stop_requested = True
                response.succeeded = True

            case _:
                response.succeeded = False

        response.remaining_poses = self.remaining_poses

        return response

    def save_path(self):
        """
            saves remaining coverage path
        """

        if self.remaining_poses <= 0:
            self.get_logger().warn("No remaining pose information available")
            return

        path = self.current_segment.copy()
        number_remaining = self.remaining_poses
        remaining_path = path[-number_remaining:]
        self.get_logger().info(
            f"for debugging purposes: nav2 remaining path: {number_remaining} requested path: {len(path)} "
        )
        self.goal_paths.insert(0, remaining_path)


def main():
    rclpy.init()
    node = LabCleanActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

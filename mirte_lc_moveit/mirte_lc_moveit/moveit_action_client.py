import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose

from mirte_lc_msgs.action import MoveToPosition


class MoveToPositionActionClient(Node):

    def __init__(self):
        super().__init__('move_to_position_action_client')
        self._action_client = ActionClient(self, MoveToPosition, 'move_to_position')
        self.result_future = None

    def send_goal(self, target_pose):
        goal_msg = MoveToPosition.Goal()
        goal_msg.target_pose = target_pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.result_future = None
            return

        self.get_logger().info('Goal accepted')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    
    def get_result_callback(self, future):
        result = future.result().success
        self.get_logger().info('Success: ' + str(result))
        # Do not shutdown here, let the loop continue

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: ' + str(feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = MoveToPositionActionClient()

    while True:
        try:
            user_input = input("Enter target position (x y z) or 'q' to quit: ").strip()
            if user_input.lower() == 'q':
                break
            parts = user_input.split()
            if len(parts) != 3:
                print("Please enter exactly 3 numbers for x y z.")
                continue
            x, y, z = map(float, parts)
        except ValueError:
            print("Invalid input. Please enter numbers for x y z.")
            continue

        goalpost = Pose()
        goalpost.position.x = x
        goalpost.position.y = y
        goalpost.position.z = z
        goalpost.orientation.x = 0.7
        goalpost.orientation.y = 0.0
        goalpost.orientation.z = 0.7
        goalpost.orientation.w = 0.0

        future = action_client.send_goal(goalpost)

        # Wait for goal response
        rclpy.spin_until_future_complete(action_client, future)

        if action_client.result_future is not None:
            # Wait for result
            rclpy.spin_until_future_complete(action_client, action_client.result_future)
            action_client.result_future = None  # Reset for next goal

    rclpy.shutdown()


if __name__ == '__main__':
    main()

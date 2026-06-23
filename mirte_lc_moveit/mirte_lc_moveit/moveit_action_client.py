import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose

from mirte_lc_msgs.action import MoveToPosition


class MoveToPositionActionClient(Node):

    def __init__(self):
        super().__init__('move_to_position_action_client')

        self._action_client = ActionClient(
            self,
            MoveToPosition,
            'move_to_position'
        )

        self.result_future = None

    def send_goal(
        self,
        arm_pose=None,
        arm_named_target="",
        gripper_named_target="",
        gripper_joint_target=None,
        wrist_joint_target=None,
        lock_wrist=False
    ):

        goal_msg = MoveToPosition.Goal()

        # -------------------------------------------------
        # Arm pose target
        # -------------------------------------------------
        if arm_pose is not None:
            goal_msg.mirte_arm_target_pose = arm_pose

        # -------------------------------------------------
        # Arm named target
        # -------------------------------------------------
        goal_msg.mirte_arm_named_target = arm_named_target

        # -------------------------------------------------
        # Gripper named target
        # -------------------------------------------------
        goal_msg.mirte_gripper_named_target = gripper_named_target

        # -------------------------------------------------
        # Gripper joint target
        # -------------------------------------------------
        if gripper_joint_target is not None:
            goal_msg.mirte_gripper_joint_target = gripper_joint_target

        # -------------------------------------------------
        # Wrist joint target
        # -------------------------------------------------
        if wrist_joint_target is not None:
            goal_msg.mirte_wrist_joint_target = wrist_joint_target

        # -------------------------------------------------
        # Lock wrist joint
        # ------------------------------------------------
        goal_msg.lock_wrist = lock_wrist

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

        return self._send_goal_future

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.result_future = None
            return

        self.get_logger().info('Goal accepted')

        self.result_future = goal_handle.get_result_async()

        self.result_future.add_done_callback(
            self.get_result_callback
        )

    def get_result_callback(self, future):

        result = future.result().result

        self.get_logger().info(
            'Success: ' + str(result.success)
        )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            'Feedback: ' + feedback.state
        )


def create_pose(x, y, z):

    pose = Pose()

    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = 0.7
    pose.orientation.y = 0.0
    pose.orientation.z = 0.7
    pose.orientation.w = 0.0

    return pose


def print_help():

    print("\nAvailable commands:")
    print("--------------------------------------------------")
    print("ARM")
    print("  arm_pose x y z [true|false]  (lock wrist joint)")
    print("  arm_name TARGET")
    print("")
    print("GRIPPER")
    print("  gripper_name TARGET")
    print("  gripper_joint VALUE")
    print("")
    print("WRIST")
    print("  wrist_joint VALUE")
    print("")
    print("GENERAL")
    print("  help")
    print("  q")
    print("--------------------------------------------------\n")


def main(args=None):

    rclpy.init(args=args)

    action_client = MoveToPositionActionClient()

    print_help()

    while True:

        user_input = input("> ").strip()

        if user_input.lower() == 'q':
            break

        if user_input.lower() == 'help':
            print_help()
            continue

        parts = user_input.split()

        if len(parts) == 0:
            continue

        try:

            command = parts[0].lower()

            # =================================================
            # ARM POSE
            # Example:
            # arm_pose 0.2 0.1 0.3
            # =================================================
            if command == "arm_pose":

                if len(parts) not in [4, 5]:
                    print("Usage: arm_pose x y z [true|false]")
                    continue

                x, y, z = map(float, parts[1:4])

                lock_wrist = False

                if len(parts) == 5:

                    lock_str = parts[4].lower()

                    if lock_str in ["true", "t", "1", "yes"]:
                        lock_wrist = True

                    elif lock_str in ["false", "f", "0", "no"]:
                        lock_wrist = False
                    
                    else:
                        continue

                pose = create_pose(x, y, z)

                future = action_client.send_goal(
                    arm_pose=pose,
                    lock_wrist=lock_wrist
                )

            # =================================================
            # ARM NAMED TARGET
            # Example:
            # arm_name home
            # =================================================
            elif command == "arm_name":

                if len(parts) < 2:
                    print("Usage: arm_name TARGET")
                    continue

                target = " ".join(parts[1:])

                future = action_client.send_goal(
                    arm_named_target=target
                )

            # =================================================
            # GRIPPER NAMED TARGET
            # Example:
            # gripper_name open
            # =================================================
            elif command == "gripper_name":

                if len(parts) < 2:
                    print("Usage: gripper_name TARGET")
                    continue

                target = " ".join(parts[1:])

                future = action_client.send_goal(
                    gripper_named_target=target
                )

            # =================================================
            # GRIPPER JOINT TARGET
            # Example:
            # gripper_joint 0.4
            # =================================================
            elif command == "gripper_joint":

                if len(parts) != 2:
                    print("Usage: gripper_joint VALUE")
                    continue

                joint_value = float(parts[1])

                future = action_client.send_goal(
                    gripper_joint_target=joint_value
                )

            # =================================================
            # WRIST JOINT TARGET
            # Example:
            # wrist_joint 1.57
            # =================================================
            elif command == "wrist_joint":

                if len(parts) != 2:
                    print("Usage: wrist_joint VALUE")
                    continue

                joint_value = float(parts[1])

                future = action_client.send_goal(
                    wrist_joint_target=joint_value
                )

            else:
                print("Unknown command.")
                print_help()
                continue

            # Wait for goal response
            rclpy.spin_until_future_complete(
                action_client,
                future
            )

            # Wait for result
            if action_client.result_future is not None:

                rclpy.spin_until_future_complete(
                    action_client,
                    action_client.result_future
                )

                action_client.result_future = None

        except ValueError:
            print("Invalid numeric input.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
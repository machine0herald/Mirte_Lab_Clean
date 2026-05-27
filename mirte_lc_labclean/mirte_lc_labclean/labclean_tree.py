'''
Lab Cleanup Behaviour Tree
See the lab cleanup tree diagram for the structure of the tree.
Understanding the tree structure is more important than the code itself,
so please refer to the diagram while reading through the code.
'''

import operator
import sys

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy

from . import behaviours
from . import mock

def create_root() -> py_trees.behaviour.Behaviour:
    """
    Lab Cleanup Behaviour Tree
    
    Subscribed Topics:
    - /explore/status (std_msgs/Bool): Triggers after exploration is complete to start the lab cleaning task
    - /dashboard/cancel (std_msgs/Bool): Trigger to cancel the current task and return home
    - /battery/state (sensor_msgs/BatteryState): Battery state to monitor for
    
    Published Topics:
    - /led_strip (std_msgs/ColorRGBA): To control the LED strip for visual feedback
    - /navigate (MoveBase action): To send navigation goals for moving out and returning home
    - /rotate (Rotate action): To send rotation goals for scanning
    - /dock (Dock action): To send docking/undocking commands

    Returns:
        the root of the tree
    """
    
    ##################################################################################
    # Root node with parallel policy to run topic subscribers and tasks concurrently #
    ##################################################################################
    root = py_trees.composites.Parallel(
        name="Lab Cleanup Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    
    ##################################
    # Branch 1: Topics to Blackboard #
    ##################################
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    
    # 1.1: Exploration status to Blackboard #
    exploration2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Exploration2BB",
        topic_name="/explore/status",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_scan_button"
    )
    
    # 1.2: Cancel button to Blackboard #
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_cancel_button"
    )
    
    # 1.3: Battery state to Blackboard #
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    
    ###################
    # Branch 2: Tasks #
    ###################
    tasks = py_trees.composites.Selector(name="Tasks", memory=False)

    # 2.1: Battery Emergency Task #
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )

    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning
    
    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )
    #---------------------------#
    
    # 2.2: Scan Task #
    scan = py_trees.composites.Sequence(name="Scan", memory=True)
    
    # 2.2.1: Is Scan Requested? #
    is_scan_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Scan?",
        check=py_trees.common.ComparisonExpression(
            variable="event_scan_button",
            value=True,
            operator=operator.eq
        )
    )
    
    # 2.2.2: Scan or Die Selector #
    scan_or_die = py_trees.composites.Selector(name="Scan or Die", memory=False)
    
    # 2.2.2.1: Die Sequence #
    die = py_trees.composites.Sequence(name="Die", memory=True)
    
    # 2.2.2.1.1: Failed Notification Parallel (Flash Red + Pause) #
    failed_notification = py_trees.composites.Parallel(
        name="Notification",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    failed_flash_green = behaviours.FlashLedStrip(name="Flash Red", colour="red")
    failed_pause = py_trees.timers.Timer("Pause", duration=3.0)
    
    # 2.2.2.1.2: Set Result to Blackboard #
    result_failed_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'failed'",
        variable_name='scan_result',
        variable_value='failed',
        overwrite=True
    )
    
    # 2.2.2.2: Ere we Go Sequence #
    ere_we_go = py_trees.composites.Sequence(name="Ere we Go", memory=True)
    
    # 2.2.2.2.1: Undock action client #
    undock = py_trees_ros.actions.ActionClient(
        name="UnDock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=False),
        generate_feedback_message=lambda msg: "undocking"
    )
    
    # 2.2.2.2.2: Scan or be Cancelled Selector #
    scan_or_be_cancelled = py_trees.composites.Selector(name="Scan or Be Cancelled", memory=False)
    
    # 2.2.2.2.2.1: Cancelling Sequence #
    cancelling = py_trees.composites.Sequence(name="Cancelling?", memory=True)
    is_cancel_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Cancel?",
        check=py_trees.common.ComparisonExpression(
            variable="event_cancel_button",
            value=True,
            operator=operator.eq
        )
    )
    move_home_after_cancel = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_cancelled_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'cancelled'",
        variable_name='scan_result',
        variable_value='cancelled',
        overwrite=True
    )
    
    # 2.2.2.2.2.2: Move Out and Scan Sequence #
    move_out_and_scan = py_trees.composites.Sequence(name="Move Out and Scan", memory=True)
    move_base = py_trees_ros.actions.ActionClient(
        name="Move Out",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving out"
    )
    scanning = py_trees.composites.Parallel(
        name="Scanning",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    scan_context_switch = behaviours.ScanContext("Context Switch")
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_type=py_trees_actions.Rotate,
        action_name="rotate",
        action_goal=py_trees_actions.Rotate.Goal(),
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
    )
    scan_flash_blue = behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'succeeded'",
        variable_name='scan_result',
        variable_value='succeeded',
        overwrite=True
    )
    
    # 2.2.2.2.3: Dock action client #
    dock = py_trees_ros.actions.ActionClient(
        name="Dock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=True),  # noqa
        generate_feedback_message=lambda msg: "docking"
    )
    
    # 2.2.2.2.4: Celebrate Parallel (Flash Green + Pause) #
    celebrate = py_trees.composites.Parallel(
        name="Celebrate",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    celebrate_flash_green = behaviours.FlashLedStrip(name="Flash Green", colour="green")
    celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)

    class SendResult(py_trees.behaviour.Behaviour):

        def __init__(self, name: str):
            super().__init__(name="Send Result")
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="scan_result",
                access=py_trees.common.Access.READ
            )

        def update(self):
            print(console.green +
                  "********** Result: {} **********".format(self.blackboard.scan_result) +
                  console.reset
                  )
            return py_trees.common.Status.SUCCESS

    send_result = SendResult(name="Send Result")

    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, scan, idle])
    scan.add_children([is_scan_requested, scan_or_die, send_result])
    scan_or_die.add_children([ere_we_go, die])
    die.add_children([failed_notification, result_failed_to_bb])
    failed_notification.add_children([failed_flash_green, failed_pause])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children([is_cancel_requested, move_home_after_cancel, result_cancelled_to_bb])
    move_out_and_scan.add_children([move_base, scanning, move_home_after_scan, result_succeeded_to_bb])
    scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
    celebrate.add_children([celebrate_flash_green, celebrate_pause])
    return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
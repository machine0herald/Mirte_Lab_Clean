"""LabClean behaviour tree entrypoint for ROS2.

This module builds and returns the root of the LabClean behaviour tree used by
ROS2 and py_trees. It wires topic subscriptions, blackboard variables, and task
branches for exploration, object handling, and coverage navigation.

Examples:
    $ py-trees-render -b mirte_lc_labclean.labclean_tree.create_root
    $ py-trees-blackboard-watcher --list
    $ py-trees-blackboard-watcher /battery.percentage
    $ ros2 run mirte_lc_labclean labclean_tree
    
"""
import operator
import sys

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy

from explore_lite_msgs.msg import ExploreStatus
from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray
from . import behaviours

import launch
import launch_ros

########
# Tree #
########
def create_root() -> py_trees.behaviour.Behaviour:
    """Build the LabClean behaviour tree root.

    The returned root node executes blackboard updates, task selection, and
    safety monitoring for the lab cleaning application.

    Returns:
        py_trees.behaviour.Behaviour: The root behaviour node for the tree.
    """

    ##################################################################################
    # Root node with parallel policy to run topic subscribers and tasks concurrently #
    ##################################################################################

    root = py_trees.composites.Parallel(
        name="Lab Cleanup Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    
    ##################################
    # Branch 1: Topics to Blackboard #
    ##################################
    # oneshot = py_trees.decorators.OneShot(name="init_oneshot", child=initiate_bb, policy=ON_SUCCESSFUL_COMPLETION)
    init_cloud = py_trees.behaviours.SetBlackboardVariable(
        name="Init Cloud Objects",
        variable_name="cloud_objects_detected",
        variable_value=[],
        overwrite=False,
    )

    init_planar = py_trees.behaviours.SetBlackboardVariable(
        name="Init Planar Objects",
        variable_name="planar_objects_detected",
        variable_value=[],
        overwrite=False,
    )

    init_explore = py_trees.behaviours.SetBlackboardVariable(
        name="Init Explore Status",
        variable_name="explore_status",
        variable_value="",
        overwrite=False,
    )

    # 1.1: Exploration status to Blackboard #
    exploration2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Exploration2BB",
        topic_name="/explore/status",
        topic_type=ExploreStatus,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={"explore_status": "status"}, # store msg.status in blackboard.explore_status
    )

    # 1.2: Cancel button to Blackboard #
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="cancel_button",
    )

    # 1.3: Start button to Blackboard #
    start2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Start2BB",
        topic_name="/dashboard/start",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="start_button",
    )

    # 1.4: Battery state to Blackboard #
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/io/power/power_watcher",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=0.25
    )

    # 1.5: Detected bounding boxes topic to Blackboard #
    detectedcloud2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Detectedcloud2BB",
        topic_name="/perception/depth/detected_objects",
        topic_type=DetectedObjectArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={
            "cloud_objects_detected": "objects",
            "num_cloud_objects_detected": "length"
        },
    )
    
    topics2bb = py_trees.composites.Sequence(
        name="Topics2BB",
        memory=True,
    )

    subscriber_parallel = py_trees.composites.Parallel(
        name="Subscribers",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        ),
    )
    

    ################################
    # Branch 2: Discovery Coverage #
    ################################
    explore_or_cover = py_trees.composites.Selector(name="Explore or Cover", memory=True)
    
    cover_and_discover =  py_trees.composites.Parallel(
        name="Cover and Discover",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        ),
    )

    cover = behaviours.CoverageTask(name="CoverageTask", planner="skeleton")
    
    def check_explored(blackboard: py_trees.blackboard.Blackboard,):
        try:
            explored = (blackboard.explore_status=="exploration_complete")
        except KeyError:
            explored = False
        return explored

    explored_check = py_trees.decorators.EternalGuard(
        name="Explored?",
        condition=check_explored,
        blackboard_keys={"explore_status"},
        child=cover_and_discover,
    )

    tasks = py_trees.composites.Selector(name="Tasks", memory=True)
    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
            return blackboard.battery_low_warning

    # 2.1: Battery Emergency Task #
    dock = py_trees.composites.Parallel(
        name="Dock",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=dock,
    )
    flash_red = behaviours.FlashLedStrip(name="Flash Red", colour=[1.0, 0.0, 0.0])
    dock_action = behaviours.NavigateToPosition(name="Dock Action", target_position=[0, 0])

    # --------------------------- #

    # 2.2: Detection and handling Task #
    def check_detected_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        try:
            detected = (len(blackboard.cloud_objects_detected) > 0)
        except KeyError:
            detected = False
        return detected


    #   2.2.2: handle Sequence   #
    approach_and_handle = py_trees.composites.Sequence(
        name="Approach and Handle",
        memory=True,
    )
    detection_check = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Objects?",
        check=py_trees.common.ComparisonExpression(
            variable="cloud_objects_detected",
            value=[],
            operator=operator.ne,   # ne = not equal, i.e. list is not empty
        )
    )
    pause_coverage = behaviours.SetCoverageStatus(name="Pause Coverage", requested_status='pause')
    resume_coverage = behaviours.SetCoverageStatus(name="Resume Coverage", requested_status='resume')
    handle = py_trees.composites.Sequence(name="handle", memory=True)
    flash_green = behaviours.FlashLedStrip(name="Flash Green", colour=[0.0, 1.0, 0.0])
    pick_up = py_trees.composites.Sequence(name="Pick Up", memory=True)

    approach = behaviours.NavigateToPosition(name="Approach", blackboard_key="cloud_objects_detected")
    deploy_arm = behaviours.MoveArm(name="Deploy Arm", predefined_pose='standby')
    pick_or_skip = py_trees.composites.Selector(name="Pick or Skip", memory=True)

    def check_planar_detected_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        try:
            detected = (len(blackboard.planar_objects_detected) > 0)
        except KeyError:
            detected = False
        return detected
    
    
    sort = py_trees.composites.Sequence(name="Sort", memory=True)

    get_planar = behaviours.GetPlanarObjects(name="Planar_Detected?")
    
    retry_planar = py_trees.decorators.Retry(
        name="Retry Planar",
        child=get_planar,
        num_failures=5  # try up to 5 times before actually failing
    )
    place = behaviours.MoveArm(name="Place", predefined_pose="place_right")

    # --------------------------- #


    # # Die Sequence #
    # die = py_trees.composites.Sequence(name="Die", memory=True)

    # # Failed Notification Parallel (Flash Red + Pause) #
    # failed_notification = py_trees.composites.Parallel(
    #     name="Notification", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    # )
    # failed_flash_green = behaviours.FlashLedStrip(name="Flash Red", colour="red")
    # failed_pause = py_trees.timers.Timer("Pause", duration=3.0)

    # result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
    #     name="Result2BB\n'succeeded'",
    #     variable_name="result",
    #     variable_value="succeeded",
    #     overwrite=True,
    # )

    # # Celebrate Parallel (Flash Green + Pause) #
    # celebrate = py_trees.composites.Parallel(
    #     name="Celebrate", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    # )
    # celebrate_flash_green = behaviours.FlashLedStrip(name="Flash Green", colour="green")
    # celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)

    class SendResult(py_trees.behaviour.Behaviour):

        def __init__(self, name: str):
            super().__init__(name="Send Result")
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="result", access=py_trees.common.Access.READ
            )

        def update(self):
            print(
                console.green
                + "********** Result: {} **********".format(self.blackboard.result)
                + console.reset
            )
            return py_trees.common.Status.SUCCESS

    # send_result = SendResult(name="Send Result")

    # Fallback task
    idle_tasks = py_trees.behaviours.Success(name="Idle")
    idle_pick = py_trees.behaviours.Success(name="Idle")
    idle_explore = py_trees.behaviours.Success(name="Idle")

    root.add_children([topics2bb, explore_or_cover])

    # 1. Topics to Blackboard branch
    topics2bb.add_children([
        init_cloud,
        init_planar,
        init_explore,
        subscriber_parallel,
    ])

    subscriber_parallel.add_children([
        exploration2bb,
        cancel2bb,
        start2bb,
        battery2bb,
        detectedcloud2bb,
    ])

    cover_and_discover.add_children([tasks, cover])
    # 2. Tasks branch
    tasks.add_children([approach_and_handle, idle_tasks])

    # 2.1: Battery Emergency dock
    dock.add_children([flash_red, dock_action])

    # 2.2: Detection and handling
    approach_and_handle.add_children([detection_check, pause_coverage, handle, resume_coverage])
    handle.add_children([flash_green, pick_up])
    pick_up.add_children([approach, deploy_arm, pick_or_skip])
    pick_or_skip.add_children([sort, idle_pick])
    
    # 3. Explore or Cover branch
    explore_or_cover.add_children([battery_emergency, explored_check, idle_explore])
    sort.add_children([retry_planar, place])
    return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, 
                                            unicode_tree_debug=False)
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red
            + "failed to setup the tree, aborting [{}]".format(str(e))
            + console.reset
        )
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=500.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

"""
Lab Cleanup Behaviour Tree
See the lab cleanup tree diagram for the structure of the tree.
Understanding the tree structure is more important than the code itself,
so please refer to the diagram while reading through the code.

$ py-trees-render -b py_trees_ros_tutorials.one_data_gathering.tutorial_create_root
$ py-trees-blackboard-watcher --list
$ py-trees-blackboard-watcher /battery.percentage
$ sudo apt install ros-humble-py-trees-ros-viewer
$ py-trees-tree-viewer
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

LOW_PERCENTAGE_THRESHOLD = 0.2

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
    def start_pressed(blackboard):
        try:
            return blackboard.start_button
        except KeyError:
            return False

    lab_cleanup = py_trees.composites.Parallel(
        name="Lab Cleanup Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    root = py_trees.decorators.EternalGuard(
        name="Start?",
        condition=start_pressed,
        blackboard_keys={"start_button"},
        child=lab_cleanup,
    )



    ##################################
    # Branch 1: Topics to Blackboard #
    ##################################
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)

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
    )

    # 1.5: Detected bounding boxes topic to Blackboard #
    detectedcloud2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Detectedcloud2BB",
        topic_name="/object_bounding_boxes",
        topic_type=DetectedObjectArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={
            "cloud_objects_detected": "objects",
            "num_cloud_objects_detected": "length"
        },
    )

    # 1.6: Detected object classes topic to Blackboard #
    detectedplanar2bb = py_trees_ros.subscribers.ToBlackboard(
        name="DetectedClasses2BB",
        topic_name="/object_bounding_boxes/planar",
        topic_type=DetectedObjectArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={
            "planar_objects_detected": "objects",
            "num_planar_objects_detected": "length",
        },
    )

    ###################
    # Branch 2: Tasks #
    ###################
    tasks = py_trees.composites.Selector(name="Tasks", memory=True)

    def check_battery_low_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        return blackboard.battery_percentage is not None and blackboard.battery_percentage < LOW_PERCENTAGE_THRESHOLD

    # 2.1: Battery Emergency Task #
    dock = py_trees.composites.Parallel(
        name="Dock",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_percentage"},
        child=dock,
    )
    flash_red = behaviours.FlashLedStrip(name="Flash Red", colour=[255, 0, 0])
    dock_action = behaviours.NavigateToPosition(name="Dock Action", target_position=[0, 0])

    # --------------------------- #

    # 2.2: Detection and handling Task #
    def check_detected_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        return blackboard.cloud_objects_detected and len(blackboard.cloud_objects_detected) > 0


    #   2.2.2: handle Sequence   #
    approach_and_handle = py_trees.composites.Sequence(
        name="Approach and Handle",
        memory=True,
    )
    detection_check = py_trees.decorators.EternalGuard(
        name="Detected?",
        condition=check_detected_on_blackboard,
        blackboard_keys={"cloud_objects_detected"},
        child=approach_and_handle,
    )
    pause_coverage = behaviours.SetCoverageStatus(name="Pause Coverage", requested_status='pause')
    resume_coverage = behaviours.SetCoverageStatus(name="Resume Coverage", requested_status='resume')
    handle = py_trees.composites.Sequence(name="handle", memory=True)
    flash_green = behaviours.FlashLedStrip(name="Flash Green", colour=[0, 255, 0])
    pick_up = py_trees.composites.Sequence(name="Pick Up", memory=True)

    # object_position =...

    approach = behaviours.NavigateToPosition(name="Approach", blackboard_key="cloud_objects_detected[0].position")
    deploy_arm = behaviours.MoveArm(name="Deploy Arm", target_position=[0.36, 0.18, 0.0])
    pick_or_skip = py_trees.composites.Selector(name="Pick or Skip", memory=True)

    def check_planar_detected_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        return blackboard.planar_objects_detected and len(blackboard.planar_objects_detected) > 0
    
    sort = py_trees.composites.Sequence(name="Sort", memory=True)

    detection_check_planar = py_trees.decorators.EternalGuard(
        name="planar_Detected?",
        condition=check_planar_detected_on_blackboard,
        blackboard_keys={"planar_objects_detected"},
        child=sort,
    )
    
    pick_place_electronic

    # position_planar = ... 

    # --------------------------- #

    ###########################
    # Branch 3: Coverage task #
    ###########################
    explore_or_cover = py_trees.composites.Selector(name="Explore or Cover", memory=True)

    cover = behaviours.CoverageTask(name="CoverageTask", planner="skeleton")
    explored_check = py_trees.decorators.EternalGuard(
        name="Explored?",
        condition=lambda blackboard: blackboard.explore_status=="exploration_complete",
        blackboard_keys={"explore_status"},
        child=cover,
    )

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

    send_result = SendResult(name="Send Result")

    # Fallback task
    idle_tasks = py_trees.behaviours.Success(name="Idle")
    idle_pick = py_trees.behaviours.Success(name="Idle")
    idle_explore = py_trees.behaviours.Success(name="Idle")

    lab_cleanup.add_children([topics2bb, tasks, explore_or_cover])

    # 1. Topics to Blackboard branch
    topics2bb.add_children([
        exploration2bb,
        cancel2bb,
        start2bb,
        battery2bb,
        detectedcloud2bb,
        detectedplanar2bb,
    ])

    # 2. Tasks branch
    tasks.add_children([battery_emergency, detection_check, idle_tasks])

    # 2.1: Battery Emergency dock
    dock.add_children([flash_red, dock_action])

    # 2.2: Detection and handling
    approach_and_handle.add_children([pause_coverage, handle, resume_coverage])
    handle.add_children([flash_green, pick_up])
    pick_up.add_children([approach, deploy_arm, pick_or_skip])
    pick_or_skip.add_children([detection_check_planar, idle_pick])

    # 3. Explore or Cover branch
    explore_or_cover.add_children([explored_check, idle_explore])
    return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
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

    tree.tick_tock(period_ms=100.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

"""LabClean behaviour tree entrypoint for ROS2.

Handy cli commands:
    $ py-trees-render -b mirte_lc_labclean.labclean_tree.create_root
    $ py-trees-blackboard-watcher --list
    $ ros2 run mirte_lc_labclean labclean_tree
    $ py-trees-tree-viewer --no-sandbox
"""
import operator
import sys

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy

from explore_lite_msgs.msg import ExploreStatus
from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray
from . import behaviours


def create_root() -> py_trees.behaviour.Behaviour:
    """Build the LabClean behaviour tree root.

    Tree structure:
        [Parallel] Lab Cleanup Root
        ├── [Sequence] Topics2BB
        │   ├── Init Cloud Objects
        │   ├── Init Planar Objects
        │   ├── Init Explore Status
        │   └── [Parallel] Subscribers
        │       ├── Exploration2BB
        │       ├── Cancel2BB
        │       ├── Start2BB
        │       ├── Battery2BB
        │       ├── Detectedcloud2BB
        │       └── DetectedClasses2BB
        ├── [Selector] Explore or Cover
        │   ├── [EternalGuard] Explored?
        │   │   └── CoverageTask
        │   └── Idle
        └── [Selector] Tasks
            ├── [EternalGuard] Battery Low?
            │   └── [Parallel] Dock
            │       ├── Flash Red
            │       └── Dock Action
            ├── [Sequence] Approach and Handle
            │   ├── Objects?
            │   ├── Pause Coverage
            │   ├── [Sequence] Handle
            │   │   ├── Flash Green
            │   │   └── [Sequence] Pick Up
            │   │       ├── Approach
            │   │       └── [Selector] Pick or Skip
            │   │           ├── [Sequence] Sort and Pick
            │   │           │   ├── [Retry] Retry Planar
            │   │           │   │   └── Planar_Detected?
            │   │           │   └── PickObject
            │   │           └── Idle
            │   └── Resume Coverage
            └── Flash Orange

    Returns:
        py_trees.behaviour.Behaviour: Root of the behaviour tree.
    """

    # -----------------------------------------------------------------------
    # Root
    # -----------------------------------------------------------------------

    root = py_trees.composites.Parallel(
        name="Lab Cleanup Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # -----------------------------------------------------------------------
    # Branch 1: Topics to Blackboard
    # -----------------------------------------------------------------------

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

    exploration2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Exploration2BB",
        topic_name="/explore/status",
        topic_type=ExploreStatus,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={"explore_status": "status"},
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="cancel_button",
    )
    start2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Start2BB",
        topic_name="/dashboard/start",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="start_button",
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/io/power/power_watcher",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=0.20,
    )
    detectedcloud2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Detectedcloud2BB",
        topic_name="/perception/depth/detected_objects",
        topic_type=DetectedObjectArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={
            "cloud_objects_detected": "objects",
            "num_cloud_objects_detected": "length",
        },
    )
    detectedplanar2bb = py_trees_ros.subscribers.ToBlackboard(
        name="DetectedClasses2BB",
        topic_name="/perception/planar/detected_objects",
        topic_type=DetectedObjectArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={
            "planar_objects_detected": "objects",
            "num_planar_objects_detected": "length",
        },
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    subscriber_parallel = py_trees.composites.Parallel(
        name="Subscribers",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # -----------------------------------------------------------------------
    # Branch 2: Explore or Cover
    # -----------------------------------------------------------------------

    cover = behaviours.CoverageTask(name="CoverageTask", planner="skeleton")

    def check_explored(blackboard: py_trees.blackboard.Blackboard) -> bool:
        try:
            return blackboard.explore_status == "exploration_complete"
        except KeyError:
            return False

    explored_check = py_trees.decorators.EternalGuard(
        name="Explored?",
        condition=check_explored,
        blackboard_keys={"explore_status"},
        child=cover,
    )

    explore_or_cover = py_trees.composites.Selector(
        name="Explore or Cover", memory=True
    )
    idle_explore = py_trees.behaviours.Success(name="Idle")

    # -----------------------------------------------------------------------
    # Branch 3: Tasks
    # -----------------------------------------------------------------------

    tasks = py_trees.composites.Selector(name="Tasks", memory=True)

    # 3.1 Battery emergency
    def check_battery_low(blackboard: py_trees.blackboard.Blackboard) -> bool:
        try:
            return blackboard.battery_low_warning
        except KeyError:
            return False

    dock = py_trees.composites.Parallel(
        name="Dock",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low,
        blackboard_keys={"battery_low_warning"},
        child=dock,
    )
    flash_red = behaviours.FlashLedStrip(name="Flash Red", colour=[1.0, 0.0, 0.0])
    dock_action = behaviours.NavigateToPosition(
        name="Dock Action", target_position=[0, 0]
    )

    # 3.2 Approach and handle
    approach_and_handle = py_trees.composites.Sequence(
        name="Approach and Handle", memory=True
    )
    detection_check = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Objects?",
        check=py_trees.common.ComparisonExpression(
            variable="cloud_objects_detected",
            value=[],
            operator=operator.ne,
        ),
    )
    pause_coverage = behaviours.SetCoverageStatus(
        name="Pause Coverage", requested_status='pause'
    )
    resume_coverage = behaviours.SetCoverageStatus(
        name="Resume Coverage", requested_status='resume'
    )

    handle = py_trees.composites.Sequence(name="Handle", memory=True)
    flash_green = behaviours.FlashLedStrip(name="Flash Green", colour=[0.0, 1.0, 0.0])

    pick_up = py_trees.composites.Sequence(name="Pick Up", memory=True)
    approach = behaviours.NavigateToPosition(
        name="Approach",
        blackboard_key="cloud_objects_detected",
        standoff=0.4,
    )

    pick_or_skip = py_trees.composites.Selector(name="Pick or Skip", memory=True)

    sort_and_pick = py_trees.composites.Sequence(name="Sort and Pick", memory=True)
    get_planar = behaviours.GetPlanarObjects(name="Planar_Detected?")
    retry_planar = py_trees.decorators.Retry(
        name="Retry Planar",
        child=get_planar,
        num_failures=5,
    )
    pick_object = behaviours.PickObject(
        name="PickObject",
        blackboard_key="planar_objects_detected_array",
    )

    idle_pick = py_trees.behaviours.Success(name="Idle")
    flash_orange = behaviours.FlashLedStrip(
        name="Flash Orange", colour=[1.0, 0.117, 0.0]
    )

    # -----------------------------------------------------------------------
    # Assembly
    # -----------------------------------------------------------------------

    root.add_children([topics2bb, explore_or_cover, tasks])

    # Branch 1
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
        detectedplanar2bb,
    ])

    # Branch 2
    explore_or_cover.add_children([explored_check, idle_explore])

    # Branch 3
    tasks.add_children([battery_emergency, approach_and_handle, flash_orange])

    dock.add_children([flash_red, dock_action])

    approach_and_handle.add_children([
        detection_check,
        # pause_coverage,
        handle,
        # resume_coverage,
    ])
    handle.add_children([flash_green, pick_up])
    pick_up.add_children([approach, pick_or_skip])
    pick_or_skip.add_children([sort_and_pick, idle_pick])
    sort_and_pick.add_children([retry_planar, pick_object])

    return root


def main():
    """Entry point for the demo script."""
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
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
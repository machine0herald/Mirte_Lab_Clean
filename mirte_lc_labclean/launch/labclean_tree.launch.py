"""
ros2 launch mirte_lc_labclean labclean_tree.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node


def generate_launch_description():

    follow_point = Node(
        package="mirte_lc_labclean",
        executable="follow_point",
        name="follow_point",
        output="screen",
    )

    pick_object = Node(
        package="mirte_lc_labclean",
        executable="pick_object_server",
        name="pick_object_server",
        output="screen",
    )

    labclean_tree = Node(
        package="mirte_lc_labclean",
        executable="labclean_tree",
        name="labclean_tree",
        output="screen",
    )

    viewer = ExecuteProcess(
        cmd=['py-trees-tree-viewer', '--no-sandbox'],
        output='screen'
    )
    

    return LaunchDescription(
        [
            # viewer,
            follow_point,
            pick_object,
            TimerAction(period=10.0, actions=[labclean_tree]),
        ]
    )
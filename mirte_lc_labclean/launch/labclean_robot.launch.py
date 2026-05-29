"""
ros2 launch mirte_lc_gazebo gazebo_mirte_lc.launch.py
"""

from sympy import true

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():

    mirte_bringup = get_package_share_directory("mirte_bringup")
    mirte_lc_labclean_pkg = get_package_share_directory("mirte_lc_labclean")

    ###########################
    # Mirte Lab Clean Bringup #
    ###########################
    mirte_lc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_lc_labclean_pkg,
                "launch",
                "labclean_bringup.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    ###################
    # Hardware launch #
    ###################
    mirte_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_bringup,
                "launch",
                "minimal_master.launch.py",
            )
        ),
        launch_arguments={
            "_start_controller_manager": "true",
            "_start_state_publishers": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value="true"),
            mirte_robot,
            TimerAction(period=20.0, actions=[mirte_lc_launch]),
        ]
    )

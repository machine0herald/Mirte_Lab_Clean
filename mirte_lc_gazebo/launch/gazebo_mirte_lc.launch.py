"""Launch file for Gazebo and labclean integration.

This launch description starts Gazebo with the MIRTE labclean world and then
launches the LabClean bringup stack after the simulator is ready.
"""

import math

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
    PythonLaunchDescriptionSource
)
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():

    mirte_gazebo = get_package_share_directory('mirte_lc_gazebo')
    mirte_lc_labclean_pkg = get_package_share_directory('mirte_lc_labclean')
    
    ###########################
    # Mirte Lab Clean Bringup #
    ###########################
    mirte_lc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_lc_labclean_pkg,
                'launch',
                'labclean_bringup.launch.py',
            )
        ),
        launch_arguments={
            "use_sim_time": "true"    
        }.items()
    )

    #################
    # Gazebo launch #
    #################
    gazebo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                mirte_gazebo,
                'launch',
                'gazebo_mirte_master_empty.launch.xml'
            )
        ),
        launch_arguments={
            'gui': 'True',
            'world': 'src/mirte_lc/mirte_lc_gazebo/worlds/floor_with_cubes_2/floor_with_cubes_2.world',
        }.items()
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value='true'),
        gazebo_launch,
        TimerAction(period=10.0, actions=[mirte_lc_launch]),
    ])

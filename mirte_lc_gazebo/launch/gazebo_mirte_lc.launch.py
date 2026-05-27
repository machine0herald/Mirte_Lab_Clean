'''
ros2 launch mirte_lc_gazebo gazebo_mirte_lc.launch.py
'''

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
    mirte_lc_nav = get_package_share_directory('mirte_lc_nav2')
    mirte_lc_labclean_pkg = get_package_share_directory('mirte_lc_labclean')
    twist_mux_yaml = os.path.join(mirte_lc_labclean_pkg, 'config', 'twist_mux.yaml')

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

    #######################
    # Mirte Moveit Launch #
    #######################
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mirte_lc_moveit_cpp'),
                'launch',
                'mirte_lc_moveit.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    #####################
    # Perception Launch #  -0.05},
    #####################

    PCnode = Node(
        package='mirte_lc_vision',
        executable= 'pc_node',
        name='pc_node',
    )

    ####################
    # Navigation Stack #
    ####################
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_lc_nav,
                'launch',
                'mirte_lc_nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    ##################
    # Twist Mux Node #
    ##################
    twist_mux = Node(
        package= 'twist_mux',
        executable= 'twist_mux',
        parameters= [twist_mux_yaml, 
                    {'use_sim_time': True}],
        remappings= [('cmd_vel_out', 'cmd_vel')],
    )

    ##################
    # Octomap Server #
    ##################
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/points'),
        ],
        parameters=[
            {'frame_id': 'odom'},
            {'base_frame_id': 'base_link'},
            {'resolution': 0.01},
            {'sensor_model.max_range': 1.0},
            {'point_cloud_min_z':  0.01},
            {'point_cloud_max_z':  0.2},
            {'occupancy_min_z': 0.01},
            {'occupancy_max_z': 0.2},
            {'filter_ground_plane': False},
            {'ground_filter/distance': 0.01},
        ]
    )
    
    ####################
    # Labclean Manager #
    ####################
    labclean_manager = Node(
        package='mirte_lc_labclean',
        executable='labclean_manager',
        name='labclean_manager',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value='true'),
        gazebo_launch,
        TimerAction(period=10.0, actions=[moveit_launch]),
        # TimerAction(period=27.0, actions=[octomap]),
        TimerAction(period=30.0, actions=[nav2]),
        TimerAction(period=60.0, actions=[
            LogInfo(msg='Starting Labclean Manager'),
            labclean_manager]), 
    ])

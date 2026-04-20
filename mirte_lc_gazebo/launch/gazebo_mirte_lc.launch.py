'''
ros2 launch mirte_lc_gazebo gazebo_mirte_lc.launch.py
'''

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


def generate_launch_description():

    mirte_gazebo = get_package_share_directory('mirte_lc_gazebo')
    mirte_navigation = get_package_share_directory('mirte_navigation')
    mirte_lc_labclean_pkg = get_package_share_directory('mirte_lc_labclean')
    twist_mux_yaml = os.path.join(mirte_lc_labclean_pkg, 'config', '.yaml')

    ####################
    # Launch Arguments #
    ####################
    exe_arg = DeclareLaunchArgument(
        name='executable',
        default_value='test_node'
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
            'world': 'src/mirte_lc/mirte_lc_gazebo/worlds/floor_with_cubes/floor_with_cubes.world',
        }.items()
    )


    #######################
    # Mirte Moveit Launch #
    #######################
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mirte_moveit_config'),
                'launch',
                'mirte_moveit.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    #######################
    # Slam Toolbox Launch #
    #######################
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_navigation,
                'launch',
                'minimal_slam_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    ########
    # Nav2 #
    ########
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_navigation,
                'launch',
                'minimal_navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    ##############
    # Executable #
    ##############
    node_name = LaunchConfiguration('executable')                                                                                                                                                                                                                                                                                                                                                                                  
    executable = Node(
        package='mirte_lc_labclean',
        executable=node_name,
        name='test_node',
        output='screen',
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
            # {'use_sim_time': True},
            {'resolution': 0.05}
        ]
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value='true'),
        exe_arg,
        gazebo_launch,
        TimerAction(period=10.0, actions=[moveit_launch]),
        TimerAction(period=20.0, actions=[slam_toolbox]),
        # TimerAction(period=27.0, actions=[octomap]),
        TimerAction(period=30.0, actions=[nav2]), 
        # TimerAction(period=25.0, actions=[twist_mux]),   
        # TimerAction(period=80.0, actions=[executable]),
    ])

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mirte_gazebo = get_package_share_directory('mirte_gazebo')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    mirte_lc_labclean_pkg = get_package_share_directory('mirte_lc_labclean')

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
            'world': "src/mirte_lc/mirte_lc_gazebo/worlds/floor_with_cubes/floor_with_cubes.world"
        }.items()
    )

    ############################
    # Mirte Moveit Demo Launch #
    ############################
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

    ##########################
    # Static TF broadcaster  #
    ##########################
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=[
            '0', '0', '0',   # xyz
            '0', '0', '0',   # rpy
            'base_link',
            'base_footprint'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    #######################
    # Slam Toolbox Launch #
    #######################
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_pkg,
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file':os.path.join(
                mirte_lc_labclean_pkg,
                'config',
                'slam_tbx.yaml',
            )
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
        output='screen'
    )

    ###################
    # Delayed startup #
    ###################

    # Start SLAM after TF is available
    delayed_slam = TimerAction(
        period=25.0,
        actions=[slam_toolbox]
    )

    # Start your node after Gazebo + SLAM
    delayed_executable = TimerAction(
        period=35.0,
        actions=[executable]
    )

    return LaunchDescription([
        exe_arg,
        gazebo_launch,
        TimerAction(period=10.0, actions=[moveit_launch]),
        TimerAction(period=26.0, actions=[base_footprint_tf]),
        TimerAction(period=27.0, actions=[slam_toolbox]),
        TimerAction(period=29.0, actions=[executable]),
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():

    mirte_pkg = get_package_share_directory('mirte_gazebo')

    gazebo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                mirte_pkg,
                'launch',
                'gazebo_mirte_master_empty.launch.xml'
            )
        ),
        launch_arguments={
            'gui': 'True',
            'world': "src/mirte_lc/mirte_lc_gazebo/worlds/floor_with_cubes/floor_with_cubes.world"
        }.items()
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/tmp/placeholder.rviz'],
        output='screen'
    )

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

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    ros2_node_launch = Node(
    package='mirte_lc_labclean',
    executable='test_node',
    name='test_node',
    output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=10.0, actions=[rviz]),
        TimerAction(period=26.0, actions=[base_footprint_tf]),
        TimerAction(period=27.0, actions=[slam_toolbox]),
        TimerAction(period=29.0, actions=[ros2_node_launch]),
    ])
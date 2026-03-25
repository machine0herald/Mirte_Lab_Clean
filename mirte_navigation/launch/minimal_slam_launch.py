# mirte_navigation/launch/slam_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    ###############
    # Launch Args #
    ###############
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    ################
    # Slam Toolbox #
    ################
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('mirte_navigation'), 'params', 'slam_params.yaml']
            )
        ],
    )

    #######
    # TFs #
    #######
    tf2_ros_link_fp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
    )
    tf2_ros_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
        output='screen',
    )

    ##############
    # Odom relay #
    ##############
    relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/mirte_base_controller/odom', '/odom'],
        output='screen',
    )

    return LaunchDescription(
        [
            sim_time_arg,
            SetParameter('use_sim_time', 
                         value=LaunchConfiguration('use_sim_time')),
            slam_toolbox,
            tf2_ros_link_fp,
            tf2_ros_link_frame,
            relay,
        ]
    )
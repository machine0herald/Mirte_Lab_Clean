# mirte_navigation/launch/slam_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

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
            SetParameter('use_sim_time', value='true'),
            slam_toolbox,
            tf2_ros_link_fp,
            tf2_ros_link_frame,
            relay,
        ]
    )
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('mirte_lc_labclean'),
                'config',
                'slam_tbx.yaml'
            ]),
            {'use_sim_time': True},
        ],
    )

    tf2_ros_link_fp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    tf2_ros_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/mirte_base_controller/odom', '/odom'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        slam_toolbox,
        tf2_ros_link_fp,
        tf2_ros_link_frame,
        relay,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
            {'use_sim_time': use_sim_time},
        ],
    )

    tf2_ros_link_fp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf2_ros_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf2_ros_link_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/mirte_base_controller/odom', '/odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        slam_toolbox,
        tf2_ros_link_fp,
        tf2_ros_link_frame,
        # tf2_ros_link_odom,
        relay,
    ])
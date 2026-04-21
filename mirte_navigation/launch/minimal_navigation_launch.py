from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('mirte_lc_labclean'),
        'config',
        'nav3.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    # -------------------
    # Topic relays
    # -------------------
    topic_tools_vel = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/cmd_vel', '/mirte_base_controller/cmd_vel'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    topic_tools_odom = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/mirte_base_controller/odom', '/odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------------------
    # Nav2 servers
    # -------------------
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # -------------------
    # Lifecycle manager
    # -------------------
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server',
            ],
        }],
    )

    # -------------------
    # Behavior Server
    # -------------------

    nav2_behavior_server = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[params_file],
    )

    return LaunchDescription([
        LogInfo(msg='Starting Mirte Nav2 stack'),

        use_sim_time_arg,

        topic_tools_vel,
        topic_tools_odom,

        nav2_planner,
        nav2_controller,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
        nav2_behavior_server,
    ])
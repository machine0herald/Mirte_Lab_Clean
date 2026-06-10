"""
ros2 launch mirte_lc_nav2 mirte_lc_nav2.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    mirte_navigation = get_package_share_directory("mirte_navigation")
    fbe_mapping = get_package_share_directory("mirte_lc_nav2")

    #######################
    # Launch Args/Configs #
    #######################
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = os.path.join(fbe_mapping, "config", "nav2_coverage_params_sim.yaml")


    #######################
    # Slam Toolbox Launch #
    #######################
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mirte_navigation, "launch", "minimal_slam_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ################
    # Topic relays #
    ################
    topic_tools_vel = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel", "/mirte_base_controller/cmd_vel"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    topic_tools_odom = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/mirte_base_controller/odom", "/odom"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ################
    # Nav2 servers #
    ################
    nav2_planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    nav2_controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            params_file, 
            {"use_sim_time": use_sim_time}
        ],
    )

    #####################
    # Lifecycle manager #
    #####################
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                ],
            }
        ],
    )

    ###################
    # Behavior Server #
    ###################
    nav2_behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    ################################
    # Lab cleanup navigation stack #
    ################################
    nav2_labclean = Node(
        package="mirte_lc_nav2",
        executable="labclean_navigator",
        name="labclean_navigator",
        output="screen",
    )

    ##############################
    # M-explore navigation stack #
    ##############################
    m_explore_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fbe_mapping, "launch", "fbe.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            SetParameter(name="use_sim_time", value=use_sim_time),
            TimerAction(
                period=20.0,
                actions=[LogInfo(msg="Starting Slam Toolbox"), slam_toolbox],
            ),
            TimerAction(
                period=30.0,
                actions=[
                    LogInfo(msg="Starting Mirte Nav2 stack"),
                    topic_tools_vel,
                    topic_tools_odom,
                    nav2_planner,
                    nav2_controller,
                    nav2_bt_navigator,
                    nav2_lifecycle_manager,
                    nav2_behavior_server,
                ],
            ),
            TimerAction(
                period=50.0,
                actions=[
                    LogInfo(msg="Starting Lab cleanup Navigation stack"),
                    nav2_labclean,
                ],
            ),
            TimerAction(
                period=50.0,
                actions=[
                    LogInfo(msg="Starting M-explore Navigation stack"),
                    m_explore_nav,
                ],
            ),
        ]
    )

"""
ros2 launch mirte_lc_labclean gazebo_mirte_lc.launch.py
"""

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
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():

    ###############
    # Launch Args #
    ###############
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    #################
    # Launch Config #
    #################
    use_sim_time = LaunchConfiguration(
        "use_sim_time",
    )

    ##################
    # File Locations #
    ##################
    mirte_lc_nav = get_package_share_directory("mirte_lc_nav2")
    mirte_lc_labclean_pkg = get_package_share_directory("mirte_lc_labclean")
    twist_mux_yaml = os.path.join(mirte_lc_labclean_pkg, "config", "twist_mux.yaml")

    #######################
    # Mirte Moveit Launch #
    #######################
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mirte_lc_moveit_cpp"),
                "launch",
                "mirte_lc_moveit.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    #####################
    # Perception Launch #
    #####################
    PCnode = Node(
        package="mirte_lc_vision",
        executable="pc_node",
        name="pc_node",
    )

    ####################
    # Navigation Stack #
    ####################
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mirte_lc_nav, "launch", "mirte_lc_nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ##################
    # Twist Mux Node #
    ##################
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_yaml, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel_out", "cmd_vel")],
    )

    ####################
    # Labclean Manager #
    ####################
    labclean_manager = Node(
        package="mirte_lc_labclean",
        executable="labclean_manager",
        name="labclean_manager",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    ###################
    # Foxglove Bridge #
    ###################
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
        launch_arguments={
            "address": "0.0.0.0",
            "port": "8766",
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            foxglove_bridge,
            TimerAction(period=10.0, actions=[moveit_launch]),
            TimerAction(period=30.0, actions=[nav2]),
            TimerAction(
                period=80.0,
                actions=[LogInfo(msg="Starting Labclean Manager"), labclean_manager],
            ),
        ]
    )

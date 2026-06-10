"""
ros2 launch mirte_lc_labclean labclean_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node


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
    points_topic = PythonExpression([
        '"/camera/points" if "',
        use_sim_time,
        '" == "true" else "/camera/depth/points"'
    ])
    camera_topic = PythonExpression([
        '"/camera/image_raw" if "',
        use_sim_time,
        '" == "true" else "/camera/color/image_raw"'
    ])

    ##################
    # File Locations #
    ##################
    mirte_lc_nav = get_package_share_directory("mirte_lc_nav2")
    mirte_lc_perception = get_package_share_directory("mirte_lc_vision")
    mirte_lc_moveit_cpp =  get_package_share_directory("mirte_lc_moveit_cpp")

    ########################
    # Labclean Tree Launch #
    ########################
    labclean_tree = Node(
        package="mirte_lc_labclean",
        executable="labclean_tree",
        name="labclean_tree",
        output="screen",
    )
    viewer = ExecuteProcess(
        cmd=['py-trees-tree-viewer', '--no-sandbox'],
        output='screen'
    )
    
    #######################
    # Mirte Moveit Launch #
    #######################
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_lc_moveit_cpp,
                "launch",
                "mirte_lc_moveit.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )

    ####################
    # Navigation Stack #
    ####################
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_lc_nav,
                "launch",
                "mirte_lc_nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    #####################
    # Perception Launch #
    #####################
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mirte_lc_perception,
                "launch",
                "perception.launch.py")
        ),
        launch_arguments={
            "points_topic": points_topic,
            "camera_topic": camera_topic,
        }.items(),
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
            # labclean_tree,
            # viewer,
            use_sim_time_arg,
            # foxglove_bridge,
            TimerAction(period=1.0, actions=[moveit_launch]),
            TimerAction(period=20.0, actions=[nav2]),
            TimerAction(period=20.0, actions=[perception])
        ]
    )

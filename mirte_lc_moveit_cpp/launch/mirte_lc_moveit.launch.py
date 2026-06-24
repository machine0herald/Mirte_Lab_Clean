# ros2 param set /ros2_arm_control_hw_interface servo_moved_dead_band 0.0001
# ros2 param set /ros2_arm_control_hw_interface_servo_update_dead_band 0.001

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Get parameters for the Servo node
    servo_yaml = load_yaml("mirte_lc_moveit_cpp", "config/servo_parameters.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    update_parameters = []

    update_parameters.append(ExecuteProcess(
            cmd=[
                'bash', '-c',
                '''
                ros2 param set /ros2_arm_control_hw_interface servo_moved_dead_band 0.0001
                '''
            ],

            shell=False,
            output='screen'
        ),
    )

    update_parameters.append(ExecuteProcess(
            cmd=[
                'bash', '-c',
                '''
                ros2 param set /ros2_arm_control_hw_interface servo_update_dead_band 0.001
                '''
            ],

            shell=False,
            output='screen'
        ),
    )

    # Declare a launch argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Launch configuration variable
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    moveit_config = (
    MoveItConfigsBuilder("mirte", package_name="mirte_moveit_config")
    .robot_description(file_path="config/mirte_master.urdf.xacro")
    .robot_description_semantic(file_path="config/mirte_master.srdf")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .planning_pipelines(
          pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
      )
    .moveit_cpp(file_path=get_package_share_directory("mirte_lc_moveit_cpp") + "/config/moveit_cpp.yaml")
    .to_moveit_configs()

    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {'publish_robot_description_semantic': True}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    labclean_config = os.path.join(
        get_package_share_directory("mirte_lc_labclean"), "config"
    )
    rviz_full_config = os.path.join(labclean_config, "labclean.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
        ],
    )

    moveit_action_server_node = Node(
        package='mirte_lc_moveit_cpp',
        executable='moveit_action_server',
        name='moveit_action_server',
        output='log',
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )


    return LaunchDescription(
        update_parameters +
        [
            use_sim_time_arg,
            SetParameter(name="use_sim_time", value=use_sim_time),
            move_group_node,
            servo_node,
            TimerAction(period = 5.0, actions = [rviz_node]),
            TimerAction(period = 20.0, actions = [moveit_action_server_node]),
        ]
    )

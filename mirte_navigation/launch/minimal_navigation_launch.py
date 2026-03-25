from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import LogInfo


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    SetParameter(name='use_sim_time', value='true'),

    ###############
    # Launch Args #
    ###############
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('mirte_navigation'), 'params', 'minimal_nav2_params.yaml']
        ),
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    ##################################
    # TF: base_link → base_footprint #
    ##################################
    tf2_ros_link_fp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
    )

    ##############################
    # TF: base_link → base_frame #
    ##############################
    tf2_ros_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
        output='screen',
    )

    ###################################################
    # Relay /cmd_vel → /mirte_base_controller/cmd_vel #
    ###################################################
    topic_tools_vel = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/cmd_vel', '/mirte_base_controller/cmd_vel'],
        output='screen',
    )

    #############################################
    # Relay /mirte_base_controller/odom → /odom #
    #############################################
    topic_tools_odom = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/mirte_base_controller/odom', '/odom'],
        output='screen',
    )

    ##############
    # Map server #
    ##############
    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file],
    )

    ########
    # AMCL #
    ########
    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    ##################
    # Planner server #
    ##################
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    #####################
    # Controller server #
    #####################
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    ################
    # BT Navigator #
    ################
    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'bt_navigator:=debug'],
    )

    #####################
    # Lifecycle manager #
    #####################
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                ],
            }
        ],
    )

    return LaunchDescription(
            [
                LogInfo(
                    msg='\033[1mNote: mirte_navigation requires proper PID gains, \
                especially an integrator term, \
                in /opt/ros/humble/share/mirte_base_control/config/mirte_base_control.yaml\033[0m'
                ),
                SetParameter('use_sim_time',
                             value=LaunchConfiguration('use_sim_time')),
                params_arg,
                use_sim_time,
                tf2_ros_link_fp,
                tf2_ros_link_frame,
                topic_tools_vel,
                topic_tools_odom,
                nav2_map_server,
                nav2_amcl,
                nav2_planner,
                nav2_controller,
                nav2_bt_navigator,
                nav2_lifecycle_manager,
            ]
        )
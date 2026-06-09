"""mirte_lc_vision Perception launch

This module defines a ROS 2 launch description for the perception
stack used by the mirte_lc robot. It starts the object locator and
2D classifier nodes and wires their topics via remapping.

The docstrings follow the Google style so they are compatible with
Sphinx (using the Napoleon extension) and typical doc tooling.

Example:

    $ ros2 launch mirte_lc_vision perception.launch.py

"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Create and return the launch description for perception.

    Returns:
        launch.LaunchDescription: A launch description that starts the
            perception-related ROS 2 nodes (object_locator and
            classifier_2d) and logs a startup message.
    """

    points_topic = LaunchConfiguration("points_topic")
    camera_topic = LaunchConfiguration("camera_topic")

    ##############
    # Perception #
    ##############
    object_detector = Node(
        package="mirte_lc_vision",
        executable="object_locator",
        name="object_locator",
        remappings=[
            ("/camera/depth/points", points_topic),
        ],
    )
    
    object_classifier = Node(
        package="mirte_lc_vision",
        executable="classifier_2d",
        name="classifier_2d",
        remappings=[
            ("/camera/color/image_raw/compressed", camera_topic),
        ],
    )

    depth_image_proc = Node(
    package='depth_image_proc',
    executable='point_cloud_xyz_node',
    name='point_cloud_xyz',
    remappings=[
        ('image_rect', '/camera/depth/image_raw'),
        ('camera_info', '/camera/depth/camera_info'),
        ('points', '/camera/depth/points'),
    ],
    arguments=[
    #     # '--ros-args',
    #     # '--qos-overrides',
    #     # '/image_rect.subscription.reliability=reliable',
        'reliability=1'
    ]
)


    return LaunchDescription(
        [
            LogInfo(msg="Starting Perception Stack"),
            object_detector,
            object_classifier,
            depth_image_proc,
        ]
    )
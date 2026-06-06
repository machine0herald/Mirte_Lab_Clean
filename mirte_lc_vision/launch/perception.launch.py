"""
ros2 launch mirte_lc_vision perception.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    points_topic = LaunchConfiguration("points_topic")

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
    )

    return LaunchDescription(
        [
            LogInfo(msg="Starting Perception Stack"),
            object_detector,
            object_classifier,
        ]
    )
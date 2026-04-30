cmake_minimum_required(VERSION 3.8)
project(mirte_lc_moveit_cpp)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(Boost REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(moveit_example src/moveit_example.cpp)
ament_target_dependencies(moveit_example rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs)

install(TARGETS 
   moveit_example
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(mirte_lc_moveit_cpp)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(Boost REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(mirte_lc_msgs REQUIRED)


#########################################
# MoveIt Action Server  #
#########################################

add_library(action_server SHARED
  src/moveit_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "MIRTE_LC_MOVEIT_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "mirte_lc_msgs"
  )
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

##########################################
# /Moveit Action Server #
##########################################

add_executable(moveit_example src/moveit_example.cpp)

ament_target_dependencies(moveit_example rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs)

add_executable(moveit_action_server src/moveit_action_server_main.cpp src/moveit_action_server.cpp)
target_include_directories(moveit_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(moveit_action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
  geometry_msgs
  mirte_lc_msgs
)

install(TARGETS 
   moveit_example
   moveit_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
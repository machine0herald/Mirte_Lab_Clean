/**
 * @file moveit_action_server_main.cpp
 * @brief Entry point for the MIRTE MoveIt action server executable.
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mirte_lc_moveit_cpp/moveit_action_server.hpp"

/**
 * @brief Initialize ROS 2 and spin the MoveIt action server node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mirte_lc_moveit_cpp::MirteLCMoveItActionServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

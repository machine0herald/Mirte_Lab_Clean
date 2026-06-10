/**
 * @file moveit_example.cpp
 * @brief Simple MoveIt example demonstrating planning and execution.
 */

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief Run an example MoveIt plan and execute it for the MIRTE arm.
 */
int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Start spinning in a separate thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {
    executor.spin();
  });

  // Wait for some initialization work
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_node");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "mirte_arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.085;
    msg.position.y = 0.0;
    msg.position.z = 0.47;
    msg.orientation.x = 0.7;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.7;
    msg.orientation.w = 0.0;
    return msg;
  }();

  move_group_interface.setApproximateJointValueTarget(target_pose);

  // Or set to a named target
  //move_group_interface.setNamedTarget("home");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Get the current pose of the end effector
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose("wrist");

  // Print the current pose
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current pose: position = [%f, %f, %f], orientation = [%f, %f, %f, %f]",
           current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
           current_pose.pose.orientation.x, current_pose.pose.orientation.y,
           current_pose.pose.orientation.z, current_pose.pose.orientation.w);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
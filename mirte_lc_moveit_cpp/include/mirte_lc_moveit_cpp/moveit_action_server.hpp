#ifndef MIRTE_LC_MOVEIT_CPP__MOVEIT_ACTION_SERVER_HPP_
#define MIRTE_LC_MOVEIT_CPP__MOVEIT_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mirte_lc_msgs/action/move_to_position.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace mirte_lc_moveit_cpp
{

class MirteLCMoveItActionServer : public rclcpp::Node
{
public:
  using MoveToPosition = mirte_lc_msgs::action::MoveToPosition;
  using GoalHandleMoveToPosition =
    rclcpp_action::ServerGoalHandle<MoveToPosition>;

  explicit MirteLCMoveItActionServer(
    const rclcpp::NodeOptions & options =
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true))
  : Node("mirte_lc_moveit_action_server", options)
  {
    using namespace std::placeholders;

    RCLCPP_INFO(
      get_logger(),
      "Constructing MoveIt Action Server...");

    action_server_ =
      rclcpp_action::create_server<MoveToPosition>(
        this,
        "move_to_position",
        std::bind(
          &MirteLCMoveItActionServer::handle_goal,
          this,
          _1,
          _2),
        std::bind(
          &MirteLCMoveItActionServer::handle_cancel,
          this,
          _1),
        std::bind(
          &MirteLCMoveItActionServer::handle_accepted,
          this,
          _1));

    RCLCPP_INFO(
      get_logger(),
      "MoveIt Action Server READY on /move_to_position");
  }

  void initialize_moveit()
  {
    move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        "mirte_arm");

    robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this());

    kinematic_model_ = robot_model_loader_->getModel();

    robot_state_ =
      std::make_shared<moveit::core::RobotState>(
        kinematic_model_);

    robot_state_->setToDefaultValues();

    joint_model_group_ =
      kinematic_model_->getJointModelGroup("mirte_arm");

    RCLCPP_INFO(get_logger(), "MoveIt initialized");
  }

private:
  /*
   * MoveIt
   */

  std::shared_ptr<robot_model_loader::RobotModelLoader>
    robot_model_loader_;

  moveit::core::RobotModelPtr kinematic_model_;

  moveit::core::RobotStatePtr robot_state_;

  const moveit::core::JointModelGroup * joint_model_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
    move_group_;

  /*
   * Action server
   */

  rclcpp_action::Server<MoveToPosition>::SharedPtr
    action_server_;

  /*
   * Action callbacks
   */

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPosition::Goal> goal)
  {
    RCLCPP_INFO(
      get_logger(),
      "Goal received: target pose [%.3f, %.3f, %.3f]",
      goal->target_pose.position.x,
      goal->target_pose.position.y,
      goal->target_pose.position.z);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPosition>)
  {
    RCLCPP_WARN(get_logger(), "Cancel request received");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPosition> goal_handle)
  {
    RCLCPP_INFO(
      get_logger(),
      "Goal accepted → starting execution thread");

    using namespace std::placeholders;

    std::thread(
      std::bind(
        &MirteLCMoveItActionServer::execute,
        this,
        _1),
      goal_handle).detach();
  }

  /*
   * Main execution
   */

  void execute(
    const std::shared_ptr<GoalHandleMoveToPosition> goal_handle)
  {
    RCLCPP_INFO(
      get_logger(),
      "Execution thread started");

    auto goal = goal_handle->get_goal();

    auto result =
      std::make_shared<MoveToPosition::Result>();

    result->success = false;

    print_move_group_info();

    /*
     * Current state
     */

    auto current_state =
      move_group_->getCurrentState(10.0);

    current_state->update();
    current_state->enforceBounds();
    current_state->update();

    move_group_->setStartState(*current_state);

    print_joint_values(current_state, "CURRENT");

    /*
     * Planner configuration
     */

    move_group_->setGoalOrientationTolerance(3.14);

    move_group_->setWorkspace(
      -0.30, -0.30, 0.00,
       0.30,  0.30, 0.30);

    print_planner_info();

    /*
     * Target pose
     */

    geometry_msgs::msg::Pose target_pose =
      goal->target_pose;

    // target_pose.orientation =
    //   move_group_->getCurrentPose("wrist")
    //     .pose.orientation;

    /*
     * Collision check
     */

    planning_scene::PlanningScene planning_scene(
      kinematic_model_);

    bool collision =
      planning_scene.isStateColliding(*current_state);

    RCLCPP_INFO(
      get_logger(),
      "Collision: %s",
      collision ? "true" : "false");

    /*
     * IK
     */

    auto target_state =
      move_group_->getCurrentState(10.0);

    bool found_ik =
      target_state->setFromIK(
        joint_model_group_,
        target_pose,
        "wrist");

    RCLCPP_INFO(
      get_logger(),
      "IK solution found: %s",
      found_ik ? "true" : "false");

    bool valid =
      target_state->satisfiesBounds(
        joint_model_group_);

    RCLCPP_INFO(
      get_logger(),
      "Bounds valid: %s",
      valid ? "true" : "false");

    if (!found_ik)
    {
      RCLCPP_ERROR(
        get_logger(),
        "IK failed");

      goal_handle->abort(result);

      return;
    }

    /*
     * Target joints
     */

    target_state->update();
    target_state->enforceBounds();

    std::vector<double> joint_values;

    target_state->copyJointGroupPositions(
      joint_model_group_,
      joint_values);

    print_joint_values(target_state, "TARGET");

    move_group_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z, "wrist");

    /*
     * Planning
     */

    auto [success, plan] =
      [this]
      {
        moveit::planning_interface::
          MoveGroupInterface::Plan msg;

        bool ok =
          static_cast<bool>(
            move_group_->plan(msg));

        return std::make_pair(ok, msg);
      }();

    if (!success)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Planning failed");

      goal_handle->abort(result);

      return;
    }
    robot_trajectory::RobotTrajectory rt(
    kinematic_model_,
    "mirte_arm");

    rt.setRobotTrajectoryMsg(
        *move_group_->getCurrentState(),
        plan.trajectory_);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;

    bool timing_success = totg.computeTimeStamps(rt);

    RCLCPP_INFO(
        this->get_logger(),
        "Time parameterization success: %s",
        timing_success ? "true" : "false");

    rt.getRobotTrajectoryMsg(plan.trajectory_);

    RCLCPP_INFO(
      get_logger(),
      "Planning successful");

    /*
     * Execution
     */

    const auto& pts = plan.trajectory_.joint_trajectory.points;

    RCLCPP_INFO(this->get_logger(),
        "Trajectory contains %zu points",
        pts.size());

    for (size_t i = 0; i < pts.size(); ++i)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Point %zu time: %f",
          i,
          pts[i].time_from_start.sec +
          pts[i].time_from_start.nanosec * 1e-9);
    }
    move_group_->execute(plan);

    /*
     * Final pose
     */

    auto current_pose =
      move_group_->getCurrentPose("wrist");

    RCLCPP_INFO(
      get_logger(),
      "Current pose: "
      "position = [%f, %f, %f], "
      "orientation = [%f, %f, %f, %f]",
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z,
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z,
      current_pose.pose.orientation.w);

    /*
     * Finish action
     */

    result->success = true;

    goal_handle->succeed(result);

    RCLCPP_INFO(
      get_logger(),
      "Goal completed successfully");
  }

  /*
   * Helper functions
   */

  void print_move_group_info()
  {
    RCLCPP_INFO(
      get_logger(),
      "Pose Reference Frame: %s",
      move_group_->getPoseReferenceFrame().c_str());

    RCLCPP_INFO(
      get_logger(),
      "End Effector Link: %s",
      move_group_->getEndEffectorLink().c_str());
  }

  void print_planner_info()
  {
    RCLCPP_INFO(
      get_logger(),
      "Planning frame: %s",
      move_group_->getPlanningFrame().c_str());

    RCLCPP_INFO(
      get_logger(),
      "Goal position tolerance: %f",
      move_group_->getGoalPositionTolerance());

    RCLCPP_INFO(
      get_logger(),
      "Goal orientation tolerance: %f",
      move_group_->getGoalOrientationTolerance());

    RCLCPP_INFO(
      get_logger(),
      "Goal joint tolerance: %f",
      move_group_->getGoalJointTolerance());
  }

  void print_joint_values(
    const moveit::core::RobotStatePtr & state,
    const std::string & label)
  {
    const auto & joint_names =
      joint_model_group_->getVariableNames();

    std::vector<double> joint_values;

    state->copyJointGroupPositions(
      joint_model_group_,
      joint_values);

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(
        get_logger(),
        "[%s] %s: %f",
        label.c_str(),
        joint_names[i].c_str(),
        joint_values[i]);
    }
  }
};

}  // namespace mirte_lc_moveit_cpp

#endif  // MIRTE_LC_MOVEIT_CPP__MOVEIT_ACTION_SERVER_HPP_
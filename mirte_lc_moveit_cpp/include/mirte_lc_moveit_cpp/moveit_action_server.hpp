/**
 * @file moveit_action_server.hpp
 * @brief Defines the MIRTE MoveIt action server node and execution helpers.
 */

#ifndef MIRTE_LC_MOVEIT_CPP__MOVEIT_ACTION_SERVER_HPP_
#define MIRTE_LC_MOVEIT_CPP__MOVEIT_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>

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

/**
 * @brief Implements a MoveIt-based action server for robot and gripper motion.
 *
 * The node exposes the /move_to_position action and handles named targets,
 * pose goals, and gripper commands for the MIRTE arm and gripper groups.
 */
class MirteLCMoveItActionServer : public rclcpp::Node
{
public:
  using MoveToPosition = mirte_lc_msgs::action::MoveToPosition;
  using GoalHandleMoveToPosition =
    rclcpp_action::ServerGoalHandle<MoveToPosition>;

  /**
   * @brief Construct a new MoveIt action server node.
   *
   * @param options Node options for ROS 2 component configuration.
   */
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

  /**
   * @brief Initialize MoveIt interfaces and configure the arm and gripper groups.
   */
  void initialize_moveit()
  {
     robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this());

    // Mirte_arm initialization
    
    mirte_arm_move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        "mirte_arm");

    mirte_arm_kinematic_model_ = robot_model_loader_->getModel();

    mirte_arm_robot_state_ =
      std::make_shared<moveit::core::RobotState>(
        mirte_arm_kinematic_model_);

    mirte_arm_robot_state_->setToDefaultValues();

    mirte_arm_joint_model_group_ =
      mirte_arm_kinematic_model_->getJointModelGroup("mirte_arm");
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    mirte_arm_move_group_->setStartStateToCurrentState();
    mirte_arm_move_group_->setNamedTarget("vigilant");
    mirte_arm_move_group_->move();

    // Mirte_gripper initialization

    mirte_gripper_move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        "mirte_gripper");

    mirte_gripper_kinematic_model_ = robot_model_loader_->getModel();

    mirte_gripper_robot_state_ =
      std::make_shared<moveit::core::RobotState>(
        mirte_gripper_kinematic_model_);

    mirte_gripper_robot_state_->setToDefaultValues();

    mirte_gripper_joint_model_group_ =
      mirte_gripper_kinematic_model_->getJointModelGroup("mirte_gripper");

    RCLCPP_INFO(get_logger(), "MoveIt initialized");
  }

private:
  /*
   * MoveIt
   */

  std::shared_ptr<robot_model_loader::RobotModelLoader>
    robot_model_loader_;

  moveit::core::RobotModelPtr mirte_arm_kinematic_model_;

  moveit::core::RobotStatePtr mirte_arm_robot_state_;

  const moveit::core::JointModelGroup * mirte_arm_joint_model_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
    mirte_arm_move_group_;

  moveit::core::RobotModelPtr mirte_gripper_kinematic_model_;

  moveit::core::RobotStatePtr mirte_gripper_robot_state_;

  const moveit::core::JointModelGroup * mirte_gripper_joint_model_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
    mirte_gripper_move_group_;

  /*
   * Action server
   */

  rclcpp_action::Server<MoveToPosition>::SharedPtr
    action_server_;

  /*
   * Action callbacks
   */

  /**
   * @brief Evaluate and accept or reject a new action goal.
   *
   * @param[in] The goal UUID (unused).
   * @param[in] goal The requested move-to-position goal.
   * @return rclcpp_action::GoalResponse Whether the goal is accepted.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPosition::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Action Contents: goal->target_pose = [%.3f, %.3f, %.3f]",
      goal->mirte_arm_target_pose.position.x,
      goal->mirte_arm_target_pose.position.y,
      goal->mirte_arm_target_pose.position.z);

    RCLCPP_INFO(get_logger(), "Action Contents: goal->mirte_arm_named_target = %s",
      goal->mirte_arm_named_target.c_str());

    RCLCPP_INFO(get_logger(), "Action Contents: goal->gripper_named_target = %s",
      goal->mirte_gripper_named_target.c_str());

    RCLCPP_INFO(get_logger(), "Action Contents: goal->gripper_joint_target = [%.3f]",
      goal->mirte_gripper_joint_target);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handle cancel requests for an active action goal.
   *
   * @param[in] Goal handle for the move-to-position action.
   * @return rclcpp_action::CancelResponse Whether cancellation is allowed.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPosition>)
  {
    RCLCPP_WARN(get_logger(), "Cancel request received");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Start asynchronous execution for an accepted goal.
   *
   * @param[in] goal_handle Handle to the accepted move-to-position goal.
   */
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

  /**
   * @brief Execute an accepted move-to-position action goal.
   *
   * @param[in] goal_handle Handle to the accepted goal being executed.
   */
  void execute(const std::shared_ptr<GoalHandleMoveToPosition> goal_handle)
  {
    RCLCPP_INFO(
      get_logger(),
      "Execution thread started");

    auto goal = goal_handle->get_goal();

    auto result =
      std::make_shared<MoveToPosition::Result>();

    result->success = false;

    if (!goal->mirte_arm_named_target.empty()){
      RCLCPP_INFO(
        get_logger(),
        "Named target specified: %s",
        goal->mirte_arm_named_target.c_str());

      mirte_arm_move_group_->setNamedTarget(
        goal->mirte_arm_named_target);

      mirte_arm_move_group_->setStartStateToCurrentState();
      mirte_arm_move_group_->move();

      result->success = true;
      goal_handle->succeed(result);

      RCLCPP_INFO(
        get_logger(),
        "Goal completed successfully");

      return;
    }

    if (goal->mirte_arm_target_pose.position.x != 0.0)
    {
      std::vector<double> mirte_arm_joint_values;
      mirte_arm_move_group_->setGoalPositionTolerance(0.001);
      mirte_arm_move_group_->setGoalJointTolerance(0.001);

      print_move_group_info();

      /*
      * Current state
      */

      auto mirte_arm_current_state = mirte_arm_move_group_->getCurrentState(10.0);

      mirte_arm_current_state->update();
      mirte_arm_current_state->enforceBounds();
      mirte_arm_current_state->update();

      mirte_arm_move_group_->setStartState(*mirte_arm_current_state);

      print_joint_values(mirte_arm_current_state, "CURRENT");

      /*
      * Planner configuration
      */


      print_planner_info();

      /*
      * Target pose
      */

      geometry_msgs::msg::Pose mirte_arm_target_pose =
        goal->mirte_arm_target_pose;
      /*
      * Collision check
      */

      planning_scene::PlanningScene planning_scene(
        mirte_arm_kinematic_model_);

      bool collision =
        planning_scene.isStateColliding(*mirte_arm_current_state);

      RCLCPP_INFO(
        get_logger(),
        "Collision: %s",
        collision ? "true" : "false");

      /*
      * IK
      */

      auto mirte_arm_target_state = mirte_arm_current_state;

      kinematics::KinematicsQueryOptions options;
      options.return_approximate_solution = false;

      bool found_ik = mirte_arm_target_state->setFromIK(
        mirte_arm_joint_model_group_,
        mirte_arm_target_pose,
        "wrist",
        0.0,
        moveit::core::GroupStateValidityCallbackFn(),
        options 
      );

      RCLCPP_INFO(
        get_logger(),
        "Exact IK solution found: %s, trying approximate IK. Disregard Target joint values.",
        found_ik ? "true" : "false");

      if (!found_ik){
        kinematics::KinematicsQueryOptions options;
        options.return_approximate_solution = true;

        found_ik = mirte_arm_target_state->setFromIK(
          mirte_arm_joint_model_group_,
          mirte_arm_target_pose,
          "wrist",
          0.0,
          moveit::core::GroupStateValidityCallbackFn(),
          options 
        );
        RCLCPP_INFO(
          get_logger(),
          "Approximate IK solution found: %s",
          found_ik ? "true" : "false");
      }

      bool valid =
        mirte_arm_target_state->satisfiesBounds(
          mirte_arm_joint_model_group_);

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

      mirte_arm_target_state->update();
      mirte_arm_target_state->enforceBounds();

      mirte_arm_target_state->copyJointGroupPositions(
        mirte_arm_joint_model_group_,
        mirte_arm_joint_values);

      print_joint_values(mirte_arm_target_state, "TARGET");

      mirte_arm_move_group_->setPositionTarget(mirte_arm_target_pose.position.x, mirte_arm_target_pose.position.y, mirte_arm_target_pose.position.z, "wrist");

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
              mirte_arm_move_group_->plan(msg));

          return std::make_pair(ok, msg);
        }();

      if (!success)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Initial planning failed, increasing tolerances");
        mirte_arm_move_group_->setGoalPositionTolerance(0.001);
        mirte_arm_move_group_->setGoalJointTolerance(0.01);
        print_planner_info();
        
        auto [success, plan] =
          [this]
          {
            moveit::planning_interface::
              MoveGroupInterface::Plan msg;

            bool ok =
              static_cast<bool>(
                mirte_arm_move_group_->plan(msg));

            return std::make_pair(ok, msg);
          }();

        if (!success){
          RCLCPP_ERROR(
            get_logger(),
            "Planning failed");
          goal_handle->abort(result);
          return;
        }
      }
      robot_trajectory::RobotTrajectory rt(
      mirte_arm_kinematic_model_,
      "mirte_arm");

      rt.setRobotTrajectoryMsg(
          *mirte_arm_move_group_->getCurrentState(),
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
      mirte_arm_move_group_->execute(plan);



      /*
      * Final pose
      */

      auto current_pose =
        mirte_arm_move_group_->getCurrentPose("wrist");

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

      mirte_arm_current_state = mirte_arm_move_group_->getCurrentState(10.0);
      mirte_arm_current_state->copyJointGroupPositions(
        mirte_arm_joint_model_group_,
        mirte_arm_joint_values);

      mirte_arm_joint_values[3] = -3.14 - mirte_arm_joint_values[1] - mirte_arm_joint_values[2];
      mirte_arm_move_group_->setJointValueTarget(mirte_arm_joint_values);
      mirte_arm_move_group_->move();


      /*
      * Finish action
      */

      result->success = true;

      goal_handle->succeed(result);

      RCLCPP_INFO(
        get_logger(),
        "Goal completed successfully");
      
      return;
    }

    if (goal->mirte_wrist_joint_target != 0.0){
      RCLCPP_INFO(
        get_logger(),
        "Wrist joint target specified: %f",
        goal->mirte_wrist_joint_target);

      std::vector<double> joint_values;

      mirte_arm_move_group_->getCurrentState()->copyJointGroupPositions(
          mirte_arm_joint_model_group_,
          joint_values);

      joint_values[3] = goal->mirte_wrist_joint_target;

      mirte_arm_move_group_->setJointValueTarget(joint_values);

      mirte_arm_move_group_->move();
    }

    if (goal->mirte_gripper_named_target != "none"){
      RCLCPP_INFO(
        get_logger(),
        "Named target specified: %s",
        goal->mirte_gripper_named_target.c_str());

      mirte_gripper_move_group_->setNamedTarget(
        goal->mirte_gripper_named_target);

      mirte_gripper_move_group_->setStartStateToCurrentState();
      mirte_gripper_move_group_->move();

      result->success = true;
      goal_handle->succeed(result);

      RCLCPP_INFO(
        get_logger(),
        "Goal completed successfully");

      return;
    }

    if (goal->mirte_gripper_joint_target != 0.0){
      RCLCPP_INFO(
        get_logger(),
        "Joint target specified: %f",
        goal->mirte_gripper_joint_target);

      std::vector<double> gripper_joint_values;
      mirte_gripper_move_group_->getCurrentState()->copyJointGroupPositions(
        mirte_gripper_joint_model_group_,
        gripper_joint_values);

      gripper_joint_values[0] = goal->mirte_gripper_joint_target;

      mirte_gripper_move_group_->setJointValueTarget(gripper_joint_values);

      mirte_gripper_move_group_->move();

      result->success = true;
      goal_handle->succeed(result);

      RCLCPP_INFO(
        get_logger(),
        "Goal completed successfully");

      return;
    }
  }
  

  /*
   * Helper functions
   */

  /**
   * @brief Log basic MoveGroup information for debugging.
   */
  void print_move_group_info()
  {
    RCLCPP_INFO(
      get_logger(),
      "Pose Reference Frame: %s",
      mirte_arm_move_group_->getPoseReferenceFrame().c_str());

    RCLCPP_INFO(
      get_logger(),
      "End Effector Link: %s",
      mirte_arm_move_group_->getEndEffectorLink().c_str());
  }

  /**
   * @brief Log currently configured planner tolerances.
   */
  void print_planner_info()
  {
    RCLCPP_INFO(
      get_logger(),
      "Planning frame: %s",
      mirte_arm_move_group_->getPlanningFrame().c_str());

    RCLCPP_INFO(
      get_logger(),
      "Goal position tolerance: %f",
      mirte_arm_move_group_->getGoalPositionTolerance());

    RCLCPP_INFO(
      get_logger(),
      "Goal orientation tolerance: %f",
      mirte_arm_move_group_->getGoalOrientationTolerance());

    RCLCPP_INFO(
      get_logger(),
      "Goal joint tolerance: %f",
      mirte_arm_move_group_->getGoalJointTolerance());
  }

  /**
   * @brief Log joint values for a robot state.
   *
   * @param[in] state Robot state to inspect.
   * @param[in] label Label used in the log output.
   */
  void print_joint_values(
    const moveit::core::RobotStatePtr & state,
    const std::string & label)
  {
    const auto & joint_names =
      mirte_arm_joint_model_group_->getVariableNames();

    std::vector<double> mirte_arm_joint_values;

    state->copyJointGroupPositions(
      mirte_arm_joint_model_group_,
      mirte_arm_joint_values);

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(
        get_logger(),
        "[%s] %s: %f",
        label.c_str(),
        joint_names[i].c_str(),
        mirte_arm_joint_values[i]);
    }
  }
};

}  // namespace mirte_lc_moveit_cpp

#endif  // MIRTE_LC_MOVEIT_CPP__MOVEIT_ACTION_SERVER_HPP_
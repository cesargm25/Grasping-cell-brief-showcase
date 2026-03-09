// goal_pose.cpp
// Plans + executes a pose target with MoveIt2 (UR manipulator group)
// Safe for real robot usage (waits for current state, resizes vectors, executes plan)

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>
#include <stdexcept>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("goal_pose_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

// Class to subscribe to the topic /yolo/detections_3d

class GoalPoseTrajectory {
public:
  explicit GoalPoseTrajectory(const rclcpp::Node::SharedPtr &base_node)
      : base_node_(base_node) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Goal Pose Trajectory...");

    // Configure node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Initialize MoveIt interface node
    move_group_node_ = rclcpp::Node::make_shared("move_group_node", node_options);

    // Spin move_group_node_ in background so MoveIt can receive joint states / TF / action feedback
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // Initialize MoveGroupInterface for the arm
    move_group_robot_ = std::make_shared<MoveGroupInterface>(move_group_node_, PLANNING_GROUP_ROBOT);

    // IMPORTANT -------- >>>>> on real hardware start state monitor
    move_group_robot_->startStateMonitor();

    // Get joint model group
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ROBOT);
    if (!joint_model_group_robot_) {
      RCLCPP_ERROR(LOGGER, "Failed to get JointModelGroup '%s'. Check your SRDF group name.",
                   PLANNING_GROUP_ROBOT.c_str());
      throw std::runtime_error("JointModelGroup not found");
    }

    // Resize joint vector (avoids out-of-range writes later)
    joint_group_positions_robot_.resize(joint_model_group_robot_->getVariableCount());

    // Print system info
    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s", move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    for (size_t i = 0; i < move_group_robot_->getJointModelGroupNames().size(); i++) {
      RCLCPP_INFO(LOGGER, "  Group %zu: %s", i,
                  move_group_robot_->getJointModelGroupNames()[i].c_str());
    }

    // Wait for current robot state
    current_state_robot_ = move_group_robot_->getCurrentState(10.0);
    if (!current_state_robot_) {
      RCLCPP_ERROR(LOGGER, "No current robot state received within timeout.");
      RCLCPP_ERROR(LOGGER, "Check that /joint_states is being published and TF is available.");
      throw std::runtime_error("No current robot state");
    }

    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // Set start state
    move_group_robot_->setStartStateToCurrentState();

    // Safer defaults for real hardware
    move_group_robot_->setMaxVelocityScalingFactor(0.2);
    move_group_robot_->setMaxAccelerationScalingFactor(0.2);
    move_group_robot_->setPlanningTime(10.0);

    RCLCPP_INFO(LOGGER, "Class Initialized: Goal Pose Trajectory");
  }

  ~GoalPoseTrajectory() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Goal Pose Trajectory");
  }

  void plan_and_execute() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Goal Pose Trajectory...");

    // Always refresh start state from real robot before planning
    move_group_robot_->setStartStateToCurrentState();

    // ---- SET YOUR TARGET HERE ----
    // NOTE: Pose target is interpreted in the planning frame unless you set a pose reference frame.
    // You can also call: move_group_robot_->setPoseReferenceFrame("base_link");
    setup_goal_pose_target(
        /*pos_x=*/+0.343f,
        /*pos_y=*/+0.132f,
        /*pos_z=*/+0.33f,
        /*quat_x=*/+1.000f,
        /*quat_y=*/+0.000f,
        /*quat_z=*/+0.000f,
        /*quat_w=*/+0.000f
    );

    RCLCPP_INFO(LOGGER, "Planning...");
    plan_trajectory();

    if (plan_success_) {
      RCLCPP_INFO(LOGGER, "Executing...");
      moveit::core::MoveItErrorCode exec_result = move_group_robot_->execute(trajectory_plan_);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Execution success!");
      } else {
        RCLCPP_ERROR(LOGGER, "Execution failed! (MoveItErrorCode=%d)", exec_result.val);
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed (no IK / collision / constraints / TF mismatch).");
    }
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  const JointModelGroup *joint_model_group_robot_{nullptr};

  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan trajectory_plan_;
  Pose target_pose_;
  bool plan_success_{false}; 

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    target_pose_.position.x = pos_x;
    target_pose_.position.y = pos_y;
    target_pose_.position.z = pos_z;
    target_pose_.orientation.x = quat_x;
    target_pose_.orientation.y = quat_y;
    target_pose_.orientation.z = quat_z;
    target_pose_.orientation.w = quat_w;

    // Set pose target for the end effector link defined in your MoveIt config
    move_group_robot_->setPoseTarget(target_pose_);
  }

  void plan_trajectory() {
    plan_success_ =
        (move_group_robot_->plan(trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto base_node = std::make_shared<rclcpp::Node>("goal_pose_trajectory");

  try {
    GoalPoseTrajectory node(base_node);
    node.plan_and_execute();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}


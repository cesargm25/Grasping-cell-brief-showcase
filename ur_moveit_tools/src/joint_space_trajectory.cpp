#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");

static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class JointSpaceTrajectory {
public:
  JointSpaceTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Joint Space Trajectory...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);

    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // ---------------- ARM MoveGroupInterface ----------------
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);

    move_group_robot_->startStateMonitor();

  
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);

    // IMPORTANT: size the joint vector before indexing
    joint_group_positions_robot_.resize(
        joint_model_group_robot_->getVariableCount());

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (size_t i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %zu: %s", i, group_names[i].c_str());
    }

    // get current state of robot (wait up to 10s)
    current_state_robot_ = move_group_robot_->getCurrentState(10.0);
    if (!current_state_robot_) {
      RCLCPP_ERROR(LOGGER,
                   "No current robot state received. Check /joint_states and TF.");
      throw std::runtime_error("No current state");
    }
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();

    // optional: safer execution on real HW
    move_group_robot_->setMaxVelocityScalingFactor(0.2);
    move_group_robot_->setMaxAccelerationScalingFactor(0.2);
    move_group_robot_->setPlanningTime(5.0);

    
    RCLCPP_INFO(LOGGER, "Class Initialized: Joint Space Trajectory");
  }

  ~JointSpaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Joint Space Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Joint Space Trajectory...");

    // ---------------- ARM MOTION ----------------
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory (ARM)...");
    setup_joint_value_target_arm(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
                                 +0.0000);

    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory (ARM)...");
    plan_trajectory_arm();

    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory (ARM)...");
    execute_trajectory_arm();
    // --------------------------- NO USED FOR NOW mantener para despues -----------------

    // ---------------- GRIPPER MOTION (COMMENTED OUT) ----------------
    // Example structure if you later want to open/close the gripper via MoveIt.
    //
    // RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory (GRIPPER)...");
    // setup_joint_value_target_gripper(0.0);  // e.g. open
    //
    // RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory (GRIPPER)...");
    // plan_trajectory_gripper();
    //
    // RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory (GRIPPER)...");
    // execute_trajectory_gripper();
    // ----------------------------------------------------------------

    RCLCPP_INFO(LOGGER, "Joint Space Trajectory Execution Complete");
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // ---------------- ARM VARIABLES ----------------
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  const JointModelGroup *joint_model_group_robot_{nullptr};

  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_robot_;
  bool plan_success_robot_ = false;

  void setup_joint_value_target_arm(float a0, float a1, float a2,
                                    float a3, float a4, float a5) {
    joint_group_positions_robot_[0] = a0; // Shoulder Pan
    joint_group_positions_robot_[1] = a1; // Shoulder Lift
    joint_group_positions_robot_[2] = a2; // Elbow
    joint_group_positions_robot_[3] = a3; // Wrist 1
    joint_group_positions_robot_[4] = a4; // Wrist 2
    joint_group_positions_robot_[5] = a5; // Wrist 3

    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void plan_trajectory_arm() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_robot_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_arm() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_robot_);
      RCLCPP_INFO(LOGGER, "ARM trajectory success!");
    } else {
      RCLCPP_ERROR(LOGGER, "ARM trajectory planning failed!");
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto base_node = std::make_shared<rclcpp::Node>("joint_space_trajectory");

  try {
    JointSpaceTrajectory joint_space_trajectory_node(base_node);
    joint_space_trajectory_node.execute_trajectory_plan();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}


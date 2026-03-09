#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <mutex>
#include <atomic>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cam_end_pose");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string GOAL_TOPIC = "/cup_goal_pose";
static const std::string REF_FRAME = "base_link";

class CamEndPoseNode : public rclcpp::Node
{
public:
  CamEndPoseNode() : Node("cam_end_pose_node")
  {
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      GOAL_TOPIC, rclcpp::QoS(10),
      std::bind(&CamEndPoseNode::goalCb, this, std::placeholders::_1));

    RCLCPP_INFO(LOGGER, "cam_end_pose_node started. Waiting for %s ...", GOAL_TOPIC.c_str());
  }

  void init_moveit()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP_ROBOT);

    move_group_->startStateMonitor();
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    move_group_->setPlanningTime(5.0);

    move_group_->setPoseReferenceFrame(REF_FRAME);

    const std::string grasp_link = "grasp_dot";
    move_group_->setEndEffectorLink(grasp_link);

    RCLCPP_INFO(LOGGER, "MoveIt initialized.");
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "Pose reference frame: %s", REF_FRAME.c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_->getEndEffectorLink().c_str());

    moveToInitialJointPose();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::mutex exec_mtx_;
  std::atomic_bool busy_{false};

  bool moveToInitialJointPose()
  {
    if (!move_group_) {
      RCLCPP_ERROR(LOGGER, "move_group_ is null. Did you call init_moveit()?");
      return false;
    }

    std::lock_guard<std::mutex> lk(exec_mtx_);
    busy_.store(true);

    struct BusyReset {
      std::atomic_bool &b;
      ~BusyReset() { b.store(false); }
    } busy_reset{busy_};

    RCLCPP_INFO(LOGGER, "Moving to initial joint pose...");

    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();

    std::vector<std::string> joint_names = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    std::vector<double> joint_values = {1.459, -2.087, 0.957, -1.31, -1.43, 0.0}; // this can be ofc changed, but this configuration can see all the objects on the table

    const auto group_joints = move_group_->getJointNames();
    for (const auto& j : joint_names) {
      if (std::find(group_joints.begin(), group_joints.end(), j) == group_joints.end()) {
        RCLCPP_ERROR(LOGGER, "Joint '%s' not found in MoveIt group '%s'.",
                     j.c_str(), PLANNING_GROUP_ROBOT.c_str());
        return false;
      }
    }

    move_group_->setJointValueTarget(joint_names, joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Initial joint pose planning failed.");
      return false;
    }

    RCLCPP_INFO(LOGGER, "Initial pose planning success. Executing...");
    auto exec_result = move_group_->execute(plan);

    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Manipulator in Initial Position!");

      // Give joint_states a moment to update after execution (MoveIt2 Humble: no waitForCurrentState())
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      move_group_->setStartStateToCurrentState();

      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Initial joint pose execution FAILED (code=%d).", exec_result.val);
      return false;
    }
  }

  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(LOGGER, "MoveIt not initialized yet (init_moveit() not called). Ignoring goal.");
      return;
    }

    if (busy_.exchange(true)) {
      RCLCPP_WARN_THROTTLE(LOGGER, *this->get_clock(), 2000, "Busy executing. Ignoring new goal.");
      return;
    }

    std::lock_guard<std::mutex> lk(exec_mtx_);
    struct BusyReset {
      std::atomic_bool &b;
      ~BusyReset() { b.store(false); }
    } busy_reset{busy_};

    RCLCPP_INFO(LOGGER, "Received goal pose in frame '%s': x=%.3f y=%.3f z=%.3f",
      msg->header.frame_id.c_str(),
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();

    const std::string ee = move_group_->getEndEffectorLink();
    if (ee.empty()) move_group_->setPoseTarget(*msg);
    else move_group_->setPoseTarget(*msg, ee);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Planning failed.");
      return;
    }

    RCLCPP_INFO(LOGGER, "Planning success. Executing...");
    auto exec_result = move_group_->execute(plan);

    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Execution SUCCESS.");
    } else {
      RCLCPP_ERROR(LOGGER, "Execution FAILED (code=%d).", exec_result.val);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamEndPoseNode>();
  node->init_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


#include <a6_y_interfaces/action/move_end_effector.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace {
using MoveEndEffector = a6_y_interfaces::action::MoveEndEffector;
using GoalHandleMoveEndEffector = rclcpp_action::ServerGoalHandle<MoveEndEffector>;

constexpr const char *ACTION_NAME = "/a6_y/move_end_effector";
constexpr const char *GROUP_NAME = "arm";
constexpr const char *TIP_LINK = "link6.";
constexpr const char *WORLD_FRAME = "world";
constexpr double DEFAULT_VELOCITY_SCALING = 1.0;
constexpr double DEFAULT_ACCELERATION_SCALING = 0.4;
constexpr double DEFAULT_PLANNING_TIME = 5.0;

enum ErrorCode { ERROR_NONE = 0, ERROR_INVALID_GOAL = 1, ERROR_PLANNING_FAILED = 2, ERROR_EXECUTION_FAILED = 3, ERROR_CANCELED = 4, ERROR_IK_FAILED = 5, ERROR_EXCEPTION = 6 };

struct Target { double x; double y; double z; double roll; double pitch; double yaw; };

bool isFinite(double value) { return std::isfinite(value); }
bool isFiniteTarget(const Target &target) { return isFinite(target.x) && isFinite(target.y) && isFinite(target.z) && isFinite(target.roll) && isFinite(target.pitch) && isFinite(target.yaw); }
bool isFinitePosition(const Target &target) { return isFinite(target.x) && isFinite(target.y) && isFinite(target.z); }

geometry_msgs::msg::Pose toPose(const Target &target) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(target.roll, target.pitch, target.yaw);
  quaternion.normalize();
  geometry_msgs::msg::Pose pose;
  pose.position.x = target.x;
  pose.position.y = target.y;
  pose.position.z = target.z;
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

std::string targetToString(const Target &target, bool position_only) {
  std::ostringstream out;
  out << "xyz=(" << target.x << ", " << target.y << ", " << target.z << ")";
  if (!position_only) {
    out << " rpy=(" << target.roll << ", " << target.pitch << ", " << target.yaw << ")";
  }
  return out.str();
}

std::string poseToXyzRpyString(const geometry_msgs::msg::Pose &pose) {
  tf2::Quaternion quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  std::ostringstream out;
  out << "xyz=(" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ") rpy=(" << roll << ", " << pitch << ", " << yaw << ")";
  return out.str();
}
} // namespace

class EndEffectorControlServer : public rclcpp::Node {
public:
  explicit EndEffectorControlServer(const rclcpp::NodeOptions &options) : rclcpp::Node("a6_y_end_effector_control_server", options) {}

  void initialize() {
    action_server_ = rclcpp_action::create_server<MoveEndEffector>(
        shared_from_this(), ACTION_NAME,
        std::bind(&EndEffectorControlServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&EndEffectorControlServer::handleCancel, this, std::placeholders::_1),
        std::bind(&EndEffectorControlServer::handleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "A6_Y end-effector action server is ready on %s", ACTION_NAME);
  }

private:
  rclcpp_action::Server<MoveEndEffector>::SharedPtr action_server_;
  std::mutex active_mutex_;
  bool active_goal_{false};

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveEndEffector::Goal> goal) {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "Rejecting goal because another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::string reason;
    if (!validateGoal(*goal, reason)) {
      RCLCPP_WARN(get_logger(), "Rejecting invalid goal: %s", reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    active_goal_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMoveEndEffector>) {
    RCLCPP_INFO(get_logger(), "Cancel requested for active end-effector goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleMoveEndEffector> goal_handle) {
    std::thread{std::bind(&EndEffectorControlServer::executeGoal, this, std::placeholders::_1), goal_handle}.detach();
  }

  void executeGoal(const std::shared_ptr<GoalHandleMoveEndEffector> goal_handle) {
    auto result = std::make_shared<MoveEndEffector::Result>();
    try {
      const auto goal = goal_handle->get_goal();
      const Target target{goal->x, goal->y, goal->z, goal->roll, goal->pitch, goal->yaw};
      publishFeedback(goal_handle, "validating", 0.05, "Goal accepted");
      RCLCPP_INFO(get_logger(), "Planning %s target for %s: %s", goal->position_only ? "position-only" : "pose", TIP_LINK, targetToString(target, goal->position_only).c_str());

      auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), GROUP_NAME);
      configureMoveGroup(move_group, *goal);
      move_group.setPoseReferenceFrame(WORLD_FRAME);
      move_group.setStartStateToCurrentState();
      move_group.clearPoseTargets();
      if (!move_group.setEndEffectorLink(TIP_LINK)) {
        abortGoal(goal_handle, result, ERROR_INVALID_GOAL, std::string("Failed to set end-effector link: ") + TIP_LINK);
        finishGoal();
        return;
      }
      const auto current_pose = move_group.getCurrentPose(TIP_LINK);
      RCLCPP_INFO(get_logger(), "Current %s pose in %s: %s", TIP_LINK, current_pose.header.frame_id.c_str(), poseToXyzRpyString(current_pose.pose).c_str());

      if (goal->position_only) {
        if (!move_group.setPositionTarget(target.x, target.y, target.z, TIP_LINK)) {
          abortGoal(goal_handle, result, ERROR_INVALID_GOAL, "Failed to set position target");
          finishGoal();
          return;
        }
      } else if (!move_group.setJointValueTarget(toPose(target), TIP_LINK)) {
        abortGoal(goal_handle, result, ERROR_IK_FAILED, "IK failed for A6_Y target");
        finishGoal();
        return;
      }

      publishFeedback(goal_handle, "planning", 0.25, "Planning arm");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_code = move_group.plan(plan);
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
        abortGoal(goal_handle, result, ERROR_PLANNING_FAILED, "MoveIt planning failed for arm");
        finishGoal();
        return;
      }
      if (!goal->execute) {
        succeedGoal(goal_handle, result, "Planning succeeded for arm");
        finishGoal();
        return;
      }
      executePlan(goal_handle, move_group, plan, result);
    } catch (const std::exception &ex) {
      result->success = false;
      result->error_code = ERROR_EXCEPTION;
      result->message = ex.what();
      RCLCPP_ERROR(get_logger(), "Goal failed with exception: %s", ex.what());
      if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }
    finishGoal();
  }

  void finishGoal() {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_goal_ = false;
  }

  void configureMoveGroup(moveit::planning_interface::MoveGroupInterface &move_group, const MoveEndEffector::Goal &goal) {
    move_group.setPlanningTime(goal.planning_time == 0.0 ? DEFAULT_PLANNING_TIME : goal.planning_time);
    move_group.setMaxVelocityScalingFactor(goal.velocity_scaling == 0.0 ? DEFAULT_VELOCITY_SCALING : goal.velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(goal.acceleration_scaling == 0.0 ? DEFAULT_ACCELERATION_SCALING : goal.acceleration_scaling);
  }

  void executePlan(const std::shared_ptr<GoalHandleMoveEndEffector> &goal_handle, moveit::planning_interface::MoveGroupInterface &move_group, const moveit::planning_interface::MoveGroupInterface::Plan &plan, std::shared_ptr<MoveEndEffector::Result> &result) {
    if (goal_handle->is_canceling()) {
      cancelGoal(goal_handle, result, "Goal canceled before execution");
      return;
    }
    publishFeedback(goal_handle, "executing", 0.7, "Executing trajectory");
    std::atomic_bool execution_done{false};
    std::thread cancel_monitor([&]() {
      while (rclcpp::ok() && !execution_done.load()) {
        if (goal_handle->is_canceling()) {
          move_group.stop();
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
    const auto execute_code = move_group.execute(plan);
    execution_done.store(true);
    cancel_monitor.join();
    if (goal_handle->is_canceling()) {
      cancelGoal(goal_handle, result, "Goal canceled during execution");
      return;
    }
    if (execute_code != moveit::core::MoveItErrorCode::SUCCESS) {
      abortGoal(goal_handle, result, ERROR_EXECUTION_FAILED, "MoveIt execution failed");
      return;
    }
    succeedGoal(goal_handle, result, "Execution finished for arm");
  }

  bool validateGoal(const MoveEndEffector::Goal &goal, std::string &reason) const {
    const Target target{goal.x, goal.y, goal.z, goal.roll, goal.pitch, goal.yaw};
    if (goal.position_only ? !isFinitePosition(target) : !isFiniteTarget(target)) {
      reason = "target contains NaN or Inf";
      return false;
    }
    if (!isValidScaling(goal.velocity_scaling) || !isValidScaling(goal.acceleration_scaling)) {
      reason = "velocity_scaling and acceleration_scaling must be 0.0 or within [0.01, 1.0]";
      return false;
    }
    if (goal.planning_time < 0.0 || !isFinite(goal.planning_time)) {
      reason = "planning_time must be finite and non-negative";
      return false;
    }
    return true;
  }

  static bool isValidScaling(double value) { return isFinite(value) && (value == 0.0 || (value >= 0.01 && value <= 1.0)); }

  void publishFeedback(const std::shared_ptr<GoalHandleMoveEndEffector> &goal_handle, const std::string &stage, double progress, const std::string &message) {
    auto feedback = std::make_shared<MoveEndEffector::Feedback>();
    feedback->stage = stage;
    feedback->progress = progress;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void succeedGoal(const std::shared_ptr<GoalHandleMoveEndEffector> &goal_handle, std::shared_ptr<MoveEndEffector::Result> &result, const std::string &message) {
    publishFeedback(goal_handle, "done", 1.0, message);
    result->success = true;
    result->error_code = ERROR_NONE;
    result->message = message;
    if (goal_handle->is_active()) goal_handle->succeed(result);
  }
  void abortGoal(const std::shared_ptr<GoalHandleMoveEndEffector> &goal_handle, std::shared_ptr<MoveEndEffector::Result> &result, int32_t error_code, const std::string &message) {
    publishFeedback(goal_handle, "failed", 1.0, message);
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    if (goal_handle->is_active()) goal_handle->abort(result);
  }
  void cancelGoal(const std::shared_ptr<GoalHandleMoveEndEffector> &goal_handle, std::shared_ptr<MoveEndEffector::Result> &result, const std::string &message) {
    result->success = false;
    result->error_code = ERROR_CANCELED;
    result->message = message;
    if (goal_handle->is_active()) goal_handle->canceled(result);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<EndEffectorControlServer>(options);
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

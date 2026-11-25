#include <a6_y_controllers/A6_y_cartesian_impedance_controller.hpp>
#include <a6_y_controllers/a5_model.hpp>
#include <a6_y_controllers/safe_zero_effort_command.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <Eigen/Cholesky>
#include <pinocchio/spatial/explog.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace a6_y_controllers
{
namespace
{
const std::vector<std::string> kJoints{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_finger_joint1"};
const std::array<std::string, 4> kCommands{"position", "velocity", "effort", "motor_enable"};
const std::array<std::string, 6> kStates{
  "position", "velocity", "effort", "position_valid", "position_age", "feedback_age"};
template<std::size_t N>
std::vector<std::string> names(const std::vector<std::string>& joints,
                               const std::array<std::string, N>& interfaces)
{
  std::vector<std::string> result;
  for (const auto& joint : joints)
    for (const auto& interface : interfaces) result.push_back(joint + "/" + interface);
  return result;
}
}  // namespace

controller_interface::CallbackReturn A6YCartesianImpedanceController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", kJoints);
  auto_declare<std::string>("urdf_path", "");
  auto_declare<std::string>("tcp_frame", "tool_tcp");
  auto_declare<bool>("enable_output", false);
  auto_declare<double>("impedance_ramp_duration", 2.0);
  auto_declare<double>("max_control_period", 0.1);
  auto_declare<double>("max_feedback_age", 0.1);
  auto_declare<std::vector<double>>("torque_scales", {1, 1, 1, 1, 1, 1, 0});
  auto_declare<std::vector<double>>("effort_signs", {1, 1, 1, 1, 1, 1, 1});
  auto_declare<std::vector<double>>("damping_gains", {0.01, 0.01, 0.01, 0.01, 0.01, 0.03, 0});
  auto_declare<std::vector<double>>("torque_limits", {10, 20, 10, 3, 4, 4, 30});
  // XYZ in base axes, then rotation about base axes. Conservative trial gains, not tuned values.
  auto_declare<std::vector<double>>("cartesian_stiffness", {50, 50, 50, 2, 2, 2});
  auto_declare<std::vector<double>>("cartesian_damping", {8, 8, 8, 0.3, 0.3, 0.3});
  auto_declare<std::vector<double>>("wrench_limits", {5, 5, 5, 0.5, 0.5, 0.5});
  auto_declare<std::vector<double>>("impedance_torque_limits", {0.8, 0.8, 0.8, 0.3, 0.3, 0.3});
  // Legacy configuration is accepted, but delayed output slew is no longer used.
  auto_declare<std::vector<double>>("impedance_torque_rates", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
A6YCartesianImpedanceController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names(joints_, kCommands)};
}
controller_interface::InterfaceConfiguration
A6YCartesianImpedanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names(joints_, kStates)};
}

Eigen::VectorXd A6YCartesianImpedanceController::vector_parameter(
    const std::string& name, int size, bool strictly_positive)
{
  const auto values = get_node()->get_parameter(name).as_double_array();
  if (values.size() != static_cast<std::size_t>(size))
    throw std::runtime_error(name + ": wrong number of entries");
  Eigen::VectorXd result(size);
  for (int i = 0; i < size; ++i)
  {
    if (!std::isfinite(values[i]) || values[i] < 0 || (strictly_positive && values[i] == 0))
      throw std::runtime_error(name + ": entries must be finite and " +
                               (strictly_positive ? "positive" : "nonnegative"));
    result[i] = values[i];
  }
  return result;
}

controller_interface::CallbackReturn A6YCartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State&)
{
  active_ = captured_ = faulted_ = false;
  try
  {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    if (joints_ != kJoints) throw std::runtime_error("Expected six A5 joints followed by gripper_finger_joint1");
    enable_output_ = get_node()->get_parameter("enable_output").as_bool();
    ramp_duration_ = get_node()->get_parameter("impedance_ramp_duration").as_double();
    max_period_ = get_node()->get_parameter("max_control_period").as_double();
    max_feedback_age_ = get_node()->get_parameter("max_feedback_age").as_double();
    for (double value : {ramp_duration_, max_period_, max_feedback_age_})
      if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Durations must be finite and positive");
    gravity_scales_ = vector_parameter("torque_scales", 7, false);
    joint_damping_ = vector_parameter("damping_gains", 7, false);
    torque_limits_ = vector_parameter("torque_limits", 7, true);
    stiffness_ = vector_parameter("cartesian_stiffness", 6, false);
    damping_ = vector_parameter("cartesian_damping", 6, false);
    wrench_limits_ = vector_parameter("wrench_limits", 6, true);
    impedance_limits_ = vector_parameter("impedance_torque_limits", 6, true);
    if (!get_node()->get_parameter("impedance_torque_rates").as_double_array().empty())
      RCLCPP_WARN(get_node()->get_logger(),
        "impedance_torque_rates is obsolete and ignored: output-history slew delays damping reversal.");
    const auto signs = get_node()->get_parameter("effort_signs").as_double_array();
    if (signs.size() != 7) throw std::runtime_error("effort_signs needs seven entries");
    gravity_signs_.resize(7);
    for (int i = 0; i < 7; ++i)
    {
      if (signs[i] != 1.0 && signs[i] != -1.0) throw std::runtime_error("effort_signs must be +1 or -1");
      gravity_signs_[i] = signs[i];
    }
    if (gravity_scales_[6] != 0 || joint_damping_[6] != 0)
      throw std::runtime_error("Gripper gravity scale and damping must be zero in this mode");
    if ((impedance_limits_.array() > torque_limits_.head<6>().array()).any())
      throw std::runtime_error("Incremental impedance limits exceed total torque limits");

    auto urdf = get_node()->get_parameter("urdf_path").as_string();
    if (urdf.empty()) urdf = ament_index_cpp::get_package_share_directory("a5_description") + "/urdf/A5.urdf";
    model_ = std::make_unique<pinocchio::Model>();
    load_mimic_model(urdf, *model_);
    map_independent_joints(*model_, joints_, q_indices_, v_indices_);
    const auto tcp = get_node()->get_parameter("tcp_frame").as_string();
    if (!model_->existFrame(tcp)) throw std::runtime_error("Unknown TCP frame: " + tcp);
    tcp_id_ = model_->getFrameId(tcp);
    // Holding a finger frame would unintentionally couple the task to gripper opening.
    if (model_->frames[tcp_id_].parentJoint != model_->getJointId("joint6"))
      throw std::runtime_error("TCP must be fixed to link6, not to a moving finger");
    data_ = std::make_unique<pinocchio::Data>(*model_);
    q_ = Eigen::VectorXd::Zero(model_->nq);
    dq_ = Eigen::VectorXd::Zero(model_->nv);
    jacobian_.setZero(6, model_->nv);
    elapsed_ = 0;
    RCLCPP_INFO(get_node()->get_logger(),
      "Implicit TCP impedance: Kxyz=[%.1f %.1f %.1f], Dxyz=[%.1f %.1f %.1f]; no output-history slew.",
      stiffness_[0], stiffness_[1], stiffness_[2], damping_[0], damping_[1], damping_[2]);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Impedance configuration: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool A6YCartesianImpedanceController::interfaces_match() const
{
  if (command_interfaces_.size() != 28 || state_interfaces_.size() != 42) return false;
  for (std::size_t i = 0; i < 7; ++i)
  {
    for (std::size_t j = 0; j < kCommands.size(); ++j)
      if (command_interfaces_[4 * i + j].get_name() != joints_[i] + "/" + kCommands[j]) return false;
    for (std::size_t j = 0; j < kStates.size(); ++j)
      if (state_interfaces_[6 * i + j].get_name() != joints_[i] + "/" + kStates[j]) return false;
  }
  return true;
}

void A6YCartesianImpedanceController::write_disabled()
{
  // Also handles incomplete/reordered interface assignments without writing a position as a torque.
  for (auto& command : command_interfaces_)
  {
    if (command.get_interface_name() == "position")
    {
      double measured = std::numeric_limits<double>::quiet_NaN();
      for (const auto& state : state_interfaces_)
        if (state.get_name() == command.get_name()) measured = state.get_value();
      command.set_value(resolve_safe_zero_effort_command(measured, command.get_value()).position);
    }
    else command.set_value(0.0);
  }
}

controller_interface::CallbackReturn A6YCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State&)
{
  write_disabled();
  if (!model_ || !data_ || !interfaces_match()) return controller_interface::CallbackReturn::ERROR;
  captured_ = faulted_ = false;
  elapsed_ = 0;
  active_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn A6YCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
  active_ = captured_ = false;
  write_disabled();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type A6YCartesianImpedanceController::fail(const char* reason)
{
  faulted_ = true;
  write_disabled();
  RCLCPP_ERROR(get_node()->get_logger(), "Impedance disabled: %s", reason);
  return controller_interface::return_type::ERROR;
}

controller_interface::return_type A6YCartesianImpedanceController::update(
    const rclcpp::Time&, const rclcpp::Duration& period)
{
  if (!active_ || faulted_) { write_disabled(); return controller_interface::return_type::ERROR; }
  if (!interfaces_match()) return fail("interface assignment changed");
  if (!enable_output_) { write_disabled(); return controller_interface::return_type::OK; }
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0 || dt > max_period_) return fail("invalid or excessive control period");
  for (int i = 0; i < 7; ++i)
  {
    const double position = state_interfaces_[6 * i].get_value();
    const double velocity = state_interfaces_[6 * i + 1].get_value();
    const double valid = state_interfaces_[6 * i + 3].get_value();
    const double position_age = state_interfaces_[6 * i + 4].get_value();
    const double feedback_age = state_interfaces_[6 * i + 5].get_value();
    if (!std::isfinite(position) || !std::isfinite(velocity) || valid != 1.0 ||
        !std::isfinite(position_age) || position_age < 0 || position_age > max_feedback_age_ ||
        !std::isfinite(feedback_age) || feedback_age < 0 || feedback_age > max_feedback_age_)
      return fail("invalid or stale joint feedback");
    q_[q_indices_[i]] = position;
    dq_[v_indices_[i]] = velocity;
  }

  try
  {
    pinocchio::computeJointJacobians(*model_, *data_, q_);
    pinocchio::updateFramePlacements(*model_, *data_);
    jacobian_.setZero();
    // Velocity and error are both at the TCP origin, expressed in base axes.
    pinocchio::getFrameJacobian(*model_, *data_, tcp_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);
    const auto& pose = data_->oMf[tcp_id_];
    if (!pose.toHomogeneousMatrix().allFinite() || !jacobian_.allFinite()) return fail("invalid TCP kinematics");
    if (!captured_)
    {
      target_ = pose;
      captured_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Captured fixed TCP target at [%.4f, %.4f, %.4f] m; orientation captured too.",
                  target_.translation().x(), target_.translation().y(), target_.translation().z());
    }
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target_.translation() - pose.translation();
    // Shortest rotation, avoiding RPY wrap-around at +/- pi.
    const Eigen::Matrix3d rotation_error = target_.rotation() * pose.rotation().transpose();
    error.tail<3>() = pinocchio::log3(rotation_error);
    const double ramp = std::min(1.0, elapsed_ / ramp_duration_);
    elapsed_ = std::min(ramp_duration_, elapsed_ + dt);
    using Matrix6 = Eigen::Matrix<double, 6, 6>;
    using Vector6 = Eigen::Matrix<double, 6, 1>;
    Matrix6 J;
    Vector6 velocity;
    for (int i = 0; i < 6; ++i)
    {
      J.col(i) = jacobian_.col(v_indices_[i]);
      velocity[i] = dq_[v_indices_[i]];
    }
    // Implicit one-step spring/damper prediction. In particular, the damping
    // must not go through an output-history slew limiter after a velocity reversal.
    pinocchio::crba(*model_, *data_, q_);
    data_->M.template triangularView<Eigen::StrictlyLower>() =
      data_->M.transpose().template triangularView<Eigen::StrictlyLower>();
    Matrix6 mass;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) mass(i, j) = data_->M(v_indices_[i], v_indices_[j]);
    const Matrix6 K = (ramp * stiffness_).asDiagonal();
    const Matrix6 D = damping_.asDiagonal();
    const Matrix6 joint_D = joint_damping_.head<6>().asDiagonal();
    const Matrix6 joint_K = J.transpose() * K * J;
    const Matrix6 total_D = J.transpose() * D * J + joint_D;
    if (!mass.allFinite()) return fail("nonfinite joint inertia");
    Eigen::LDLT<Matrix6> mass_solver(mass);
    if (mass_solver.info() != Eigen::Success || mass_solver.vectorD().minCoeff() <= 0)
      return fail("invalid joint inertia");
    const Matrix6 step_matrix = mass + dt * total_D + dt * dt * joint_K;
    if (!step_matrix.allFinite()) return fail("nonfinite discrete impedance matrix");
    Eigen::LDLT<Matrix6> step_solver(step_matrix);
    if (step_solver.info() != Eigen::Success || step_solver.vectorD().minCoeff() <= 0)
      return fail("invalid discrete impedance solve");
    const Vector6 acceleration = step_solver.solve(
      J.transpose() * K * error - (total_D + dt * joint_K) * velocity);
    const Vector6 predicted_velocity = velocity + dt * acceleration;
    const Vector6 predicted_twist = J * predicted_velocity;
    const Vector6 wrench = K * (error - dt * predicted_twist) - D * predicted_twist;
    const Vector6 task_torque = J.transpose() * wrench - joint_D * predicted_velocity;
    if (!wrench.allFinite() || !task_torque.allFinite()) return fail("nonfinite task torque");
    pinocchio::computeGeneralizedGravity(*model_, *data_, q_);
    if (!data_->g.allFinite()) return fail("nonfinite gravity compensation");
    Eigen::Matrix<double, 7, 1> commands = Eigen::Matrix<double, 7, 1>::Zero();
    // Scale the complete task vector uniformly. Component-wise clipping can
    // change its direction and destroy dissipativity of the coupled damper.
    double task_scale = 1.0;
    const std::array<const char*, 6> wrench_axes{"X", "Y", "Z", "Rx", "Ry", "Rz"};
    const char* limit_kind = "none";
    const char* limit_axis = "none";
    double limit_requested = 0.0, limit_available = 0.0;
    // Record the active minimum without changing the existing saturation rule.
    const auto note_limit = [&](double magnitude, double available,
                                const char* kind, const char* axis) {
      const double ratio = available / magnitude;
      if (ratio < task_scale)
      {
        limit_kind = kind;
        limit_axis = axis;
        limit_requested = magnitude;
        limit_available = available;
      }
      task_scale = std::min(task_scale, ratio);
    };
    for (int i = 0; i < 6; ++i)
    {
      if (std::abs(wrench[i]) > wrench_limits_[i])
        note_limit(std::abs(wrench[i]), wrench_limits_[i], "wrench", wrench_axes[i]);
      if (std::abs(task_torque[i]) > impedance_limits_[i])
        note_limit(std::abs(task_torque[i]), impedance_limits_[i], "joint_task", joints_[i].c_str());
      const double gravity = gravity_signs_[i] * gravity_scales_[i] * data_->g[v_indices_[i]];
      if (!std::isfinite(gravity)) return fail("nonfinite joint gravity");
      commands[i] = std::clamp(gravity, -torque_limits_[i], torque_limits_[i]);
      const double room = task_torque[i] >= 0 ? torque_limits_[i] - commands[i] : torque_limits_[i] + commands[i];
      if (std::abs(task_torque[i]) > room)
        note_limit(std::abs(task_torque[i]), room, "joint_total", joints_[i].c_str());
    }
    for (int i = 0; i < 6; ++i)
      commands[i] = std::clamp(commands[i] + task_scale * task_torque[i], -torque_limits_[i], torque_limits_[i]);
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Impedance status: xyz_error_mm=[%.2f %.2f %.2f], rotation_error_deg=%.2f, "
      "joint_speed_norm=%.3f rad/s, task_scale=%.3f, period_ms=%.2f, "
      "limit=%s:%s, limit_requested_abs=%.3f, limit_available=%.3f, "
      "task_Fz_requested_N=%.3f, task_Fz_scaled_N=%.3f",
      1000 * error[0], 1000 * error[1], 1000 * error[2],
      error.tail<3>().norm() * 180.0 / std::acos(-1.0), velocity.norm(), task_scale, 1000 * dt,
      limit_kind, limit_axis, limit_requested, limit_available, wrench[2], task_scale * wrench[2]);
    // Complete finite batch before motor enable. Gripper contributes mass but receives zero effort.
    for (int i = 0; i < 7; ++i)
    {
      command_interfaces_[4 * i].set_value(q_[q_indices_[i]]);
      command_interfaces_[4 * i + 1].set_value(0.0);
      command_interfaces_[4 * i + 2].set_value(commands[i]);
      command_interfaces_[4 * i + 3].set_value(1.0);
    }
  }
  catch (const std::exception& error) { return fail(error.what()); }
  return controller_interface::return_type::OK;
}
}  // namespace a6_y_controllers

PLUGINLIB_EXPORT_CLASS(a6_y_controllers::A6YCartesianImpedanceController,
                      controller_interface::ControllerInterface)

#include <a6_y_ros2_control/A6_y_damiao_system.hpp>
#include <a6_y_ros2_control/motor_command_guard.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("a6_y_ros2_control.A6YDamiaoSystem");
// Allow a small calibration/noise margin at mechanical boundaries, then clamp
// the outgoing command back into the configured soft limits before encoding it.
constexpr double kSoftLimitTolerance = 1e-1;
constexpr double kInitialFeedbackPerRequestDrain = 0.01;
constexpr double kInitialFeedbackPassDrain = 0.02;
constexpr int kDisableCommandPasses = 3;
constexpr auto kDisablePassDelay = std::chrono::milliseconds(20);

struct DurationMeasurement {
  double& result;
  std::chrono::steady_clock::time_point start{std::chrono::steady_clock::now()};
  ~DurationMeasurement() { result=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(); }
};

}

namespace a6_y_ros2_control
{
A6YDamiaoSystem::~A6YDamiaoSystem()
{
  if (!dry_run_)
  {
    deactivate_real_hardware();
  }
}

// Opt-in bridge for stock position controllers and the existing MIT gravity
// controller. No trajectory or following calculation belongs in this adapter.
std::set<std::string> A6YDamiaoSystem::next_claims(
    const std::vector<std::string>& start, const std::vector<std::string>& stop) const
{
  auto claims = controller_claims_;
  for (const auto& key : stop) claims.erase(key);
  for (const auto& key : start) {
    const auto slash = key.find('/');
    if (joint_index_by_name_.count(key.substr(0, slash))) claims.insert(key);
  }
  return claims;
}

int A6YDamiaoSystem::claimed_mode(const std::set<std::string>& claims) const
{
  int mode = -2;
  for (const auto& joint : joints_) {
    if (joint.read_only) continue;
    const auto prefix = joint.name + "/";
    const bool p = claims.count(prefix + "position");
    const bool v = claims.count(prefix + "velocity");
    const bool e = claims.count(prefix + "effort");
    const bool enable = claims.count(prefix + "motor_enable");
    const int current = !p && !v && !e && !enable ? 0 :
                        p && !v && !e && !enable ? 2 :
                        p && v && e && enable ? 1 : -1;
    if (current < 0 || (mode != -2 && mode != current)) return -1;
    mode = current;
  }
  return mode == -2 ? 0 : mode;
}

hardware_interface::return_type A6YDamiaoSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start, const std::vector<std::string>& stop)
{
  if (!controller_mode_switching_) return hardware_interface::return_type::OK;
  if (mode_switch_failed_ || claimed_mode(next_claims(start, stop)) < 0) {
    RCLCPP_ERROR(LOGGER, "Refusing incomplete/mixed arm command interfaces or latched mode-switch failure.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type A6YDamiaoSystem::perform_command_mode_switch(
    const std::vector<std::string>& start, const std::vector<std::string>& stop)
{
  if (!controller_mode_switching_) return hardware_interface::return_type::OK;
  const auto claims = next_claims(start, stop);
  const int mode = claimed_mode(claims);
  if (mode_switch_failed_ || mode < 0) return hardware_interface::return_type::ERROR;
  const std::string desired = mode == 1 ? "mit" : "pos_vel";
  if (!dry_run_ && auto_enable_ && (mode == 0 || desired != control_mode_)) {
    disable_enabled_motors();
    const double now = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
    bool disabled = verify_enable_feedback(now, "before controller mode switch");
    for (const auto& joint : joints_) {
      if (!joint.read_only && joint.motor_error != 0) disabled = false;
    }
    if (!disabled) {
      mode_switch_failed_ = true;
      RCLCPP_ERROR(LOGGER, "Mode switch aborted: actual disabled feedback was not confirmed.");
      return hardware_interface::return_type::ERROR;
    }
    if (mode != 0 && desired != control_mode_) {
      control_mode_ = desired;
      if (!switch_control_modes()) {
        mode_switch_failed_ = true;
        disable_enabled_motors();
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  if (mode != 0) control_mode_ = desired;
  for (auto& joint : joints_) {
    joint.command_position = joint.state_position;
    joint.command_velocity = 0.0;
    joint.command_effort = 0.0;
    joint.command_motor_enable = 0.0;
  }
  controller_mode_ = mode;
  controller_claims_ = claims;
  command_watchdog_.reset();
  RCLCPP_INFO(LOGGER, "Controller mode confirmed: %s; next output batch must preload before enabling.",
              mode == 0 ? "disabled" : control_mode_.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type A6YDamiaoSystem::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
  // At the next read boundary, both timings from the previous read/write pair
  // are complete. This excludes controller computation and idle period time.
  if (last_read_duration_>=0 && last_write_duration_>=0)
    last_bus_cycle_duration_=last_read_duration_+last_write_duration_;
  DurationMeasurement measurement{last_read_duration_};
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }

  const double cycle_time = std::chrono::duration<double>(measurement.start.time_since_epoch()).count();
  if (control_cycle_count_ > 0) {
    control_period_ = cycle_time - control_time_;
    // Count delayed cycles, not a motor fault. Allow 50% scheduling jitter.
    if (diagnostic_period_ > 0 && control_period_ > 1.5 * diagnostic_period_)
      control_overrun_count_ += 1.0;
  }
  control_time_ = cycle_time;
  control_cycle_count_ += 1.0;

  if (!dry_run_)
  {
    const double now_seconds = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
    const bool has_read_only_joint =
        std::any_of(joints_.begin(), joints_.end(), [](const auto& joint) { return joint.read_only; });
    const auto request_scope = runtime_status_request_scope(
        auto_enable_, has_read_only_joint,
        defer_enable_until_controller_ready_,
        all_commandable_motors_enabled());
    if (!request_status_updates(now_seconds, request_scope))
    {
      return hardware_interface::return_type::ERROR;
    }
    drain_can(receive_timeout_, 64, now_seconds);

    const bool strict_feedback = !allow_partial_feedback_ || auto_enable_;
    for (auto& joint : joints_)
    {
      update_joint_diagnostics(joint, now_seconds);
      if (!joint.has_feedback)
      {
        if (strict_feedback)
        {
          RCLCPP_ERROR(LOGGER, "No feedback has been received from %s.", joint.name.c_str());
          disable_enabled_motors();
          return hardware_interface::return_type::ERROR;
        }
        if (!joint.missing_feedback_warned)
        {
          RCLCPP_WARN(LOGGER, "No feedback has been received from %s; keeping last state because "
                              "allow_partial_feedback=true and auto_enable=false.",
                      joint.name.c_str());
          joint.missing_feedback_warned = true;
        }
        continue;
      }
      if (feedback_timeout_ > 0.0 && (now_seconds - joint.last_feedback_time) > feedback_timeout_)
      {
        if (strict_feedback)
        {
          RCLCPP_ERROR(LOGGER, "Feedback timeout for %s: %.3f s exceeds %.3f s.", joint.name.c_str(),
                       now_seconds - joint.last_feedback_time, feedback_timeout_);
          disable_enabled_motors();
          return hardware_interface::return_type::ERROR;
        }
        if (!joint.feedback_timeout_warned)
        {
          RCLCPP_WARN(LOGGER, "Feedback timeout for %s: %.3f s exceeds %.3f s; keeping last state because "
                              "allow_partial_feedback=true and auto_enable=false.",
                      joint.name.c_str(), now_seconds - joint.last_feedback_time, feedback_timeout_);
          joint.feedback_timeout_warned = true;
        }
      }
      if (joint.faulted && auto_enable_ && !joint.read_only)
      {
        RCLCPP_ERROR(LOGGER, "Motor %s is faulted with error 0x%X. Disabling enabled motors.", joint.name.c_str(),
                     joint.motor_error);
        disable_enabled_motors();
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }

  const double dt = std::max(period.seconds(), 1e-6);
  for (auto& joint : joints_)
  {
    joint.last_position = joint.state_position;
    joint.state_position = joint.command_position;
    joint.state_velocity = (joint.state_position - joint.last_position) / dt;
    joint.state_effort = joint.command_effort;
    joint.state_motor_error = static_cast<double>(joint.motor_error);
    joint.state_fault = joint.faulted ? 1.0 : 0.0;
    joint.state_has_feedback = 1.0;
    joint.state_feedback_age = 0.0;
    joint.state_position_age = 0.0;
    joint.state_position_valid = 1.0;
    joint.state_position_rejections = 0.0;
    joint.state_enabled = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type A6YDamiaoSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  DurationMeasurement measurement{last_write_duration_};
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }

  if (mode_switch_failed_) return hardware_interface::return_type::ERROR;
  if (!dry_run_)
  {
    if (!auto_enable_)
    {
      if (!warned_motion_disabled_)
      {
        RCLCPP_WARN(LOGGER, "auto_enable=false, so real motor command frames are not being sent.");
        warned_motion_disabled_ = true;
      }
      return hardware_interface::return_type::OK;
    }

    const double command_write_period = command_watchdog_.observe();
    if (command_timeout_ > 0.0 && command_write_period > command_timeout_)
    {
      RCLCPP_ERROR(LOGGER, "Command write period %.3f s exceeds command_timeout %.3f s. Disabling enabled motors.",
                   command_write_period, command_timeout_);
      disable_enabled_motors();
      return hardware_interface::return_type::ERROR;
    }

    const auto controller_enable_state = controller_enable_batch_state(
        joints_.begin(), joints_.end(),
        [](const auto& joint) { return joint.read_only; },
        [this](const auto& joint) {
          if (!controller_mode_switching_) return joint.command_motor_enable;
          if (controller_mode_ == 2) return 1.0;  // stock position controllers
          if (controller_mode_ == 1) return joint.command_motor_enable;
          return 0.0;
        });
    const auto enable_action = deferred_motor_enable_action(
        defer_enable_until_controller_ready_, controller_enable_state,
        any_commandable_motor_enabled());
    if (enable_action == DeferredMotorEnableAction::wait_disabled)
    {
      if (!warned_waiting_for_controller_)
      {
        RCLCPP_WARN(
            LOGGER,
            "Motor enable is deferred until the active controller publishes "
            "a complete safe command batch.");
        warned_waiting_for_controller_ = true;
      }
      return hardware_interface::return_type::OK;
    }
    if (enable_action == DeferredMotorEnableAction::disable)
    {
      disable_enabled_motors();
      warned_waiting_for_controller_ = false;
      return hardware_interface::return_type::OK;
    }
    if (enable_action == DeferredMotorEnableAction::enable_and_send)
    {
      for (const auto& joint : joints_)
      {
        if (!joint.read_only && !validate_motor_command(joint))
        {
          return hardware_interface::return_type::ERROR;
        }
      }
      for (auto& joint : joints_)
      {
        if (!joint.read_only && !send_motor_command(joint))
        {
          return hardware_interface::return_type::ERROR;
        }
      }
      std::string flush_error;
      if (!bus_->flush(&flush_error))
      {
        RCLCPP_ERROR(
            LOGGER,
            "Failed to flush the controller-ready MIT preload: %s",
            flush_error.c_str());
        return hardware_interface::return_type::ERROR;
      }
      if (!enable_commandable_motors())
      {
        return hardware_interface::return_type::ERROR;
      }
      warned_waiting_for_controller_ = false;
      RCLCPP_INFO(
          LOGGER,
          "Controller-ready handshake complete; commandable motors are now enabled.");
    }

    const auto batch_disposition = motor_command_batch_disposition(
        joints_.begin(), joints_.end(), [](const auto& joint) {
          return motor_command_disposition(joint.read_only, joint.enabled, joint.faulted);
        });
    if (batch_disposition == MotorCommandDisposition::block_disabled ||
        batch_disposition == MotorCommandDisposition::block_faulted)
    {
      for (const auto& joint : joints_)
      {
        const auto disposition = motor_command_disposition(joint.read_only, joint.enabled, joint.faulted);
        if (disposition == MotorCommandDisposition::block_disabled)
        {
          RCLCPP_ERROR(
              LOGGER, "Refusing command batch because %s is disabled. Disabling all commandable motors.",
              joint.name.c_str());
          break;
        }
        if (disposition == MotorCommandDisposition::block_faulted)
        {
          RCLCPP_ERROR(
              LOGGER, "Refusing command batch because %s has latched motor error 0x%X. Disabling enabled motors.",
              joint.name.c_str(), joint.motor_error);
          break;
        }
      }
      disable_enabled_motors();
      return hardware_interface::return_type::ERROR;
    }

    for (auto& joint : joints_)
    {
      if (joint.read_only)
      {
        continue;
      }
      if (!send_motor_command(joint))
      {
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

A6YDamiaoSystem::CallbackReturn A6YDamiaoSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  dry_run_ = parse_bool(hardware_parameter("dry_run", "true"), true);
  auto_enable_ = parse_bool(hardware_parameter("auto_enable", "false"), false);
  defer_enable_until_controller_ready_ = parse_bool(
      hardware_parameter("defer_enable_until_controller_ready", "false"),
      false);
  set_control_mode_on_configure_ =
      parse_bool(hardware_parameter("set_control_mode_on_configure", "false"), false);
  require_feedback_on_configure_ =
      parse_bool(hardware_parameter("require_feedback_on_configure", "true"), true);
  allow_partial_feedback_ = parse_bool(hardware_parameter("allow_partial_feedback", "false"), false);
  disable_on_deactivate_ = parse_bool(hardware_parameter("disable_on_deactivate", "true"), true);
  can_channel_ = hardware_parameter("can_channel", can_channel_);
  can_bitrate_ = parse_uint(hardware_parameter("can_bitrate", "1000000"), 1000000);
  tty_baudrate_ = parse_uint(hardware_parameter("tty_baudrate", "115200"), 115200);
  controller_mode_switching_ = parse_bool(hardware_parameter("controller_mode_switching", "false"), false);
  control_mode_ = hardware_parameter("control_mode", "pos_vel");
  command_timeout_ = parse_double(hardware_parameter("command_timeout", "0.2"), 0.2);
  max_start_delta_ = parse_double(hardware_parameter("max_start_delta", "0.25"), 0.25);
  receive_timeout_ = parse_double(hardware_parameter("receive_timeout", "0.001"), 0.001);
  feedback_timeout_ = parse_double(hardware_parameter("feedback_timeout", "0.5"), 0.5);
  status_request_period_ = parse_double(hardware_parameter("status_request_period", "0.05"), 0.05);
  const double diagnostic_rate = parse_double(hardware_parameter("diagnostic_update_rate", "0"), 0.0);
  diagnostic_period_ = std::isfinite(diagnostic_rate) && diagnostic_rate > 0 ? 1.0 / diagnostic_rate : 0.0;
  initial_feedback_attempts_ = parse_uint(hardware_parameter("initial_feedback_attempts", "20"), 20);
  mit_kp_ = parse_double(hardware_parameter("mit_kp", "0.0"), 0.0);
  mit_kd_ = parse_double(hardware_parameter("mit_kd", "0.5"), 0.5);

  if (!configure_joints(info))
  {
    return CallbackReturn::ERROR;
  }

  const auto read_only_count =
      std::count_if(joints_.begin(), joints_.end(), [](const auto& joint) { return joint.read_only; });
  RCLCPP_INFO(LOGGER,
              "Configured A6_Y Damiao system with %zu joints (%zu read-only), dry_run=%s, channel=%s, "
              "control_mode=%s, auto_enable=%s, allow_partial_feedback=%s, command_timeout=%.3f, "
              "initial_feedback_attempts=%u, mit_kp=%.3f, mit_kd=%.3f, "
              "defer_enable_until_controller_ready=%s",
              joints_.size(), read_only_count, dry_run_ ? "true" : "false", can_channel_.c_str(),
              control_mode_.c_str(), auto_enable_ ? "true" : "false",
              allow_partial_feedback_ ? "true" : "false", command_timeout_, initial_feedback_attempts_, mit_kp_,
              mit_kd_, defer_enable_until_controller_ready_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

A6YDamiaoSystem::CallbackReturn A6YDamiaoSystem::on_configure(const rclcpp_lifecycle::State&)
{
  if (!dry_run_)
  {
    return configure_real_hardware() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  for (auto& joint : joints_)
  {
    joint.command_position = joint.state_position;
    joint.command_velocity = 0.0;
    joint.command_effort = 0.0;
    joint.command_motor_enable = 0.0;
    joint.last_position = joint.state_position;
  }

  return CallbackReturn::SUCCESS;
}

A6YDamiaoSystem::CallbackReturn A6YDamiaoSystem::on_activate(const rclcpp_lifecycle::State&)
{
  for (const auto& joint : joints_)
  {
    if (std::abs(joint.command_position - joint.state_position) > max_start_delta_)
    {
      RCLCPP_ERROR(LOGGER, "Refusing to activate: %s command/state delta %.3f exceeds max_start_delta %.3f",
                   joint.name.c_str(), std::abs(joint.command_position - joint.state_position), max_start_delta_);
      return CallbackReturn::ERROR;
    }
  }

  if (!dry_run_ && !activate_real_hardware())
  {
    return CallbackReturn::ERROR;
  }

  command_watchdog_.reset();
  active_ = true;
  warned_motion_disabled_ = false;
  warned_waiting_for_controller_ = false;
  RCLCPP_INFO(LOGGER, "A6_Y Damiao system activated in %s mode", dry_run_ ? "dry-run" : control_mode_.c_str());
  return CallbackReturn::SUCCESS;
}

A6YDamiaoSystem::CallbackReturn A6YDamiaoSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  active_ = false;
  if (!dry_run_)
  {
    deactivate_real_hardware();
  }
  for (auto& joint : joints_)
  {
    joint.command_position = joint.state_position;
    joint.command_velocity = 0.0;
    joint.command_effort = 0.0;
    joint.command_motor_enable = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> A6YDamiaoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& joint : joints_)
  {
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.state_effort);
    state_interfaces.emplace_back(joint.name, "motor_error", &joint.state_motor_error);
    state_interfaces.emplace_back(joint.name, "fault", &joint.state_fault);
    state_interfaces.emplace_back(joint.name, "has_feedback", &joint.state_has_feedback);
    state_interfaces.emplace_back(joint.name, "feedback_age", &joint.state_feedback_age);
    state_interfaces.emplace_back(joint.name, "enabled", &joint.state_enabled);
    state_interfaces.emplace_back(joint.name, "position_age", &joint.state_position_age);
    state_interfaces.emplace_back(joint.name, "position_valid", &joint.state_position_valid);
    state_interfaces.emplace_back(joint.name, "position_rejections", &joint.state_position_rejections);
    state_interfaces.emplace_back(joint.name, "dry_run", &joint.state_dry_run);
    state_interfaces.emplace_back(joint.name, "bus_cycle_duration", &last_bus_cycle_duration_);
    if (diagnostic_period_ > 0) {
      state_interfaces.emplace_back(joint.name, "control_time", &control_time_);
      state_interfaces.emplace_back(joint.name, "control_cycle_count", &control_cycle_count_);
      state_interfaces.emplace_back(joint.name, "control_period", &control_period_);
      state_interfaces.emplace_back(joint.name, "control_overrun_count", &control_overrun_count_);
      state_interfaces.emplace_back(joint.name, "read_duration", &last_read_duration_);
      state_interfaces.emplace_back(joint.name, "write_duration", &last_write_duration_);
      state_interfaces.emplace_back(joint.name, "command_count", &joint.command_count);
      state_interfaces.emplace_back(joint.name, "feedback_count", &joint.feedback_count);
      state_interfaces.emplace_back(joint.name, "sent_position", &joint.sent_position);
      state_interfaces.emplace_back(joint.name, "sent_motor_position", &joint.sent_motor_position);
      state_interfaces.emplace_back(joint.name, "sent_velocity_limit", &joint.sent_velocity_limit);
      state_interfaces.emplace_back(joint.name, "send_begin_time", &joint.send_begin_time);
      state_interfaces.emplace_back(joint.name, "send_complete_time", &joint.send_complete_time);
      state_interfaces.emplace_back(joint.name, "feedback_receive_time", &joint.feedback_receive_time);
      state_interfaces.emplace_back(joint.name, "sent_mode", &joint.sent_mode);
      state_interfaces.emplace_back(joint.name, "motor_position", &joint.motor_position);
      state_interfaces.emplace_back(joint.name, "motor_velocity", &joint.motor_velocity);
    }
    state_interfaces.emplace_back(joint.name, "commanded_effort", &joint.command_effort);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> A6YDamiaoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& joint : joints_)
  {
    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.command_position);
    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command_velocity);
    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.command_effort);
    command_interfaces.emplace_back(joint.name, kMotorEnableInterfaceName, &joint.command_motor_enable);
  }
  return command_interfaces;
}

bool A6YDamiaoSystem::configure_joints(const hardware_interface::HardwareInfo& info)
{
  joints_.clear();
  joint_index_by_name_.clear();
  joint_index_by_can_id_.clear();

  if (info.joints.empty())
  {
    RCLCPP_ERROR(LOGGER, "No joints were provided in ros2_control hardware info.");
    return false;
  }

  for (const auto& joint_info : info.joints)
  {
    if (!has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_POSITION))
    {
      RCLCPP_ERROR(LOGGER, "Joint %s is missing required position command interface.", joint_info.name.c_str());
      return false;
    }
    if (defer_enable_until_controller_ready_ &&
        !has_interface(joint_info.command_interfaces, kMotorEnableInterfaceName))
    {
      RCLCPP_ERROR(
          LOGGER,
          "Joint %s is missing required %s command interface for deferred enable.",
          joint_info.name.c_str(), kMotorEnableInterfaceName);
      return false;
    }
    if (!has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(LOGGER, "Joint %s must provide position and velocity state interfaces.", joint_info.name.c_str());
      return false;
    }

    JointData joint;
    joint.name = joint_info.name;
    joint.motor_type = joint_parameter(joint_info, "motor_type", "UNKNOWN");
    joint.slave_id = parse_uint(joint_parameter(joint_info, "slave_id", "0"), 0);
    joint.master_id = parse_uint(joint_parameter(joint_info, "master_id", "0"), 0);
    joint.sign = parse_double(joint_parameter(joint_info, "sign", "1.0"), 1.0);
    joint.offset = parse_double(joint_parameter(joint_info, "offset", "0.0"), 0.0);
    joint.position_scale = parse_double(joint_parameter(joint_info, "position_scale", "1.0"), 1.0);
    joint.min_position =
        parse_double(joint_parameter(joint_info, "min_position", "-inf"), -std::numeric_limits<double>::infinity());
    joint.max_position =
        parse_double(joint_parameter(joint_info, "max_position", "inf"), std::numeric_limits<double>::infinity());
    joint.max_velocity = parse_double(joint_parameter(joint_info, "max_velocity", "0.5"), 0.5);
    joint.mit_kp = parse_double(joint_parameter(joint_info, "mit_kp", std::to_string(mit_kp_)), mit_kp_);
    joint.mit_kd = parse_double(joint_parameter(joint_info, "mit_kd", std::to_string(mit_kd_)), mit_kd_);
    joint.read_only = parse_bool(joint_parameter(joint_info, "read_only", "false"), false);
    joint.limits = motor_limits_from_type(joint.motor_type);
    // Current four-arm adapters have different gripper PMAX values. The same
    // protocol range must be used for feedback decoding and MIT encoding.
    const auto pmax = joint_parameter(joint_info, "protocol_pmax", "");
    if (!pmax.empty())
    {
      try
      {
        std::size_t consumed = 0;
        const double value = std::stod(pmax, &consumed);
        if (consumed != pmax.size() || !std::isfinite(value) || value <= 0.0)
          throw std::invalid_argument("invalid PMAX");
        joint.limits.q_max = value;
      }
      catch (const std::exception&)
      {
        RCLCPP_ERROR(LOGGER, "Joint %s has invalid protocol_pmax.", joint.name.c_str());
        return false;
      }
    }
    if (!apply_protocol_vmax(joint_parameter(joint_info, "protocol_vmax", ""), joint.limits))
    {
      RCLCPP_ERROR(LOGGER, "Joint %s has invalid protocol_vmax; expected positive finite motor VMAX.", joint.name.c_str());
      return false;
    }
    joint.state_dry_run = dry_run_ ? 1.0 : 0.0;
    joint.feedback_position_jump = parse_double(joint_parameter(joint_info, "feedback_position_jump",
        std::to_string(0.2 * joint.position_scale)), 0.2 * joint.position_scale);
    joint.feedback_position_alpha = parse_double(joint_parameter(joint_info, "feedback_position_alpha", "0.4"), 0.4);
    if (!std::isfinite(joint.feedback_position_jump) || joint.feedback_position_jump < 0 ||
        !std::isfinite(joint.feedback_position_alpha) || joint.feedback_position_alpha <= 0 || joint.feedback_position_alpha > 1) {
      RCLCPP_ERROR(LOGGER, "Invalid position acceptance parameters for %s", joint.name.c_str());
      return false;
    }

    if (!dry_run_ && (joint.motor_type == "UNKNOWN" || joint.slave_id == 0))
    {
      RCLCPP_ERROR(LOGGER, "Joint %s is missing motor_type or slave_id for real hardware mode.",
                   joint_info.name.c_str());
      return false;
    }
    if (joint.min_position >= joint.max_position)
    {
      RCLCPP_ERROR(LOGGER, "Joint %s has invalid soft limits [%.3f, %.3f].", joint_info.name.c_str(),
                   joint.min_position, joint.max_position);
      return false;
    }
    if (!std::isfinite(joint.position_scale) || joint.position_scale <= 0.0)
    {
      RCLCPP_ERROR(LOGGER, "Joint %s has invalid position_scale %.12f.", joint_info.name.c_str(),
                   joint.position_scale);
      return false;
    }

    for (const auto& state_interface : joint_info.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION && !state_interface.initial_value.empty())
      {
        joint.state_position = parse_double(state_interface.initial_value, 0.0);
      }
    }
    joint.command_position = joint.state_position;
    joint.command_motor_enable = 0.0;
    joint.last_position = joint.state_position;

    joint_index_by_name_[joint.name] = joints_.size();
    if (joint.slave_id != 0)
    {
      joint_index_by_can_id_[joint.slave_id] = joints_.size();
    }
    if (joint.master_id != 0)
    {
      joint_index_by_can_id_[joint.master_id] = joints_.size();
    }
    joints_.push_back(joint);
  }

  return true;
}

bool A6YDamiaoSystem::configure_real_hardware()
{
  bus_ = std::make_unique<SlcanBus>();
  std::string error_message;
  if (!bus_->open(can_channel_, can_bitrate_, tty_baudrate_, &error_message))
  {
    RCLCPP_ERROR(LOGGER, "Failed to open SLCAN bus: %s", error_message.c_str());
    return false;
  }
  last_status_request_time_ = 0.0;

  const std::uint32_t mode = control_mode_code();
  if (mode == 0)
  {
    RCLCPP_ERROR(LOGGER, "Unsupported control_mode '%s'. Use pos_vel, pos_vel_csp, or mit.", control_mode_.c_str());
    return false;
  }

  if (set_control_mode_on_configure_ && !switch_control_modes())
  {
    return false;
  }

  rclcpp::Clock clock(RCL_STEADY_TIME);
  for (std::uint32_t attempt = 0; attempt < initial_feedback_attempts_; ++attempt)
  {
    bool requested_any = false;
    for (const auto& joint : joints_)
    {
      if (joint.has_feedback)
      {
        continue;
      }
      if (!bus_->send(make_refresh_status_command(joint.slave_id), &error_message))
      {
        RCLCPP_ERROR(LOGGER, "Failed to request %s status: %s", joint.name.c_str(), error_message.c_str());
        return false;
      }
      requested_any = true;
      drain_can(kInitialFeedbackPerRequestDrain, 64, clock.now().seconds());
    }
    if (!requested_any)
    {
      break;
    }
    for (int i = 0; i < 5; ++i)
    {
      drain_can(kInitialFeedbackPassDrain, 128, clock.now().seconds());
    }
    if ((attempt + 1U) % 10U == 0U || (attempt + 1U) == initial_feedback_attempts_)
    {
      std::ostringstream missing;
      for (const auto& joint : joints_)
      {
        if (joint.has_feedback)
        {
          continue;
        }
        if (missing.tellp() > 0)
        {
          missing << ", ";
        }
        missing << joint.name << "(0x" << std::hex << std::uppercase << joint.slave_id << std::dec << ")";
      }
      if (missing.tellp() > 0)
      {
        RCLCPP_WARN(LOGGER, "Still waiting for initial feedback after %u/%u attempts: %s", attempt + 1U,
                    initial_feedback_attempts_, missing.str().c_str());
      }
    }
  }

  const bool strict_configure_feedback = require_feedback_on_configure_ && (!allow_partial_feedback_ || auto_enable_);
  bool all_feedback_received = true;
  for (auto& joint : joints_)
  {
    if (!joint.has_feedback)
    {
      all_feedback_received = false;
      if (strict_configure_feedback)
      {
        RCLCPP_ERROR(LOGGER, "No initial feedback received from %s (slave_id=0x%X).", joint.name.c_str(),
                     joint.slave_id);
      }
      else
      {
        RCLCPP_WARN(LOGGER, "No initial feedback received from %s (slave_id=0x%X); continuing because "
                            "allow_partial_feedback=true and auto_enable=false.",
                    joint.name.c_str(), joint.slave_id);
      }
      continue;
    }
    joint.command_position = joint.state_position;
    joint.command_velocity = 0.0;
    joint.command_effort = 0.0;
    update_joint_diagnostics(joint, clock.now().seconds());
    joint.last_position = joint.state_position;
  }

  if (strict_configure_feedback && !all_feedback_received)
  {
    return false;
  }

  RCLCPP_INFO(LOGGER, "Opened SLCAN bus %s at CAN %u, tty %u. auto_enable=%s", can_channel_.c_str(), can_bitrate_,
              tty_baudrate_, auto_enable_ ? "true" : "false");
  return true;
}

bool A6YDamiaoSystem::activate_real_hardware()
{
  if (!bus_ || !bus_->is_open())
  {
    RCLCPP_ERROR(LOGGER, "Cannot activate real hardware: SLCAN bus is not open.");
    return false;
  }

  warned_motion_disabled_ = false;
  if (!auto_enable_)
  {
    RCLCPP_WARN(LOGGER, "auto_enable=false: hardware is active for state readback, but motors will not be enabled.");
    return true;
  }

  std::string error_message;
  rclcpp::Clock clock(RCL_STEADY_TIME);
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    for (const auto& joint : joints_)
    {
      if (!bus_->send(make_refresh_status_command(joint.slave_id), &error_message))
      {
        RCLCPP_ERROR(LOGGER, "Failed to refresh %s before enable: %s", joint.name.c_str(), error_message.c_str());
        return false;
      }
      // SLCAN runs over a 115200-baud serial link.  Sending all seven
      // requests back-to-back before reading can leave an early response in
      // the tty queue long enough to fail the deliberately tight D2
      // freshness gate.  Drain after each request, as configure does, so the
      // age check measures a completed refresh rather than serial backlog.
      drain_can(kInitialFeedbackPerRequestDrain, 64, clock.now().seconds());
    }
    drain_can(kInitialFeedbackPassDrain, 128, clock.now().seconds());
  }
  if (!verify_enable_feedback(clock.now().seconds(), "before enable after refresh"))
  {
    return false;
  }

  if (!switch_control_modes())
  {
    return false;
  }
  if (!verify_enable_feedback(rclcpp::Clock(RCL_STEADY_TIME).now().seconds(), "after switching control mode"))
  {
    return false;
  }

  if (defer_enable_until_controller_ready_)
  {
    RCLCPP_INFO(
        LOGGER,
        "Damiao mode is prepared; motor enable is waiting for the controller-ready handshake.");
    return true;
  }
  return enable_commandable_motors();
}

bool A6YDamiaoSystem::enable_commandable_motors()
{
  if (!bus_ || !bus_->is_open())
  {
    RCLCPP_ERROR(LOGGER, "Cannot enable motors: SLCAN bus is not open.");
    return false;
  }

  std::string error_message;
  for (auto& joint : joints_)
  {
    if (joint.read_only)
    {
      joint.enabled = false;
      joint.state_enabled = 0.0;
      continue;
    }
    if (!bus_->send(make_control_command(joint.slave_id, 0xFC), &error_message))
    {
      RCLCPP_ERROR(LOGGER, "Failed to enable %s: %s", joint.name.c_str(), error_message.c_str());
      disable_enabled_motors();
      return false;
    }
    joint.enabled = true;
    joint.state_enabled = 1.0;
  }
  drain_can(0.01, 64, rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
  if (!verify_enable_feedback(rclcpp::Clock(RCL_STEADY_TIME).now().seconds(), "after enable command"))
  {
    disable_enabled_motors();
    return false;
  }
  return true;
}

void A6YDamiaoSystem::deactivate_real_hardware()
{
  if (!bus_ || !bus_->is_open())
  {
    return;
  }

  if (disable_on_deactivate_)
  {
    disable_enabled_motors();
  }
  bus_->close();
}

void A6YDamiaoSystem::disable_enabled_motors()
{
  if (!bus_ || !bus_->is_open())
  {
    return;
  }

  std::string error_message;
  std::size_t sent_count = 0;
  std::size_t commandable_count = 0;
  for (int pass = 0; pass < kDisableCommandPasses; ++pass)
  {
    for (auto& joint : joints_)
    {
      if (joint.read_only)
      {
        continue;
      }
      if (pass == 0)
      {
        ++commandable_count;
      }
      if (!bus_->send(make_control_command(joint.slave_id, 0xFD), &error_message))
      {
        RCLCPP_ERROR(LOGGER, "Failed to disable %s on pass %d/%d: %s", joint.name.c_str(), pass + 1,
                     kDisableCommandPasses, error_message.c_str());
      }
      else
      {
        ++sent_count;
      }
    }
    if (!bus_->flush(&error_message))
    {
      RCLCPP_WARN(LOGGER, "Failed to flush disable commands on pass %d/%d: %s", pass + 1, kDisableCommandPasses,
                  error_message.c_str());
    }
    if (pass + 1 < kDisableCommandPasses)
    {
      std::this_thread::sleep_for(kDisablePassDelay);
    }
  }
  for (auto& joint : joints_)
  {
    joint.enabled = false;
    joint.state_enabled = 0.0;
  }
  if (sent_count > 0)
  {
    drain_can(0.05, 128, rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
    RCLCPP_INFO(LOGGER,
                "Sent %zu disable command frame(s) across %d pass(es) for %zu commandable Damiao motor(s); "
                "%zu read-only motor(s).",
                sent_count, kDisableCommandPasses, commandable_count, joints_.size() - commandable_count);
  }
}

bool A6YDamiaoSystem::switch_control_modes()
{
  if (!bus_ || !bus_->is_open())
  {
    RCLCPP_ERROR(LOGGER, "Cannot switch control modes: SLCAN bus is not open.");
    return false;
  }

  const std::uint32_t mode = control_mode_code();
  if (mode == 0)
  {
    RCLCPP_ERROR(LOGGER, "Unsupported control_mode '%s'. Use pos_vel, pos_vel_csp, or mit.", control_mode_.c_str());
    return false;
  }

  std::string error_message;
  std::size_t commandable_count = 0;
  for (auto& joint : joints_)
  {
    if (joint.read_only)
    {
      joint.control_mode_feedback = static_cast<int>(mode);
      continue;
    }
    ++commandable_count;
    joint.control_mode_feedback = -1;
    if (!bus_->send(make_switch_mode_command(joint.slave_id, mode), &error_message))
    {
      RCLCPP_ERROR(LOGGER, "Failed to switch %s control mode: %s", joint.name.c_str(), error_message.c_str());
      return false;
    }
  }

  rclcpp::Clock clock(RCL_STEADY_TIME);
  const double deadline = clock.now().seconds() + 0.5;
  while (clock.now().seconds() < deadline)
  {
    drain_can(0.05, 128, clock.now().seconds());
    bool all_confirmed = true;
    for (const auto& joint : joints_)
    {
      if (joint.read_only)
      {
        continue;
      }
      if (joint.control_mode_feedback != static_cast<int>(mode))
      {
        all_confirmed = false;
        break;
      }
    }
    if (all_confirmed)
    {
      RCLCPP_INFO(LOGGER, "Confirmed Damiao control mode %u for %zu commandable motor(s); %zu read-only motor(s).",
                  mode, commandable_count, joints_.size() - commandable_count);
      return true;
    }
  }

  for (const auto& joint : joints_)
  {
    if (joint.read_only)
    {
      continue;
    }
    if (joint.control_mode_feedback != static_cast<int>(mode))
    {
      RCLCPP_ERROR(LOGGER, "Control mode switch for %s was not confirmed: expected %u, got %d.",
                   joint.name.c_str(), mode, joint.control_mode_feedback);
    }
  }
  return false;
}

bool A6YDamiaoSystem::verify_enable_feedback(double now_seconds, const std::string& context) const
{
  for (const auto& joint : joints_)
  {
    if (!joint.has_feedback)
    {
      RCLCPP_ERROR(LOGGER, "Refusing to enable %s %s: no feedback has been received.", joint.name.c_str(),
                   context.c_str());
      return false;
    }
    if (feedback_timeout_ > 0.0 && (now_seconds - joint.last_feedback_time) > feedback_timeout_)
    {
      RCLCPP_ERROR(LOGGER, "Refusing to enable %s %s: feedback age %.3f s exceeds %.3f s.", joint.name.c_str(),
                   context.c_str(), now_seconds - joint.last_feedback_time, feedback_timeout_);
      return false;
    }
    if (joint.faulted)
    {
      RCLCPP_ERROR(LOGGER, "Refusing to enable %s %s: fault code 0x%X is latched.", joint.name.c_str(),
                   context.c_str(), joint.motor_error);
      return false;
    }
  }
  return true;
}

bool A6YDamiaoSystem::validate_motor_command(const JointData& joint) const
{
  if (joint.read_only)
  {
    return true;
  }

  if (!std::isfinite(joint.command_position))
  {
    RCLCPP_ERROR(LOGGER, "Refusing %s command because position is not finite.", joint.name.c_str());
    return false;
  }
  const bool mit_zero_stiffness = control_mode_ == "mit" && std::abs(joint.mit_kp) <= 1e-9;
  if (!mit_zero_stiffness && (joint.command_position < (joint.min_position - kSoftLimitTolerance) ||
                              joint.command_position > (joint.max_position + kSoftLimitTolerance)))
  {
    RCLCPP_ERROR(LOGGER, "Refusing %s command %.3f outside soft limits [%.3f, %.3f] with %.3f tolerance.",
                 joint.name.c_str(), joint.command_position, joint.min_position, joint.max_position,
                 kSoftLimitTolerance);
    return false;
  }
  if (!std::isfinite(joint.command_velocity) ||
      !std::isfinite(joint.command_effort))
  {
    RCLCPP_ERROR(
        LOGGER,
        "Refusing %s command because velocity or effort is not finite.",
        joint.name.c_str());
    return false;
  }
  return true;
}

bool A6YDamiaoSystem::send_motor_command(JointData& joint)
{
  if (joint.read_only)
  {
    return true;
  }
  if (!validate_motor_command(joint))
  {
    return false;
  }
  if (!bus_ || !bus_->is_open())
  {
    RCLCPP_ERROR(LOGGER, "Cannot send command for %s: SLCAN bus is not open.", joint.name.c_str());
    return false;
  }

  const bool mit_zero_stiffness = control_mode_ == "mit" && std::abs(joint.mit_kp) <= 1e-9;
  // In zero-stiffness MIT force mode the position field is only a protocol reference;
  // rejecting or clamping it at a soft limit can interrupt hand-guided gravity compensation.
  const double joint_position = mit_zero_stiffness ?
                                    joint.command_position :
                                    std::clamp(joint.command_position, joint.min_position, joint.max_position);
  const double motor_position = to_motor_position(joint, joint_position);

  const bool has_velocity_command = std::isfinite(joint.command_velocity) && std::abs(joint.command_velocity) > 1e-6;
  const double motor_velocity =
      has_velocity_command ? to_motor_velocity(joint, joint.command_velocity) :
                             (control_mode_ == "mit" ? 0.0 : to_motor_velocity_limit(joint));
  const double motor_torque = to_motor_torque(joint, joint.command_effort);

  CanFrame frame;
  if (control_mode_ == "mit")
  {
    frame = make_mit_command(joint.slave_id, joint.limits, joint.mit_kp, joint.mit_kd, motor_position, motor_velocity,
                             motor_torque);
  }
  else if (control_mode_ == "pos_vel_csp")
  {
    frame = make_pos_vel_csp_command(joint.slave_id, motor_position, std::abs(motor_velocity));
  }
  else
  {
    frame = make_pos_vel_command(joint.slave_id, motor_position, std::abs(motor_velocity));
  }

  std::string error_message;
  const double send_begin = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  if (!bus_->send(frame, &error_message))
  {
    RCLCPP_ERROR(LOGGER, "Failed to send command for %s: %s", joint.name.c_str(), error_message.c_str());
    return false;
  }
  // A complete SLCAN frame accepted by the host serial driver, not a CAN ACK.
  joint.command_count += 1.0;
  joint.send_begin_time = send_begin;
  joint.send_complete_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  joint.sent_mode = static_cast<double>(control_mode_code());
  // POS_VEL encodes float32 motor coordinates. MIT gravity has no position
  // servo target; keep its reference field out of position-response metrics.
  if (control_mode_ == "pos_vel") {
    joint.sent_motor_position = static_cast<double>(static_cast<float>(motor_position));
    joint.sent_position = joint.sign * joint.sent_motor_position * joint.position_scale + joint.offset;
    joint.sent_velocity_limit = static_cast<double>(static_cast<float>(std::abs(motor_velocity)));
  } else {
    joint.sent_position = joint.sent_motor_position = joint.sent_velocity_limit =
        std::numeric_limits<double>::quiet_NaN();
  }
  return true;
}

bool A6YDamiaoSystem::any_commandable_motor_enabled() const
{
  return std::any_of(
      joints_.begin(), joints_.end(),
      [](const auto& joint) { return !joint.read_only && joint.enabled; });
}

bool A6YDamiaoSystem::all_commandable_motors_enabled() const
{
  bool has_commandable_motor = false;
  for (const auto& joint : joints_)
  {
    if (joint.read_only)
    {
      continue;
    }
    has_commandable_motor = true;
    if (!joint.enabled)
    {
      return false;
    }
  }
  return has_commandable_motor;
}

bool A6YDamiaoSystem::request_status_updates(double now_seconds, StatusRequestScope scope)
{
  if (scope == StatusRequestScope::none)
  {
    return true;
  }
  if (!bus_ || !bus_->is_open())
  {
    RCLCPP_ERROR(LOGGER, "Cannot request status updates: SLCAN bus is not open.");
    return false;
  }
  if (status_request_period_ < 0.0)
  {
    return true;
  }
  if (last_status_request_time_ > 0.0 && (now_seconds - last_status_request_time_) < status_request_period_)
  {
    return true;
  }

  std::string error_message;
  for (const auto& joint : joints_)
  {
    if (!status_request_includes_joint(scope, joint.read_only))
    {
      continue;
    }
    if (!bus_->send(make_refresh_status_command(joint.slave_id), &error_message))
    {
      RCLCPP_ERROR(LOGGER, "Failed to request %s status: %s", joint.name.c_str(), error_message.c_str());
      disable_enabled_motors();
      return false;
    }
  }
  last_status_request_time_ = now_seconds;
  return true;
}

void A6YDamiaoSystem::process_rx_frame(const CanFrame& frame, double now_seconds)
{
  DamiaoParamFeedback param_feedback;
  if (parse_damiao_param_feedback(frame, joint_index_by_can_id_, param_feedback))
  {
    auto param_it = joint_index_by_can_id_.find(param_feedback.slave_id);
    if (param_it != joint_index_by_can_id_.end() && param_feedback.rid == 10)
    {
      auto& joint = joints_[param_it->second];
      joint.control_mode_feedback = static_cast<int>(param_feedback.value);
      RCLCPP_DEBUG(LOGGER, "Received CTRL_MODE feedback for %s: opcode=0x%02X value=%u.", joint.name.c_str(),
                   param_feedback.opcode, param_feedback.value);
    }
    return;
  }

  auto it = joint_index_by_can_id_.find(frame.id);
  if (it == joint_index_by_can_id_.end() && frame.dlc > 0)
  {
    it = joint_index_by_can_id_.find(frame.data[0] & 0x0F);
  }
  if (it == joint_index_by_can_id_.end())
  {
    return;
  }

  auto& joint = joints_[it->second];
  DamiaoFeedback feedback;
  if (!parse_damiao_feedback(frame, joint.limits, feedback))
  {
    return;
  }
  joint.feedback_count += 1.0;
  joint.feedback_receive_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  joint.motor_position = feedback.q;
  joint.motor_velocity = feedback.dq;

  const double raw_position = joint.sign * feedback.q * joint.position_scale + joint.offset;
  joint.last_position = joint.state_position;
  joint.accepted_position.update(raw_position, now_seconds, joint.feedback_position_jump,
                                 joint.feedback_position_alpha, joint.state_position);
  joint.state_velocity = joint.sign * feedback.dq * joint.position_scale;
  joint.state_effort = joint.sign * feedback.tau / joint.position_scale;
  joint.last_feedback_time = now_seconds;
  joint.has_feedback = true;
  joint.missing_feedback_warned = false;
  joint.feedback_timeout_warned = false;

  const int previous_error = joint.motor_error;
  joint.motor_error = feedback.error;
  if (is_fault_code(feedback.error))
  {
    const bool first_report = !joint.faulted || previous_error != feedback.error;
    joint.faulted = true;
    if (first_report)
    {
      RCLCPP_ERROR(LOGGER, "Motor %s reported fault code 0x%X; fault latched.", joint.name.c_str(), feedback.error);
    }
  }
  update_joint_diagnostics(joint, now_seconds);
}

void A6YDamiaoSystem::drain_can(double timeout_seconds, std::size_t max_frames, double now_seconds)
{
  if (!bus_ || !bus_->is_open())
  {
    return;
  }

  std::vector<CanFrame> frames;
  std::string error_message;
  bus_->drain(frames, timeout_seconds, max_frames, &error_message);
  for (const auto& frame : frames)
  {
    process_rx_frame(frame, now_seconds);
  }
}

void A6YDamiaoSystem::update_joint_diagnostics(JointData& joint, double now_seconds)
{
  joint.state_motor_error = static_cast<double>(joint.motor_error);
  joint.state_fault = joint.faulted ? 1.0 : 0.0;
  joint.state_has_feedback = joint.has_feedback ? 1.0 : 0.0;
  joint.state_feedback_age = joint.has_feedback ? std::max(0.0, now_seconds - joint.last_feedback_time) : -1.0;
  joint.state_enabled = joint.enabled ? 1.0 : 0.0;
  joint.state_position_age = joint.accepted_position.age(now_seconds);
  joint.state_position_valid = joint.accepted_position.valid ? 1.0 : 0.0;
  joint.state_position_rejections = static_cast<double>(joint.accepted_position.consecutive_rejections);
}

std::uint32_t A6YDamiaoSystem::control_mode_code() const
{
  if (control_mode_ == "mit")
  {
    return 1;
  }
  if (control_mode_ == "pos_vel")
  {
    return 2;
  }
  if (control_mode_ == "pos_vel_csp")
  {
    return 5;
  }
  return 0;
}

bool A6YDamiaoSystem::is_fault_code(int code) const
{
  return code >= 0x08 && code <= 0x0E;
}

double A6YDamiaoSystem::to_motor_position(const JointData& joint, double joint_position) const
{
  return joint.sign * (joint_position - joint.offset) / joint.position_scale;
}

double A6YDamiaoSystem::to_motor_velocity(const JointData& joint, double joint_velocity) const
{
  const double velocity = joint.sign * joint_velocity / joint.position_scale;
  const double limit = to_motor_velocity_limit(joint);
  return std::clamp(velocity, -limit, limit);
}

double A6YDamiaoSystem::to_motor_velocity_limit(const JointData& joint) const
{
  const double configured_limit = std::abs(joint.max_velocity / joint.position_scale);
  if (!std::isfinite(configured_limit) || configured_limit <= 0.0)
  {
    return joint.limits.dq_max;
  }
  return std::min(configured_limit, joint.limits.dq_max);
}

double A6YDamiaoSystem::to_motor_torque(const JointData& joint, double joint_torque) const
{
  return std::clamp(joint.sign * joint_torque * joint.position_scale, -joint.limits.tau_max, joint.limits.tau_max);
}

bool A6YDamiaoSystem::has_interface(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                                   const std::string& name) const
{
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [&](const auto& interface) { return interface.name == name; });
}

std::string A6YDamiaoSystem::hardware_parameter(const std::string& name, const std::string& default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  return it == info_.hardware_parameters.end() ? default_value : it->second;
}

std::string A6YDamiaoSystem::joint_parameter(const hardware_interface::ComponentInfo& joint, const std::string& name,
                                            const std::string& default_value) const
{
  const auto it = joint.parameters.find(name);
  return it == joint.parameters.end() ? default_value : it->second;
}

bool A6YDamiaoSystem::parse_bool(const std::string& text, bool default_value) const
{
  if (text == "true" || text == "True" || text == "1")
  {
    return true;
  }
  if (text == "false" || text == "False" || text == "0")
  {
    return false;
  }
  return default_value;
}

double A6YDamiaoSystem::parse_double(const std::string& text, double default_value) const
{
  try
  {
    return std::stod(text);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

std::uint32_t A6YDamiaoSystem::parse_uint(const std::string& text, std::uint32_t default_value) const
{
  try
  {
    return static_cast<std::uint32_t>(std::stoul(text, nullptr, 0));
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}
}  // namespace a6_y_ros2_control

PLUGINLIB_EXPORT_CLASS(a6_y_ros2_control::A6YDamiaoSystem, hardware_interface::SystemInterface)

#include <a6_y_ros2_control/a6_y_damiao_system.hpp>

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
constexpr double kFeedbackPositionAlpha = 0.4;
constexpr double kFeedbackPositionSpikeThreshold = 0.2;
constexpr int kDisableCommandPasses = 3;
constexpr auto kDisablePassDelay = std::chrono::milliseconds(20);

bool parse_param_feedback(const a6_y_ros2_control::CanFrame& frame, std::uint32_t& slave_id, std::uint8_t& opcode,
                          std::uint8_t& rid, std::uint32_t& value)
{
  if (frame.dlc < 8)
  {
    return false;
  }
  opcode = frame.data[2];
  if (opcode != 0x33 && opcode != 0x55 && opcode != 0xAA)
  {
    return false;
  }

  slave_id = static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8);
  rid = frame.data[3];
  value = static_cast<std::uint32_t>(frame.data[4]) | (static_cast<std::uint32_t>(frame.data[5]) << 8) |
          (static_cast<std::uint32_t>(frame.data[6]) << 16) | (static_cast<std::uint32_t>(frame.data[7]) << 24);
  return true;
}
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

hardware_interface::return_type A6YDamiaoSystem::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!dry_run_)
  {
    const double now_seconds = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
    if (!auto_enable_ && !request_status_updates(now_seconds))
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
    joint.state_enabled = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type A6YDamiaoSystem::write(const rclcpp::Time&, const rclcpp::Duration& period)
{
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }

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

    if (!first_write_after_activate_ && command_timeout_ > 0.0 && period.seconds() > command_timeout_)
    {
      RCLCPP_ERROR(LOGGER, "Command write period %.3f s exceeds command_timeout %.3f s. Disabling enabled motors.",
                   period.seconds(), command_timeout_);
      disable_enabled_motors();
      return hardware_interface::return_type::ERROR;
    }
    first_write_after_activate_ = false;

    for (const auto& joint : joints_)
    {
      if (joint.read_only)
      {
        continue;
      }
      if (joint.faulted)
      {
        RCLCPP_ERROR(LOGGER, "Refusing to command %s because motor error 0x%X is latched. Disabling enabled motors.",
                     joint.name.c_str(), joint.motor_error);
        disable_enabled_motors();
        return hardware_interface::return_type::ERROR;
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
  set_control_mode_on_configure_ =
      parse_bool(hardware_parameter("set_control_mode_on_configure", "false"), false);
  require_feedback_on_configure_ =
      parse_bool(hardware_parameter("require_feedback_on_configure", "true"), true);
  allow_partial_feedback_ = parse_bool(hardware_parameter("allow_partial_feedback", "false"), false);
  disable_on_deactivate_ = parse_bool(hardware_parameter("disable_on_deactivate", "true"), true);
  can_channel_ = hardware_parameter("can_channel", can_channel_);
  can_bitrate_ = parse_uint(hardware_parameter("can_bitrate", "1000000"), 1000000);
  tty_baudrate_ = parse_uint(hardware_parameter("tty_baudrate", "115200"), 115200);
  control_mode_ = hardware_parameter("control_mode", "pos_vel");
  command_timeout_ = parse_double(hardware_parameter("command_timeout", "0.2"), 0.2);
  max_start_delta_ = parse_double(hardware_parameter("max_start_delta", "0.25"), 0.25);
  receive_timeout_ = parse_double(hardware_parameter("receive_timeout", "0.001"), 0.001);
  feedback_timeout_ = parse_double(hardware_parameter("feedback_timeout", "0.5"), 0.5);
  status_request_period_ = parse_double(hardware_parameter("status_request_period", "0.05"), 0.05);
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
              "initial_feedback_attempts=%u, mit_kp=%.3f, mit_kd=%.3f",
              joints_.size(), read_only_count, dry_run_ ? "true" : "false", can_channel_.c_str(),
              control_mode_.c_str(), auto_enable_ ? "true" : "false",
              allow_partial_feedback_ ? "true" : "false", command_timeout_, initial_feedback_attempts_, mit_kp_,
              mit_kd_);
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

  first_write_after_activate_ = true;
  active_ = true;
  warned_motion_disabled_ = false;
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
    // Port adaptation only: the four calibrated arms have different protocol
    // ranges (in particular ID 7). Use measured values without writing motors.
    joint.limits.q_max = parse_double(joint_parameter(joint_info, "protocol_pmax", std::to_string(joint.limits.q_max)), joint.limits.q_max);
    joint.limits.dq_max = parse_double(joint_parameter(joint_info, "protocol_vmax", std::to_string(joint.limits.dq_max)), joint.limits.dq_max);
    if (!std::isfinite(joint.limits.q_max) || joint.limits.q_max <= 0.0 ||
        !std::isfinite(joint.limits.dq_max) || joint.limits.dq_max <= 0.0)
    {
      RCLCPP_ERROR(LOGGER, "Invalid calibrated protocol range for %s", joint.name.c_str());
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
    }
    drain_can(0.02, 128, clock.now().seconds());
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

bool A6YDamiaoSystem::send_motor_command(const JointData& joint)
{
  if (joint.read_only)
  {
    return true;
  }

  if (!bus_ || !bus_->is_open())
  {
    RCLCPP_ERROR(LOGGER, "Cannot send command for %s: SLCAN bus is not open.", joint.name.c_str());
    return false;
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
  if (!bus_->send(frame, &error_message))
  {
    RCLCPP_ERROR(LOGGER, "Failed to send command for %s: %s", joint.name.c_str(), error_message.c_str());
    return false;
  }
  return true;
}

bool A6YDamiaoSystem::request_status_updates(double now_seconds)
{
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
  std::uint32_t param_slave_id = 0;
  std::uint8_t param_opcode = 0;
  std::uint8_t param_rid = 0;
  std::uint32_t param_value = 0;
  if (parse_param_feedback(frame, param_slave_id, param_opcode, param_rid, param_value))
  {
    auto param_it = joint_index_by_can_id_.find(param_slave_id);
    if (param_it != joint_index_by_can_id_.end() && param_rid == 10)
    {
      auto& joint = joints_[param_it->second];
      joint.control_mode_feedback = static_cast<int>(param_value);
      RCLCPP_DEBUG(LOGGER, "Received CTRL_MODE feedback for %s: opcode=0x%02X value=%u.", joint.name.c_str(),
                   param_opcode, param_value);
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

  const double raw_position = joint.sign * feedback.q * joint.position_scale + joint.offset;
  joint.last_position = joint.state_position;
  if (!joint.has_feedback)
  {
    joint.state_position = raw_position;
  }
  else if (std::abs(raw_position - joint.state_position) > kFeedbackPositionSpikeThreshold)
  {
    joint.state_position = joint.last_position;
  }
  else
  {
    joint.state_position = kFeedbackPositionAlpha * raw_position + (1.0 - kFeedbackPositionAlpha) * joint.state_position;
  }
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

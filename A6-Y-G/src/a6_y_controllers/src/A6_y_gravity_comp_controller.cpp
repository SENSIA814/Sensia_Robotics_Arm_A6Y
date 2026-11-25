#include <a6_y_controllers/A6_y_gravity_comp_controller.hpp>
#include <a6_y_controllers/a5_model.hpp>
#include <a6_y_controllers/controller_motor_enable.hpp>
#include <a6_y_controllers/haptic_effort_composer.hpp>
#include <a6_y_controllers/safe_zero_effort_command.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

namespace a6_y_controllers
{
namespace
{
constexpr std::size_t kBaseCommandInterfacesPerJoint = 3;
constexpr std::size_t kStateInterfacesPerJoint = 3;

std::vector<std::string> command_interface_names(
    const std::vector<std::string>& joints,
    const std::string& hardware_enable_interface)
{
  std::vector<std::string> names;
  const std::size_t interface_count =
      kBaseCommandInterfacesPerJoint +
      (hardware_enable_interface.empty() ? 0U : 1U);
  names.reserve(joints.size() * interface_count);
  for (const auto& joint : joints)
  {
    names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    if (!hardware_enable_interface.empty())
    {
      names.push_back(joint + "/" + hardware_enable_interface);
    }
  }
  return names;
}

std::vector<std::string> state_interface_names(
    const std::vector<std::string>& joints)
{
  std::vector<std::string> names;
  names.reserve(joints.size() * kStateInterfacesPerJoint);
  for (const auto& joint : joints)
  {
    names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return names;
}
}  // namespace

controller_interface::CallbackReturn A6YGravityCompController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<bool>("enable_output", false);
  auto_declare<bool>("live_tuning_enabled", false);
  auto_declare<bool>("planned_home_enabled", false);
  auto_declare<bool>("teaching_follow_enabled", false);
  auto_declare<bool>("teaching_position_output", false);
  auto_declare<bool>("home_mock_follow", false);
  auto_declare<std::vector<double>>("feedback_velocity_limits", {});
  auto_declare<std::string>("hardware_enable_interface", "");
  auto_declare<double>("torque_scale", 0.0);
  auto_declare<double>("damping_gain", 0.0);
  auto_declare<double>("torque_limit", 0.0);
  auto_declare<bool>("friction_compensation_enabled", false);
  auto_declare<std::vector<double>>("friction_velocity_deadbands", {});
  auto_declare<std::vector<double>>("friction_velocity_scales", {});
  auto_declare<double>("friction_ramp_duration", 0.2);
  auto_declare<bool>("friction_pulse_enabled", false);
  auto_declare<double>("friction_intent_velocity_threshold", 0.01);
  auto_declare<double>("friction_release_velocity_threshold", 0.003);
  auto_declare<double>("friction_pulse_duration", 0.12);
  auto_declare<double>("friction_decay_duration", 0.18);
  auto_declare<double>("friction_retrigger_delay", 0.10);
  auto_declare<bool>("haptic_enable_output", false);
  auto_declare<std::string>("haptic_effort_topic", "/leader/arm_force_feedback/effort");
  auto_declare<double>("haptic_command_timeout", 0.1);
  auto_declare<std::vector<double>>("haptic_effort_limits", {});
  auto_declare<double>("startup_ramp_duration", 0.0);
  auto_declare<std::string>("haptic_output_enable_topic", "");
  auto_declare<double>("haptic_output_enable_timeout", 0.1);
  auto_declare<double>("haptic_activation_ramp_duration", 2.0);
  auto_declare<std::vector<double>>("damping_gains", {});
  auto_declare<std::vector<double>>("torque_limits", {});
  auto_declare<std::vector<double>>("torque_scales", {});
  auto_declare<std::vector<double>>("effort_signs", {});
  auto_declare<std::vector<double>>("friction_compensations", {});
  auto_declare<std::string>("urdf_path", "");
  auto_declare<std::string>("urdf_package", "a5_description");
  auto_declare<std::string>("urdf_relative_path", "urdf/A5.urdf");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration A6YGravityCompController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interface_names(joint_names_, hardware_enable_interface_)};
}

controller_interface::InterfaceConfiguration A6YGravityCompController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    state_interface_names(joint_names_)};
}

controller_interface::CallbackReturn A6YGravityCompController::on_configure(const rclcpp_lifecycle::State&)
{
  tuning_service_.reset();
  home_service_.reset();reset_home();
  follow_service_.reset();follow_sub_.reset();reset_follow();
  follow_enabled_=get_node()->get_parameter("teaching_follow_enabled").as_bool();
  home_enabled_=get_node()->get_parameter("planned_home_enabled").as_bool();
  home_mock_follow_=get_node()->get_parameter("home_mock_follow").as_bool();
  feedback_velocity_limits_=get_node()->get_parameter("feedback_velocity_limits").as_double_array();
  if((home_enabled_ || follow_enabled_) &&
     (feedback_velocity_limits_.size()!=6 ||
      std::any_of(feedback_velocity_limits_.begin(),feedback_velocity_limits_.end(),
                  [](double v){return !std::isfinite(v) || v<=0.;}))) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Home/teaching requires six positive feedback_velocity_limits from the motor register record");
    return controller_interface::CallbackReturn::ERROR;
  }
  teaching_position_output_=get_node()->get_parameter("teaching_position_output").as_bool();
  if(teaching_position_output_ && (!home_enabled_ || !follow_enabled_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Position output requires teaching and home services");
    return controller_interface::CallbackReturn::ERROR;
  }
  live_tuning_enabled_ = get_node()->get_parameter("live_tuning_enabled").as_bool();
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  enable_output_ = get_node()->get_parameter("enable_output").as_bool();
  hardware_enable_interface_ =
      get_node()->get_parameter("hardware_enable_interface").as_string();
  torque_scale_ = get_node()->get_parameter("torque_scale").as_double();
  damping_gain_ = get_node()->get_parameter("damping_gain").as_double();
  torque_limit_ = get_node()->get_parameter("torque_limit").as_double();
  friction_compensation_enabled_ = get_node()->get_parameter("friction_compensation_enabled").as_bool();
  friction_ramp_duration_ = get_node()->get_parameter("friction_ramp_duration").as_double();
  friction_pulse_enabled_ = get_node()->get_parameter("friction_pulse_enabled").as_bool();
  friction_intent_velocity_threshold_ = get_node()->get_parameter("friction_intent_velocity_threshold").as_double();
  friction_release_velocity_threshold_ = get_node()->get_parameter("friction_release_velocity_threshold").as_double();
  friction_pulse_duration_ = get_node()->get_parameter("friction_pulse_duration").as_double();
  friction_decay_duration_ = get_node()->get_parameter("friction_decay_duration").as_double();
  friction_retrigger_delay_ = get_node()->get_parameter("friction_retrigger_delay").as_double();
  haptic_enable_output_ = get_node()->get_parameter("haptic_enable_output").as_bool();
  haptic_effort_topic_ = get_node()->get_parameter("haptic_effort_topic").as_string();
  haptic_command_timeout_ = get_node()->get_parameter("haptic_command_timeout").as_double();
  startup_ramp_duration_ = get_node()->get_parameter("startup_ramp_duration").as_double();
  haptic_output_enable_topic_ = get_node()->get_parameter("haptic_output_enable_topic").as_string();
  haptic_output_enable_timeout_ = get_node()->get_parameter("haptic_output_enable_timeout").as_double();
  haptic_activation_ramp_duration_ =
      get_node()->get_parameter("haptic_activation_ramp_duration").as_double();
  urdf_path_ = get_node()->get_parameter("urdf_path").as_string();
  urdf_package_ = get_node()->get_parameter("urdf_package").as_string();
  urdf_relative_path_ = get_node()->get_parameter("urdf_relative_path").as_string();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must contain at least one joint.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!hardware_enable_interface_.empty() &&
      hardware_enable_interface_ != "motor_enable")
  {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Parameter 'hardware_enable_interface' must be empty or motor_enable.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(torque_scale_) || torque_scale_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'torque_scale' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(damping_gain_) || damping_gain_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'damping_gain' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(torque_limit_) || torque_limit_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'torque_limit' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (friction_compensation_enabled_ && friction_pulse_enabled_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Continuous friction and legacy friction pulse are mutually exclusive.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (live_tuning_enabled_ && friction_pulse_enabled_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Live tuning requires legacy friction pulse to be disabled.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(friction_ramp_duration_) || friction_ramp_duration_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'friction_ramp_duration' must be finite and positive.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(friction_intent_velocity_threshold_) || friction_intent_velocity_threshold_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'friction_intent_velocity_threshold' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(friction_release_velocity_threshold_) || friction_release_velocity_threshold_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'friction_release_velocity_threshold' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (friction_release_velocity_threshold_ > friction_intent_velocity_threshold_)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'friction_release_velocity_threshold' must be <= friction_intent_velocity_threshold.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(friction_pulse_duration_) || friction_pulse_duration_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'friction_pulse_duration' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(friction_decay_duration_) || friction_decay_duration_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'friction_decay_duration' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(friction_retrigger_delay_) || friction_retrigger_delay_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'friction_retrigger_delay' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (haptic_effort_topic_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'haptic_effort_topic' must not be empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(haptic_command_timeout_) || haptic_command_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'haptic_command_timeout' must be finite and positive.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(startup_ramp_duration_) || startup_ramp_duration_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'startup_ramp_duration' must be finite and non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!haptic_output_enable_topic_.empty() &&
      (!std::isfinite(haptic_output_enable_timeout_) || haptic_output_enable_timeout_ <= 0.0))
  {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Parameter 'haptic_output_enable_timeout' must be finite and positive when "
        "haptic_output_enable_topic is set.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!haptic_output_enable_topic_.empty() &&
      (!std::isfinite(haptic_activation_ramp_duration_) || haptic_activation_ramp_duration_ <= 0.0))
  {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Parameter 'haptic_activation_ramp_duration' must be finite and positive when "
        "haptic_output_enable_topic is set.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!configure_vector_parameter("damping_gains", damping_gain_, damping_gains_) ||
      !configure_vector_parameter("torque_limits", torque_limit_, torque_limits_) ||
      !configure_vector_parameter("torque_scales", torque_scale_, torque_scales_) ||
      !configure_vector_parameter("haptic_effort_limits", 0.0, haptic_effort_limits_) ||
      !configure_signed_vector_parameter("friction_compensations", 0.0, friction_compensations_) ||
      !configure_vector_parameter("friction_velocity_deadbands", 0.02, friction_velocity_deadbands_) ||
      !configure_vector_parameter("friction_velocity_scales", 0.05, friction_velocity_scales_) ||
      !configure_effort_signs())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (friction_velocity_deadbands_[i] <= 0.0 || friction_velocity_scales_[i] <= 0.0 ||
        (friction_compensation_enabled_ &&
         (friction_compensations_[i] < 0.0 ||
          (friction_compensations_[i] > 0.0 &&
           (torque_limits_[i] <= 0.0 || friction_compensations_[i] > torque_limits_[i])))))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid friction parameters for %s: deadband/scale must be positive; "
                   "continuous amplitude must be non-negative and bounded by a positive torque limit.",
                   joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  smooth_friction_states_.assign(joint_names_.size(), SmoothFrictionCompensation{});
  if (!configure_pinocchio_model() || !configure_pinocchio_joint_mapping())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  reset_friction_pulse_state();
  reset_haptic_command();
  reset_haptic_output_gate();
  base_activation_ramp_.reset();
  requested_tuning_ = LiveTuning{torque_scales_, damping_gains_, friction_compensations_,
                               friction_compensation_enabled_, 0};
  tuning_buffer_.writeFromNonRT(requested_tuning_);
  applied_tuning_revision_.store(0);
  rt_tuning_revision_ = 0;
  tuning_remaining_seconds_ = 0.0;
  if (live_tuning_enabled_)
  {
    tuning_service_ = get_node()->create_service<TuningService>(
      "~/tune", [this](const TuningService::Request::SharedPtr request,
                        TuningService::Response::SharedPtr response) {
        accept_live_tuning(*request, *response);
      });
  }
  if(home_enabled_) {
    for(int i=0;i<6;++i) {
      home_lower_[i]=pinocchio_model_->lowerPositionLimit[pinocchio_q_indices_[i]];
      home_upper_[i]=pinocchio_model_->upperPositionLimit[pinocchio_q_indices_[i]];
    }
    home_service_=get_node()->create_service<HomeService>("~/home",
      [this](const HomeService::Request::SharedPtr r,HomeService::Response::SharedPtr out) {accept_home(*r,*out);});
    if(follow_enabled_) {
      follow_service_=get_node()->create_service<FollowService>("~/teaching",
        [this](const FollowService::Request::SharedPtr r,FollowService::Response::SharedPtr out) {accept_follow(*r,*out);});
      follow_sub_=get_node()->create_subscription<FollowReference>("~/teaching_reference",rclcpp::QoS(1).best_effort(),
        [this](const FollowReference::SharedPtr r) {accept_follow_reference(*r);});
    }
  }
  haptic_effort_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    haptic_effort_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr message)
    {
      accept_haptic_command(message);
    });
  if (!haptic_output_enable_topic_.empty())
  {
    haptic_output_gate_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        haptic_output_enable_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr message) { accept_haptic_output_gate(message); });
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured A6YGravityCompController for %zu joints. enable_output=%s, torque_scale=%.3f, "
              "urdf=%s",
              joint_names_.size(), enable_output_ ? "true" : "false", torque_scale_, urdf_path_.c_str());
  RCLCPP_INFO(get_node()->get_logger(),
              "Per-joint torque_scales, effort_signs, and friction_compensations are active. "
              "friction_compensation_enabled=%s, friction_pulse_enabled=%s, haptic_enable_output=%s, haptic_effort_topic=%s, "
              "startup_ramp=%.3f, haptic_output_enable_topic=%s, haptic_ramp=%.3f",
              friction_compensation_enabled_ ? "true" : "false",
              friction_pulse_enabled_ ? "true" : "false", haptic_enable_output_ ? "true" : "false",
              haptic_effort_topic_.c_str(), startup_ramp_duration_,
              haptic_output_enable_topic_.empty() ? "<always>" : haptic_output_enable_topic_.c_str(),
              haptic_activation_ramp_duration_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn A6YGravityCompController::on_activate(const rclcpp_lifecycle::State&)
{
  reset_home();
  // Inactive edits must be part of the complete preload before motor enable.
  update_live_tuning(0.0, true);
  reset_friction_pulse_state();
  reset_haptic_command();
  reset_haptic_output_gate();
  base_activation_ramp_.reset();
  if (!write_zero_commands())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  home_controller_active_.store(true);
  if(teaching_position_output_) {
    for(int i=0;i<6;++i) position_target_[i]=std::clamp(
      command_interfaces_[i*command_interfaces_per_joint()].get_value(),home_lower_[i],home_upper_[i]);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn A6YGravityCompController::on_deactivate(const rclcpp_lifecycle::State&)
{
  reset_home();
  reset_friction_pulse_state();
  reset_haptic_command();
  reset_haptic_output_gate();
  base_activation_ramp_.reset();
  return write_zero_commands() ?
         controller_interface::CallbackReturn::SUCCESS :
         controller_interface::CallbackReturn::ERROR;
}

controller_interface::return_type A6YGravityCompController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (!validate_interface_counts())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Interface count mismatch.");
    return controller_interface::return_type::ERROR;
  }
  if (!read_state(q_, dq_))
  {
    write_zero_commands();
    return controller_interface::return_type::ERROR;
  }
  update_live_tuning(period.seconds());

  const double staged_now_seconds = staged_output_steady_time_seconds();
  double base_activation_scale = enable_output_ ? 1.0 : 0.0;
  if (enable_output_ && startup_ramp_duration_ > 0.0)
  {
    base_activation_scale = base_activation_ramp_.update(
        true, staged_now_seconds, startup_ramp_duration_);
  }

  const HapticCommand* haptic_command = haptic_command_buffer_.readFromRT();
  const bool haptic_command_fresh = haptic_command_is_fresh(
    staged_now_seconds, haptic_command->received_time_seconds, haptic_command_timeout_,
    haptic_command->efforts.size(), joint_names_.size());

  double haptic_activation_scale = enable_output_ ? 1.0 : 0.0;
  if (!haptic_output_enable_topic_.empty())
  {
    const TimedOutputGateCommand* output_gate = haptic_output_gate_buffer_.readFromRT();
    const bool gate_active = timed_output_gate_is_active(
        staged_now_seconds, haptic_output_enable_timeout_, *output_gate);
    haptic_activation_scale = haptic_activation_ramp_.update(
        staged_output_is_ready(enable_output_, gate_active, haptic_command_fresh),
        staged_now_seconds, haptic_activation_ramp_duration_);
  }

  Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
  if (!teaching_position_output_ && base_activation_scale > 0.0 &&
      std::any_of(torque_scales_.begin(), torque_scales_.end(), [](double scale) { return scale > 0.0; }))
  {
    try
    {
      pinocchio::computeGeneralizedGravity(*pinocchio_model_, *pinocchio_data_, q_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio gravity computation failed: %s", e.what());
      write_zero_commands();
      return controller_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      tau_gravity[static_cast<Eigen::Index>(i)] =
          pinocchio_data_->g[static_cast<Eigen::Index>(pinocchio_v_indices_[i])];
      if (!std::isfinite(tau_gravity[static_cast<Eigen::Index>(i)]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Non-finite gravity effort for %s.", joint_names_[i].c_str());
        write_zero_commands();
        return controller_interface::return_type::ERROR;
      }
    }
  }

  JointVector return_effort=JointVector::Zero();
  bool returning=false;
  try {
    returning=home_effort(period.seconds(),staged_now_seconds,tau_gravity,return_effort);
    if(!returning) returning=follow_effort(period.seconds(),staged_now_seconds,tau_gravity,return_effort);
  }
  catch(const std::exception&) {write_zero_commands();return controller_interface::return_type::ERROR;}
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::size_t offset = i * command_interfaces_per_joint();
    const double velocity = dq_[static_cast<Eigen::Index>(pinocchio_v_indices_[i])];
    const double now_seconds = time.seconds();
    double effort = 0.0;
    if (enable_output_ && !teaching_position_output_)
    {
      const double requested_base_effort =
          base_activation_scale *
          (effort_signs_[i] * torque_scales_[i] *
               tau_gravity[static_cast<Eigen::Index>(i)] -
           damping_gains_[i] * velocity +
           friction_compensation_effort(velocity, i, now_seconds, period.seconds()));
      const double requested_haptic_effort =
        haptic_activation_scale * (haptic_command_fresh ? haptic_command->efforts[i] : 0.0);
      const auto composition = compose_haptic_effort(
        requested_base_effort, requested_haptic_effort, haptic_effort_limits_[i],
        torque_limits_[i], haptic_enable_output_, haptic_command_fresh);
      effort = composition.output_effort;
      if (std::abs(effort - composition.requested_total_effort) > 1e-9)
      {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Total effort for %s clamped: requested %.3f Nm, sending %.3f Nm (limit %.3f Nm).",
                             joint_names_[i].c_str(), composition.requested_total_effort, effort,
                             torque_limits_[i]);
      }
    }

    if(returning) effort=i<6?return_effort[i]:0.;
    command_interfaces_[offset + 0].set_value(q_[static_cast<Eigen::Index>(pinocchio_q_indices_[i])]);
    if(returning && (home_mock_follow_ || teaching_position_output_) && i<6) {
      if(follow_phase_.load()!=0) command_interfaces_[offset].set_value(follow_q_[i]);
      else {
        std::array<double,6> reference{},speed{};
        home_buffer_.readFromRT()->sample(home_elapsed_,reference,speed);
        command_interfaces_[offset].set_value(reference[i]);
      }
    }
    if(teaching_position_output_ && i<6) {
      if(returning) position_target_[i]=command_interfaces_[offset].get_value();
      command_interfaces_[offset].set_value(std::clamp(position_target_[i],home_lower_[i],home_upper_[i]));
      effort=0.; // POS_VEL firmware owns the position loop; no torque servo.
    }
    command_interfaces_[offset + 1].set_value(0.0);
    command_interfaces_[offset + 2].set_value(effort);
    if (!hardware_enable_interface_.empty())
    {
      command_interfaces_[offset + 3].set_value(
          gravity_motor_enable_command(
              hardware_enable_interface_, enable_output_,
              base_activation_scale));
    }
  }

  return controller_interface::return_type::OK;
}

bool A6YGravityCompController::write_zero_commands()
{
  home_controller_active_.store(false);
  for (auto& state : smooth_friction_states_) state.reset();
  if (!validate_interface_counts())
  {
    return false;
  }

  bool all_measured_positions_are_finite = true;
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::size_t command_offset =
        i * command_interfaces_per_joint();
    const std::size_t state_offset = i * kStateInterfacesPerJoint;
    const auto command = resolve_safe_zero_effort_command(
      state_interfaces_[state_offset + 0].get_value(),
      command_interfaces_[command_offset + 0].get_value());
    all_measured_positions_are_finite &=
      command.measured_position_is_finite;
    command_interfaces_[command_offset + 0].set_value(command.position);
    command_interfaces_[command_offset + 1].set_value(command.velocity);
    command_interfaces_[command_offset + 2].set_value(command.effort);
    if (!hardware_enable_interface_.empty())
    {
      command_interfaces_[command_offset + 3].set_value(0.0);
    }
  }
  return all_measured_positions_are_finite;
}

bool A6YGravityCompController::configure_pinocchio_model()
{
  if (urdf_path_.empty())
  {
    try
    {
      urdf_path_ = ament_index_cpp::get_package_share_directory(urdf_package_) + "/" + urdf_relative_path_;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to resolve URDF package '%s': %s", urdf_package_.c_str(),
                   e.what());
      return false;
    }
  }

  try
  {
    pinocchio_model_ = std::make_unique<pinocchio::Model>();
    // One DM3507 drives both fingers. Honor URDF mimic constraints so the
    // second finger follows the measured opening and its force is projected
    // onto the independent gripper coordinate by Pinocchio.
    load_mimic_model(urdf_path_, *pinocchio_model_);
    pinocchio_data_ = std::make_unique<pinocchio::Data>(*pinocchio_model_);
    q_ = Eigen::VectorXd::Zero(pinocchio_model_->nq);
    dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build Pinocchio model from %s: %s", urdf_path_.c_str(),
                 e.what());
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Loaded Pinocchio model: nq=%d nv=%d joints=%d", pinocchio_model_->nq,
              pinocchio_model_->nv, static_cast<int>(pinocchio_model_->njoints));
  return true;
}

bool A6YGravityCompController::configure_pinocchio_joint_mapping()
{
  try {
    map_independent_joints(*pinocchio_model_, joint_names_, pinocchio_q_indices_, pinocchio_v_indices_);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid model joint mapping: %s", e.what());
    return false;
  }
}

bool A6YGravityCompController::configure_vector_parameter(const std::string& name, double scalar_default,
                                                         std::vector<double>& values)
{
  values = get_node()->get_parameter(name).as_double_array();
  if (values.empty())
  {
    values.assign(joint_names_.size(), scalar_default);
    return true;
  }
  if (values.size() != joint_names_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' must be empty or contain %zu values, got %zu.",
                 name.c_str(), joint_names_.size(), values.size());
    return false;
  }
  for (double value : values)
  {
    if (!std::isfinite(value) || value < 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' values must be finite and non-negative.", name.c_str());
      return false;
    }
  }
  return true;
}

bool A6YGravityCompController::configure_signed_vector_parameter(const std::string& name, double scalar_default,
                                                                std::vector<double>& values)
{
  values = get_node()->get_parameter(name).as_double_array();
  if (values.empty())
  {
    values.assign(joint_names_.size(), scalar_default);
    return true;
  }
  if (values.size() != joint_names_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' must be empty or contain %zu values, got %zu.",
                 name.c_str(), joint_names_.size(), values.size());
    return false;
  }
  for (double value : values)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' values must be finite.", name.c_str());
      return false;
    }
  }
  return true;
}

bool A6YGravityCompController::configure_effort_signs()
{
  effort_signs_ = get_node()->get_parameter("effort_signs").as_double_array();
  if (effort_signs_.empty())
  {
    effort_signs_.assign(joint_names_.size(), 1.0);
    return true;
  }
  if (effort_signs_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'effort_signs' must be empty or contain %zu values, got %zu.",
                 joint_names_.size(), effort_signs_.size());
    return false;
  }
  for (double& value : effort_signs_)
  {
    if (value >= 0.0)
    {
      value = 1.0;
    }
    else
    {
      value = -1.0;
    }
  }
  return true;
}

bool A6YGravityCompController::read_state(Eigen::VectorXd& q, Eigen::VectorXd& dq)
{
  q.setZero();
  dq.setZero();
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::size_t offset = i * kStateInterfacesPerJoint;
    const double position = state_interfaces_[offset + 0].get_value();
    const double velocity = state_interfaces_[offset + 1].get_value();
    if (!std::isfinite(position) || !std::isfinite(velocity))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite state for %s.", joint_names_[i].c_str());
      return false;
    }
    q[static_cast<Eigen::Index>(pinocchio_q_indices_[i])] = position;
    dq[static_cast<Eigen::Index>(pinocchio_v_indices_[i])] = velocity;
  }
  return true;
}

bool A6YGravityCompController::validate_interface_counts() const
{
  return command_interfaces_.size() ==
             joint_names_.size() * command_interfaces_per_joint() &&
         state_interfaces_.size() ==
             joint_names_.size() * kStateInterfacesPerJoint;
}

std::size_t A6YGravityCompController::command_interfaces_per_joint() const
{
  return kBaseCommandInterfacesPerJoint +
         (hardware_enable_interface_.empty() ? 0U : 1U);
}

double A6YGravityCompController::friction_compensation_effort(double velocity, std::size_t joint_index,
                                                             double now_seconds, double period_seconds)
{
  const double compensation = friction_compensations_[joint_index];
  if (friction_compensation_enabled_)
  {
    return smooth_friction_states_[joint_index].update(
        velocity, period_seconds, compensation, friction_velocity_deadbands_[joint_index],
        friction_velocity_scales_[joint_index], friction_ramp_duration_);
  }
  if (!friction_pulse_enabled_ || compensation == 0.0 || !std::isfinite(velocity) || !std::isfinite(now_seconds))
  {
    return 0.0;
  }

  const double abs_velocity = std::abs(velocity);
  int& pulse_direction = friction_pulse_directions_[joint_index];
  double& pulse_start_time = friction_pulse_start_times_[joint_index];
  double& last_trigger_time = friction_last_trigger_times_[joint_index];
  char& pulse_armed = friction_pulse_armed_[joint_index];

  if (pulse_direction != 0)
  {
    const double elapsed = std::max(0.0, now_seconds - pulse_start_time);
    const double full_duration = friction_pulse_duration_;
    const double total_duration = friction_pulse_duration_ + friction_decay_duration_;
    if (elapsed <= total_duration)
    {
      double gain = 1.0;
      if (elapsed > full_duration)
      {
        if (friction_decay_duration_ <= 0.0)
        {
          gain = 0.0;
        }
        else
        {
          gain = 1.0 - std::clamp((elapsed - full_duration) / friction_decay_duration_, 0.0, 1.0);
        }
      }
      return compensation * gain * static_cast<double>(pulse_direction);
    }

    pulse_direction = 0;
  }

  if (abs_velocity <= friction_release_velocity_threshold_)
  {
    pulse_armed = 1;
    return 0.0;
  }

  if (!pulse_armed || abs_velocity < friction_intent_velocity_threshold_ ||
      (now_seconds - last_trigger_time) < friction_retrigger_delay_)
  {
    return 0.0;
  }

  pulse_direction = velocity > 0.0 ? 1 : -1;
  pulse_start_time = now_seconds;
  last_trigger_time = now_seconds;
  pulse_armed = 0;
  return compensation * static_cast<double>(pulse_direction);
}

void A6YGravityCompController::accept_live_tuning(
    const TuningService::Request& request, TuningService::Response& response)
{
  response.success = true;
  if (request.apply)
  {
    if(home_pending_.load() || home_phase_.load()==1 || home_phase_.load()==2 || follow_phase_.load()!=0) {
      response.success=false;response.message="Cannot tune during planned return";return;
    }
    const auto valid = [this](const std::vector<double>& values, double maximum) {
      return values.size() == joint_names_.size() &&
        std::all_of(values.begin(), values.end(), [maximum](double x) {
          return std::isfinite(x) && x >= 0.0 && x <= maximum;
        });
    };
    response.success = valid(request.torque_scales, 3.0) && valid(request.damping_gains, 5.0) &&
                       valid(request.friction_compensations, std::numeric_limits<double>::max());
    if (response.success)
    {
      for (std::size_t i = 0; i < joint_names_.size(); ++i)
      {
        if (request.friction_compensations[i] > torque_limits_[i] ||
            (joint_names_[i] == "gripper_finger_joint1" &&
             (request.torque_scales[i] != 0.0 || request.damping_gains[i] != 0.0 ||
              request.friction_compensations[i] != 0.0)))
          response.success = false;
      }
    }
    if (response.success)
    {
      requested_tuning_ = LiveTuning{request.torque_scales, request.damping_gains,
        request.friction_compensations, request.friction_compensation_enabled,
        requested_tuning_.revision + 1};
      // Commit the complete validated batch; never mutate RT-owned vectors here.
      tuning_buffer_.writeFromNonRT(requested_tuning_);
    }
    else
      response.message = "Rejected batch: seven finite nonnegative values required; scale <= 3, "
                         "damping <= 5, friction <= torque limit, gripper output must stay zero.";
  }
  response.revision = requested_tuning_.revision;
  response.applied_revision = applied_tuning_revision_.load();
  response.torque_scales = requested_tuning_.scales;
  response.damping_gains = requested_tuning_.damping;
  response.friction_compensations = requested_tuning_.friction;
  response.friction_compensation_enabled = requested_tuning_.friction_enabled;
}

void A6YGravityCompController::update_live_tuning(double period_seconds, bool immediate)
{
  if (!live_tuning_enabled_) return;
  const auto* target = tuning_buffer_.readFromRT();
  if (immediate)
  {
    torque_scales_ = target->scales;
    damping_gains_ = target->damping;
    friction_compensations_ = target->friction;
    friction_compensation_enabled_ = target->friction_enabled;
    rt_tuning_revision_ = target->revision;
    tuning_remaining_seconds_ = 0.0;
    applied_tuning_revision_.store(target->revision);
    return;
  }
  if (target->revision != rt_tuning_revision_)
  {
    rt_tuning_revision_ = target->revision;
    tuning_remaining_seconds_ = 0.3;
    if (!friction_compensation_enabled_)
      std::fill(friction_compensations_.begin(), friction_compensations_.end(), 0.0);
    friction_compensation_enabled_ = friction_compensation_enabled_ || target->friction_enabled;
  }
  if (tuning_remaining_seconds_ <= 0.0 || !std::isfinite(period_seconds) ||
      period_seconds <= 0.0 || period_seconds > 0.1) return;
  const double fraction = std::min(period_seconds / tuning_remaining_seconds_, 1.0);
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    torque_scales_[i] += (target->scales[i] - torque_scales_[i]) * fraction;
    damping_gains_[i] += (target->damping[i] - damping_gains_[i]) * fraction;
    const double amplitude = target->friction_enabled ? target->friction[i] : 0.0;
    friction_compensations_[i] += (amplitude - friction_compensations_[i]) * fraction;
  }
  tuning_remaining_seconds_ = std::max(0.0, tuning_remaining_seconds_ - period_seconds);
  if (tuning_remaining_seconds_ <= 1e-9)
  {
    friction_compensation_enabled_ = target->friction_enabled;
    tuning_remaining_seconds_ = 0.0;
    applied_tuning_revision_.store(target->revision);
  }
}

void A6YGravityCompController::accept_haptic_command(
  const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  if (message->data.size() != joint_names_.size())
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Ignoring haptic effort command with %zu values; expected %zu.",
      message->data.size(), joint_names_.size());
    reset_haptic_command();
    return;
  }

  for (const double effort : message->data)
  {
    if (!std::isfinite(effort))
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ignoring haptic effort command containing a non-finite value.");
      reset_haptic_command();
      return;
    }
  }

  HapticCommand command;
  command.efforts.assign(message->data.begin(), message->data.end());
  command.received_time_seconds = staged_output_steady_time_seconds();
  haptic_command_buffer_.writeFromNonRT(command);
}

void A6YGravityCompController::reset_haptic_command()
{
  HapticCommand command;
  command.efforts.assign(joint_names_.size(), 0.0);
  haptic_command_buffer_.writeFromNonRT(command);
}

void A6YGravityCompController::accept_haptic_output_gate(
    const std_msgs::msg::Bool::SharedPtr message)
{
  haptic_output_gate_buffer_.writeFromNonRT(
      TimedOutputGateCommand{message->data,
                             staged_output_steady_time_seconds()});
}

void A6YGravityCompController::reset_haptic_output_gate()
{
  haptic_output_gate_buffer_.writeFromNonRT(TimedOutputGateCommand{});
  haptic_activation_ramp_.reset();
}

void A6YGravityCompController::reset_friction_pulse_state()
{
  friction_pulse_directions_.assign(joint_names_.size(), 0);
  friction_pulse_start_times_.assign(joint_names_.size(), 0.0);
  friction_last_trigger_times_.assign(joint_names_.size(), -std::numeric_limits<double>::infinity());
  friction_pulse_armed_.assign(joint_names_.size(), 1);
}
}  // namespace a6_y_controllers

PLUGINLIB_EXPORT_CLASS(a6_y_controllers::A6YGravityCompController, controller_interface::ControllerInterface)

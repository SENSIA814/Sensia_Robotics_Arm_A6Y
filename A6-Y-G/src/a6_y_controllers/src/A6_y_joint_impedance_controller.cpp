#include <a6_y_controllers/A6_y_joint_impedance_controller.hpp>
#include <a6_y_controllers/a5_model.hpp>
#include <a6_y_controllers/safe_zero_effort_command.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <a6_y_controllers/bounded_joint_impedance.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <fstream>
#include <chrono>

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

controller_interface::CallbackReturn A6YJointImpedanceController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", kJoints);
  auto_declare<std::string>("urdf_path", "");
  auto_declare<bool>("enable_output", false);
  auto_declare<std::vector<double>>("position_velocity_limit", {.03,.03,.03,.03,.03,.03});
  auto_declare<double>("position_prepare_duration", .5);
  auto_declare<bool>("vr_gripper_control", false);
  auto_declare<double>("vr_gripper_speed", .003);
  auto_declare<double>("vr_gripper_acceleration", .01);
  auto_declare<double>("vr_gripper_actual_speed", .01);
  auto_declare<double>("vr_gripper_tracking_error", .004);
  auto_declare<double>("impedance_ramp_duration", 2.0);
  auto_declare<double>("max_control_period", 0.1);
  auto_declare<double>("max_feedback_age", 0.1);
  auto_declare<std::vector<double>>("torque_scales", {1, 1, 1, 1, 1, 1, 0});
  auto_declare<std::vector<double>>("effort_signs", {1, 1, 1, 1, 1, 1, 1});
  auto_declare<std::vector<double>>("damping_gains", {0.01, 0.01, 0.01, 0.01, 0.01, 0.03, 0});
  auto_declare<std::vector<double>>("torque_limits", {10, 20, 10, 3, 4, 4, 30});
  // Trial values in joint order; not Cartesian gains or calibrated hardware limits.
  auto_declare<std::vector<double>>("joint_stiffness", {6, 8, 6, 1.5, 1.5, 1});
  auto_declare<std::vector<double>>("joint_damping", {1.2, 1.5, 1.2, 0.12, 0.12, 0.08});
  // Zero disables this per-axis spring displacement cap. The fixed target and integral use full error.
  auto_declare<std::vector<double>>("spring_error_limits", {0, 0, 0, 0, 0, 0});
  // Optional slow position-error correction. Zero gains preserve pure impedance.
  auto_declare<std::vector<double>>("joint_integral_gains", {0, 0, 0, 0, 0, 0});
  auto_declare<std::vector<double>>("integral_torque_limits", {1.8, 1.8, 1.8, .6, .6, .6});
  auto_declare<double>("integral_deadband", 0.005);
  auto_declare<double>("integral_error_window", 0.35);
  auto_declare<double>("integral_velocity_threshold", 0.25);
  auto_declare<std::vector<double>>("impedance_torque_limits", {1.8, 1.8, 1.8, 0.6, 0.6, 0.6});
  auto_declare<std::string>("reference_mode", "captured");
  auto_declare<bool>("vr_allow_prepare", false);
  auto_declare<bool>("vr_allow_arm", true);
  auto_declare<bool>("vr_auto_rearm_after_release", false);
  auto_declare<bool>("vr_trajectory_goals", false);
  auto_declare<bool>("vr_tracking_governor", false);
  auto_declare<bool>("vr_ramp_gravity_on_prepare", false);
  auto_declare<double>("vr_gravity_ramp_duration", 2.0);
  auto_declare<double>("vr_session_reference_travel", 0.);
  auto_declare<double>("vr_session_actual_travel", 0.);
  auto_declare<bool>("vr_allow_test_input", false);
  auto_declare<std::vector<double>>("vr_lower", {-2.62,0,0,-1.55,-.9,-3.14});
  auto_declare<std::vector<double>>("vr_upper", {2.62,3.16,3.16,1.55,.9,3.14});
  auto_declare<std::vector<double>>("vr_zero_tolerance", {0.,0.,0.,0.,0.,0.,0.});
  auto_declare<double>("vr_gripper_lower", -.03);
  auto_declare<double>("vr_gripper_upper", 0.);
  auto_declare<std::vector<double>>("vr_reference_velocity", {.1,.1,.1,.1,.1,.1});
  auto_declare<std::vector<double>>("vr_reference_acceleration", {.3,.3,.3,.3,.3,.3});
  auto_declare<std::vector<double>>("vr_actual_velocity", {.25,.25,.25,.25,.25,.25});
  auto_declare<std::vector<double>>("vr_impedance_slew", {3,3,3,1,1,1});
  for (const auto& pair : std::vector<std::pair<std::string,double>>{
      {"vr_tracking_error",.05},{"vr_feedback_tolerance",.005},{"vr_input_timeout",.15},
      {"vr_reference_timeout",.05},{"vr_settle_speed",.03},{"vr_settle_error",.02},
      {"vr_settle_duration",.15},{"vr_stop_timeout",2.},{"vr_saturation_timeout",.5}})
    auto_declare<double>(pair.first,pair.second);
  return controller_interface::CallbackReturn::SUCCESS;
}

uint64_t A6YJointImpedanceController::steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void A6YJointImpedanceController::configure_vr()
{
  vr_reference_sub_.reset();vr_control_sub_.reset();vr_state_pub_.reset();reference_gate_.reset();
  const auto mode=get_node()->get_parameter("reference_mode").as_string();
  if (mode!="captured" && mode!="external_vr") throw std::runtime_error("Invalid reference_mode");
  external_vr_=mode=="external_vr";
  if(position_output() && !external_vr_) throw std::runtime_error("Position output requires external_vr permission gate");
  if (!external_vr_) return;
  if (integral_gains_.cwiseAbs().maxCoeff()!=0)
    throw std::runtime_error("external_vr requires zero joint_integral_gains");
  JointReferenceGate::Config cfg;
  auto load=[this](const char* key,auto& out) {
    const auto values=get_node()->get_parameter(key).as_double_array();
    if(values.size()!=6) throw std::runtime_error(std::string(key)+" needs six entries");
    std::copy(values.begin(),values.end(),out.begin());
  };
  load("vr_lower",cfg.lower);load("vr_upper",cfg.upper);
  const auto zero_tolerance=get_node()->get_parameter("vr_zero_tolerance").as_double_array();
  if(zero_tolerance.size()!=7 || std::any_of(zero_tolerance.begin(),zero_tolerance.end(),
      [](double value){return !std::isfinite(value) || value<0.;}))
    throw std::runtime_error("vr_zero_tolerance needs seven finite nonnegative entries (rad, gripper m)");
  std::copy_n(zero_tolerance.begin(),6,cfg.zero_tolerance.begin());
  vr_gripper_zero_tolerance_=zero_tolerance[6];
  vr_gripper_lower_=get_node()->get_parameter("vr_gripper_lower").as_double();
  vr_gripper_upper_=get_node()->get_parameter("vr_gripper_upper").as_double();
  if(!std::isfinite(vr_gripper_lower_) || !std::isfinite(vr_gripper_upper_) || vr_gripper_lower_>=vr_gripper_upper_)
    throw std::runtime_error("Invalid VR gripper limits");
  load("vr_reference_velocity",cfg.speed);load("vr_reference_acceleration",cfg.acceleration);
  load("vr_actual_velocity",cfg.actual_speed);
  cfg.tracking_error=get_node()->get_parameter("vr_tracking_error").as_double();
  cfg.feedback_tolerance=get_node()->get_parameter("vr_feedback_tolerance").as_double();
  cfg.input_timeout=get_node()->get_parameter("vr_input_timeout").as_double();
  cfg.reference_timeout=get_node()->get_parameter("vr_reference_timeout").as_double();
  cfg.settle_speed=get_node()->get_parameter("vr_settle_speed").as_double();
  cfg.settle_error=get_node()->get_parameter("vr_settle_error").as_double();
  cfg.settle_duration=get_node()->get_parameter("vr_settle_duration").as_double();
  cfg.stop_timeout=get_node()->get_parameter("vr_stop_timeout").as_double();
  cfg.max_period=max_period_;
  cfg.allow_prepare=get_node()->get_parameter("vr_allow_prepare").as_bool();
  cfg.allow_arm=get_node()->get_parameter("vr_allow_arm").as_bool();
  cfg.auto_rearm_after_release=get_node()->get_parameter("vr_auto_rearm_after_release").as_bool();
  cfg.trajectory_goals=get_node()->get_parameter("vr_trajectory_goals").as_bool();
  cfg.tracking_governor=get_node()->get_parameter("vr_tracking_governor").as_bool();
  if(cfg.trajectory_goals && !position_output())
    throw std::runtime_error("Trajectory goals require the native position plugin");
  cfg.gripper_control=get_node()->get_parameter("vr_gripper_control").as_bool();
  if(cfg.gripper_control && !position_output()) throw std::runtime_error("Gripper teleoperation requires native position plugin");
  cfg.gripper_lower=vr_gripper_lower_;cfg.gripper_upper=vr_gripper_upper_;
  cfg.gripper_zero_tolerance=vr_gripper_zero_tolerance_;
  cfg.gripper_speed=get_node()->get_parameter("vr_gripper_speed").as_double();
  cfg.gripper_acceleration=get_node()->get_parameter("vr_gripper_acceleration").as_double();
  cfg.gripper_actual_speed=get_node()->get_parameter("vr_gripper_actual_speed").as_double();
  cfg.gripper_tracking_error=get_node()->get_parameter("vr_gripper_tracking_error").as_double();
  vr_ramp_gravity_on_prepare_=!position_output() && get_node()->get_parameter("vr_ramp_gravity_on_prepare").as_bool();
  vr_gravity_ramp_duration_=get_node()->get_parameter("vr_gravity_ramp_duration").as_double();
  if(!std::isfinite(vr_gravity_ramp_duration_) || vr_gravity_ramp_duration_<=0)
    throw std::runtime_error("Invalid vr_gravity_ramp_duration");
  cfg.session_reference_travel=get_node()->get_parameter("vr_session_reference_travel").as_double();
  cfg.session_actual_travel=get_node()->get_parameter("vr_session_actual_travel").as_double();
  vr_allow_prepare_=cfg.allow_prepare;
  cfg.allow_test_input=get_node()->get_parameter("vr_allow_test_input").as_bool();
  if(!cfg.valid()) throw std::runtime_error("Invalid VR reference constraints");
  if(position_output()) {
    position_speed_=vector_parameter("position_velocity_limit",6,true);
    for(int i=0;i<6;++i)
      if(position_speed_[i]>cfg.actual_speed[i] || position_speed_[i]<cfg.speed[i])
        throw std::runtime_error("Position speed cap must cover reference speed and not exceed actual speed protection");
  }
  vr_slew_=vector_parameter("vr_impedance_slew",6,true);
  saturation_timeout_=get_node()->get_parameter("vr_saturation_timeout").as_double();
  if(!std::isfinite(saturation_timeout_) || saturation_timeout_<=0)
    throw std::runtime_error("Invalid vr_saturation_timeout");
  // Stable boot identifier shared with Python, independent of ROS wall time.
  std::ifstream stream("/proc/sys/kernel/random/boot_id");std::string boot;stream>>boot;
  if(boot.empty()) throw std::runtime_error("Cannot read Linux boot identifier");
  clock_id_=14695981039346656037ULL;
  for(unsigned char c:boot) {clock_id_^=c;clock_id_*=1099511628211ULL;}
  reference_gate_=std::make_unique<JointReferenceGate>(cfg);
  vr_reference_sub_=get_node()->create_subscription<JointReferenceGate::Reference>("~/vr_reference",
    rclcpp::QoS(1).best_effort().durability_volatile(),
    [this](JointReferenceGate::Reference::ConstSharedPtr msg) { vr_reference_.writeFromNonRT(*msg); });
  vr_control_sub_=get_node()->create_subscription<JointReferenceGate::Control>("~/vr_control",
    rclcpp::QoS(10).reliable().durability_volatile(),
    [this](JointReferenceGate::Control::ConstSharedPtr msg) {
      if(msg->controller_epoch!=published_epoch_.load() ||
         !JointReferenceGate::age_ok(steady_ns(),msg->generated_monotonic_ns,.5)) return;
      // Stops cannot be lost if a later arm/capture overwrites the latest buffer.
      if(msg->action==JointReferenceGate::Control::DISARM)
        urgent_vr_action_.store(msg->action);
      else if(msg->action==JointReferenceGate::Control::PAUSE) {
        uint8_t empty=0;urgent_vr_action_.compare_exchange_strong(empty,msg->action);
      }
      vr_control_.writeFromNonRT(*msg);
    });
  vr_state_pub_=std::make_unique<realtime_tools::RealtimePublisher<a6_y_teleop_msgs::msg::VrState>>(
    get_node()->create_publisher<a6_y_teleop_msgs::msg::VrState>("~/vr_state",rclcpp::QoS(10).best_effort()));
}

void A6YJointImpedanceController::publish_vr(uint64_t now,bool feedback_valid)
{
  if(!reference_gate_ || !vr_state_pub_ || !interfaces_match() || !vr_state_pub_->trylock()) return;
  auto& m=vr_state_pub_->msg_;const auto& g=*reference_gate_;
  m.clock_id=clock_id_;m.controller_epoch=g.epoch();m.takeover_id=g.takeover();
  m.producer_id=g.producer();m.session_id=g.session();m.monotonic_ns=now;
  m.anchor_monotonic_ns=g.anchor_time();m.accepted_sequence=g.accepted();
  m.request_id=g.request_id();m.request_accepted=g.request_accepted();
  m.feedback_valid=feedback_valid;m.output_permitted=vr_allow_prepare_;
  m.position_control=position_output();
  m.gripper_control=g.gripper_control();m.gripper_reference_position=g.gripper_target();
  m.gripper_anchor_position=g.gripper_anchor();
  m.prepare_ramp_complete=g.enabled() && elapsed_>=prepare_duration();
  m.follow_permitted=g.follow_permitted();
  m.auto_rearm_after_release=g.auto_rearm_after_release();
  m.session_reference_travel_limit=g.session_reference_travel();
  m.session_actual_travel_limit=g.session_actual_travel();
  m.session_origin_positions=g.session_origin();
  m.prepared=g.enabled();m.state=g.state();m.reason=g.reason();
  m.reference_positions=g.target();m.reference_velocities=g.velocity();m.anchor_positions=g.anchor();
  for(int i=0;i<7;++i) {
    m.actual_positions[i]=state_interfaces_[6*i].get_value();m.actual_velocities[i]=state_interfaces_[6*i+1].get_value();
    m.position_age[i]=state_interfaces_[6*i+4].get_value();
    m.feedback_age[i]=state_interfaces_[6*i+5].get_value();
    if(i<6) {m.impedance_effort[i]=vr_effort_[i];m.saturated[i]=vr_saturated_[i];}
  }
  vr_state_pub_->unlockAndPublish();
}

controller_interface::InterfaceConfiguration
A6YJointImpedanceController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names(joints_, kCommands)};
}
controller_interface::InterfaceConfiguration
A6YJointImpedanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names(joints_, kStates)};
}

Eigen::VectorXd A6YJointImpedanceController::vector_parameter(
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

controller_interface::CallbackReturn A6YJointImpedanceController::on_configure(
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
    position_prepare_duration_=get_node()->get_parameter("position_prepare_duration").as_double();
    if(!std::isfinite(position_prepare_duration_) || position_prepare_duration_<=0)
      throw std::runtime_error("Invalid position_prepare_duration");
    for (double value : {ramp_duration_, max_period_, max_feedback_age_})
      if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Durations must be finite and positive");
    if(!position_output()) {
    gravity_scales_ = vector_parameter("torque_scales", 7, false);
    base_damping_ = vector_parameter("damping_gains", 7, false);
    torque_limits_ = vector_parameter("torque_limits", 7, true);
    stiffness_ = vector_parameter("joint_stiffness", 6, false);
    damping_ = vector_parameter("joint_damping", 6, false);
    spring_error_limits_ = vector_parameter("spring_error_limits", 6, false);
    impedance_limits_ = vector_parameter("impedance_torque_limits", 6, true);
    integral_gains_ = vector_parameter("joint_integral_gains", 6, false);
    integral_limits_ = vector_parameter("integral_torque_limits", 6, false);
    integral_deadband_ = get_node()->get_parameter("integral_deadband").as_double();
    integral_window_ = get_node()->get_parameter("integral_error_window").as_double();
    integral_velocity_ = get_node()->get_parameter("integral_velocity_threshold").as_double();
    if (!std::isfinite(integral_deadband_) || integral_deadband_ < 0 ||
        !std::isfinite(integral_window_) || integral_window_ <= integral_deadband_ ||
        !std::isfinite(integral_velocity_) || integral_velocity_ <= 0)
      throw std::runtime_error("Invalid integral deadband/window/velocity threshold");
    for (int i = 0; i < 6; ++i)
      if (integral_gains_[i] > 0 && (stiffness_[i] <= 0 || integral_limits_[i] <= 0 ||
          integral_limits_[i] > impedance_limits_[i]))
        throw std::runtime_error("Active integral needs positive stiffness and limits within the impedance budget");
    const auto signs = get_node()->get_parameter("effort_signs").as_double_array();
    if (signs.size() != 7) throw std::runtime_error("effort_signs needs seven entries");
    gravity_signs_.resize(7);
    for (int i = 0; i < 7; ++i)
    {
      if (signs[i] != 1.0 && signs[i] != -1.0) throw std::runtime_error("effort_signs must be +1 or -1");
      gravity_signs_[i] = signs[i];
    }
    if (gravity_scales_[6] != 0 || base_damping_[6] != 0)
      throw std::runtime_error("Gripper gravity scale and damping must be zero in this mode");
    if ((impedance_limits_.array() > torque_limits_.head<6>().array()).any())
      throw std::runtime_error("Incremental impedance limits exceed total torque limits");
    } else {
      integral_gains_=Eigen::VectorXd::Zero(6);
    }

    auto urdf = get_node()->get_parameter("urdf_path").as_string();
    if (urdf.empty()) urdf = ament_index_cpp::get_package_share_directory("a5_description") + "/urdf/A5.urdf";
    model_ = std::make_unique<pinocchio::Model>();
    load_mimic_model(urdf, *model_);
    map_independent_joints(*model_, joints_, q_indices_, v_indices_);
    data_ = std::make_unique<pinocchio::Data>(*model_);
    q_ = Eigen::VectorXd::Zero(model_->nq);
    dq_ = Eigen::VectorXd::Zero(model_->nv);
    elapsed_ = 0;
    integral_.setZero();
    configure_vr();
    if(position_output()) {
      RCLCPP_INFO(get_node()->get_logger(), "VR native position output: zero feedforward effort; gripper %s; PREPARE captures actual joints",
        reference_gate_->gripper_control() ? "position control" : "read-only");
      return controller_interface::CallbackReturn::SUCCESS;
    }
    RCLCPP_INFO(get_node()->get_logger(),
      "Implicit joint impedance: K=[%.2f %.2f %.2f %.2f %.2f %.2f] Nm/rad, "
      "D=[%.2f %.2f %.2f %.2f %.2f %.2f] Nms/rad; per-joint bounds in solve.",
      stiffness_[0], stiffness_[1], stiffness_[2], stiffness_[3], stiffness_[4], stiffness_[5],
      damping_[0], damping_[1], damping_[2], damping_[3], damping_[4], damping_[5]);
    RCLCPP_INFO(get_node()->get_logger(),
      "Spring displacement caps: [%.3f %.3f %.3f %.3f %.3f %.3f] rad; zero disables a cap; fixed target and integral retain full error.",
      spring_error_limits_[0], spring_error_limits_[1], spring_error_limits_[2],
      spring_error_limits_[3], spring_error_limits_[4], spring_error_limits_[5]);
    RCLCPP_INFO(get_node()->get_logger(),
      "Slow joint return: Ki=[%.2f %.2f %.2f %.2f %.2f %.2f] Nm/(rad*s), "
      "limits=[%.2f %.2f %.2f %.2f %.2f %.2f] Nm, deadband=%.4f rad, "
      "error_clip=%.3f rad, velocity_threshold=%.3f rad/s; opposing bias resets outside deadband.",
      integral_gains_[0], integral_gains_[1], integral_gains_[2],
      integral_gains_[3], integral_gains_[4], integral_gains_[5],
      integral_limits_[0], integral_limits_[1], integral_limits_[2],
      integral_limits_[3], integral_limits_[4], integral_limits_[5],
      integral_deadband_, integral_window_, integral_velocity_);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint controller configuration: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool A6YJointImpedanceController::interfaces_match() const
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

void A6YJointImpedanceController::write_disabled()
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

controller_interface::CallbackReturn A6YJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State&)
{
  write_disabled();
  if (!model_ || !data_ || !interfaces_match()) return controller_interface::CallbackReturn::ERROR;
  captured_ = faulted_ = false;
  elapsed_ = 0;
  integral_.setZero();
  active_ = true;
  if(external_vr_) {
    const auto epoch=steady_ns();reference_gate_->reset(epoch,clock_id_);published_epoch_.store(epoch);
    vr_reference_.reset();vr_control_.reset();urgent_vr_action_.store(0);
    vr_effort_.setZero();saturation_elapsed_=0;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn A6YJointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
  active_ = captured_ = false;
  integral_.setZero();
  published_epoch_.store(0);
  write_disabled();
  return controller_interface::CallbackReturn::SUCCESS;
}
double A6YJointImpedanceController::prepare_duration() const
{
  if(position_output()) return position_prepare_duration_;
  return std::max(ramp_duration_,vr_ramp_gravity_on_prepare_ ? vr_gravity_ramp_duration_ : ramp_duration_);
}

controller_interface::return_type A6YJointImpedanceController::fail(const char* reason)
{
  faulted_ = true;
  if(reference_gate_) {
    if(reference_gate_->state()!=JointReferenceGate::FAULT) reference_gate_->fault(JointReferenceGate::COMPUTE_FAILURE);
    publish_vr(steady_ns(),false);
  }
  integral_.setZero();
  write_disabled();
  RCLCPP_ERROR(get_node()->get_logger(), "Joint %s disabled: %s", position_output() ? "position" : "impedance", reason);
  return controller_interface::return_type::ERROR;
}

controller_interface::return_type A6YJointImpedanceController::update(
    const rclcpp::Time&, const rclcpp::Duration& period)
{
  if (!active_ || faulted_) { write_disabled(); return controller_interface::return_type::ERROR; }
  if (!interfaces_match()) return fail("interface assignment changed");
  if (!enable_output_) { write_disabled(); return controller_interface::return_type::OK; }
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0 || dt > max_period_) {
    // Repeated driver disable frames may exceed a motion-control period. While
    // already disabled, emit no output and accept no PREPARE on this cycle.
    // Enabled control retains the strict period fault below.
    if (external_vr_ && reference_gate_->state()==JointReferenceGate::DISABLED) {
      write_disabled();publish_vr(steady_ns(),false);return controller_interface::return_type::OK;
    }
    return fail("invalid or excessive control period");
  }
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
    {
      // Disabled readback can wait for a complete fresh sample without latching
      // a motion fault. Enabling/following still requires strict fresh feedback.
      if (external_vr_ && reference_gate_->state()==JointReferenceGate::DISABLED) {
        write_disabled();publish_vr(steady_ns(),false);return controller_interface::return_type::OK;
      }
      return fail("invalid or stale joint feedback");
    }
    q_[q_indices_[i]] = position;
    dq_[v_indices_[i]] = velocity;
  }

  if(external_vr_) {
    JointReferenceGate::Joints q,dq;
    for(int i=0;i<6;++i) {q[i]=q_[q_indices_[i]];dq[i]=dq_[v_indices_[i]];}
    const auto now=steady_ns();
    const bool gripper_valid=JointReferenceGate::position_allowed(q_[q_indices_[6]],
      vr_gripper_lower_,vr_gripper_upper_,vr_gripper_zero_tolerance_);
    reference_gate_->step(now,dt,q,dq,true,vr_reference_.readFromRT(),vr_control_.readFromRT(),
      urgent_vr_action_.exchange(0),elapsed_>=prepare_duration(),gripper_valid,q_[q_indices_[6]],dq_[v_indices_[6]]);
    if(reference_gate_->state()==JointReferenceGate::FAULT) return fail("VR local state protection");
    if(!reference_gate_->enabled()) {
      write_disabled();vr_effort_.setZero();saturation_elapsed_=0;
      publish_vr(now,true);return controller_interface::return_type::OK;
    }
    if(reference_gate_->just_prepared()) {elapsed_=0;vr_effort_.setZero();}
    for(int i=0;i<6;++i) target_[i]=reference_gate_->target()[i];
    captured_=true;integral_.setZero();
  }

  if(position_output()) {
    // In pos_vel the velocity field is a positive motor-side speed ceiling,
    // not feedforward velocity. Sending zero would use the driver's fallback.
    for(int i=0;i<7;++i) {
      const bool gripper=reference_gate_->gripper_control();
      command_interfaces_[4*i].set_value(i<6 ? target_[i] : gripper ? reference_gate_->gripper_target() : q_[q_indices_[i]]);
      command_interfaces_[4*i+1].set_value(i<6 ? position_speed_[i] : gripper ? reference_gate_->gripper_speed() : 0.);
      command_interfaces_[4*i+2].set_value(0.);
      command_interfaces_[4*i+3].set_value(i<6 || gripper ? 1. : 0.);
    }
    elapsed_=std::min(prepare_duration(),elapsed_+dt);
    publish_vr(steady_ns(),true);
    return controller_interface::return_type::OK;
  }

  try
  {
    JointVector position, velocity;
    for (int i = 0; i < 6; ++i)
    {
      position[i] = q_[q_indices_[i]];
      velocity[i] = dq_[v_indices_[i]];
    }
    if (!captured_)
    {
      target_ = position;
      captured_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
        "Captured fixed joint target: [%.5f %.5f %.5f %.5f %.5f %.5f] rad.",
        target_[0], target_[1], target_[2], target_[3], target_[4], target_[5]);
    }
    // These URDF joints are bounded revolute joints, not continuous joints:
    // preserve their measured coordinates instead of wrapping across a stop.
    const JointVector error = target_ - position;
    const double ramp = std::min(1.0, elapsed_ / ramp_duration_);
    const double gravity_ramp = external_vr_ && vr_ramp_gravity_on_prepare_
      ? std::min(1.0, elapsed_ / vr_gravity_ramp_duration_) : 1.;
    const double prepare_duration = external_vr_ && vr_ramp_gravity_on_prepare_
      ? std::max(ramp_duration_,vr_gravity_ramp_duration_) : ramp_duration_;
    elapsed_ = std::min(prepare_duration, elapsed_ + dt);
    pinocchio::crba(*model_, *data_, q_);
    data_->M.template triangularView<Eigen::StrictlyLower>() =
      data_->M.transpose().template triangularView<Eigen::StrictlyLower>();
    JointMatrix mass;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) mass(i, j) = data_->M(v_indices_[i], v_indices_[j]);
    pinocchio::computeGeneralizedGravity(*model_, *data_, q_);
    if (!data_->g.allFinite()) return fail("nonfinite gravity compensation");
    Eigen::Matrix<double, 7, 1> commands = Eigen::Matrix<double, 7, 1>::Zero();
    JointVector lower, upper;
    for (int i = 0; i < 6; ++i)
    {
      // Supported commissioning loads gravity gradually. Normal captured/VR
      // profiles keep their existing full-gravity behavior unless opted in.
      const double gravity = gravity_ramp * gravity_signs_[i] * gravity_scales_[i] * data_->g[v_indices_[i]];
      if (!std::isfinite(gravity)) return fail("nonfinite joint gravity");
      commands[i] = std::clamp(gravity, -torque_limits_[i], torque_limits_[i]);
      lower[i] = std::max(-impedance_limits_[i], -torque_limits_[i] - commands[i]);
      upper[i] = std::min(impedance_limits_[i], torque_limits_[i] - commands[i]);
    }
    // A sign reversal outside the target deadband invalidates the old load
    // estimate. Clear it before this sample's solve, including fast pushes.
    for (int i = 0; i < 6; ++i)
      if (std::abs(error[i]) > integral_deadband_ && error[i] * integral_[i] < 0)
        integral_[i] = 0;
    // Slow integral bias estimates the static balancing torque. Reserve its
    // torque budget BEFORE solving the spring/damper. Both shifted bounds still
    // contain zero, preserving the bounded solver's damping contract.
    for (int i = 0; i < 6; ++i) integral_[i] = std::clamp(integral_[i], lower[i], upper[i]);
    const JointVector applied_integral = integral_;
    JointVector spring_error = error;
    for (int i = 0; i < 6; ++i)
      if (spring_error_limits_[i] > 0)
        spring_error[i] = std::clamp(error[i], -spring_error_limits_[i], spring_error_limits_[i]);
    const auto step = bounded_joint_impedance(mass, spring_error, velocity, ramp * stiffness_,
      damping_ + base_damping_.head<6>(), dt, lower - applied_integral, upper - applied_integral);
    if (!step.valid) return fail("bounded joint impedance solve failed");
    JointVector impedance = applied_integral + step.torque;
    if(external_vr_) {
      bool saturated=false;
      for(int i=0;i<6;++i) {
        impedance[i]=std::clamp(impedance[i],vr_effort_[i]-vr_slew_[i]*dt,vr_effort_[i]+vr_slew_[i]*dt);
        impedance[i]=std::clamp(impedance[i],lower[i],upper[i]);
        vr_saturated_[i]=step.saturated[i];saturated|=step.saturated[i];
      }
      vr_effort_=impedance;
      saturation_elapsed_=saturated ? saturation_elapsed_+dt : 0;
      if(saturation_elapsed_>saturation_timeout_ && reference_gate_->state()==JointReferenceGate::TELEOP) {
        JointReferenceGate::Joints measured;
        for(int i=0;i<6;++i) measured[i]=position[i];
        reference_gate_->request_stop(measured,steady_ns(),JointReferenceGate::SATURATION);
      }
    }
    for (int i = 0; i < 6; ++i)
      commands[i] = std::clamp(commands[i] + impedance[i], -torque_limits_[i], torque_limits_[i]);
    // Integrate for the NEXT sample only. The error window limits accumulation
    // RATE; it must not disable return when the user pushes beyond that angle.
    // The opposite bias was already reset before solving this sample. Retain
    // balancing torque inside the deadband and freeze accumulation during motion.
    if (ramp >= 1.0)
      for (int i = 0; i < 6; ++i)
      {
        const bool slow = std::abs(velocity[i]) <= integral_velocity_;
        if (std::abs(error[i]) <= integral_deadband_) continue;
        if (!slow || (step.saturated[i] && error[i] * impedance[i] >= 0)) continue;
        const double clipped_error = std::clamp(error[i], -integral_window_, integral_window_);
        const double next = integral_[i] + dt * integral_gains_[i] * clipped_error;
        integral_[i] = std::clamp(next, -integral_limits_[i], integral_limits_[i]);
      }
    if (!integral_.allFinite()) return fail("nonfinite integral state");
    const JointVector error_deg = error * (180.0 / std::acos(-1.0));
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Joint impedance status: error_deg=[%.2f %.2f %.2f %.2f %.2f %.2f], "
      "dq_rad_s=[%.3f %.3f %.3f %.3f %.3f %.3f], "
      "impedance_nm=[%.3f %.3f %.3f %.3f %.3f %.3f], "
      "integral_nm=[%.3f %.3f %.3f %.3f %.3f %.3f], "
      "saturated=[%d %d %d %d %d %d], period_ms=%.2f, solver_iterations=%d",
      error_deg[0], error_deg[1], error_deg[2], error_deg[3], error_deg[4], error_deg[5],
      velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5],
      impedance[0], impedance[1], impedance[2], impedance[3], impedance[4], impedance[5],
      applied_integral[0], applied_integral[1], applied_integral[2],
      applied_integral[3], applied_integral[4], applied_integral[5],
      int(step.saturated[0]), int(step.saturated[1]), int(step.saturated[2]),
      int(step.saturated[3]), int(step.saturated[4]), int(step.saturated[5]), 1000 * dt, step.iterations);
    // Complete finite batch before motor enable. Gripper contributes mass but receives zero effort.
    for (int i = 0; i < 7; ++i)
    {
      command_interfaces_[4 * i].set_value(q_[q_indices_[i]]);
      command_interfaces_[4 * i + 1].set_value(0.0);
      command_interfaces_[4 * i + 2].set_value(commands[i]);
      command_interfaces_[4 * i + 3].set_value(1.0);
    }
    if(external_vr_) publish_vr(steady_ns(),true);
  }
  catch (const std::exception& error) { return fail(error.what()); }
  return controller_interface::return_type::OK;
}
}  // namespace a6_y_controllers

PLUGINLIB_EXPORT_CLASS(a6_y_controllers::A6YJointImpedanceController,
                      controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(a6_y_controllers::A6YVRPositionController,
                      controller_interface::ControllerInterface)

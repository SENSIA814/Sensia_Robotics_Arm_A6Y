#ifndef A6_Y_CONTROLLERS__A6_Y_GRAVITY_COMP_CONTROLLER_HPP_
#define A6_Y_CONTROLLERS__A6_Y_GRAVITY_COMP_CONTROLLER_HPP_

#include <a6_y_controllers/staged_output_gate.hpp>
#include <a6_y_controllers/smooth_friction_compensation.hpp>
#include <a6_y_teleop_msgs/srv/gravity_tuning.hpp>
#include <a6_y_teleop_msgs/srv/gravity_home.hpp>
#include <a6_y_teleop_msgs/srv/teaching_control.hpp>
#include <a6_y_teleop_msgs/msg/teaching_reference.hpp>
#include <a6_y_controllers/gravity_home_path.hpp>
#include <a6_y_controllers/bounded_joint_impedance.hpp>

#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>

#include <limits>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace a6_y_controllers
{
class A6YGravityCompController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  using FollowService = a6_y_teleop_msgs::srv::TeachingControl;
  using FollowReference = a6_y_teleop_msgs::msg::TeachingReference;
  struct FollowCommand {
    uint64_t revision{0}, sequence{0};
    int command{0};
    double received{0};
    std::array<double,6> positions{};
  };
  void accept_follow(const FollowService::Request&, FollowService::Response&);
  void accept_follow_reference(const FollowReference&);
  bool follow_effort(double dt, double now, const Eigen::VectorXd&, JointVector&);
  void reset_follow();
  bool follow_enabled_{false};
  bool teaching_position_output_{false};
  JointVector position_target_{JointVector::Zero()};
  FollowCommand follow_requested_; // Mutually exclusive ROS callbacks only.
  uint64_t follow_next_revision_{0};
  realtime_tools::RealtimeBuffer<FollowCommand> follow_buffer_;
  std::atomic<uint64_t> follow_rt_revision_{0};
  std::atomic<int> follow_phase_{0}, follow_fault_{0};
  std::atomic<int> follow_fault_joint_{-1};
  std::atomic<double> follow_fault_value_{0.}, follow_fault_limit_{0.};
  JointVector follow_q_{JointVector::Zero()}, follow_v_{JointVector::Zero()};
  JointVector follow_integral_{JointVector::Zero()};
  double follow_elapsed_{0};
  rclcpp::Service<FollowService>::SharedPtr follow_service_;
  rclcpp::Subscription<FollowReference>::SharedPtr follow_sub_;
  using HomeService = a6_y_teleop_msgs::srv::GravityHome;
  void accept_home(const HomeService::Request&, HomeService::Response&);
  bool home_effort(double dt, double now, const Eigen::VectorXd& gravity, JointVector& effort);
  void reset_home();
  enum class HomeFault {
    None, StartPosition, StartVelocity, ControlPeriod, Heartbeat,
    TrackingError, JointSpeed, LowerLimit, UpperLimit, Solver, SettlingTimeout
  };
  void clear_home_fault();
  bool fail_home(HomeFault fault, int joint, double value, double limit,
                 double position = 0., double velocity = 0.);
  std::string home_fault_message();  // Formatting/logging stays outside the RT loop.
  std::atomic<HomeFault> home_fault_{HomeFault::None};
  std::atomic<uint64_t> home_fault_revision_{0};
  std::atomic<int> home_fault_joint_{-1};
  std::atomic<double> home_fault_value_{0.}, home_fault_limit_{0.}, home_fault_elapsed_{0.};
  std::atomic<double> home_fault_position_{0.}, home_fault_velocity_{0.};
  uint64_t home_logged_failure_revision_{0};  // Mutually-exclusive service callback only.
  bool home_enabled_{false};
  bool home_mock_follow_{false};
  std::atomic<bool> home_controller_active_{false};
  std::atomic<bool> home_pending_{false};
  std::atomic<int> home_phase_{0};
  std::atomic<uint64_t> home_revision_{0};
  std::atomic<double> home_heartbeat_{0.};
  realtime_tools::RealtimeBuffer<GravityHomePath> home_buffer_;
  rclcpp::Service<HomeService>::SharedPtr home_service_;
  uint64_t home_next_revision_{0};
  std::atomic<uint64_t> home_rt_revision_{0};
  double home_elapsed_{0}, home_settled_{0};
  JointVector home_integral_{JointVector::Zero()};
  std::array<double,6> home_lower_{}, home_upper_{};
  std::vector<double> feedback_velocity_limits_;
  struct LiveTuning
  {
    std::vector<double> scales, damping, friction;
    bool friction_enabled{false};
    uint64_t revision{0};
  };
  using TuningService = a6_y_teleop_msgs::srv::GravityTuning;
  void accept_live_tuning(const TuningService::Request& request, TuningService::Response& response);
  void update_live_tuning(double period_seconds, bool immediate = false);
  bool live_tuning_enabled_{false};
  LiveTuning requested_tuning_;  // Only the mutually-exclusive service callback writes this.
  realtime_tools::RealtimeBuffer<LiveTuning> tuning_buffer_;
  std::atomic<uint64_t> applied_tuning_revision_{0};
  uint64_t rt_tuning_revision_{0};
  double tuning_remaining_seconds_{0.0};
  rclcpp::Service<TuningService>::SharedPtr tuning_service_;

  struct HapticCommand
  {
    std::vector<double> efforts;
    double received_time_seconds{ -std::numeric_limits<double>::infinity() };
  };

  bool write_zero_commands();
  bool configure_pinocchio_model();
  bool configure_pinocchio_joint_mapping();
  bool configure_vector_parameter(const std::string& name, double scalar_default, std::vector<double>& values);
  bool configure_signed_vector_parameter(const std::string& name, double scalar_default, std::vector<double>& values);
  bool configure_effort_signs();
  bool read_state(Eigen::VectorXd& q, Eigen::VectorXd& dq);
  bool validate_interface_counts() const;
  std::size_t command_interfaces_per_joint() const;
  double friction_compensation_effort(double velocity, std::size_t joint_index,
                                     double now_seconds, double period_seconds);
  void accept_haptic_command(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void accept_haptic_output_gate(const std_msgs::msg::Bool::SharedPtr message);
  void reset_haptic_command();
  void reset_haptic_output_gate();
  void reset_friction_pulse_state();

  std::vector<std::string> joint_names_;
  bool enable_output_{ false };
  std::string hardware_enable_interface_;
  double torque_scale_{ 0.0 };
  double damping_gain_{ 0.0 };
  double torque_limit_{ 0.0 };
  bool friction_compensation_enabled_{ false };
  double friction_ramp_duration_{ 0.2 };
  bool friction_pulse_enabled_{ false };
  double friction_intent_velocity_threshold_{ 0.0 };
  double friction_release_velocity_threshold_{ 0.0 };
  double friction_pulse_duration_{ 0.0 };
  double friction_decay_duration_{ 0.0 };
  double friction_retrigger_delay_{ 0.0 };
  bool haptic_enable_output_{ false };
  std::string haptic_effort_topic_{ "/leader/arm_force_feedback/effort" };
  double haptic_command_timeout_{ 0.1 };
  double startup_ramp_duration_{ 0.0 };
  std::string haptic_output_enable_topic_;
  double haptic_output_enable_timeout_{ 0.1 };
  double haptic_activation_ramp_duration_{ 2.0 };
  std::string urdf_path_;
  std::string urdf_package_{ "a5_description" };
  std::string urdf_relative_path_{ "urdf/A5.urdf" };

  std::vector<double> damping_gains_;
  std::vector<double> torque_limits_;
  std::vector<double> torque_scales_;
  std::vector<double> effort_signs_;
  std::vector<double> friction_compensations_;
  std::vector<double> friction_velocity_deadbands_;
  std::vector<double> friction_velocity_scales_;
  std::vector<SmoothFrictionCompensation> smooth_friction_states_;
  std::vector<double> haptic_effort_limits_;
  std::vector<int> friction_pulse_directions_;
  std::vector<double> friction_pulse_start_times_;
  std::vector<double> friction_last_trigger_times_;
  std::vector<char> friction_pulse_armed_;
  std::vector<int> pinocchio_q_indices_;
  std::vector<int> pinocchio_v_indices_;

  std::unique_ptr<pinocchio::Model> pinocchio_model_;
  std::unique_ptr<pinocchio::Data> pinocchio_data_;
  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;

  realtime_tools::RealtimeBuffer<HapticCommand> haptic_command_buffer_;
  realtime_tools::RealtimeBuffer<TimedOutputGateCommand> haptic_output_gate_buffer_;
  OutputActivationRamp base_activation_ramp_;
  OutputActivationRamp haptic_activation_ramp_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr haptic_effort_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr haptic_output_gate_sub_;
};
}  // namespace a6_y_controllers

#endif  // A6_Y_CONTROLLERS__A6_Y_GRAVITY_COMP_CONTROLLER_HPP_

#ifndef A6_Y_ROS2_CONTROL__A6_Y_DAMIAO_SYSTEM_HPP_
#define A6_Y_ROS2_CONTROL__A6_Y_DAMIAO_SYSTEM_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <a6_y_ros2_control/damiao_can.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace a6_y_ros2_control
{
class A6YDamiaoSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ~A6YDamiaoSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  struct JointData
  {
    std::string name;
    std::string motor_type;
    std::uint32_t slave_id{ 0 };
    std::uint32_t master_id{ 0 };
    double sign{ 1.0 };
    double offset{ 0.0 };
    double position_scale{ 1.0 };
    double min_position{ -std::numeric_limits<double>::infinity() };
    double max_position{ std::numeric_limits<double>::infinity() };
    double max_velocity{ 0.5 };
    double mit_kp{ 0.0 };
    double mit_kd{ 0.5 };
    MotorLimits limits;

    double state_position{ 0.0 };
    double state_velocity{ 0.0 };
    double state_effort{ 0.0 };
    double state_motor_error{ 0.0 };
    double state_fault{ 0.0 };
    double state_has_feedback{ 0.0 };
    double state_feedback_age{ -1.0 };
    double state_enabled{ 0.0 };
    double command_position{ 0.0 };
    double command_velocity{ 0.0 };
    double command_effort{ 0.0 };
    double last_position{ 0.0 };
    double last_feedback_time{ 0.0 };
    bool has_feedback{ false };
    bool enabled{ false };
    bool missing_feedback_warned{ false };
    bool feedback_timeout_warned{ false };
    bool faulted{ false };
    bool read_only{ false };
    int motor_error{ 0 };
    int control_mode_feedback{ -1 };
  };

  bool configure_joints(const hardware_interface::HardwareInfo& info);
  bool configure_real_hardware();
  bool activate_real_hardware();
  void deactivate_real_hardware();
  void disable_enabled_motors();
  bool switch_control_modes();
  bool verify_enable_feedback(double now_seconds, const std::string& context) const;
  bool send_motor_command(const JointData& joint);
  bool request_status_updates(double now_seconds);
  void process_rx_frame(const CanFrame& frame, double now_seconds);
  void drain_can(double timeout_seconds, std::size_t max_frames, double now_seconds);
  void update_joint_diagnostics(JointData& joint, double now_seconds);
  std::uint32_t control_mode_code() const;
  bool is_fault_code(int code) const;
  double to_motor_position(const JointData& joint, double joint_position) const;
  double to_motor_velocity(const JointData& joint, double joint_velocity) const;
  double to_motor_velocity_limit(const JointData& joint) const;
  double to_motor_torque(const JointData& joint, double joint_torque) const;
  bool has_interface(const std::vector<hardware_interface::InterfaceInfo>& interfaces, const std::string& name) const;
  std::string hardware_parameter(const std::string& name, const std::string& default_value) const;
  std::string joint_parameter(const hardware_interface::ComponentInfo& joint, const std::string& name,
                              const std::string& default_value) const;
  bool parse_bool(const std::string& text, bool default_value) const;
  double parse_double(const std::string& text, double default_value) const;
  std::uint32_t parse_uint(const std::string& text, std::uint32_t default_value) const;

  std::vector<JointData> joints_;
  std::unordered_map<std::string, std::size_t> joint_index_by_name_;
  std::unordered_map<std::uint32_t, std::size_t> joint_index_by_can_id_;
  std::unique_ptr<SlcanBus> bus_;

  bool dry_run_{ true };
  bool active_{ false };
  bool auto_enable_{ false };
  bool set_control_mode_on_configure_{ false };
  bool require_feedback_on_configure_{ true };
  bool allow_partial_feedback_{ false };
  bool disable_on_deactivate_{ true };
  bool warned_motion_disabled_{ false };
  bool first_write_after_activate_{ true };
  std::string can_channel_{ "/dev/ttyACM0" };
  std::uint32_t can_bitrate_{ 1000000 };
  std::uint32_t tty_baudrate_{ 115200 };
  std::string control_mode_{ "pos_vel" };
  double command_timeout_{ 0.2 };
  double max_start_delta_{ 0.25 };
  double receive_timeout_{ 0.001 };
  double feedback_timeout_{ 0.5 };
  double status_request_period_{ 0.05 };
  double last_status_request_time_{ 0.0 };
  std::uint32_t initial_feedback_attempts_{ 20 };
  double mit_kp_{ 0.0 };
  double mit_kd_{ 0.5 };
};
}  // namespace a6_y_ros2_control

#endif  // A6_Y_ROS2_CONTROL__A6_Y_DAMIAO_SYSTEM_HPP_

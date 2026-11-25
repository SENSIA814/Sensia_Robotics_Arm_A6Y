#pragma once

#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <a6_y_controllers/joint_reference_gate.hpp>
#include <a6_y_teleop_msgs/msg/vr_state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace a6_y_controllers
{
// Fixed six-joint configuration captured on the first active update.
// Commands the existing seven-motor MIT effort interface; the gripper stays zero-force.
class A6YJointImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

protected:
  // Fixed by the plugin type, never switched while a controller is running.
  virtual bool position_output() const { return false; }

private:
  double prepare_duration() const;
  Eigen::VectorXd position_speed_;
  double position_prepare_duration_{0.5};
  bool interfaces_match() const;
  void write_disabled();
  controller_interface::return_type fail(const char* reason);
  Eigen::VectorXd vector_parameter(const std::string& name, int size, bool strictly_positive);
  void configure_vr();
  void publish_vr(uint64_t now, bool feedback_valid);
  static uint64_t steady_ns();

  std::vector<std::string> joints_;
  std::vector<int> q_indices_, v_indices_;
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::Matrix<double, 6, 1> target_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::VectorXd q_, dq_, gravity_scales_, gravity_signs_, base_damping_, torque_limits_;
  Eigen::VectorXd stiffness_, damping_, spring_error_limits_, impedance_limits_, integral_gains_, integral_limits_;
  Eigen::Matrix<double,6,1> integral_{Eigen::Matrix<double,6,1>::Zero()};
  double integral_deadband_{0.005}, integral_window_{0.35}, integral_velocity_{0.25};
  bool enable_output_{false}, active_{false}, captured_{false}, faulted_{false};
  double elapsed_{0.0}, ramp_duration_{2.0}, max_period_{0.1}, max_feedback_age_{0.1};
  bool external_vr_{false}, vr_allow_prepare_{false};
  bool vr_ramp_gravity_on_prepare_{false};
  double vr_gravity_ramp_duration_{2.0};
  double vr_gripper_lower_{-.03}, vr_gripper_upper_{0.}, vr_gripper_zero_tolerance_{0.};
  uint64_t clock_id_{0};
  std::unique_ptr<JointReferenceGate> reference_gate_;
  realtime_tools::RealtimeBuffer<JointReferenceGate::Reference> vr_reference_;
  realtime_tools::RealtimeBuffer<JointReferenceGate::Control> vr_control_;
  std::atomic<uint8_t> urgent_vr_action_{0};
  std::atomic<uint64_t> published_epoch_{0};
  rclcpp::Subscription<JointReferenceGate::Reference>::SharedPtr vr_reference_sub_;
  rclcpp::Subscription<JointReferenceGate::Control>::SharedPtr vr_control_sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<a6_y_teleop_msgs::msg::VrState>> vr_state_pub_;
  Eigen::Matrix<double,6,1> vr_effort_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> vr_slew_{Eigen::Matrix<double,6,1>::Constant(1.0)};
  std::array<bool,6> vr_saturated_{};
  double saturation_elapsed_{0}, saturation_timeout_{0.5};
};

// Reuses feedback validation and the VR permission gate. Its output branch
// sends only joint positions and speed limits; no dynamics/effort calculation.
class A6YVRPositionController final : public A6YJointImpedanceController
{
protected:
  bool position_output() const override { return true; }
};
}  // namespace a6_y_controllers

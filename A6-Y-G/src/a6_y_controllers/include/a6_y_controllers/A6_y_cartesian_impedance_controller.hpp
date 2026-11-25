#pragma once

#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace a6_y_controllers
{
// Fixed TCP pose captured from feedback on the first active update.
// Commands the existing seven-motor MIT effort interface; the gripper stays zero-force.
class A6YCartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  bool interfaces_match() const;
  void write_disabled();
  controller_interface::return_type fail(const char* reason);
  Eigen::VectorXd vector_parameter(const std::string& name, int size, bool strictly_positive);

  std::vector<std::string> joints_;
  std::vector<int> q_indices_, v_indices_;
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex tcp_id_{0};
  pinocchio::SE3 target_{pinocchio::SE3::Identity()};
  Eigen::VectorXd q_, dq_, gravity_scales_, gravity_signs_, joint_damping_, torque_limits_;
  Eigen::VectorXd stiffness_, damping_, wrench_limits_, impedance_limits_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_;
  bool enable_output_{false}, active_{false}, captured_{false}, faulted_{false};
  double elapsed_{0.0}, ramp_duration_{2.0}, max_period_{0.1}, max_feedback_age_{0.1};
};
}  // namespace a6_y_controllers

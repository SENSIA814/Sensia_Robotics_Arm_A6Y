#include <a6_y_controllers/A6_y_gravity_comp_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <array>
#include <algorithm>
#include <limits>
#include <chrono>
#include <thread>

namespace
{
using Controller = a6_y_controllers::A6YGravityCompController;
using CallbackReturn = controller_interface::CallbackReturn;
using ReturnType = controller_interface::return_type;
const std::vector<std::string> kJoints{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_finger_joint1"};

class A5GravityControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void configure(Controller& controller, const std::vector<std::string>& joints)
  {
    ASSERT_EQ(controller.init("a5_controller_test"), ReturnType::OK);
    const auto results = controller.get_node()->set_parameters({
      rclcpp::Parameter("urdf_path", std::string(A5_TEST_URDF_PATH)),
      rclcpp::Parameter("joints", joints),
      rclcpp::Parameter("enable_output", true),
      rclcpp::Parameter("hardware_enable_interface", "motor_enable"),
      rclcpp::Parameter("feedback_velocity_limits", std::vector<double>{10,10,10,50,50,50}),
      // The vector must work even when the scalar fallback is zero.
      rclcpp::Parameter("torque_scale", 0.0),
      rclcpp::Parameter("torque_scales", std::vector<double>{1, 1, 1, 1, 1, 1, 0}),
      rclcpp::Parameter("damping_gains", std::vector<double>{0.05, 0.05, 0.05, 0.05, 0.05, 0.03, 0}),
      rclcpp::Parameter("torque_limits", std::vector<double>{10, 10, 10, 3, 4, 4, 30})});
    for (const auto& result : results) { ASSERT_TRUE(result.successful) << result.reason; }
  }
};

TEST_F(A5GravityControllerTest, UsesCompletePayloadAndMeasuredOpeningAndDisablesOnInvalidState)
{
  // Handles must outlive the loaned interfaces owned by the controller.
  std::array<double, 21> states{};
  std::array<double, 28> commands{};
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;
  state_handles.reserve(21);
  command_handles.reserve(28);
  for (std::size_t i = 0; i < kJoints.size(); ++i)
  {
    const std::array<std::string, 4> names{"position", "velocity", "effort", "motor_enable"};
    for (std::size_t j = 0; j < 3; ++j)
      state_handles.emplace_back(kJoints[i], names[j], &states[3 * i + j]);
    for (std::size_t j = 0; j < 4; ++j)
      command_handles.emplace_back(kJoints[i], names[j], &commands[4 * i + j]);
  }
  Controller controller;
  configure(controller, kJoints);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.command_interface_configuration().names.size(), 28U);
  std::vector<hardware_interface::LoanedStateInterface> state_loans;
  std::vector<hardware_interface::LoanedCommandInterface> command_loans;
  for (auto& handle : state_handles) state_loans.emplace_back(handle);
  for (auto& handle : command_handles) command_loans.emplace_back(handle);
  controller.assign_interfaces(std::move(command_loans), std::move(state_loans));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  for (std::size_t i = 0; i < 7; ++i) EXPECT_DOUBLE_EQ(commands[4 * i + 3], 0);

  // Independent 8-DoF model + constrained potential-energy gradient, rather
  // than calling the same mimic gravity routine as the controller under test.
  pinocchio::Model full;
  pinocchio::urdf::buildModel(A5_TEST_URDF_PATH, full, false, false);
  pinocchio::Data data(full);
  const std::array<double, 7> damping{0.05, 0.05, 0.05, 0.05, 0.05, 0.03, 0};
  const std::array<double, 7> limits{10, 10, 10, 3, 4, 4, 30};
  for (const double opening : {0.0, -0.015, -0.03})
  {
    Eigen::VectorXd q(8);
    q << 0.2, 0.8, 1.1, -0.7, 0.4, 1.0, opening, -opening;
    for (std::size_t i = 0; i < 7; ++i)
    {
      states[3 * i] = q[static_cast<int>(i)];
      states[3 * i + 1] = i == 6 ? -0.01 : 0.1;
    }
    ASSERT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), ReturnType::OK);
    for (int i = 0; i < 7; ++i)
    {
      auto plus = q; auto minus = q;
      plus[i] += 1e-6; minus[i] -= 1e-6;
      if (i == 6) { plus[7] -= 1e-6; minus[7] += 1e-6; }
      const double gradient = (pinocchio::computePotentialEnergy(full, data, plus) -
                              pinocchio::computePotentialEnergy(full, data, minus)) / 2e-6;
      const double requested = i == 6 ? 0 : gradient - damping[i] * states[3 * i + 1];
      EXPECT_NEAR(commands[4 * i + 2], std::clamp(requested, -limits[i], limits[i]), 1e-7);
      EXPECT_DOUBLE_EQ(commands[4 * i], states[3 * i]);
      EXPECT_DOUBLE_EQ(commands[4 * i + 1], 0);
      EXPECT_DOUBLE_EQ(commands[4 * i + 3], 1);
    }
  }
  states[18] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), ReturnType::ERROR);
  for (std::size_t i = 0; i < 7; ++i)
  {
    EXPECT_DOUBLE_EQ(commands[4 * i + 2], 0);
    EXPECT_DOUBLE_EQ(commands[4 * i + 3], 0);
  }
}

TEST_F(A5GravityControllerTest, RejectsMissingGripperState)
{
  Controller controller;
  configure(controller, kJoints);
  controller.get_node()->set_parameter(rclcpp::Parameter("joints",
    std::vector<std::string>(kJoints.begin(), kJoints.end() - 1)));
  for (const auto* name : {"torque_scales", "torque_limits", "damping_gains"})
    controller.get_node()->set_parameter(rclcpp::Parameter(name, std::vector<double>(6, 1)));
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

struct TestHardware
{
  std::array<double, 21> states{};
  std::array<double, 28> commands{};
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;

  void bind(Controller& controller)
  {
    state_handles.reserve(21);
    command_handles.reserve(28);
    const std::array<std::string, 4> names{"position", "velocity", "effort", "motor_enable"};
    for (std::size_t i = 0; i < kJoints.size(); ++i)
    {
      for (std::size_t j = 0; j < 3; ++j)
        state_handles.emplace_back(kJoints[i], names[j], &states[3 * i + j]);
      for (std::size_t j = 0; j < 4; ++j)
        command_handles.emplace_back(kJoints[i], names[j], &commands[4 * i + j]);
    }
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    for (auto& handle : state_handles) state_loans.emplace_back(handle);
    for (auto& handle : command_handles) command_loans.emplace_back(handle);
    controller.assign_interfaces(std::move(command_loans), std::move(state_loans));
  }
};

TEST_F(A5GravityControllerTest, ContinuousFrictionRespectsCompositionAndLifecycle)
{
  TestHardware hardware;
  Controller controller;
  configure(controller, kJoints);
  controller.get_node()->set_parameters({
    rclcpp::Parameter("friction_compensation_enabled", true),
    rclcpp::Parameter("friction_compensations", std::vector<double>{0.15, 0, 0.15, 0, 0, 0, 0}),
    rclcpp::Parameter("damping_gains", std::vector<double>(7, 0)),
    rclcpp::Parameter("torque_limits", std::vector<double>{10, 10, 1, 3, 4, 4, 30})});
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  hardware.bind(controller);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const auto tick = [&controller](double dt = 0.02) {
    return controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
  };
  ASSERT_EQ(tick(), ReturnType::OK);
  const auto resting = hardware.commands;
  hardware.states[1] = hardware.states[4] = hardware.states[7] = 0.1;
  for (int i = 0; i < 50; ++i) ASSERT_EQ(tick(), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2] - resting[2], 0.15, 1e-12);
  // J2 amplitude is zero; J3 gravity + friction still obeys the total limit.
  EXPECT_DOUBLE_EQ(hardware.commands[6], resting[6]);
  EXPECT_DOUBLE_EQ(hardware.commands[10], 1.0);
  for (std::size_t i = 3; i < 7; ++i)
    EXPECT_DOUBLE_EQ(hardware.commands[4 * i + 2], resting[4 * i + 2]);

  hardware.states[1] = -0.1;
  ASSERT_EQ(tick(), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2], resting[2], 1e-12);
  ASSERT_EQ(tick(), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2] - resting[2], -0.015, 1e-12);
  ASSERT_EQ(tick(0.2), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2], resting[2], 1e-12);
  ASSERT_EQ(tick(), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2] - resting[2], -0.015, 1e-12);

  hardware.states[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tick(), ReturnType::ERROR);
  for (std::size_t i = 0; i < 7; ++i)
  {
    EXPECT_DOUBLE_EQ(hardware.commands[4 * i + 2], 0.0);
    EXPECT_DOUBLE_EQ(hardware.commands[4 * i + 3], 0.0);
  }
  hardware.states[1] = 0.1;
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(tick(), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2] - resting[2], 0.015, 1e-12);
  hardware.states[1] = 0.007326;
  ASSERT_EQ(tick(), ReturnType::OK);
  EXPECT_NEAR(hardware.commands[2], resting[2], 1e-12);
}

TEST_F(A5GravityControllerTest, DisabledOutputCannotProduceFrictionOrEnableMotor)
{
  TestHardware hardware;
  Controller controller;
  configure(controller, kJoints);
  controller.get_node()->set_parameters({
    rclcpp::Parameter("enable_output", false),
    rclcpp::Parameter("friction_compensation_enabled", true),
    rclcpp::Parameter("friction_compensations", std::vector<double>{0.15, 0, 0.15, 0, 0, 0, 0})});
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  hardware.bind(controller);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  hardware.states[1] = 0.1;
  for (int i = 0; i < 20; ++i)
    ASSERT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), ReturnType::OK);
  for (std::size_t i = 0; i < 7; ++i)
  {
    EXPECT_DOUBLE_EQ(hardware.commands[4 * i + 2], 0.0);
    EXPECT_DOUBLE_EQ(hardware.commands[4 * i + 3], 0.0);
  }
}

TEST_F(A5GravityControllerTest, RejectsInvalidContinuousFrictionConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("friction_pulse_enabled", true),
    rclcpp::Parameter("friction_ramp_duration", 0.0),
    rclcpp::Parameter("friction_ramp_duration", nan),
    rclcpp::Parameter("friction_compensations", std::vector<double>{-0.1, 0, 0, 0, 0, 0, 0}),
    rclcpp::Parameter("friction_compensations", std::vector<double>{11, 0, 0, 0, 0, 0, 0}),
    rclcpp::Parameter("friction_compensations", std::vector<double>{nan, 0, 0, 0, 0, 0, 0}),
    rclcpp::Parameter("friction_compensations", std::vector<double>(6, 0.1)),
    rclcpp::Parameter("friction_velocity_deadbands", std::vector<double>(7, 0)),
    rclcpp::Parameter("friction_velocity_scales", std::vector<double>(7, 0)),
    rclcpp::Parameter("friction_velocity_scales", std::vector<double>(7, nan)),
    rclcpp::Parameter("torque_limits", std::vector<double>(7, 0))};
  for (const auto& parameter : invalid)
  {
    SCOPED_TRACE(parameter.get_name());
    Controller controller;
    configure(controller, kJoints);
    controller.get_node()->set_parameters({
      rclcpp::Parameter("friction_compensation_enabled", true),
      rclcpp::Parameter("friction_compensations", std::vector<double>{0.15, 0, 0, 0, 0, 0, 0}),
      parameter});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  }
}

TEST_F(A5GravityControllerTest, LiveTuningRampsAtomicallyRejectsInvalidAndPreloadsOnReactivation)
{
  using Service = a6_y_teleop_msgs::srv::GravityTuning;
  TestHardware hardware;
  Controller controller;
  configure(controller, kJoints);
  controller.get_node()->set_parameter(rclcpp::Parameter("live_tuning_enabled", true));
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  hardware.bind(controller);
  hardware.states[3] = 0.8;
  hardware.states[6] = 1.1;
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const auto tick = [&](double dt = 0.02) {
    EXPECT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt)), ReturnType::OK);
  };
  tick();
  const double initial_j3 = hardware.commands[10];
  ASSERT_GT(initial_j3, 1.0);
  auto client_node = std::make_shared<rclcpp::Node>("tuning_test_client");
  auto client = client_node->create_client<Service>("/a5_controller_test/tune");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(controller.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  const auto call = [&](const Service::Request& request) {
    auto future = client->async_send_request(std::make_shared<Service::Request>(request));
    EXPECT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(2)),
              rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  Service::Request query;
  auto baseline = call(query);
  Service::Request request;
  request.apply = true;
  request.torque_scales = baseline->torque_scales;
  request.damping_gains = baseline->damping_gains;
  request.friction_compensations = baseline->friction_compensations;
  request.torque_scales[2] = 1.2;
  ASSERT_TRUE(call(request)->success);
  tick();
  EXPECT_NEAR(hardware.commands[10], initial_j3 * (1.0 + 0.2 / 15.0), 1e-10);
  EXPECT_EQ(call(query)->applied_revision, 0U);
  tick(0.2);  // A late cycle must not skip the transition.
  EXPECT_NEAR(hardware.commands[10], initial_j3 * (1.0 + 0.2 / 15.0), 1e-10);
  for (int i = 0; i < 15; ++i) tick();
  EXPECT_NEAR(hardware.commands[10], initial_j3 * 1.2, 1e-10);
  EXPECT_EQ(call(query)->applied_revision, 1U);

  auto invalid = request;
  invalid.torque_scales[1] = 1.8;  // Must not partially commit this valid entry.
  invalid.damping_gains[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(call(invalid)->success);
  EXPECT_EQ(call(query)->revision, 1U);
  EXPECT_EQ(call(query)->torque_scales[1], 1.0);
  invalid = request; invalid.friction_compensations[2] = 11.0;
  EXPECT_FALSE(call(invalid)->success);
  invalid = request; invalid.torque_scales[6] = 0.1;
  EXPECT_FALSE(call(invalid)->success);
  invalid = request; invalid.damping_gains.pop_back();
  EXPECT_FALSE(call(invalid)->success);

  // Continuous assistance fades in/out and cannot survive a disable request.
  request.friction_compensation_enabled = true;
  request.friction_compensations[0] = 0.3;
  hardware.states[1] = 0.1;
  ASSERT_TRUE(call(request)->success);
  for (int i = 0; i < 30; ++i) tick();
  EXPECT_NEAR(hardware.commands[2], 0.3 - 0.05 * 0.1, 1e-10);
  request.friction_compensation_enabled = false;
  ASSERT_TRUE(call(request)->success);
  tick();
  EXPECT_GT(hardware.commands[2], -0.005);
  EXPECT_LT(hardware.commands[2], 0.295);
  for (int i = 0; i < 15; ++i) tick();
  EXPECT_NEAR(hardware.commands[2], -0.005, 1e-10);

  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  request.torque_scales[2] = 1.1;
  const auto pending = call(request);
  ASSERT_TRUE(pending->success);
  EXPECT_LT(pending->applied_revision, pending->revision);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  tick();
  EXPECT_NEAR(hardware.commands[10], initial_j3 * 1.1, 1e-10);
  EXPECT_EQ(call(query)->applied_revision, pending->revision);
  EXPECT_DOUBLE_EQ(hardware.commands[26], 0.0);
}

TEST_F(A5GravityControllerTest, PlannedHomeRequiresActivationAndChecksPathTrackingAndCancellation)
{
  using Service=a6_y_teleop_msgs::srv::GravityHome;
  TestHardware hardware;Controller controller;configure(controller,kJoints);
  controller.get_node()->set_parameter(rclcpp::Parameter("planned_home_enabled",true));
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  hardware.bind(controller);hardware.states[0]=.1;
  auto node=std::make_shared<rclcpp::Node>("home_test_client");
  auto client=node->create_client<Service>("/a5_controller_test/home");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  executor.add_node(controller.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  const auto call=[&](const Service::Request& r) {
    auto f=client->async_send_request(std::make_shared<Service::Request>(r));
    EXPECT_EQ(executor.spin_until_future_complete(f,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    return f.get();
  };
  const auto tick=[&]() {EXPECT_EQ(controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.008)),ReturnType::OK);};
  Service::Request r;r.command=1;r.times={0.,2.,5.};r.positions=std::vector<double>(18,0.);
  r.positions[0]=r.positions[6]=.1;r.velocities=std::vector<double>(18,0.);
  EXPECT_FALSE(call(r)->success);  // Neither the service nor a queued path may enable motors.
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);tick();
  auto invalid=r;invalid.times={0.,.01,.02};EXPECT_FALSE(call(invalid)->success);
  invalid=r;invalid.positions.back()=.2;EXPECT_FALSE(call(invalid)->success);
  invalid=r;invalid.positions[0]=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(call(invalid)->success);
  auto accepted=call(r);ASSERT_TRUE(accepted->success);
  EXPECT_FALSE(call(r)->success);  // Pending request cannot be replaced before the RT tick.
  tick();
  Service::Request status;status.revision=accepted->revision;
  ASSERT_EQ(call(status)->phase,1);
  // Independent ideal encoder path: exercise all RT interpolation and limits,
  // never use a mode that changes real MIT position/stiffness commands.
  a6_y_controllers::GravityHomePath path;path.times=r.times;path.positions=r.positions;path.velocities=r.velocities;
  for(int i=1;i<690;++i) {
    std::array<double,6> q{},v{};path.sample(i*.008,q,v);
    for(int j=0;j<6;++j) {hardware.states[3*j]=q[j];hardware.states[3*j+1]=v[j];}
    tick();
    if(i%20==0) {ASSERT_NE(call(status)->phase,3);}
    for(int j=0;j<6;++j) {
      EXPECT_TRUE(std::isfinite(hardware.commands[4*j+2]));
      EXPECT_LE(std::abs(hardware.commands[4*j+2]),j<3?10.:j==3?3.:4.);
      EXPECT_DOUBLE_EQ(hardware.commands[4*j],hardware.states[3*j]);
      EXPECT_DOUBLE_EQ(hardware.commands[4*j+3],1.);
    }
    EXPECT_DOUBLE_EQ(hardware.commands[26],0.);
  }
  EXPECT_EQ(call(status)->phase,2);
  Service::Request cancel;cancel.command=2;cancel.revision=status.revision;ASSERT_TRUE(call(cancel)->success);tick();
  EXPECT_EQ(call(Service::Request())->phase,0);
  // Start mismatch is rejected by the RT loop before any return torque.
  accepted=call(r);ASSERT_TRUE(accepted->success);tick();
  status.revision=accepted->revision;EXPECT_EQ(call(status)->phase,3);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  for(int i=0;i<7;++i) EXPECT_DOUBLE_EQ(hardware.commands[4*i+3],0.);
  EXPECT_FALSE(call(r)->success);
}

TEST(GravityHomePathTest, RejectsCubicOvershootBetweenValidKnots)
{
  a6_y_controllers::GravityHomePath p;
  p.times={0.,1.,3.};p.positions=std::vector<double>(18,0.);p.velocities=std::vector<double>(18,0.);
  std::array<double,6> lo{},hi{};hi.fill(1.);
  ASSERT_TRUE(p.valid(lo,hi));
  p.velocities[6]=.1; // negative excursion in the segment after this knot
  EXPECT_FALSE(p.valid(lo,hi));
  p.velocities[6]=0;p.times[1]=0;EXPECT_FALSE(p.valid(lo,hi));
}

TEST_F(A5GravityControllerTest, HomeRejectsMissingOrInvalidMotorVelocityRanges)
{
  for(const auto& limits:std::vector<std::vector<double>>{
      {},{10,10,10,50,50},{10,10,10,0,50,50},{10,10,10,-1,50,50},
      {10,10,10,std::numeric_limits<double>::quiet_NaN(),50,50}}) {
    Controller controller;configure(controller,kJoints);
    controller.get_node()->set_parameters({rclcpp::Parameter("planned_home_enabled",true),
      rclcpp::Parameter("feedback_velocity_limits",limits)});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),CallbackReturn::ERROR);
  }
}

TEST_F(A5GravityControllerTest, PlannedHomeReportsSpecificFirstFaultAndClearsItOnCancellation)
{
  using Service=a6_y_teleop_msgs::srv::GravityHome;
  TestHardware hardware;Controller controller;configure(controller,kJoints);
  controller.get_node()->set_parameter(rclcpp::Parameter("planned_home_enabled",true));
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  hardware.bind(controller);
  auto node=std::make_shared<rclcpp::Node>("home_fault_test_client");
  auto client=node->create_client<Service>("/a5_controller_test/home");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  executor.add_node(controller.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  const auto call=[&](const Service::Request& request) {
    auto future=client->async_send_request(std::make_shared<Service::Request>(request));
    EXPECT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  const auto tick=[&](double dt=.008) {
    EXPECT_EQ(controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(dt)),ReturnType::OK);
  };
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);tick();
  Service::Request path;path.command=1;path.times={0.,2.,3.};
  path.positions=std::vector<double>(18,0.);path.velocities=std::vector<double>(18,0.);
  Service::Request status;
  const auto start=[&]() {
    const auto accepted=call(path);EXPECT_TRUE(accepted->success)<<accepted->message;
    status.revision=accepted->revision;
  };
  const auto fault=[&](const std::string& reason,const std::string& joint,
                       const std::string& value,const std::string& limit) {
    const auto result=call(status);EXPECT_EQ(result->phase,3);
    EXPECT_NE(result->message.find("Return aborted: "+reason),std::string::npos)<<result->message;
    if(!joint.empty()) {EXPECT_NE(result->message.find("joint="+joint),std::string::npos)<<result->message;}
    if(!value.empty()) {EXPECT_NE(result->message.find("value="+value),std::string::npos)<<result->message;}
    EXPECT_NE(result->message.find("limit="+limit),std::string::npos)<<result->message;
    EXPECT_NE(result->message.find("elapsed="),std::string::npos);
    // Failure is latched, even after feedback changes, and does not disable
    // the otherwise healthy gravity controller or command the gripper.
    hardware.states.fill(0.);tick();
    EXPECT_EQ(call(status)->message,result->message);
    for(int j=0;j<6;++j) EXPECT_DOUBLE_EQ(hardware.commands[4*j+3],1.);
    EXPECT_DOUBLE_EQ(hardware.commands[26],0.);
    Service::Request cancel;cancel.command=2;cancel.revision=status.revision;
    const auto cancelled=call(cancel);ASSERT_TRUE(cancelled->success);tick();
    status.revision=cancelled->revision;
    const auto idle=call(status);EXPECT_EQ(idle->phase,0);EXPECT_EQ(idle->message,"OK");
  };

  hardware.states[9]=.04;start();tick();
  fault("start_position_mismatch","joint4","0.040000","0.030000");
  hardware.states[10]=-.081;start();tick();
  fault("start_velocity","joint4","0.081000","0.080000");
  start();tick();tick(.101);
  fault("invalid_control_period","","0.101000","0.100000");
  start();tick();std::this_thread::sleep_for(std::chrono::milliseconds(650));tick();
  fault("heartbeat_timeout","","","0.600000");
  start();tick();hardware.states[9]=-.151;tick();
  fault("tracking_error","joint4","0.151000","0.150000");
  start();tick();
  // Regress both actual startup and protected-exit failures, on both arms.
  for(double speed:{.525031,-.549451,50.,-50.}) {
    hardware.states[10]=speed;tick();EXPECT_EQ(call(status)->phase,1);
  }
  hardware.states[10]=-50.001;tick();
  fault("joint_speed","joint4","50.001000","50.000000");
  start();tick();hardware.states[1]=10.001;tick();
  fault("joint_speed","joint1","10.001000","10.000000");
  start();tick();hardware.states[3]=-.011;tick();
  fault("joint_lower_limit","joint2","-0.011000","-0.010000");
  path.times.back()=25.;path.positions[3]=path.positions[9]=1.55;
  hardware.states[9]=1.55;start();tick();hardware.states[9]=1.561;tick();
  fault("joint_upper_limit","joint4","1.561000","1.560000");

  // Cancel/retry must log an already latched fault even when no status query
  // observed it. Changing revision must not make that first failure disappear.
  path.times.back()=3.;path.positions[3]=path.positions[9]=0.;
  hardware.states[9]=.04;start();tick();
  Service::Request cancel;cancel.command=2;cancel.revision=status.revision;
  testing::internal::CaptureStderr();const auto cancelled=call(cancel);
  const auto cancel_log=testing::internal::GetCapturedStderr();
  EXPECT_TRUE(cancelled->success);
  EXPECT_NE(cancel_log.find("start_position_mismatch"),std::string::npos)<<cancel_log;
  EXPECT_EQ(cancel_log.find("start_position_mismatch"),cancel_log.rfind("start_position_mismatch"));
  tick();EXPECT_EQ(call(Service::Request())->message,"OK");
  hardware.states[9]=.04;start();tick();
  hardware.states[9]=0.;testing::internal::CaptureStderr();start();
  const auto retry_log=testing::internal::GetCapturedStderr();
  EXPECT_NE(retry_log.find("start_position_mismatch"),std::string::npos)<<retry_log;
  EXPECT_EQ(retry_log.find("start_position_mismatch"),retry_log.rfind("start_position_mismatch"));
  tick();EXPECT_EQ(call(status)->phase,1);
  EXPECT_EQ(call(status)->message,"OK");
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  EXPECT_EQ(call(status)->message,"OK");
}

TEST_F(A5GravityControllerTest, PlannedHomeDistinguishesZeroSettlingTimeoutFromTrackingFailure)
{
  using Service=a6_y_teleop_msgs::srv::GravityHome;
  TestHardware hardware;Controller controller;configure(controller,kJoints);
  controller.get_node()->set_parameter(rclcpp::Parameter("planned_home_enabled",true));
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  hardware.bind(controller);
  auto node=std::make_shared<rclcpp::Node>("home_settling_test_client");
  auto client=node->create_client<Service>("/a5_controller_test/home");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  executor.add_node(controller.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  const auto call=[&](const Service::Request& request) {
    auto future=client->async_send_request(std::make_shared<Service::Request>(request));
    EXPECT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  const auto tick=[&]() {
    EXPECT_EQ(controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.008)),ReturnType::OK);
  };
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);tick();
  Service::Request path;path.command=1;path.times={0.,2.,3.};
  path.positions=std::vector<double>(18,0.);path.velocities=std::vector<double>(18,0.);
  const auto accepted=call(path);ASSERT_TRUE(accepted->success);tick();
  Service::Request status;status.revision=accepted->revision;
  hardware.states[9]=.006; // Below tracking tolerance, outside zero tolerance.
  for(int i=0;i<1640;++i) {tick();if(i%20==0) call(status);}
  const auto result=call(status);EXPECT_EQ(result->phase,3);
  EXPECT_NE(result->message.find("zero_settling_timeout"),std::string::npos)<<result->message;
  EXPECT_NE(result->message.find("joint=joint4"),std::string::npos)<<result->message;
  EXPECT_NE(result->message.find("limit=10.000000"),std::string::npos)<<result->message;
  EXPECT_NE(result->message.find("position=0.006000"),std::string::npos)<<result->message;
  EXPECT_NE(result->message.find("velocity=0.000000"),std::string::npos)<<result->message;

  // A smaller position error on J1 must not conceal J4's velocity violation.
  Service::Request cancel;cancel.command=2;cancel.revision=status.revision;
  ASSERT_TRUE(call(cancel)->success);tick();hardware.states.fill(0.);
  const auto retry=call(path);ASSERT_TRUE(retry->success);tick();status.revision=retry->revision;
  hardware.states[0]=.004;hardware.states[10]=.1;
  for(int i=0;i<1640;++i) {tick();if(i%20==0) call(status);}
  const auto moving=call(status);EXPECT_EQ(moving->phase,3);
  EXPECT_NE(moving->message.find("zero_settling_timeout"),std::string::npos)<<moving->message;
  EXPECT_NE(moving->message.find("joint=joint4"),std::string::npos)<<moving->message;
  EXPECT_NE(moving->message.find("position=0.000000"),std::string::npos)<<moving->message;
  EXPECT_NE(moving->message.find("velocity=0.100000"),std::string::npos)<<moving->message;
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
}
class TeachingModeTest : public A5GravityControllerTest, public ::testing::WithParamInterface<bool>
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_P(TeachingModeTest, TeachingReferenceIsBoundedAndTimeoutLatchesUntilExplicitHold)
{
  using Service=a6_y_teleop_msgs::srv::TeachingControl;
  using Reference=a6_y_teleop_msgs::msg::TeachingReference;
  TestHardware hardware;Controller controller;configure(controller,kJoints);
  controller.get_node()->set_parameters({rclcpp::Parameter("planned_home_enabled",true),
    rclcpp::Parameter("teaching_follow_enabled",true),rclcpp::Parameter("home_mock_follow",!GetParam()),
    rclcpp::Parameter("teaching_position_output",GetParam())});
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  hardware.bind(controller);
  auto node=std::make_shared<rclcpp::Node>("teaching_test_client");
  auto client=node->create_client<Service>("/a5_controller_test/teaching");
  auto home=node->create_client<a6_y_teleop_msgs::srv::GravityHome>("/a5_controller_test/home");
  auto pub=node->create_publisher<Reference>("/a5_controller_test/teaching_reference",rclcpp::QoS(1).best_effort());
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  executor.add_node(controller.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  const auto call=[&](int command,uint64_t revision=0) {
    auto request=std::make_shared<Service::Request>();request->command=command;request->revision=revision;
    auto future=client->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  const auto tick=[&]() {
    EXPECT_EQ(controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.008)),ReturnType::OK);
    for(int i=0;i<6;++i) {
      EXPECT_LE(std::abs(hardware.commands[4*i+2]),i<3?10.:4.);
      if(GetParam()) EXPECT_DOUBLE_EQ(hardware.commands[4*i+2],0.);
    }
    EXPECT_DOUBLE_EQ(hardware.commands[26],0.);
  };
  EXPECT_FALSE(call(2)->success); // Cannot arm an inactive controller.
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);tick();
  EXPECT_FALSE(call(1)->success); // Must hold/anchor first.
  auto held=call(2);ASSERT_TRUE(held->success);tick();EXPECT_EQ(call(0)->phase,2);
  auto started=call(1,held->revision);ASSERT_TRUE(started->success);tick();
  uint64_t revision=started->revision;
  EXPECT_FALSE(call(3,revision)->success); // No free-gravity transition during streaming.
  auto hr=std::make_shared<a6_y_teleop_msgs::srv::GravityHome::Request>();hr->command=1;
  auto hf=home->async_send_request(hr);
  ASSERT_EQ(executor.spin_until_future_complete(hf,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(hf.get()->success);
  Reference reference;reference.revision=revision;reference.positions.fill(.05);
  double previous=0.,previous_v=0.;
  for(int k=0;k<100;++k) {
    reference.sequence=k+1;reference.stamp_ns=node->now().nanoseconds();pub->publish(reference);
    executor.spin_some(std::chrono::milliseconds(2));tick();
    double current=hardware.commands[0],velocity=(current-previous)/.008;
    EXPECT_LE(std::abs(velocity),.100001);EXPECT_LE(std::abs(velocity-previous_v)/.008,.200001);
    previous=current;previous_v=velocity;
    for(int i=0;i<6;++i)hardware.states[3*i]=hardware.commands[4*i];
  }
  EXPECT_GT(previous,.005);
  // Stale timestamp, duplicate sequence, stale revision, NaN and limits may
  // not refresh the watchdog. Status queries may not refresh it either.
  for(int k=0;k<6;++k) {
    reference.stamp_ns=node->now().nanoseconds()-1000000000LL;reference.sequence=200+k;
    pub->publish(reference);executor.spin_some();call(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));tick();
  }
  EXPECT_EQ(call(0)->phase,3);EXPECT_NE(call(0)->message.find("timeout"),std::string::npos);
  reference.sequence=300;reference.stamp_ns=node->now().nanoseconds();pub->publish(reference);executor.spin_some();tick();
  EXPECT_EQ(call(0)->phase,3); // Fresh packets cannot auto-resume.
  EXPECT_FALSE(call(1,revision)->success);
  held=call(2);ASSERT_TRUE(held->success);tick();
  started=call(1,held->revision);ASSERT_TRUE(started->success);tick();revision=started->revision;
  reference.revision=revision-1;reference.sequence=1000;reference.stamp_ns=node->now().nanoseconds();
  pub->publish(reference);executor.spin_some();tick();
  reference.revision=revision;reference.positions[0]=std::numeric_limits<double>::quiet_NaN();
  pub->publish(reference);executor.spin_some();tick();
  std::this_thread::sleep_for(std::chrono::milliseconds(280));tick();EXPECT_EQ(call(0)->phase,3);
  held=call(2);tick();started=call(1,held->revision);tick();
  for(double speed:{.525031,-.549451,50.,-50.}) {
    hardware.states[10]=speed;tick();EXPECT_EQ(call(0)->phase,1);
  }
  hardware.states[10]=50.001;tick();EXPECT_EQ(call(0)->phase,3);
  const auto speed_fault=call(0)->message;
  EXPECT_NE(speed_fault.find("speed"),std::string::npos);
  EXPECT_NE(speed_fault.find("joint=joint4"),std::string::npos);
  EXPECT_NE(speed_fault.find("limit=50.000000"),std::string::npos);
  hardware.states[10]=0.;tick();EXPECT_EQ(call(0)->message,speed_fault);
  held=call(2);tick();started=call(1,held->revision);tick();
  hardware.states[0]+=.151;tick();EXPECT_EQ(call(0)->phase,3);
  EXPECT_NE(call(0)->message.find("Tracking"),std::string::npos);
  held=call(2);tick();auto off=call(3,held->revision);ASSERT_TRUE(off->success);tick();EXPECT_EQ(call(0)->phase,0);
  if(GetParam()) {
    const double stopped=hardware.commands[0];hardware.states[0]+=.01;tick();
    EXPECT_DOUBLE_EQ(hardware.commands[0],stopped); // Idle is a latched position hold.
  }
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  for(int i=0;i<7;++i)EXPECT_DOUBLE_EQ(hardware.commands[4*i+3],0.);
}
INSTANTIATE_TEST_SUITE_P(MitAndPosVel, TeachingModeTest, ::testing::Bool());
}  // namespace

#include <a6_y_controllers/A6_y_cartesian_impedance_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <Eigen/Cholesky>
#include <random>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Geometry>
#include <array>
#include <limits>

namespace
{
using Controller = a6_y_controllers::A6YCartesianImpedanceController;
using Callback = controller_interface::CallbackReturn;
using Result = controller_interface::return_type;
const std::vector<std::string> kJoints{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_finger_joint1"};

struct Hardware
{
  std::array<double, 42> states{};
  std::array<double, 28> commands{};
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;
  Hardware()
  {
    const std::array<double, 7> start{0.2, 0.8, 1.1, -0.7, 0.4, 1.0, -0.015};
    for (int i = 0; i < 7; ++i) { states[6 * i] = start[i]; states[6 * i + 3] = 1.0; }
  }
  void bind(Controller& controller, bool reorder = false)
  {
    state_handles.reserve(42); command_handles.reserve(28);
    const std::array<std::string, 6> state_names{
      "position", "velocity", "effort", "position_valid", "position_age", "feedback_age"};
    const std::array<std::string, 4> command_names{"position", "velocity", "effort", "motor_enable"};
    for (int i = 0; i < 7; ++i)
    {
      for (int j = 0; j < 6; ++j) state_handles.emplace_back(kJoints[i], state_names[j], &states[6 * i + j]);
      for (int j = 0; j < 4; ++j) command_handles.emplace_back(kJoints[i], command_names[j], &commands[4 * i + j]);
    }
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    for (auto& h : state_handles) state_loans.emplace_back(h);
    for (int i = 0; i < 28; ++i)
    {
      const int index = reorder && i == 0 ? 2 : (reorder && i == 2 ? 0 : i);
      command_loans.emplace_back(command_handles[index]);
    }
    controller.assign_interfaces(std::move(command_loans), std::move(state_loans));
  }
  void expect_disabled() const
  {
    for (int i = 0; i < 7; ++i)
    {
      EXPECT_TRUE(std::isfinite(commands[4 * i]));
      EXPECT_DOUBLE_EQ(commands[4 * i + 1], 0.0);
      EXPECT_DOUBLE_EQ(commands[4 * i + 2], 0.0);
      EXPECT_DOUBLE_EQ(commands[4 * i + 3], 0.0);
    }
  }
  Eigen::VectorXd full_q() const
  {
    Eigen::VectorXd q(8);
    for (int i = 0; i < 7; ++i) q[i] = states[6 * i];
    q[7] = -q[6];
    return q;
  }
};

class ImpedanceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void init(Controller& controller, std::vector<rclcpp::Parameter> extra = {})
  {
    ASSERT_EQ(controller.init("a5_impedance_test"), Result::OK);
    auto params = std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("urdf_path", std::string(A5_TEST_URDF_PATH)),
      rclcpp::Parameter("enable_output", true),
      rclcpp::Parameter("torque_scales", std::vector<double>{0, 0, 0, 0, 0, 0, 0}),
      rclcpp::Parameter("damping_gains", std::vector<double>(7, 0.0)),
      rclcpp::Parameter("cartesian_damping", std::vector<double>(6, 0.0)),
      // Disable saturation for the independent energy/geometry checks.
      rclcpp::Parameter("torque_limits", std::vector<double>(7, 10000.0)),
      rclcpp::Parameter("impedance_torque_limits", std::vector<double>(6, 10000.0)),
      rclcpp::Parameter("wrench_limits", std::vector<double>(6, 10000.0)),
      rclcpp::Parameter("impedance_ramp_duration", 0.02)};
    params.insert(params.end(), extra.begin(), extra.end());
    for (const auto& p : params) ASSERT_TRUE(controller.get_node()->set_parameter(p).successful);
  }
  void start(Controller& controller, Hardware& hardware)
  {
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::SUCCESS);
    hardware.bind(controller);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
    hardware.expect_disabled();
    ASSERT_EQ(tick(controller), Result::OK);
  }
  Result tick(Controller& controller, double dt = 0.02)
  {
    return controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
  }
};

TEST_F(ImpedanceTest, ContinuousLimitMatchesPotentialGradientAndRecapturesTarget)
{
  Hardware hardware; Controller controller;
  init(controller);
  start(controller, hardware);
  for (int i = 0; i < 7; ++i) EXPECT_NEAR(hardware.commands[4 * i + 2], 0, 1e-12);
  pinocchio::Model full;
  pinocchio::urdf::buildModel(A5_TEST_URDF_PATH, full, false, false);
  pinocchio::Data data(full);
  const auto tcp = full.getFrameId("tool_tcp");
  pinocchio::framesForwardKinematics(full, data, hardware.full_q());
  const pinocchio::SE3 target = data.oMf[tcp];
  // Do not compute the reference with the controller's Jacobian or log3 code.
  const auto potential = [&](const Eigen::VectorXd& q) {
    pinocchio::framesForwardKinematics(full, data, q);
    const auto& current = data.oMf[tcp];
    const Eigen::AngleAxisd delta(target.rotation() * current.rotation().transpose());
    return 0.5 * 50 * (current.translation() - target.translation()).squaredNorm() +
           0.5 * 2 * delta.angle() * delta.angle();
  };
  for (int displaced = 0; displaced < 6; ++displaced)
  {
    hardware.states[6 * displaced] += 0.03;
    // Continuous-time limit of the discrete controller, checked against the
    // independent potential gradient rather than reproducing its matrix solve.
    ASSERT_EQ(tick(controller, 1e-7), Result::OK);
    const auto q = hardware.full_q();
    for (int i = 0; i < 6; ++i)
    {
      auto plus = q; auto minus = q;
      plus[i] += 1e-6; minus[i] -= 1e-6;
      const double torque = -(potential(plus) - potential(minus)) / 2e-6;
      EXPECT_NEAR(hardware.commands[4 * i + 2], torque, 1e-7) << "joint " << i;
    }
    EXPECT_DOUBLE_EQ(hardware.commands[26], 0.0);
  }
  const auto old_commands = hardware.commands;
  for (int i = 0; i < 10; ++i) ASSERT_EQ(tick(controller, 1e-7), Result::OK);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(hardware.commands[4 * i + 2], old_commands[4 * i + 2], 1e-12);
  // Reactivation captures the new pose, instead of pulling towards a previous session's target.
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  hardware.expect_disabled();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(tick(controller), Result::OK);
  ASSERT_EQ(tick(controller), Result::OK);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(hardware.commands[4 * i + 2], 0, 1e-10);
}

TEST_F(ImpedanceTest, GravityRemainsLiveAndIncludesBothMovingFingers)
{
  Hardware hardware; Controller controller;
  const std::vector<double> scales{1.3, 1.1, 1, 1.1, 1.3, 1.3, 0};
  init(controller, {
    rclcpp::Parameter("torque_scales", scales),
    rclcpp::Parameter("cartesian_stiffness", std::vector<double>(6, 0))});
  start(controller, hardware);
  pinocchio::Model full;
  pinocchio::urdf::buildModel(A5_TEST_URDF_PATH, full, false, false);
  pinocchio::Data data(full);
  for (const double opening : {0.0, -0.015, -0.03})
  {
    hardware.states[6] += 0.1;
    hardware.states[36] = opening;
    ASSERT_EQ(tick(controller), Result::OK);
    const auto q = hardware.full_q();
    for (int i = 0; i < 6; ++i)
    {
      auto plus = q; auto minus = q;
      plus[i] += 1e-6; minus[i] -= 1e-6;
      const double gradient = (pinocchio::computePotentialEnergy(full, data, plus) -
                              pinocchio::computePotentialEnergy(full, data, minus)) / 2e-6;
      EXPECT_NEAR(hardware.commands[4 * i + 2], scales[i] * gradient, 1e-7);
    }
    EXPECT_DOUBLE_EQ(hardware.commands[26], 0.0);
  }
}

TEST_F(ImpedanceTest, DampingOpposesMotionAtCapturedPose)
{
  Hardware hardware; Controller controller;
  init(controller, {rclcpp::Parameter("cartesian_damping", std::vector<double>{8, 8, 8, 0.3, 0.3, 0.3})});
  start(controller, hardware);
  for (int i = 0; i < 6; ++i) hardware.states[6 * i + 1] = (i % 2 ? -0.1 : 0.15);
  ASSERT_EQ(tick(controller), Result::OK);
  double power = 0;
  for (int i = 0; i < 6; ++i) power += hardware.commands[4 * i + 2] * hardware.states[6 * i + 1];
  EXPECT_LT(power, -1e-5);
  const auto previous = hardware.commands;
  for (int i = 0; i < 6; ++i) hardware.states[6 * i + 1] *= -1;
  ASSERT_EQ(tick(controller), Result::OK);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(hardware.commands[4 * i + 2], -previous[4 * i + 2], 1e-10);
}

TEST_F(ImpedanceTest, OrientationCrossingPiUsesShortRotation)
{
  Hardware hardware; Controller controller;
  const double pi = std::acos(-1.0), offset = 0.01;
  hardware.states[30] = pi - offset;
  init(controller, {rclcpp::Parameter("cartesian_stiffness", std::vector<double>{0, 0, 0, 2, 2, 2})});
  start(controller, hardware);
  hardware.states[30] = -pi + offset;
  ASSERT_EQ(tick(controller, 1e-7), Result::OK);
  EXPECT_NEAR(hardware.commands[22], -2 * 2 * offset, 1e-10);
}

TEST_F(ImpedanceTest, StartupRampAndUniformTaskLimit)
{
  Hardware hardware; Controller controller;
  init(controller, {
    rclcpp::Parameter("impedance_ramp_duration", 2.0),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>(6, 0.2)),
    rclcpp::Parameter("torque_limits", std::vector<double>(7, 0.3))});
  start(controller, hardware);
  hardware.states[6] += 0.2;
  hardware.states[18] += 0.2;
  double initial_norm = 0;
  for (int n = 0; n < 150; ++n)
  {
    ASSERT_EQ(tick(controller), Result::OK);
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_LE(std::abs(hardware.commands[4 * i + 2]), 0.2 + 1e-12);
      if (n == 0) initial_norm += std::pow(hardware.commands[4 * i + 2], 2);
    }
  }
  double final_norm = 0;
  for (int i = 0; i < 6; ++i) final_norm += std::pow(hardware.commands[4 * i + 2], 2);
  EXPECT_GT(final_norm, initial_norm);
}

TEST_F(ImpedanceTest, TotalTorqueIsClampedAfterGravityAndImpedance)
{
  Hardware hardware; Controller controller;
  init(controller, {
    rclcpp::Parameter("torque_scales", std::vector<double>{1.3, 1.1, 1, 1.1, 1.3, 1.3, 0}),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>(6, 0.1)),
    rclcpp::Parameter("torque_limits", std::vector<double>(7, 0.1))});
  start(controller, hardware);
  // Full gravity is present already on the first cycle, with spring ramp initially zero.
  EXPECT_NEAR(std::abs(hardware.commands[6]), 0.1, 1e-12);
  hardware.states[6] += 0.3;
  ASSERT_EQ(tick(controller), Result::OK);
  for (int i = 0; i < 7; ++i) EXPECT_LE(std::abs(hardware.commands[4 * i + 2]), 0.1 + 1e-12);
}

TEST_F(ImpedanceTest, InvalidOrStaleStateLatchesDisabledUntilReactivation)
{
  for (const auto& invalid : std::vector<std::pair<int, double>>{
         {0, std::numeric_limits<double>::quiet_NaN()}, {1, std::numeric_limits<double>::infinity()},
         {3, 0}, {4, -1}, {4, 0.11}, {5, 0.11}, {5, std::numeric_limits<double>::quiet_NaN()},
         {36, std::numeric_limits<double>::quiet_NaN()}})
  {
    Hardware hardware; Controller controller;
    init(controller); start(controller, hardware);
    const double old = hardware.states[invalid.first];
    hardware.states[invalid.first] = invalid.second;
    EXPECT_EQ(tick(controller), Result::ERROR);
    hardware.expect_disabled();
    hardware.states[invalid.first] = old;
    EXPECT_EQ(tick(controller), Result::ERROR);
    hardware.expect_disabled();
    ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
    EXPECT_EQ(tick(controller), Result::OK);
  }
}

TEST_F(ImpedanceTest, ExcessiveOrZeroPeriodDisablesAndOutputOffNeverEnables)
{
  for (double dt : {0.0, -0.02, 0.11})
  {
    Hardware hardware; Controller controller;
    init(controller); start(controller, hardware);
    EXPECT_EQ(tick(controller, dt), Result::ERROR);
    hardware.expect_disabled();
  }
  Hardware hardware; Controller controller;
  init(controller, {rclcpp::Parameter("enable_output", false)});
  start(controller, hardware);
  hardware.expect_disabled();
}

TEST_F(ImpedanceTest, RejectsInvalidParametersAndWrongInterfaceOrder)
{
  for (const auto& parameter : std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("tcp_frame", "missing"),
    rclcpp::Parameter("tcp_frame", "gripper_finger_link1"),
    rclcpp::Parameter("cartesian_stiffness", std::vector<double>(5, 1)),
    rclcpp::Parameter("cartesian_stiffness", std::vector<double>(6, -1)),
    rclcpp::Parameter("cartesian_damping", std::vector<double>(6, std::numeric_limits<double>::quiet_NaN())),
    rclcpp::Parameter("torque_scales", std::vector<double>(7, 1)),
    rclcpp::Parameter("damping_gains", std::vector<double>(7, 1)),
    rclcpp::Parameter("effort_signs", std::vector<double>(7, 0)),
    rclcpp::Parameter("max_feedback_age", 0.0),
    rclcpp::Parameter("impedance_ramp_duration", -1.0),
    rclcpp::Parameter("torque_limits", std::vector<double>(7, 0.01))})
  {
    SCOPED_TRACE(parameter.get_name());
    Controller controller;
    init(controller, {parameter});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::ERROR);
  }
  Hardware hardware; Controller controller;
  init(controller);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::SUCCESS);
  hardware.bind(controller, true);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  hardware.expect_disabled();
}

TEST_F(ImpedanceTest, ReleaseSettlesAt50HzAndSlowerWithFeedbackDelay)
{
  // Preserve both the original failing gains and the user's later tuning.
  for (const bool retuned : {false, true})
  for (const double dt : {0.02, 0.04})
  for (const bool delayed : {false, true})
  {
    SCOPED_TRACE(std::to_string(dt) + (delayed ? " delayed" : " immediate") +
                 (retuned ? " retuned" : " original gains"));
    Hardware hardware; Controller controller;
    init(controller, {
      rclcpp::Parameter("cartesian_stiffness", retuned ? std::vector<double>{100, 100, 100, 12, 12, 12} :
        std::vector<double>{50, 50, 100, 2, 2, 2}),
      rclcpp::Parameter("cartesian_damping", retuned ? std::vector<double>{18, 18, 12, 0.63, 0.63, 0.63} :
        std::vector<double>{8, 8, 12, 0.3, 0.3, 0.3}),
      rclcpp::Parameter("damping_gains", std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.03, 0}),
      rclcpp::Parameter("wrench_limits", std::vector<double>{5, 5, 5, 0.5, 0.5, 0.5}),
      rclcpp::Parameter("impedance_torque_limits", std::vector<double>{0.8, 0.8, 0.8, 0.3, 0.3, 0.3}),
      rclcpp::Parameter("impedance_ramp_duration", 0.02)});
    start(controller, hardware);
    ASSERT_EQ(tick(controller), Result::OK);
    pinocchio::Model plant;
    // Eight coordinates make the dynamics independent of mimic implementation.
    // Both fingers remain at fixed measured openings in this release test.
    pinocchio::urdf::buildModel(A5_TEST_URDF_PATH, plant, false, false);
    plant.gravity.setZero(); // Isolate impedance with ideal gravity compensation.
    pinocchio::Data data(plant);
    Eigen::VectorXd q = hardware.full_q();
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(8);
    const auto target = q;
    q[0] += 0.04;
    const auto tcp = plant.getFrameId("tool_tcp");
    pinocchio::framesForwardKinematics(plant, data, target);
    const auto target_position = data.oMf[tcp].translation().eval();
    double early_peak = 0, late_peak = 0, late_speed = 0;
    Eigen::VectorXd measured_q = q, measured_dq = dq;
    const int cycles = static_cast<int>(8.0 / dt);
    for (int cycle = 0; cycle < cycles; ++cycle)
    {
      for (int i = 0; i < 6; ++i) { hardware.states[6*i] = delayed ? measured_q[i] : q[i]; hardware.states[6*i+1] = delayed ? measured_dq[i] : dq[i]; }
      measured_q = q; measured_dq = dq;
      ASSERT_EQ(tick(controller, dt), Result::OK);
      Eigen::Matrix<double, 6, 1> torque;
      for (int i = 0; i < 6; ++i) torque[i] = hardware.commands[4*i+2];
      for (int substep = 0; substep < static_cast<int>(dt / 0.001); ++substep)
      {
        pinocchio::crba(plant, data, q);
        Eigen::Matrix<double, 6, 6> mass = data.M.topLeftCorner<6,6>().selfadjointView<Eigen::Upper>();
        const Eigen::VectorXd nonlinear = pinocchio::nonLinearEffects(plant, data, q, dq);
        // Modest physical viscous friction, with immediate or one-period-old feedback.
        const Eigen::Matrix<double, 6, 1> ddq = mass.ldlt().solve(torque - nonlinear.head<6>() - 0.05*dq.head<6>());
        dq.head<6>() += 0.001 * ddq;
        q.head<6>() += 0.001 * dq.head<6>();
        ASSERT_TRUE(q.allFinite() && dq.allFinite());
      }
      pinocchio::framesForwardKinematics(plant, data, q);
      const double error = (data.oMf[tcp].translation() - target_position).norm();
      if (cycle * dt < 1.0) early_peak = std::max(early_peak, error);
      if (cycle * dt >= 6.0) { late_peak = std::max(late_peak, error); late_speed = std::max(late_speed, dq.head<6>().norm()); }
    }
    std::cout << "release dt=" << dt << " delayed=" << delayed << " retuned=" << retuned
              << ": initial_peak=" << early_peak << " final_peak=" << late_peak
              << " final_speed=" << late_speed << "\n";
    EXPECT_LT(late_peak, 0.25 * early_peak);
    EXPECT_LT(late_speed, 0.1);
  }
}

TEST_F(ImpedanceTest, DampingMustNotPushAfterVelocityReverses)
{
  Hardware hardware; Controller controller;
  init(controller, {
    rclcpp::Parameter("cartesian_stiffness", std::vector<double>(6, 0)),
    rclcpp::Parameter("cartesian_damping", std::vector<double>{8, 8, 12, 0.3, 0.3, 0.3}),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>{0.8, 0.8, 0.8, 0.3, 0.3, 0.3}),
    rclcpp::Parameter("impedance_torque_rates", std::vector<double>{2, 2, 2, 1, 1, 1})});
  start(controller, hardware);
  hardware.states[1] = 0.2;
  for (int cycle = 0; cycle < 30; ++cycle) ASSERT_EQ(tick(controller), Result::OK);
  EXPECT_LT(hardware.commands[2], 0.0);
  hardware.states[1] = -0.2;
  ASSERT_EQ(tick(controller), Result::OK);
  const double damping_power = hardware.commands[2] * hardware.states[1];
  std::cout << "damping power after reversal = " << damping_power << " W\n";
  EXPECT_LE(damping_power, 0.0);
}

TEST_F(ImpedanceTest, CoupledDampingRemainsDissipativeUnderTaskSaturation)
{
  Hardware hardware; Controller controller;
  const std::vector<double> caps{0.05, 0.04, 0.02, 0.01, 0.02, 0.03};
  init(controller, {
    rclcpp::Parameter("cartesian_stiffness", std::vector<double>(6, 0)),
    rclcpp::Parameter("cartesian_damping", std::vector<double>{8, 8, 12, 0.3, 0.3, 0.3}),
    rclcpp::Parameter("impedance_torque_limits", caps)});
  start(controller, hardware);
  std::mt19937 random(20261007);
  std::uniform_real_distribution<double> speed(-0.4, 0.4);
  for (int sample = 0; sample < 200; ++sample)
  {
    for (int i = 0; i < 6; ++i) hardware.states[6 * i + 1] = speed(random);
    ASSERT_EQ(tick(controller), Result::OK);
    double power = 0;
    for (int i = 0; i < 6; ++i)
    {
      power += hardware.commands[4 * i + 2] * hardware.states[6 * i + 1];
      EXPECT_LE(std::abs(hardware.commands[4 * i + 2]), caps[i] + 1e-12);
    }
    EXPECT_LE(power, 1e-12) << "sample " << sample;
  }
}
}  // namespace

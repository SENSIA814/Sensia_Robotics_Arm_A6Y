#include <a6_y_controllers/bounded_joint_impedance.hpp>
#include <a6_y_controllers/A6_y_joint_impedance_controller.hpp>
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
#include <thread>
#include <chrono>
#include <limits>
#include <fstream>
#include <cstdlib>
#include <rclcpp/parameter_map.hpp>

namespace
{
using Controller = a6_y_controllers::A6YJointImpedanceController;
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


class JointControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void init(Controller& controller, std::vector<rclcpp::Parameter> extra = {})
  {
    ASSERT_EQ(controller.init("a5_joint_impedance_test"), Result::OK);
    auto params = std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("urdf_path", std::string(A5_TEST_URDF_PATH)),
      rclcpp::Parameter("enable_output", true),
      rclcpp::Parameter("torque_scales", std::vector<double>{0, 0, 0, 0, 0, 0, 0}),
      rclcpp::Parameter("damping_gains", std::vector<double>(7, 0.0)),
      rclcpp::Parameter("joint_damping", std::vector<double>(6, 0.0)),
      // Disable saturation for the independent energy/geometry checks.
      rclcpp::Parameter("torque_limits", std::vector<double>(7, 10000.0)),
      rclcpp::Parameter("impedance_torque_limits", std::vector<double>(6, 10000.0)),
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
TEST_F(JointControllerTest, CapturesFixedAnglesRestoresEachJointAndRecaptures)
{
  Hardware hardware; Controller controller;
  const std::vector<double> stiffness{6, 8, 6, 1.5, 1.5, 1};
  init(controller, {rclcpp::Parameter("joint_stiffness", stiffness)});
  start(controller, hardware);
  for (int axis = 0; axis < 6; ++axis)
  {
    hardware.states[6 * axis] += 0.02;
    ASSERT_EQ(tick(controller, 1e-7), Result::OK);
    for (int i = 0; i < 6; ++i)
      EXPECT_NEAR(hardware.commands[4*i+2], i == axis ? -0.02*stiffness[i] : 0, 1e-8);
    EXPECT_DOUBLE_EQ(hardware.commands[26], 0);
    hardware.states[6 * axis] -= 0.02;
  }
  hardware.states[0] += 0.2;
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  hardware.expect_disabled();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(tick(controller), Result::OK);
  ASSERT_EQ(tick(controller), Result::OK);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(hardware.commands[4*i+2], 0, 1e-12);
  // Reconfigure must reload a fresh model and capture again.
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(tick(controller), Result::OK);
}

TEST_F(JointControllerTest, SpringDisplacementCapKeepsFullErrorForIntegral)
{
  Hardware hardware; Controller controller;
  init(controller, {rclcpp::Parameter("spring_error_limits",std::vector<double>(6,.1)),
    rclcpp::Parameter("joint_integral_gains",std::vector<double>{1,0,0,0,0,0})});
  start(controller,hardware);
  hardware.states[0]-=.3;
  pinocchio::Model plant; pinocchio::urdf::buildModel(A5_TEST_URDF_PATH,plant,false,false);
  pinocchio::Data data(plant);pinocchio::crba(plant,data,hardware.full_q());
  const Eigen::Matrix<double,6,6> M=data.M.topLeftCorner<6,6>().selfadjointView<Eigen::Upper>();
  a6_y_controllers::JointVector e=a6_y_controllers::JointVector::Zero();e[0]=.1;
  a6_y_controllers::JointVector K;K<<6,8,6,1.5,1.5,1;
  const auto expected=a6_y_controllers::bounded_joint_impedance(M,e,
    a6_y_controllers::JointVector::Zero(),K,a6_y_controllers::JointVector::Zero(),.02,
    a6_y_controllers::JointVector::Constant(-10000),a6_y_controllers::JointVector::Constant(10000));
  ASSERT_TRUE(expected.valid);
  ASSERT_EQ(tick(controller),Result::OK);
  for(int j=0;j<6;++j)EXPECT_NEAR(hardware.commands[4*j+2],expected.torque[j],1e-9);
  const double first=hardware.commands[2];
  for(int k=0;k<10;++k)ASSERT_EQ(tick(controller),Result::OK);
  // Integral sees the real 0.3 rad error, not the capped 0.1 rad spring input.
  EXPECT_NEAR(hardware.commands[2]-first,.06,1e-9);
}

TEST_F(JointControllerTest, RejectsInvalidSpringCaps)
{
  for(const auto& values : std::vector<std::vector<double>>{
      std::vector<double>(5,.1),std::vector<double>(6,-.1),
      std::vector<double>(6,std::numeric_limits<double>::infinity())}){
    Controller controller;init(controller,{rclcpp::Parameter("spring_error_limits",values)});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),Callback::ERROR);
  }
}

TEST_F(JointControllerTest, GravityUpdatesWithAllSevenCoordinatesAndGripperStaysZero)
{
  Hardware hardware; Controller controller;
  const std::vector<double> scales{1.3, 1.1, 1, 1.1, 1.3, 1.3, 0};
  const std::vector<double> signs{1, -1, 1, 1, 1, -1, 1};
  init(controller, {rclcpp::Parameter("joint_stiffness", std::vector<double>(6, 0)),
    rclcpp::Parameter("torque_scales", scales), rclcpp::Parameter("effort_signs", signs)});
  start(controller, hardware);
  pinocchio::Model plant;
  pinocchio::urdf::buildModel(A5_TEST_URDF_PATH, plant, false, false);
  pinocchio::Data data(plant);
  for (int sample = 0; sample < 3; ++sample)
  {
    hardware.states[6] += 0.05;
    hardware.states[12] -= 0.03;
    hardware.states[36] = -0.005 - sample * 0.01;
    ASSERT_EQ(tick(controller), Result::OK);
    const auto expected = pinocchio::computeGeneralizedGravity(plant, data, hardware.full_q());
    for (int i = 0; i < 6; ++i)
      EXPECT_NEAR(hardware.commands[4*i+2], signs[i]*scales[i]*expected[i], 1e-9);
    EXPECT_DOUBLE_EQ(hardware.commands[26], 0);
    EXPECT_DOUBLE_EQ(hardware.commands[24], hardware.states[36]);
  }
}

TEST_F(JointControllerTest, RampAndPerJointCapsBoundTheAdditionalTorque)
{
  Hardware hardware; Controller controller;
  init(controller, {rclcpp::Parameter("impedance_ramp_duration", 2.0),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>{0.08, 0.09, 0.07, 0.04, 0.03, 0.02})});
  start(controller, hardware);
  hardware.states[6] += 0.2;
  double first = 0, last = 0;
  const std::array<double, 6> caps{0.08, 0.09, 0.07, 0.04, 0.03, 0.02};
  for (int cycle = 0; cycle < 120; ++cycle)
  {
    ASSERT_EQ(tick(controller), Result::OK);
    for (int i = 0; i < 6; ++i) EXPECT_LE(std::abs(hardware.commands[4*i+2]), caps[i]+1e-12);
    if (cycle == 0) first = std::abs(hardware.commands[6]);
    last = std::abs(hardware.commands[6]);
  }
  EXPECT_GT(last, first);
}

TEST_F(JointControllerTest, TotalLimitIncludesFullGravityFromTheFirstTick)
{
  Hardware hardware; Controller controller;
  init(controller, {rclcpp::Parameter("torque_scales", std::vector<double>{1.3,1.1,1,1.1,1.3,1.3,0}),
    rclcpp::Parameter("torque_limits", std::vector<double>(7, 0.1)),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>(6, 0.05))});
  start(controller, hardware);
  EXPECT_NEAR(std::abs(hardware.commands[6]), 0.1, 1e-12);
  for (int i = 0; i < 6; ++i) { hardware.states[6*i] += 0.2; hardware.states[6*i+1] = 0.2; }
  ASSERT_EQ(tick(controller), Result::OK);
  for (int i = 0; i < 7; ++i) EXPECT_LE(std::abs(hardware.commands[4*i+2]), 0.1+1e-12);
}

TEST_F(JointControllerTest, DampingReversalAndAsymmetricSaturationRemainDissipative)
{
  Hardware hardware; Controller controller;
  init(controller, {rclcpp::Parameter("joint_stiffness", std::vector<double>(6, 0)),
    rclcpp::Parameter("joint_damping", std::vector<double>{1.2,1.5,1.2,0.12,0.12,0.08}),
    rclcpp::Parameter("damping_gains", std::vector<double>{.01,.01,.01,.01,.01,.03,0}),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>{.08,.02,.05,.03,.01,.02})});
  start(controller, hardware);
  std::mt19937 rng(55239);
  std::uniform_real_distribution<double> speed(-0.4, 0.4);
  for (int sample = 0; sample < 100; ++sample)
  {
    for (int i = 0; i < 6; ++i) hardware.states[6*i+1] = speed(rng);
    for (int reversed = 0; reversed < 2; ++reversed)
    {
      ASSERT_EQ(tick(controller), Result::OK);
      double power = 0;
      for (int i = 0; i < 6; ++i)
      {
        power += hardware.commands[4*i+2]*hardware.states[6*i+1];
        hardware.states[6*i+1] *= -1;
      }
      EXPECT_LE(power, 1e-10);
    }
  }
}

TEST_F(JointControllerTest, InvalidFeedbackDisablesAndLatchesUntilReactivation)
{
  for (const auto& invalid : std::vector<std::pair<int, double>>{
    {0, std::numeric_limits<double>::quiet_NaN()}, {1, std::numeric_limits<double>::infinity()},
    {3, 0}, {4, -1}, {4, 0.11}, {5, 0.11}, {36, std::numeric_limits<double>::quiet_NaN()}})
  {
    Hardware hardware; Controller controller;
    init(controller); start(controller, hardware);
    const double original = hardware.states[invalid.first];
    hardware.states[invalid.first] = invalid.second;
    EXPECT_EQ(tick(controller), Result::ERROR);
    hardware.expect_disabled();
    hardware.states[invalid.first] = original;
    EXPECT_EQ(tick(controller), Result::ERROR);
    hardware.expect_disabled();
    ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
    EXPECT_EQ(tick(controller), Result::OK);
  }
}

TEST_F(JointControllerTest, InvalidPeriodAndOutputDisabledNeverEnable)
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

TEST_F(JointControllerTest, RejectsBadParametersAndReorderedInterfaces)
{
  for (const auto& parameter : std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("joint_stiffness", std::vector<double>(5, 1)),
    rclcpp::Parameter("joint_stiffness", std::vector<double>(6, -1)),
    rclcpp::Parameter("joint_damping", std::vector<double>(6, std::numeric_limits<double>::quiet_NaN())),
    rclcpp::Parameter("torque_scales", std::vector<double>(7, 1)),
    rclcpp::Parameter("damping_gains", std::vector<double>(7, 1)),
    rclcpp::Parameter("effort_signs", std::vector<double>(7, 0)),
    rclcpp::Parameter("impedance_torque_limits", std::vector<double>(6, 0)),
    rclcpp::Parameter("max_feedback_age", 0.0),
    rclcpp::Parameter("impedance_ramp_duration", -1.0),
    rclcpp::Parameter("torque_limits", std::vector<double>(7, 0.01))})
  {
    Controller controller;
    init(controller, {parameter});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::ERROR) << parameter.get_name();
  }
  Hardware hardware; Controller controller;
  init(controller);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::SUCCESS);
  hardware.bind(controller, true);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::ERROR);
  hardware.expect_disabled();
}

TEST_F(JointControllerTest, EachJointReturnsAfterReleaseAt50HzAndWithDelay)
{
  for (const bool tight_caps : {false, true})
  for (const double dt : {0.02, 0.04})
  for (const bool delayed : {false, true})
  for (int displaced = 0; displaced < 6; ++displaced)
  {
    SCOPED_TRACE(std::to_string(dt) + " delay=" + std::to_string(delayed) + " joint=" +
                 std::to_string(displaced+1) + " tight_caps=" + std::to_string(tight_caps));
    Hardware hardware; Controller controller;
    const std::vector<double> caps = tight_caps ? std::vector<double>{.1,.1,.1,.005,.005,.001} :
      std::vector<double>{1.8,1.8,1.8,.6,.6,.6};
    init(controller, {
      rclcpp::Parameter("joint_damping", std::vector<double>{1.2,1.5,1.2,0.12,0.12,0.08}),
      rclcpp::Parameter("damping_gains", std::vector<double>{.01,.01,.01,.01,.01,.03,0}),
      rclcpp::Parameter("impedance_torque_limits", caps)});
    start(controller, hardware);
    ASSERT_EQ(tick(controller), Result::OK);
    pinocchio::Model plant;
    pinocchio::urdf::buildModel(A5_TEST_URDF_PATH, plant, false, false);
    plant.gravity.setZero();  // Isolate impedance under ideal gravity compensation.
    pinocchio::Data data(plant);
    Eigen::VectorXd q = hardware.full_q(), dq = Eigen::VectorXd::Zero(8);
    const auto target = q;
    q[displaced] += 0.05;
    Eigen::VectorXd measured_q = q, measured_dq = dq;
    double late_error = 0, late_speed = 0;
    bool hit_limit = false;
    for (int cycle = 0; cycle < static_cast<int>(8.0 / dt); ++cycle)
    {
      for (int i = 0; i < 6; ++i)
      {
        hardware.states[6*i] = delayed ? measured_q[i] : q[i];
        hardware.states[6*i+1] = delayed ? measured_dq[i] : dq[i];
      }
      measured_q = q; measured_dq = dq;
      ASSERT_EQ(tick(controller, dt), Result::OK);
      Eigen::Matrix<double, 6, 1> torque;
      for (int i = 0; i < 6; ++i)
      {
        torque[i] = hardware.commands[4*i+2];
        EXPECT_LE(std::abs(torque[i]), caps[i] + 1e-12);
        if (std::abs(torque[i]) >= caps[i] - 1e-9) hit_limit = true;
      }
      // Continuous plant integrates held command at 1 ms; fingers are fixed.
      for (int substep = 0; substep < static_cast<int>(dt/0.001); ++substep)
      {
        pinocchio::crba(plant, data, q);
        const Eigen::Matrix<double, 6, 6> M = data.M.topLeftCorner<6,6>().selfadjointView<Eigen::Upper>();
        const Eigen::VectorXd nonlinear = pinocchio::nonLinearEffects(plant, data, q, dq);
        const Eigen::Matrix<double, 6, 1> acceleration = M.ldlt().solve(torque - nonlinear.head<6>() - .05*dq.head<6>());
        dq.head<6>() += .001*acceleration;
        q.head<6>() += .001*dq.head<6>();
        ASSERT_TRUE(q.allFinite() && dq.allFinite());
      }
      if (cycle*dt >= 6.0)
      {
        late_error = std::max(late_error, (target.head<6>()-q.head<6>()).norm());
        late_speed = std::max(late_speed, dq.head<6>().norm());
      }
    }
    std::cout << "joint_release dt=" << dt << " delayed=" << delayed << " axis=" << displaced+1
              << " tight_caps=" << tight_caps
              << " late_error_rad=" << late_error << " late_speed_rad_s=" << late_speed << "\n";
    if (tight_caps) { EXPECT_TRUE(hit_limit); }
    EXPECT_LT(late_error, 0.005);
    EXPECT_LT(late_speed, 0.04);
  }
}


TEST_F(JointControllerTest, IntegralGatesResetsAndAccumulatesAtRest)
{
  for (const auto& scenario : std::vector<std::pair<double,double>>{{.001,0},{.4,0},{.1,.3},{.1,0}})
  {
    Hardware a,b; Controller integral,plain;
    init(integral,{rclcpp::Parameter("joint_integral_gains",std::vector<double>{4,4,4,.4,.4,.2})});
    init(plain); start(integral,a); start(plain,b);
    a.states[0]+=scenario.first; b.states[0]+=scenario.first;
    a.states[1]=b.states[1]=scenario.second;
    for(int k=0;k<100;++k){ ASSERT_EQ(tick(integral),Result::OK); ASSERT_EQ(tick(plain),Result::OK); }
    double difference=0;
    for(int j=0;j<6;++j) difference+=std::abs(a.commands[4*j+2]-b.commands[4*j+2]);
    if((scenario.first==.1 || scenario.first==.4) && scenario.second==0) EXPECT_GT(difference,.01);
    else EXPECT_NEAR(difference,0,1e-10);
    // Reactivation discards both the old bias and the old target.
    ASSERT_EQ(integral.on_deactivate(rclcpp_lifecycle::State()),Callback::SUCCESS);
    a.states[1]=0;
    ASSERT_EQ(integral.on_activate(rclcpp_lifecycle::State()),Callback::SUCCESS);
    ASSERT_EQ(tick(integral),Result::OK);
    for(int j=0;j<6;++j) EXPECT_NEAR(a.commands[4*j+2],0,1e-10);
  }
}

TEST_F(JointControllerTest, IntegralCannotWindUpIntoSaturation)
{
  Hardware hardware; Controller controller;
  init(controller,{rclcpp::Parameter("joint_integral_gains",std::vector<double>{4,4,4,.4,.4,.2}),
    rclcpp::Parameter("impedance_torque_limits",std::vector<double>(6,.02)),
    rclcpp::Parameter("integral_torque_limits",std::vector<double>(6,.02))});
  start(controller,hardware);
  const double original=hardware.states[0]; hardware.states[0]-=.3;
  for(int k=0;k<1000;++k){
    ASSERT_EQ(tick(controller),Result::OK);
    for(int j=0;j<6;++j) EXPECT_LE(std::abs(hardware.commands[4*j+2]),.02+1e-12);
  }
  // With zero velocity and the error removed, a wound-up integrator would
  // still drive the arm. The saturated positive error must not have accumulated.
  hardware.states[0]=original;
  ASSERT_EQ(tick(controller),Result::OK);
  for(int j=0;j<6;++j) EXPECT_NEAR(hardware.commands[4*j+2],0,1e-10);
}

TEST_F(JointControllerTest, RejectsInvalidIntegralConfiguration)
{
  for(const auto& parameter : std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("joint_integral_gains",std::vector<double>(5,1)),
    rclcpp::Parameter("joint_integral_gains",std::vector<double>(6,-1)),
    rclcpp::Parameter("integral_torque_limits",std::vector<double>(6,0)),
    rclcpp::Parameter("integral_torque_limits",std::vector<double>(6,10001.0)),
    rclcpp::Parameter("integral_torque_limits",std::vector<double>(6,std::numeric_limits<double>::infinity())),
    rclcpp::Parameter("joint_stiffness",std::vector<double>(6,0)),
    rclcpp::Parameter("integral_deadband",-1.0),
    rclcpp::Parameter("integral_error_window",0.001),
    rclcpp::Parameter("integral_velocity_threshold",0.0),
    rclcpp::Parameter("integral_velocity_threshold",std::numeric_limits<double>::quiet_NaN())})
  {
    Controller controller;
    init(controller,{rclcpp::Parameter("joint_integral_gains",std::vector<double>(6,1)),parameter});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),Callback::ERROR)<<parameter.get_name();
  }
}

// Independent nonsmooth plant: implicit Coulomb friction, including sticking at
// zero velocity. Coordinate minimization of 1/2 v'Mv - rhs'v + h Fc'|v|.
// This is a velocity-space friction solve, not the controller's torque-space QP.
Eigen::Matrix<double,6,1> friction_step(
  const Eigen::Matrix<double,6,6>& mass, const Eigen::Matrix<double,6,1>& v,
  const Eigen::Matrix<double,6,1>& applied, const Eigen::Matrix<double,6,1>& friction, double h)
{
  const Eigen::Matrix<double,6,1> rhs = mass*v + h*applied;
  Eigen::Matrix<double,6,1> out = v;
  for (int iteration=0; iteration<300; ++iteration)
  {
    const auto previous = out;
    for (int j=0; j<6; ++j)
    {
      const double r = rhs[j] - mass.row(j).dot(out) + mass(j,j)*out[j];
      out[j] = std::copysign(std::max(0.0, std::abs(r)-h*friction[j]), r)/mass(j,j);
    }
    if ((out-previous).lpNorm<Eigen::Infinity>() < 1e-11) return out;
  }
  ADD_FAILURE() << "Friction plant did not converge";
  return out;
}


TEST_F(JointControllerTest, FrictionPlantSticksAndDissipates)
{
  std::mt19937 rng(8172);
  std::normal_distribution<double> normal;
  using V = Eigen::Matrix<double,6,1>;
  using M = Eigen::Matrix<double,6,6>;
  for(int k=0;k<50;++k) {
    const M a=M::NullaryExpr([&](){return normal(rng);});
    const M mass=a.transpose()*a+M::Identity();
    const V friction=V::Constant(.1);
    const V still=friction_step(mass,V::Zero(),V::Constant(.09),friction,.001);
    EXPECT_DOUBLE_EQ(still.norm(),0);
    const V v=V::NullaryExpr([&](){return normal(rng);});
    const V after=friction_step(mass,v,V::Zero(),friction,.001);
    EXPECT_LE(after.dot(mass*after),v.dot(mass*v)+1e-12);
  }
}

TEST_F(JointControllerTest, ReversedLargePushReleasesOpposingIntegral)
{
  Hardware hardware; Controller controller;
  const std::array<double,6> recorded{-.04520,.53769,.36183,.14019,.11387,.10853};
  for(int j=0;j<6;++j) hardware.states[6*j]=recorded[j];
  init(controller,rclcpp::parameter_map_from_yaml_file(A5_TEST_CONFIG_PATH)
    .at("/arm6/a6_y_joint_impedance_controller"));
  start(controller,hardware);
  for(int k=0;k<101;++k) ASSERT_EQ(tick(controller),Result::OK);
  // The arm is held to one side until its slow static compensation accumulates.
  hardware.states[30]=recorded[5]+.10;
  for(int k=0;k<1000;++k) ASSERT_EQ(tick(controller),Result::OK);
  EXPECT_LT(hardware.commands[22],-.01);
  // Reproduce the log's reversal: e6=+23.17 deg, v6~=0. The old negative
  // integral must not remain latched merely because the error exceeds 20 deg.
  hardware.states[30]=recorded[5]-.4044;
  ASSERT_EQ(tick(controller),Result::OK);
  EXPECT_GT(hardware.commands[22],0.0); // old bias must be gone on the first sample
  for(int k=0;k<99;++k) ASSERT_EQ(tick(controller),Result::OK);
  std::cout << "reversed_push e6_deg=23.17 torque6_nm=" << hardware.commands[22] << std::endl;
  EXPECT_GT(hardware.commands[22],.01);
}


TEST_F(JointControllerTest, FastLargeReversalClearsButDoesNotAccumulate)
{
  Hardware a,b; Controller integral,plain;
  auto parameters=rclcpp::parameter_map_from_yaml_file(A5_TEST_CONFIG_PATH).at("/arm6/a6_y_joint_impedance_controller");
  init(integral,parameters); init(plain,parameters);
  ASSERT_TRUE(plain.get_node()->set_parameter(rclcpp::Parameter("joint_integral_gains",std::vector<double>(6,0))).successful);
  start(integral,a); start(plain,b);
  for(int k=0;k<101;++k){ASSERT_EQ(tick(integral),Result::OK); ASSERT_EQ(tick(plain),Result::OK);}
  const double target=a.states[30];
  a.states[30]=b.states[30]=target+.1;
  for(int k=0;k<1000;++k){ASSERT_EQ(tick(integral),Result::OK); ASSERT_EQ(tick(plain),Result::OK);}
  a.states[30]=b.states[30]=target-.5;
  a.states[31]=b.states[31]=.3;
  for(int k=0;k<51;++k){ASSERT_EQ(tick(integral),Result::OK); ASSERT_EQ(tick(plain),Result::OK);}
  for(int j=0;j<6;++j) EXPECT_NEAR(a.commands[4*j+2],b.commands[4*j+2],1e-10);
}

TEST_F(JointControllerTest, IntegralPreservesStaticBiasInsideTargetDeadband)
{
  Hardware hardware; Controller controller;
  init(controller,{rclcpp::Parameter("joint_integral_gains",std::vector<double>{4,4,4,.4,.4,.2})});
  start(controller,hardware);
  const double target=hardware.states[0];hardware.states[0]=target-.1;
  for(int k=0;k<100;++k)ASSERT_EQ(tick(controller),Result::OK);
  hardware.states[0]=target+.001;
  ASSERT_EQ(tick(controller),Result::OK);
  const double balancing=hardware.commands[2];EXPECT_GT(balancing,.01);
  for(int k=0;k<100;++k)ASSERT_EQ(tick(controller),Result::OK);
  EXPECT_NEAR(hardware.commands[2],balancing,1e-10);
}

TEST_F(JointControllerTest, RepeatedLargePushesWithStrongerResistance)
{
  const auto parameters=rclcpp::parameter_map_from_yaml_file(A5_TEST_CONFIG_PATH).at("/arm6/a6_y_joint_impedance_controller");
  const char* artifact=std::getenv("A6Y_LARGE_RETURN_CSV");
  std::ofstream csv;
  if(artifact){csv.open(artifact);ASSERT_TRUE(csv.good());csv<<"case_id,axis,round,dt,delay,friction_scale,time,error1,error2,error3,error4,error5,error6,speed1,speed2,speed3,speed4,speed5,speed6,torque1,torque2,torque3,torque4,torque5,torque6\n";}
  int case_id=0;
  for(double friction_scale : {0.,1.,1.25})
  for(bool delayed : {false,true})
  for(int axis=0;axis<7;++axis)
  {
    ++case_id; SCOPED_TRACE("large case="+std::to_string(case_id));
    Hardware hardware; Controller controller;
    const std::array<double,6> recorded{-.04520,.53769,.36183,.14019,.11387,.10853};
    for(int j=0;j<6;++j) hardware.states[6*j]=recorded[j];
    init(controller,parameters);
    ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("damping_gains",std::vector<double>{.01,.01,.01,.01,.01,.03,0})).successful);
    start(controller,hardware);
    for(int k=0;k<101;++k) ASSERT_EQ(tick(controller),Result::OK);
    pinocchio::Model plant;pinocchio::urdf::buildModel(A5_TEST_URDF_PATH,plant,false,false);plant.gravity.setZero();
    pinocchio::Data data(plant);
    const Eigen::VectorXd target=hardware.full_q();
    Eigen::VectorXd q=target,v=Eigen::VectorXd::Zero(8);
    const auto caps=controller.get_node()->get_parameter("impedance_torque_limits").as_double_array();
    Eigen::Matrix<double,6,1> friction;friction<<1.,1.6,2.2,.25,.25,.20;friction*=friction_scale;
    const std::array<double,6> deflection{.817,.25,.31,.48,.475,.4044};
    for(int round=0;round<2;++round)
    {
      const double sign=round==0 ? -1 : 1;
      q=target;v.setZero();
      // External hand holds opposite-side error first, building a history.
      for(int j=0;j<6;++j) if(axis==6 || axis==j) q[j]+=sign*.10;
      for(int k=0;k<250;++k){
        for(int j=0;j<6;++j){hardware.states[6*j]=q[j];hardware.states[6*j+1]=0;}
        ASSERT_EQ(tick(controller),Result::OK);
      }
      q=target;
      for(int j=0;j<6;++j) if(axis==6 || axis==j) q[j]-=sign*deflection[j];
      Eigen::VectorXd measured_q=q,measured_v=v;
      double late_error=0,late_speed=0,peak_speed=0,return_fraction=0;
      for(int cycle=0;cycle<1000;++cycle)
      {
        for(int j=0;j<6;++j){
          hardware.states[6*j]=delayed ? measured_q[j] : q[j];
          hardware.states[6*j+1]=delayed ? measured_v[j] : v[j];
          hardware.states[6*j+4]=hardware.states[6*j+5]=delayed ? .02 : 0;
        }
        measured_q=q;measured_v=v;
        ASSERT_EQ(tick(controller),Result::OK);
        Eigen::Matrix<double,6,1> torque;
        for(int j=0;j<6;++j){torque[j]=hardware.commands[4*j+2];ASSERT_LE(std::abs(torque[j]),caps[j]+1e-10);}
        if(csv){
          csv<<case_id<<','<<axis+1<<','<<round<<",0.02,"<<delayed<<','<<friction_scale<<','<<cycle*.02;
          for(int j=0;j<6;++j)csv<<','<<target[j]-q[j];
          for(int j=0;j<6;++j)csv<<','<<v[j];
          for(int j=0;j<6;++j)csv<<','<<torque[j];
          csv<<'\n';
        }
        for(int sub=0;sub<20;++sub){
          pinocchio::crba(plant,data,q);
          const Eigen::Matrix<double,6,6> M=data.M.topLeftCorner<6,6>().selfadjointView<Eigen::Upper>();
          const Eigen::VectorXd nle=pinocchio::nonLinearEffects(plant,data,q,v);
          // No viscous friction in the zero-friction stress case.
          const double viscous=friction_scale==0 ? 0 : .05;
          v.head<6>()=friction_step(M,v.head<6>(),torque-nle.head<6>()-viscous*v.head<6>(),friction,.001);
          q.head<6>()+=.001*v.head<6>();
          ASSERT_TRUE(q.allFinite() && v.allFinite());
          peak_speed=std::max(peak_speed,v.head<6>().lpNorm<Eigen::Infinity>());
          if(cycle>=150) for(int j=0;j<6;++j) if(axis==6 || axis==j)
            return_fraction=std::max(return_fraction,std::abs(target[j]-q[j])/deflection[j]);
        }
        if(cycle>=800){
          late_error=std::max(late_error,(target.head<6>()-q.head<6>()).lpNorm<Eigen::Infinity>());
          late_speed=std::max(late_speed,v.head<6>().lpNorm<Eigen::Infinity>());
        }
      }
      std::cout<<"large_release case="<<case_id<<" axis="<<axis+1<<" round="<<round<<" friction="<<friction_scale<<" delay="<<delayed
        <<" late_error_deg="<<late_error*180/M_PI<<" late_speed="<<late_speed<<" peak_speed="<<peak_speed<<std::endl;
      EXPECT_LT(late_error,2*M_PI/180);EXPECT_LT(late_speed,.03);
      EXPECT_LT(return_fraction,.5); EXPECT_LT(peak_speed,3.5);
    }
  }
}

TEST_F(JointControllerTest, FrictionReleaseAtRecordedPose)
{
  const auto parameters = rclcpp::parameter_map_from_yaml_file(A5_TEST_CONFIG_PATH)
    .at("/arm6/a6_y_joint_impedance_controller");
  const char* artifact = std::getenv("A6Y_RETURN_CSV");
  std::ofstream csv;
  if (artifact) {
    csv.open(artifact); ASSERT_TRUE(csv.good());
    csv << "profile,case_id,pose,axis,dt,delay,friction_scale,mass_scale,quantized,time,error1,error2,error3,error4,error5,error6,speed1,speed2,speed3,speed4,speed5,speed6,torque1,torque2,torque3,torque4,torque5,torque6\n";
  }
  struct Scenario { double dt; bool delay; double friction; double mass; bool quantized; double residual; double viscous; };
  const std::vector<Scenario> cases{
    {.02,false,1,1,false,0,.05}, {.02,true,1,1,false,0,.05},
    {.02,false,0,1,false,0,0}, {.02,true,0,1,false,0,0},
    {.04,false,1,1,false,0,.05}, {.04,true,1,1,false,0,.05},
    {.04,true,0,1,false,0,.05},
    {.02,true,.5,.7,true,.1,.03}, {.02,true,1.5,1.3,true,-.1,.08}};
  int case_id=0;
  for(bool baseline : {true,false})
  for(int pose : {0,1})
  for(const auto& c : cases)
  for(int axis=0;axis<6;++axis)
  {
    // Keep the old-gain negative control minimal and deterministic.
    if(baseline && (pose!=0 || &c!=&cases.front() || (axis!=0 && axis!=4))) continue;
    ++case_id;
    SCOPED_TRACE("case="+std::to_string(case_id));
    Hardware hardware; Controller controller;
    const std::array<double,6> recorded{-.05360,-.00591,.00515,-.29927,.18368,.02346};
    if(pose==0) for(int j=0;j<6;++j) hardware.states[6*j]=recorded[j];
    init(controller, parameters);
    ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("damping_gains",std::vector<double>{.01,.01,.01,.01,.01,.03,0})).successful);
    if(baseline) {
      for(const auto& p : std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("joint_stiffness",std::vector<double>{6,8,6,1.5,1.5,1}),
        rclcpp::Parameter("joint_damping",std::vector<double>{1.2,1.5,1.2,.12,.12,.08}),
        rclcpp::Parameter("joint_integral_gains",std::vector<double>(6,0)),
        rclcpp::Parameter("spring_error_limits",std::vector<double>(6,0))})
        ASSERT_TRUE(controller.get_node()->set_parameter(p).successful);
    }
    start(controller,hardware);
    // Release after the same 2 s startup ramp used in the actual launch.
    for(int k=0;k<101;++k) ASSERT_EQ(tick(controller),Result::OK);
    pinocchio::Model plant;
    pinocchio::urdf::buildModel(A5_TEST_URDF_PATH,plant,false,false);
    plant.gravity.setZero(); // ideal gravity cancellation + explicit residual below
    pinocchio::Data data(plant);
    Eigen::VectorXd q=hardware.full_q(), v=Eigen::VectorXd::Zero(8);
    const Eigen::VectorXd target=q;
    q[axis]+=(axis==1 || axis==2) ? .15 : -.15;
    Eigen::Matrix<double,6,1> friction; friction << .55,.25,.85,.08,.10,.05; friction *= c.friction;
    Eigen::VectorXd measured_q=q, measured_v=v;
    double late_error=0,late_speed=0,peak_speed=0,peak_error=0,return_fraction=0;
    const auto caps=controller.get_node()->get_parameter("impedance_torque_limits").as_double_array();
    for(int cycle=0;cycle<static_cast<int>(8/c.dt);++cycle)
    {
      for(int j=0;j<6;++j){
        double sensed_q=c.delay ? measured_q[j] : q[j];
        double sensed_v=c.delay ? measured_v[j] : v[j];
        if(c.quantized) {
          sensed_q=std::round(sensed_q/(25./65535))*(25./65535);
          const double v_step=(j<3 ? 20. : 60.)/4095;
          sensed_v=std::round(sensed_v/v_step)*v_step;
        }
        hardware.states[6*j]=sensed_q; hardware.states[6*j+1]=sensed_v;
        hardware.states[6*j+4]=hardware.states[6*j+5]=c.delay ? c.dt : 0;
      }
      measured_q=q; measured_v=v;
      ASSERT_EQ(tick(controller,c.dt),Result::OK);
      Eigen::Matrix<double,6,1> torque;
      for(int j=0;j<6;++j){
        torque[j]=hardware.commands[4*j+2];
        ASSERT_LE(std::abs(torque[j]),caps[j]+1e-10);
        if(c.quantized) { const double t_step=(j<3 ? 56. : 20.)/4095; torque[j]=std::round(torque[j]/t_step)*t_step; }
      }
      if(csv) {
        csv<<(baseline?"baseline":"configured")<<','<<case_id<<','<<pose<<','<<axis+1<<','<<c.dt<<','<<c.delay<<','<<c.friction<<','<<c.mass<<','<<c.quantized<<','<<cycle*c.dt;
        for(int j=0;j<6;++j) csv << ',' << target[j]-q[j];
        for(int j=0;j<6;++j) csv << ',' << v[j];
        for(int j=0;j<6;++j) csv << ',' << torque[j];
        csv << '\n';
      }
      for(int sub=0;sub<static_cast<int>(c.dt/.001);++sub){
        pinocchio::crba(plant,data,q);
        const Eigen::Matrix<double,6,6> M=c.mass*Eigen::Matrix<double,6,6>(data.M.topLeftCorner<6,6>().selfadjointView<Eigen::Upper>());
        const Eigen::VectorXd nle=c.mass*pinocchio::nonLinearEffects(plant,data,q,v);
        Eigen::Matrix<double,6,1> disturbance; disturbance << c.residual,-c.residual,c.residual,.2*c.residual,-.2*c.residual,.1*c.residual;
        v.head<6>()=friction_step(M,v.head<6>(),torque-nle.head<6>()-c.viscous*v.head<6>()+disturbance,friction,.001);
        q.head<6>()+=.001*v.head<6>();
        ASSERT_TRUE(q.allFinite() && v.allFinite());
        peak_speed=std::max(peak_speed,v.head<6>().lpNorm<Eigen::Infinity>());
        if(cycle*c.dt>=3) return_fraction=std::max(return_fraction,std::abs(target[axis]-q[axis])/.15);
      }
      const double err=(target.head<6>()-q.head<6>()).lpNorm<Eigen::Infinity>();
      peak_error=std::max(peak_error,err);
      if(cycle*c.dt>=6){late_error=std::max(late_error,err);late_speed=std::max(late_speed,v.head<6>().lpNorm<Eigen::Infinity>());}
    }
    std::cout<<"friction_release profile="<<(baseline?"baseline":"configured")<<" case="<<case_id<<" pose="<<pose<<" axis="<<axis+1
      <<" dt="<<c.dt<<" delay="<<c.delay<<" friction="<<c.friction<<" mass="<<c.mass<<" quantized="<<c.quantized
      <<" late_error_deg="<<late_error*180/M_PI<<" late_speed="<<late_speed<<" peak_speed="<<peak_speed<<" peak_error_deg="<<peak_error*180/M_PI<<std::endl;
    if(baseline) { EXPECT_GT(late_error,5*M_PI/180); }
    else { EXPECT_LT(late_error,2*M_PI/180); EXPECT_LT(late_speed,c.quantized ? .04 : .03); EXPECT_LT(peak_error,.2);
      EXPECT_LT(return_fraction,.5); EXPECT_LT(peak_speed,2.0); }
  }
}
TEST_F(JointControllerTest, SmallReleaseWithStrongerAssumedResistance)
{
 const auto params=rclcpp::parameter_map_from_yaml_file(A5_TEST_CONFIG_PATH).at("/arm6/a6_y_joint_impedance_controller");
 for(int axis:{0,2,4,5}) {
  SCOPED_TRACE("joint="+std::to_string(axis+1));
  Hardware hardware;Controller controller;
  const std::array<double,6> captured{-.09747,.59987,.30842,.33169,.10586,.03147};
  for(int j=0;j<6;++j)hardware.states[6*j]=captured[j];
  init(controller,params);
  controller.get_node()->set_parameter(rclcpp::Parameter("damping_gains",std::vector<double>{.01,.01,.01,.01,.01,.03,0}));
  start(controller,hardware);for(int k=0;k<101;++k)ASSERT_EQ(tick(controller),Result::OK);
  pinocchio::Model plant;pinocchio::urdf::buildModel(A5_TEST_URDF_PATH,plant,false,false);plant.gravity.setZero();
  pinocchio::Data data(plant);
  const Eigen::VectorXd target=hardware.full_q();Eigen::VectorXd q=target,v=Eigen::VectorXd::Zero(8);
  const double angle=5*M_PI/180;q[axis]-=angle;
  Eigen::Matrix<double,6,1> friction;friction<<1,1.6,2.2,.25,.25,.2;
  double onset=-1,fraction_at_3=-1;
  for(int cycle=0;cycle<400;++cycle){
   for(int j=0;j<6;++j){hardware.states[6*j]=q[j];hardware.states[6*j+1]=v[j];}
   ASSERT_EQ(tick(controller),Result::OK);
   Eigen::Matrix<double,6,1> torque;for(int j=0;j<6;++j)torque[j]=hardware.commands[4*j+2];
   for(int sub=0;sub<20;++sub){
    pinocchio::crba(plant,data,q);
    const Eigen::Matrix<double,6,6> M=data.M.topLeftCorner<6,6>().selfadjointView<Eigen::Upper>();
    const Eigen::VectorXd nle=pinocchio::nonLinearEffects(plant,data,q,v);
    v.head<6>()=friction_step(M,v.head<6>(),torque-nle.head<6>()-.05*v.head<6>(),friction,.001);
    q.head<6>()+=.001*v.head<6>();
    if(onset<0 && std::abs(q[axis]-(target[axis]-angle))>.001)onset=cycle*.02+(sub+1)*.001;
   }
   if(cycle==149)fraction_at_3=std::abs(target[axis]-q[axis])/angle;
  }
  std::cout<<"small_strong_friction joint="<<axis+1<<" onset_1mrad_s="<<onset<<" fraction_at_3s="<<fraction_at_3<<" final_error_deg="<<(target[axis]-q[axis])*180/M_PI<<std::endl;
  EXPECT_LT(fraction_at_3,.5);
 }
}

TEST_F(JointControllerTest, ExternalVrRosHandshakeDynamicTrackingAndLocalStop)
{
  using Gate=a6_y_controllers::JointReferenceGate;
  using State=a6_y_teleop_msgs::msg::VrState;
  Hardware hw;Controller controller;
  auto parameters=rclcpp::parameter_map_from_yaml_file(A5_TEST_CONFIG_PATH).at("/arm6/a6_y_joint_impedance_controller");
  init(controller,parameters);
  for(const auto& p:std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("reference_mode","external_vr"),rclcpp::Parameter("vr_allow_prepare",true),
      rclcpp::Parameter("joint_integral_gains",std::vector<double>(6,0.)),
      rclcpp::Parameter("max_control_period",.04),rclcpp::Parameter("impedance_ramp_duration",.02)})
    ASSERT_TRUE(controller.get_node()->set_parameter(p).successful);
  start(controller,hw);hw.expect_disabled();
  auto client=std::make_shared<rclcpp::Node>("vr_controller_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client);executor.add_node(controller.get_node()->get_node_base_interface());
  State state;bool received=false;
  auto sub=client->create_subscription<State>("/a5_joint_impedance_test/vr_state",rclcpp::QoS(1).best_effort(),
    [&](State::ConstSharedPtr s){state=*s;received=true;});
  auto refpub=client->create_publisher<Gate::Reference>("/a5_joint_impedance_test/vr_reference",rclcpp::QoS(1).best_effort());
  auto commandpub=client->create_publisher<Gate::Control>("/a5_joint_impedance_test/vr_control",10);
  auto now=[](){return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());};
  auto pump=[&](int n){for(int i=0;i<n;++i){executor.spin_some();EXPECT_EQ(tick(controller,.008),Result::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));executor.spin_some();}};
  for(int i=0;i<300 && !received;++i)pump(1);
  ASSERT_TRUE(received);ASSERT_EQ(state.state,State::DISABLED);hw.expect_disabled();
  Gate::Reference ref;ref.clock_id=state.clock_id;ref.controller_epoch=state.controller_epoch;
  ref.producer_id=33;ref.session_id=44;ref.valid=ref.live=true;
  for(int j=0;j<6;++j)ref.positions[j]=hw.states[6*j];
  auto send=[&](bool held){if(held!=ref.squeeze)++ref.clutch_epoch;ref.squeeze=held;
    ++ref.sequence;++ref.pose_sequence;ref.takeover_id=state.takeover_id;
    ref.generated_monotonic_ns=ref.pose_monotonic_ns=ref.buttons_monotonic_ns=ref.feedback_monotonic_ns=now();
    refpub->publish(ref);pump(1);};
  auto command=[&](uint8_t action){Gate::Control c;c.action=action;c.controller_epoch=state.controller_epoch;
    c.request_id=c.generated_monotonic_ns=now();c.producer_id=33;c.session_id=44;c.clutch_epoch=ref.clutch_epoch;
    commandpub->publish(c);pump(5);EXPECT_EQ(state.request_id,c.request_id);EXPECT_TRUE(state.request_accepted);};
  for(int i=0;i<10;++i)send(false);
  command(Gate::Control::PREPARE);ASSERT_EQ(state.state,State::HOLD);
  for(int j=0;j<7;++j)EXPECT_EQ(hw.commands[4*j+3],1.);
  send(false);command(Gate::Control::ARM);ASSERT_EQ(state.state,State::ARMED_HOLD);
  send(true);command(Gate::Control::CAPTURE);ASSERT_EQ(state.state,State::TELEOP);
  ref.positions=state.anchor_positions;for(int i=0;i<5;++i)send(true);
  const auto initial=state.anchor_positions;
  pinocchio::Model plant;pinocchio::urdf::buildModel(A5_TEST_URDF_PATH,plant,false,false);plant.gravity.setZero();
  pinocchio::Data data(plant);Eigen::VectorXd q=hw.full_q(),v=Eigen::VectorXd::Zero(8);
  double peak_speed=0.,peak_lead=0.;
  // Coupled six-axis dynamics with fixed fingers and ideal gravity cancellation.
  // These are simulation assumptions, not physical friction or stop evidence.
  for(int k=0;k<500;++k) {
    ref.positions=initial;ref.positions[0]+=.008*std::min(1.,k/200.);
    send(true);ASSERT_EQ(state.state,State::TELEOP);
    Eigen::Matrix<double,6,1> torque;
    for(int j=0;j<6;++j)torque[j]=hw.commands[4*j+2];
    for(int step=0;step<8;++step){
      pinocchio::crba(plant,data,q);data.M.template triangularView<Eigen::StrictlyLower>()=data.M.transpose().template triangularView<Eigen::StrictlyLower>();
      const auto bias=pinocchio::rnea(plant,data,q,v,Eigen::VectorXd::Zero(8));
      Eigen::Matrix<double,6,1> acceleration=data.M.topLeftCorner<6,6>().ldlt().solve(torque-bias.head<6>()-.02*v.head<6>());
      v.head<6>()+=acceleration*.001;q.head<6>()+=v.head<6>()*.001;
    }
    for(int j=0;j<6;++j){hw.states[6*j]=q[j];hw.states[6*j+1]=v[j];
      peak_speed=std::max(peak_speed,std::abs(v[j]));peak_lead=std::max(peak_lead,std::abs(state.reference_positions[j]-q[j]));}
  }
  EXPECT_GT(q[0]-initial[0],.004);EXPECT_LT(peak_speed,.25);EXPECT_LT(peak_lead,.05);
  send(false);ASSERT_EQ(state.state,State::STOPPING);const auto stop=state.reference_positions;
  for(int k=0;k<250;++k){
    send(false);ASSERT_NE(state.state,State::FAULT);
    for(int step=0;step<8;++step){
      pinocchio::crba(plant,data,q);data.M.template triangularView<Eigen::StrictlyLower>()=data.M.transpose().template triangularView<Eigen::StrictlyLower>();
      Eigen::Matrix<double,6,1> torque;for(int j=0;j<6;++j)torque[j]=hw.commands[4*j+2];
      const auto bias=pinocchio::rnea(plant,data,q,v,Eigen::VectorXd::Zero(8));
      v.head<6>()+=data.M.topLeftCorner<6,6>().ldlt().solve(torque-bias.head<6>()-.02*v.head<6>())*.001;
      q.head<6>()+=v.head<6>()*.001;
    }
    for(int j=0;j<6;++j){hw.states[6*j]=q[j];hw.states[6*j+1]=v[j];EXPECT_DOUBLE_EQ(state.reference_positions[j],stop[j]);}
  }
  EXPECT_EQ(state.state,State::ARMED_HOLD);EXPECT_LT(v.head<6>().cwiseAbs().maxCoeff(),.03);
  // A vanished bridge revokes permission while the C++ loop continues alone.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));pump(5);
  EXPECT_EQ(state.state,State::STOPPING);
  command(Gate::Control::DISARM);EXPECT_EQ(state.state,State::DISABLED);hw.expect_disabled();
  std::cout<<"VR coupled dynamics peak_speed="<<peak_speed<<" peak_lead="<<peak_lead<<std::endl;
}


TEST_F(JointControllerTest, ExternalDisabledWaitsForFreshFeedbackWithoutEnabling)
{
  Hardware hw;Controller controller;
  init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
                   rclcpp::Parameter("max_feedback_age",.05)});
  hw.states[4]=.08;
  start(controller,hw);hw.expect_disabled();
  hw.states[4]=0.;EXPECT_EQ(tick(controller,.008),Result::OK);hw.expect_disabled();
}

TEST_F(JointControllerTest, ExternalDisabledWaitsForFreshPeriodAfterDisableBatch)
{
  Hardware hw;Controller controller;
  init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
                   rclcpp::Parameter("vr_allow_prepare",true),
                   rclcpp::Parameter("max_control_period",.04)});
  start(controller,hw);hw.expect_disabled();
  // The driver's repeated disable frames take longer than an enabled control
  // cycle. Keep output off and wait for the next valid period without latching.
  EXPECT_EQ(tick(controller,.06),Result::OK);hw.expect_disabled();
  EXPECT_EQ(tick(controller,.008),Result::OK);hw.expect_disabled();
}

TEST_F(JointControllerTest, ExternalZeroToleranceRejectsInvalidParameters)
{
  for(const auto& values:std::vector<std::vector<double>>{
      std::vector<double>(6,0.),std::vector<double>(7,-.01),
      std::vector<double>(7,std::numeric_limits<double>::quiet_NaN())}) {
    Controller controller;
    init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
                     rclcpp::Parameter("vr_zero_tolerance",values)});
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),Callback::ERROR);
  }
}

TEST_F(JointControllerTest, ExternalZeroTolerancePreservesMeasuredHoldAndChecksGripper)
{
  using Gate=a6_y_controllers::JointReferenceGate;
  using State=a6_y_teleop_msgs::msg::VrState;
  Hardware hw;Controller controller;
  constexpr double arm_tolerance=.03490658503988659, gripper_tolerance=.0008342673824532896;
  init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
    rclcpp::Parameter("vr_allow_prepare",true),rclcpp::Parameter("vr_feedback_tolerance",0.),
    rclcpp::Parameter("vr_zero_tolerance",std::vector<double>{arm_tolerance,arm_tolerance,arm_tolerance,
      arm_tolerance,arm_tolerance,arm_tolerance,gripper_tolerance})});
  hw.states[6]=-.004768444342717615;hw.states[36]=-.04077230301060503;
  start(controller,hw);hw.expect_disabled();
  auto client=std::make_shared<rclcpp::Node>("zero_tolerance_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client);executor.add_node(controller.get_node()->get_node_base_interface());
  State state;bool received=false;
  auto sub=client->create_subscription<State>("/a5_joint_impedance_test/vr_state",rclcpp::QoS(1).best_effort(),
    [&](State::ConstSharedPtr message){state=*message;received=true;});
  auto pub=client->create_publisher<Gate::Control>("/a5_joint_impedance_test/vr_control",10);
  auto pump=[&](){executor.spin_some();EXPECT_EQ(tick(controller,.008),Result::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));executor.spin_some();};
  for(int i=0;i<300 && (!received || !pub->get_subscription_count());++i)pump();
  ASSERT_TRUE(received);ASSERT_GT(pub->get_subscription_count(),0U);
  auto prepare=[&](){Gate::Control command;command.action=Gate::Control::PREPARE;
    command.controller_epoch=state.controller_epoch;
    command.request_id=command.generated_monotonic_ns=uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
    pub->publish(command);
    for(int i=0;i<100 && state.request_id!=command.request_id;++i)pump();
    EXPECT_EQ(state.request_id,command.request_id);};
  prepare();EXPECT_FALSE(state.request_accepted);EXPECT_EQ(state.state,State::DISABLED);hw.expect_disabled();
  hw.states[36]=.0003895297016861095;
  prepare();ASSERT_TRUE(state.request_accepted);ASSERT_EQ(state.state,State::HOLD);
  for(int j=0;j<6;++j) {
    EXPECT_DOUBLE_EQ(state.reference_positions[j],hw.states[6*j]);
    EXPECT_DOUBLE_EQ(state.actual_positions[j],hw.states[6*j]);
    EXPECT_NEAR(hw.commands[4*j+2],0.,1e-12);  // No spring pull toward numerical zero.
  }
  EXPECT_DOUBLE_EQ(state.actual_positions[6],hw.states[36]);
  hw.states[36]=gripper_tolerance+1e-6;
  EXPECT_EQ(tick(controller,.008),Result::ERROR);hw.expect_disabled();
}

TEST_F(JointControllerTest, SupportedCommissioningRampsGravityFromZeroOnPrepare)
{
  using Gate=a6_y_controllers::JointReferenceGate;
  using State=a6_y_teleop_msgs::msg::VrState;
  Hardware hw;Controller controller;
  init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
    rclcpp::Parameter("vr_allow_prepare",true),rclcpp::Parameter("vr_allow_arm",false),
    rclcpp::Parameter("torque_scales",std::vector<double>{1.3,1.1,1.,1.1,1.3,1.3,0.}),
    rclcpp::Parameter("joint_stiffness",std::vector<double>(6,0.)),
    rclcpp::Parameter("impedance_ramp_duration",2.)});
  ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("vr_ramp_gravity_on_prepare",true)).successful);
  start(controller,hw);
  auto client=std::make_shared<rclcpp::Node>("gravity_ramp_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client);executor.add_node(controller.get_node()->get_node_base_interface());
  State state;bool received=false;
  auto sub=client->create_subscription<State>("/a5_joint_impedance_test/vr_state",rclcpp::QoS(1).best_effort(),
    [&](State::ConstSharedPtr s){state=*s;received=true;});
  auto pub=client->create_publisher<Gate::Control>("/a5_joint_impedance_test/vr_control",10);
  auto pump=[&](){executor.spin_some();EXPECT_EQ(tick(controller,.008),Result::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));executor.spin_some();};
  for(int i=0;i<300 && (!received || !pub->get_subscription_count());++i)pump();
  ASSERT_TRUE(received);ASSERT_GT(pub->get_subscription_count(),0U);
  Gate::Control command;command.action=Gate::Control::PREPARE;command.controller_epoch=state.controller_epoch;
  command.request_id=command.generated_monotonic_ns=uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
  pub->publish(command);
  std::vector<std::array<double,6>> torques;
  for(int i=0;i<400 && torques.size()<251;++i){
    pump();
    if(hw.commands[3]==1.) {
      std::array<double,6> value{};for(int j=0;j<6;++j)value[j]=hw.commands[4*j+2];torques.push_back(value);
    }
  }
  ASSERT_EQ(torques.size(),251U);
  // The final sample is full compensation; initial samples must ramp up, not step.
  EXPECT_GT(std::abs(torques.back()[2]),1.);
  for(int j=0;j<6;++j) {
    EXPECT_NEAR(torques.front()[j],0.,1e-10);
    for(size_t k=0;k<torques.size();++k)
      EXPECT_NEAR(torques[k][j],torques.back()[j]*std::min(1.,k*.008/2.),1e-9);
  }
}
TEST_F(JointControllerTest, IndependentGravityRampUsesFiveSecondsBeforeReady)
{
  using Gate=a6_y_controllers::JointReferenceGate;
  using State=a6_y_teleop_msgs::msg::VrState;
  Hardware hw;Controller controller;
  init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
    rclcpp::Parameter("vr_allow_prepare",true),rclcpp::Parameter("vr_allow_arm",false),
    rclcpp::Parameter("torque_scales",std::vector<double>{1.3,1.1,1.,1.1,1.3,1.3,0.}),
    rclcpp::Parameter("joint_stiffness",std::vector<double>(6,0.)),
    rclcpp::Parameter("impedance_ramp_duration",2.)});
  ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("vr_ramp_gravity_on_prepare",true)).successful);
  ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("vr_gravity_ramp_duration",5.)).successful);
  start(controller,hw);
  auto client=std::make_shared<rclcpp::Node>("independent_gravity_ramp_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client);executor.add_node(controller.get_node()->get_node_base_interface());
  State state;bool received=false;
  auto sub=client->create_subscription<State>("/a5_joint_impedance_test/vr_state",rclcpp::QoS(1).best_effort(),
    [&](State::ConstSharedPtr s){state=*s;received=true;});
  auto pub=client->create_publisher<Gate::Control>("/a5_joint_impedance_test/vr_control",10);
  auto pump=[&](){executor.spin_some();EXPECT_EQ(tick(controller,.008),Result::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));executor.spin_some();};
  for(int i=0;i<300 && (!received || !pub->get_subscription_count());++i)pump();
  ASSERT_TRUE(received);ASSERT_GT(pub->get_subscription_count(),0U);
  Gate::Control command;command.action=Gate::Control::PREPARE;command.controller_epoch=state.controller_epoch;
  command.request_id=command.generated_monotonic_ns=uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
  pub->publish(command);
  std::vector<std::array<double,6>> torques;
  for(int i=0;i<800 && torques.size()<626;++i){
    pump();
    if(hw.commands[3]==1.) {
      std::array<double,6> value{};for(int j=0;j<6;++j)value[j]=hw.commands[4*j+2];torques.push_back(value);
      if(torques.size()<600) EXPECT_FALSE(state.prepare_ramp_complete);
    }
  }
  ASSERT_EQ(torques.size(),626U);
  EXPECT_TRUE(state.prepare_ramp_complete);
  // The final sample is full compensation; initial samples must ramp up, not step.
  EXPECT_GT(std::abs(torques.back()[2]),1.);
  for(int j=0;j<6;++j) {
    EXPECT_NEAR(torques.front()[j],0.,1e-10);
    for(size_t k=0;k<torques.size();++k)
      EXPECT_NEAR(torques[k][j],torques.back()[j]*std::min(1.,k*.008/5.),1e-9);
  }
}
} // namespace

TEST_F(JointControllerTest, NativePositionCapturesWithoutEffortTracksAndStops)
{
  using Gate=a6_y_controllers::JointReferenceGate;
  using State=a6_y_teleop_msgs::msg::VrState;
  Hardware hw; a6_y_controllers::A6YVRPositionController controller;
  hw.states[6]=-.0048; // Approved zero tolerance must not snap to nominal zero.
  init(controller,{rclcpp::Parameter("reference_mode","external_vr"),
    rclcpp::Parameter("vr_allow_prepare",true),
    rclcpp::Parameter("vr_zero_tolerance",std::vector<double>(7,.01)),
    rclcpp::Parameter("vr_reference_velocity",std::vector<double>(6,.03)),
    rclcpp::Parameter("position_velocity_limit",std::vector<double>(6,.03)),
    rclcpp::Parameter("position_prepare_duration",.04)});
  start(controller,hw);hw.expect_disabled();
  auto client=std::make_shared<rclcpp::Node>("vr_position_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client);executor.add_node(controller.get_node()->get_node_base_interface());
  State state;bool received=false;
  auto sub=client->create_subscription<State>("/a5_joint_impedance_test/vr_state",rclcpp::QoS(1).best_effort(),
    [&](State::ConstSharedPtr s){state=*s;received=true;});
  auto refpub=client->create_publisher<Gate::Reference>("/a5_joint_impedance_test/vr_reference",rclcpp::QoS(1).best_effort());
  auto pub=client->create_publisher<Gate::Control>("/a5_joint_impedance_test/vr_control",10);
  auto now=[](){return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());};
  auto pump=[&](int n){for(int i=0;i<n;++i){executor.spin_some();EXPECT_EQ(tick(controller,.008),Result::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));executor.spin_some();}};
  for(int i=0;i<300 && !received;++i)pump(1);
  ASSERT_TRUE(received);ASSERT_TRUE(state.position_control);ASSERT_EQ(state.state,State::DISABLED);
  Gate::Reference ref;ref.clock_id=state.clock_id;ref.controller_epoch=state.controller_epoch;
  ref.producer_id=33;ref.session_id=44;ref.valid=ref.live=true;
  for(int j=0;j<6;++j)ref.positions[j]=hw.states[6*j];
  auto send=[&](bool held){if(held!=ref.squeeze)++ref.clutch_epoch;ref.squeeze=held;
    ++ref.sequence;++ref.pose_sequence;ref.takeover_id=state.takeover_id;
    ref.generated_monotonic_ns=ref.pose_monotonic_ns=ref.buttons_monotonic_ns=ref.feedback_monotonic_ns=now();
    refpub->publish(ref);pump(1);};
  auto command=[&](uint8_t action){Gate::Control c;c.action=action;c.controller_epoch=state.controller_epoch;
    c.request_id=c.generated_monotonic_ns=now();c.producer_id=33;c.session_id=44;c.clutch_epoch=ref.clutch_epoch;
    pub->publish(c);pump(5);ASSERT_EQ(state.request_id,c.request_id);ASSERT_TRUE(state.request_accepted);};
  for(int i=0;i<10;++i)send(false);
  command(Gate::Control::PREPARE);ASSERT_EQ(state.state,State::HOLD);
  for(int j=0;j<7;++j) {
    EXPECT_DOUBLE_EQ(hw.commands[4*j],hw.states[6*j]);
    EXPECT_DOUBLE_EQ(hw.commands[4*j+1],j<6 ? .03 : 0.);
    EXPECT_DOUBLE_EQ(hw.commands[4*j+2],0.);
    EXPECT_DOUBLE_EQ(hw.commands[4*j+3],j<6 ? 1. : 0.);
  }
  pump(5);send(false);ASSERT_TRUE(state.prepare_ramp_complete);
  command(Gate::Control::ARM);send(true);command(Gate::Control::CAPTURE);
  ASSERT_EQ(state.state,State::TELEOP);
  const auto anchor=state.anchor_positions;ref.positions=anchor;
  for(int k=0;k<100;++k) {
    ref.positions[0]=anchor[0]+.00001*k;send(true);ASSERT_EQ(state.state,State::TELEOP);
    for(int j=0;j<6;++j) {
      EXPECT_DOUBLE_EQ(hw.commands[4*j+2],0.);
      EXPECT_DOUBLE_EQ(hw.commands[4*j],state.reference_positions[j]);
      hw.states[6*j]=hw.commands[4*j];
    }
  }
  EXPECT_GT(hw.commands[0],anchor[0]+.0005);
  send(false);const auto stopped=state.reference_positions;
  // Real wall time is used for stop settling, independent of supplied update dt.
  for(int k=0;k<200;++k)send(false);
  EXPECT_EQ(state.state,State::ARMED_HOLD);
  EXPECT_EQ(state.reference_positions,stopped);
  command(Gate::Control::DISARM);hw.expect_disabled();
  command(Gate::Control::PREPARE);ASSERT_EQ(state.state,State::HOLD);
  hw.states[4]=1.; // Accepted-position age, not the latest CAN packet age.
  EXPECT_EQ(tick(controller),Result::ERROR);hw.expect_disabled();
}

TEST_F(JointControllerTest, NativePositionRejectsUngatedModeAndInconsistentSpeed)
{
  a6_y_controllers::A6YVRPositionController ungated;
  init(ungated);
  EXPECT_EQ(ungated.on_configure(rclcpp_lifecycle::State()),Callback::ERROR);
  a6_y_controllers::A6YVRPositionController too_slow;
  init(too_slow,{rclcpp::Parameter("reference_mode","external_vr")});
  EXPECT_EQ(too_slow.on_configure(rclcpp_lifecycle::State()),Callback::ERROR);
}

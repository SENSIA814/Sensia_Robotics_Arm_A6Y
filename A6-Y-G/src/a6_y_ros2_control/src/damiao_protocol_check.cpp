#include <a6_y_ros2_control/damiao_can.hpp>
#include <a6_y_ros2_control/deferred_motor_enable.hpp>
#include <a6_y_ros2_control/motor_command_guard.hpp>
#include <a6_y_ros2_control/status_request_policy.hpp>
#include <a6_y_ros2_control/steady_command_watchdog.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>

namespace
{
bool check_frame(const std::string& name, const a6_y_ros2_control::CanFrame& frame, std::uint32_t expected_id,
                 std::uint8_t expected_dlc, const std::array<std::uint8_t, 8>& expected_data)
{
  bool ok = true;
  if (frame.id != expected_id)
  {
    std::cerr << name << ": expected id 0x" << std::hex << expected_id << ", got 0x" << frame.id << std::dec
              << '\n';
    ok = false;
  }
  if (frame.dlc != expected_dlc)
  {
    std::cerr << name << ": expected dlc " << static_cast<int>(expected_dlc) << ", got "
              << static_cast<int>(frame.dlc) << '\n';
    ok = false;
  }
  for (std::size_t i = 0; i < expected_dlc; ++i)
  {
    if (frame.data[i] != expected_data[i])
    {
      std::cerr << name << ": data[" << i << "] expected 0x" << std::hex << static_cast<int>(expected_data[i])
                << ", got 0x" << static_cast<int>(frame.data[i]) << std::dec << '\n';
      ok = false;
    }
  }
  return ok;
}

bool near_zero(const std::string& name, double value, double tolerance)
{
  if (std::abs(value) <= tolerance)
  {
    return true;
  }
  std::cerr << name << ": expected near zero, got " << value << '\n';
  return false;
}
}  // namespace

int main()
{
  using namespace a6_y_ros2_control;

  bool ok = true;
  ok &= check_frame("enable", make_control_command(0x01, 0xFC), 0x01, 8,
                    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC });
  ok &= check_frame("disable", make_control_command(0x01, 0xFD), 0x01, 8,
                    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD });
  ok &= check_frame("set_zero", make_zero_position_command(0x07), 0x07, 8,
                    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE });
  ok &= check_frame("refresh_status", make_refresh_status_command(0x0A), 0x7FF, 8,
                    { 0x0A, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00 });
  ok &= check_frame("switch_pos_vel", make_switch_mode_command(0x01, 2), 0x7FF, 8,
                    { 0x01, 0x00, 0x55, 0x0A, 0x02, 0x00, 0x00, 0x00 });
  ok &= check_frame("pos_vel", make_pos_vel_command(0x01, 1.0, 0.5), 0x101, 8,
                    { 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x3F });
  ok &= check_frame("pos_vel_csp", make_pos_vel_csp_command(0x01, 1.0, 0.5), 0x401, 8,
                    { 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x3F });
  ok &= check_frame("mit_zero", make_mit_command(0x01, MotorLimits{ 12.5, 10.0, 28.0 }, 0.0, 0.0, 0.0, 0.0, 0.0),
                    0x01, 8, { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF });

  CanFrame feedback_frame;
  feedback_frame.id = 0x01;
  feedback_frame.dlc = 6;
  feedback_frame.data = { 0x00, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x00, 0x00 };

  DamiaoFeedback feedback;
  if (!parse_damiao_feedback(feedback_frame, MotorLimits{ 12.5, 10.0, 28.0 }, feedback))
  {
    std::cerr << "feedback_parse: parse failed\n";
    ok = false;
  }
  else
  {
    ok &= near_zero("feedback.q", feedback.q, 0.001);
    ok &= near_zero("feedback.dq", feedback.dq, 0.01);
    ok &= near_zero("feedback.tau", feedback.tau, 0.02);
  }

  const std::unordered_map<std::uint32_t, std::size_t> known_can_ids{ { 0x06, 0 }, { 0x16, 0 } };
  CanFrame joint6_status_frame;
  joint6_status_frame.id = 0x16;
  joint6_status_frame.dlc = 8;
  joint6_status_frame.data = { 0x06, 0x80, 0x55, 0x7F, 0xF8, 0x01, 0x1C, 0x1A };
  DamiaoParamFeedback param_feedback;
  if (parse_damiao_param_feedback(joint6_status_frame, known_can_ids, param_feedback))
  {
    std::cerr << "feedback_classification: ordinary joint6 status was misclassified as parameter feedback\n";
    ok = false;
  }

  CanFrame control_mode_feedback;
  control_mode_feedback.id = 0x7FF;
  control_mode_feedback.dlc = 8;
  control_mode_feedback.data = { 0x06, 0x00, 0x55, 0x0A, 0x02, 0x00, 0x00, 0x00 };
  if (!parse_damiao_param_feedback(control_mode_feedback, known_can_ids, param_feedback))
  {
    std::cerr << "feedback_classification: valid parameter feedback was rejected\n";
    ok = false;
  }

  SteadyCommandWatchdog watchdog;
  const auto watchdog_start = SteadyCommandWatchdog::Clock::time_point{};
  ok &= near_zero("watchdog.first_observation", watchdog.observe_at(watchdog_start), 1e-12);
  const double watchdog_period = watchdog.observe_at(watchdog_start + std::chrono::milliseconds(20));
  if (std::abs(watchdog_period - 0.02) > 1e-12)
  {
    std::cerr << "watchdog.period: expected 0.02, got " << watchdog_period << '\n';
    ok = false;
  }
  watchdog.reset();
  ok &= near_zero("watchdog.first_observation_after_reset",
                  watchdog.observe_at(watchdog_start + std::chrono::hours(1)), 1e-12);

  if (motor_command_disposition(true, false, true) != MotorCommandDisposition::skip_read_only)
  {
    std::cerr << "motor_command_guard: read-only motor was not skipped\n";
    ok = false;
  }
  if (motor_command_disposition(false, true, false) != MotorCommandDisposition::send)
  {
    std::cerr << "motor_command_guard: enabled healthy motor was not allowed\n";
    ok = false;
  }
  if (motor_command_disposition(false, false, false) != MotorCommandDisposition::block_disabled)
  {
    std::cerr << "motor_command_guard: disabled motor was not blocked\n";
    ok = false;
  }
  if (motor_command_disposition(false, true, true) != MotorCommandDisposition::block_faulted)
  {
    std::cerr << "motor_command_guard: faulted motor was not blocked\n";
    ok = false;
  }

  const auto batch_disposition = [](const auto& states) {
    return motor_command_batch_disposition(states.begin(), states.end(), [](const auto& state) {
      return motor_command_disposition(state[0], state[1], state[2]);
    });
  };
  const std::array<std::array<bool, 3>, 3> healthy_batch{ {
      { false, true, false },
      { true, false, true },
      { false, true, false },
  } };
  if (batch_disposition(healthy_batch) != MotorCommandDisposition::send)
  {
    std::cerr << "motor_command_guard: healthy command batch was not allowed\n";
    ok = false;
  }
  const std::array<std::array<bool, 3>, 3> late_disabled_batch{ {
      { false, true, false },
      { true, false, true },
      { false, false, false },
  } };
  if (batch_disposition(late_disabled_batch) != MotorCommandDisposition::block_disabled)
  {
    std::cerr << "motor_command_guard: later disabled motor did not block the entire batch\n";
    ok = false;
  }
  const std::array<std::array<bool, 3>, 3> late_faulted_batch{ {
      { false, true, false },
      { true, false, true },
      { false, true, true },
  } };
  if (batch_disposition(late_faulted_batch) != MotorCommandDisposition::block_faulted)
  {
    std::cerr << "motor_command_guard: later faulted motor did not block the entire batch\n";
    ok = false;
  }
  const std::array<std::array<bool, 3>, 2> read_only_batch{ {
      { true, false, true },
      { true, false, false },
  } };
  if (batch_disposition(read_only_batch) != MotorCommandDisposition::skip_read_only)
  {
    std::cerr << "motor_command_guard: all-read-only batch was not skipped\n";
    ok = false;
  }

  const std::array<std::array<double, 2>, 3> controller_not_ready{ {
      { 0.0, 0.0 },
      { 1.0, 1.0 },
      { 0.0, 1.0 },
  } };
  if (controller_enable_batch_state(
          controller_not_ready.begin(), controller_not_ready.end(),
          [](const auto& state) { return state[0] > 0.5; },
          [](const auto& state) { return state[1]; }) !=
      ControllerEnableBatchState::waiting)
  {
    std::cerr << "deferred_motor_enable: partial controller readiness did not keep motors disabled\n";
    ok = false;
  }
  const std::array<std::array<double, 2>, 3> controller_ready{ {
      { 0.0, 1.0 },
      { 1.0, 0.0 },
      { 0.0, 1.0 },
  } };
  if (controller_enable_batch_state(
          controller_ready.begin(), controller_ready.end(),
          [](const auto& state) { return state[0] > 0.5; },
          [](const auto& state) { return state[1]; }) !=
      ControllerEnableBatchState::ready)
  {
    std::cerr << "deferred_motor_enable: complete controller readiness did not allow enable\n";
    ok = false;
  }
  if (deferred_motor_enable_action(true, ControllerEnableBatchState::waiting, false) !=
          DeferredMotorEnableAction::wait_disabled ||
      deferred_motor_enable_action(true, ControllerEnableBatchState::ready, false) !=
          DeferredMotorEnableAction::enable_and_send ||
      deferred_motor_enable_action(true, ControllerEnableBatchState::waiting, true) !=
          DeferredMotorEnableAction::disable ||
      deferred_motor_enable_action(false, ControllerEnableBatchState::waiting, true) !=
          DeferredMotorEnableAction::legacy)
  {
    std::cerr << "deferred_motor_enable: unsafe lifecycle transition\n";
    ok = false;
  }

  if (status_request_scope(false, false) != StatusRequestScope::all_joints ||
      status_request_scope(false, true) != StatusRequestScope::all_joints)
  {
    std::cerr << "status_request_policy: manual-enable mode must poll all joints\n";
    ok = false;
  }
  if (status_request_scope(true, true) != StatusRequestScope::read_only_joints)
  {
    std::cerr << "status_request_policy: auto-enable mode did not poll read-only joints\n";
    ok = false;
  }
  if (status_request_scope(true, false) != StatusRequestScope::none)
  {
    std::cerr << "status_request_policy: auto-enable all-commandable mode added redundant polling\n";
    ok = false;
  }
  if (!status_request_includes_joint(StatusRequestScope::all_joints, false) ||
      !status_request_includes_joint(StatusRequestScope::all_joints, true) ||
      status_request_includes_joint(StatusRequestScope::read_only_joints, false) ||
      !status_request_includes_joint(StatusRequestScope::read_only_joints, true) ||
      status_request_includes_joint(StatusRequestScope::none, true))
  {
    std::cerr << "status_request_policy: request scope selected the wrong joints\n";
    ok = false;
  }
  if (runtime_status_request_scope(true, true, true, false) !=
          StatusRequestScope::all_joints ||
      runtime_status_request_scope(true, true, true, true) !=
          StatusRequestScope::read_only_joints)
  {
    std::cerr << "status_request_policy: deferred enable did not poll all joints while waiting\n";
    ok = false;
  }

  // Same feedback bits decode to twice the speed at J3's measured VMAX=20.
  auto normal_limits = motor_limits_from_type("DM4340P");
  auto j3_limits = normal_limits;
  CanFrame velocity_frame; velocity_frame.dlc=8;
  velocity_frame.data={0x13,0x80,0x00,0xA0,0x08,0x00,30,29};
  DamiaoFeedback normal_feedback, j3_feedback;
  ok &= apply_protocol_vmax("20",j3_limits);
  ok &= parse_damiao_feedback(velocity_frame,normal_limits,normal_feedback);
  ok &= parse_damiao_feedback(velocity_frame,j3_limits,j3_feedback);
  ok &= std::abs(j3_feedback.dq-2.0*normal_feedback.dq)<1e-12;
  ok &= j3_feedback.q==normal_feedback.q && j3_feedback.tau==normal_feedback.tau;
  ok &= normal_limits.dq_max==10.0 && j3_limits.dq_max==20.0;
  ok &= apply_protocol_vmax("",normal_limits) && normal_limits.dq_max==10.0;
  for(const auto* invalid : {"0","-1","nan","inf","20junk"}) {
    ok &= !apply_protocol_vmax(invalid,j3_limits) && j3_limits.dq_max==20.0;
  }

  if (!ok)
  {
    std::cerr << "Damiao protocol self-check failed\n";
    return 1;
  }

  std::cout << "Damiao protocol self-check passed\n";
  return 0;
}

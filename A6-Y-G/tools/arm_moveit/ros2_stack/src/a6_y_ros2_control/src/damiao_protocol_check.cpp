#include <a6_y_ros2_control/damiao_can.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

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

  if (!ok)
  {
    std::cerr << "Damiao protocol self-check failed\n";
    return 1;
  }

  std::cout << "Damiao protocol self-check passed\n";
  return 0;
}

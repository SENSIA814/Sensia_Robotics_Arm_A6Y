#ifndef A6_Y_ROS2_CONTROL__DAMIAO_CAN_HPP_
#define A6_Y_ROS2_CONTROL__DAMIAO_CAN_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace a6_y_ros2_control
{
struct CanFrame
{
  std::uint32_t id{ 0 };
  std::array<std::uint8_t, 8> data{};
  std::uint8_t dlc{ 0 };
};

struct MotorLimits
{
  double q_max{ 12.5 };
  double dq_max{ 10.0 };
  double tau_max{ 28.0 };
};

struct DamiaoFeedback
{
  double q{ 0.0 };
  double dq{ 0.0 };
  double tau{ 0.0 };
  int error{ 0 };
};

class SlcanBus
{
public:
  SlcanBus() = default;
  ~SlcanBus();

  bool open(const std::string& channel, std::uint32_t can_bitrate, std::uint32_t tty_baudrate,
            std::string* error_message);
  void close();
  bool is_open() const;

  bool send(const CanFrame& frame, std::string* error_message);
  bool flush(std::string* error_message);
  bool receive(CanFrame& frame, double timeout_seconds, std::string* error_message);
  std::size_t drain(std::vector<CanFrame>& frames, double timeout_seconds, std::size_t max_frames,
                    std::string* error_message);

private:
  bool configure_serial(std::uint32_t tty_baudrate, std::string* error_message);
  bool write_command(const std::string& command, std::string* error_message);
  bool wait_for_line(std::string& line, double timeout_seconds, std::string* error_message);
  bool parse_line(const std::string& line, CanFrame& frame) const;

  int fd_{ -1 };
  std::string rx_buffer_;
};

MotorLimits motor_limits_from_type(const std::string& motor_type);
bool parse_damiao_feedback(const CanFrame& frame, const MotorLimits& limits, DamiaoFeedback& feedback);

CanFrame make_control_command(std::uint32_t slave_id, std::uint8_t command);
CanFrame make_refresh_status_command(std::uint32_t slave_id);
CanFrame make_switch_mode_command(std::uint32_t slave_id, std::uint32_t mode);
CanFrame make_pos_vel_command(std::uint32_t slave_id, double position, double velocity);
CanFrame make_pos_vel_csp_command(std::uint32_t slave_id, double position, double velocity);
CanFrame make_mit_command(std::uint32_t slave_id, const MotorLimits& limits, double kp, double kd, double position,
                          double velocity, double torque);

}  // namespace a6_y_ros2_control

#endif  // A6_Y_ROS2_CONTROL__DAMIAO_CAN_HPP_

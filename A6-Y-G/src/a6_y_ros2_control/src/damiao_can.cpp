#include <a6_y_ros2_control/damiao_can.hpp>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

namespace a6_y_ros2_control
{
namespace
{
constexpr std::uint32_t kParamCanId = 0x7FF;
constexpr std::uint32_t kPosVelOffset = 0x100;
constexpr std::uint32_t kPosVelCspOffset = 0x400;
constexpr std::uint32_t kMitOffset = 0x000;
constexpr std::uint8_t kCtrlModeRid = 10;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

std::uint32_t float_to_uint(double value, double minimum, double maximum, int bits)
{
  value = clamp(value, minimum, maximum);
  const double span = maximum - minimum;
  return static_cast<std::uint32_t>((value - minimum) * ((1U << bits) - 1U) / span);
}

double uint_to_float(std::uint32_t raw, double minimum, double maximum, int bits)
{
  const double span = maximum - minimum;
  return (static_cast<double>(raw) / static_cast<double>((1U << bits) - 1U)) * span + minimum;
}

void pack_f32_le(std::array<std::uint8_t, 8>& data, std::size_t offset, double value)
{
  static_assert(std::numeric_limits<float>::is_iec559, "float32 packing requires IEC559 floats");
  const float as_float = static_cast<float>(value);
  std::uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, &as_float, sizeof(float));
  data[offset + 0] = bytes[0];
  data[offset + 1] = bytes[1];
  data[offset + 2] = bytes[2];
  data[offset + 3] = bytes[3];
}

void pack_u32_le(std::array<std::uint8_t, 8>& data, std::size_t offset, std::uint32_t value)
{
  data[offset + 0] = static_cast<std::uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  data[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
  data[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
}

speed_t tty_speed_from_baudrate(std::uint32_t baudrate)
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    default:
      return B115200;
  }
}

char slcan_speed_from_bitrate(std::uint32_t bitrate)
{
  switch (bitrate)
  {
    case 10000:
      return '0';
    case 20000:
      return '1';
    case 50000:
      return '2';
    case 100000:
      return '3';
    case 125000:
      return '4';
    case 250000:
      return '5';
    case 500000:
      return '6';
    case 800000:
      return '7';
    case 1000000:
      return '8';
    default:
      return '8';
  }
}

std::uint8_t hex_to_nibble(char c)
{
  if (c >= '0' && c <= '9')
  {
    return static_cast<std::uint8_t>(c - '0');
  }
  if (c >= 'a' && c <= 'f')
  {
    return static_cast<std::uint8_t>(10 + c - 'a');
  }
  if (c >= 'A' && c <= 'F')
  {
    return static_cast<std::uint8_t>(10 + c - 'A');
  }
  return 0;
}

bool is_hex(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

}  // namespace

SlcanBus::~SlcanBus()
{
  close();
}

bool SlcanBus::open(const std::string& channel, std::uint32_t can_bitrate, std::uint32_t tty_baudrate,
                    std::string* error_message)
{
  close();

  fd_ = ::open(channel.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    if (error_message)
    {
      *error_message = "failed to open " + channel + ": " + std::strerror(errno);
    }
    return false;
  }

  while (::flock(fd_, LOCK_EX | LOCK_NB) != 0)
  {
    const int lock_error = errno;
    if (lock_error == EINTR)
    {
      continue;
    }
    if (error_message)
    {
      if (lock_error == EWOULDBLOCK)
      {
        *error_message = "SLCAN device is already locked by another A6-Y process: " + channel;
      }
      else
      {
        *error_message = "failed to lock SLCAN device " + channel + ": " + std::strerror(lock_error);
      }
    }
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  if (!configure_serial(tty_baudrate, error_message))
  {
    close();
    return false;
  }

  ::tcflush(fd_, TCIOFLUSH);
  write_command("\r", nullptr);
  write_command("C\r", nullptr);

  const std::string speed_command = std::string("S") + slcan_speed_from_bitrate(can_bitrate) + "\r";
  if (!write_command(speed_command, error_message))
  {
    close();
    return false;
  }
  if (!write_command("O\r", error_message))
  {
    close();
    return false;
  }

  rx_buffer_.clear();
  return true;
}

void SlcanBus::close()
{
  if (fd_ >= 0)
  {
    write_command("C\r", nullptr);
    ::close(fd_);
    fd_ = -1;
  }
  rx_buffer_.clear();
}

bool SlcanBus::is_open() const
{
  return fd_ >= 0;
}

bool SlcanBus::configure_serial(std::uint32_t tty_baudrate, std::string* error_message)
{
  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0)
  {
    if (error_message)
    {
      *error_message = std::string("tcgetattr failed: ") + std::strerror(errno);
    }
    return false;
  }

  cfmakeraw(&tty);
  const speed_t speed = tty_speed_from_baudrate(tty_baudrate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    if (error_message)
    {
      *error_message = std::string("tcsetattr failed: ") + std::strerror(errno);
    }
    return false;
  }

  return true;
}

bool SlcanBus::write_command(const std::string& command, std::string* error_message)
{
  if (fd_ < 0)
  {
    if (error_message)
    {
      *error_message = "SLCAN device is not open";
    }
    return false;
  }

  std::size_t written = 0;
  while (written < command.size())
  {
    const ssize_t rc = ::write(fd_, command.data() + written, command.size() - written);
    if (rc < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        continue;
      }
      if (error_message)
      {
        *error_message = std::string("serial write failed: ") + std::strerror(errno);
      }
      return false;
    }
    written += static_cast<std::size_t>(rc);
  }

  return true;
}

bool SlcanBus::send(const CanFrame& frame, std::string* error_message)
{
  std::string command;
  if (!encode_slcan_command(frame, command, error_message))
  {
    return false;
  }
  return write_command(command, error_message);
}

bool encode_slcan_command(const CanFrame& frame, std::string& command_text, std::string* error_message)
{
  if (frame.id > 0x7FF || frame.dlc > 8)
  {
    if (error_message)
    {
      *error_message = "only standard CAN frames with DLC <= 8 are supported";
    }
    return false;
  }

  std::ostringstream command;
  command << 't' << std::uppercase << std::hex << std::setw(3) << std::setfill('0') << frame.id
          << std::nouppercase << std::dec << static_cast<int>(frame.dlc);
  for (std::size_t i = 0; i < frame.dlc; ++i)
  {
    command << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]);
  }
  command << '\r';

  command_text = command.str();
  return true;
}

bool SlcanBus::flush(std::string* error_message)
{
  if (fd_ < 0)
  {
    if (error_message)
    {
      *error_message = "SLCAN device is not open";
    }
    return false;
  }
  if (::tcdrain(fd_) != 0)
  {
    if (error_message)
    {
      *error_message = std::string("serial drain failed: ") + std::strerror(errno);
    }
    return false;
  }
  return true;
}

bool SlcanBus::receive(CanFrame& frame, double timeout_seconds, std::string* error_message)
{
  const double deadline_seconds = timeout_seconds;
  while (true)
  {
    std::string line;
    if (!wait_for_line(line, deadline_seconds, error_message))
    {
      return false;
    }
    if (line.empty() || line == "\a")
    {
      continue;
    }
    if (parse_line(line, frame))
    {
      return true;
    }
  }
}

std::size_t SlcanBus::drain(std::vector<CanFrame>& frames, double timeout_seconds, std::size_t max_frames,
                            std::string* error_message)
{
  std::size_t count = 0;
  for (; count < max_frames; ++count)
  {
    CanFrame frame;
    if (!receive(frame, count == 0 ? timeout_seconds : 0.0, error_message))
    {
      break;
    }
    frames.push_back(frame);
  }
  return count;
}

bool SlcanBus::wait_for_line(std::string& line, double timeout_seconds, std::string* error_message)
{
  while (true)
  {
    const auto pos = rx_buffer_.find_first_of("\r\n");
    if (pos != std::string::npos)
    {
      line = rx_buffer_.substr(0, pos);
      rx_buffer_.erase(0, pos + 1);
      return true;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    timeval timeout{};
    timeval* timeout_ptr = nullptr;
    if (timeout_seconds >= 0.0)
    {
      timeout.tv_sec = static_cast<long>(timeout_seconds);
      timeout.tv_usec = static_cast<long>((timeout_seconds - static_cast<double>(timeout.tv_sec)) * 1000000.0);
      timeout_ptr = &timeout;
    }

    const int rc = ::select(fd_ + 1, &read_fds, nullptr, nullptr, timeout_ptr);
    if (rc < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      if (error_message)
      {
        *error_message = std::string("serial select failed: ") + std::strerror(errno);
      }
      return false;
    }
    if (rc == 0)
    {
      return false;
    }

    char buffer[128];
    const ssize_t bytes_read = ::read(fd_, buffer, sizeof(buffer));
    if (bytes_read < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return false;
      }
      if (error_message)
      {
        *error_message = std::string("serial read failed: ") + std::strerror(errno);
      }
      return false;
    }
    if (bytes_read == 0)
    {
      return false;
    }

    rx_buffer_.append(buffer, static_cast<std::size_t>(bytes_read));
  }
}

bool SlcanBus::parse_line(const std::string& line, CanFrame& frame) const
{
  if (line.size() < 5 || line[0] != 't')
  {
    return false;
  }
  for (std::size_t i = 1; i < 5; ++i)
  {
    if (!is_hex(line[i]))
    {
      return false;
    }
  }

  const std::uint32_t id = (hex_to_nibble(line[1]) << 8) | (hex_to_nibble(line[2]) << 4) | hex_to_nibble(line[3]);
  const std::uint8_t dlc = hex_to_nibble(line[4]);
  if (dlc > 8 || line.size() < 5 + static_cast<std::size_t>(dlc) * 2)
  {
    return false;
  }

  frame = CanFrame{};
  frame.id = id;
  frame.dlc = dlc;
  for (std::size_t i = 0; i < dlc; ++i)
  {
    const std::size_t offset = 5 + i * 2;
    if (!is_hex(line[offset]) || !is_hex(line[offset + 1]))
    {
      return false;
    }
    frame.data[i] = static_cast<std::uint8_t>((hex_to_nibble(line[offset]) << 4) | hex_to_nibble(line[offset + 1]));
  }

  return true;
}

MotorLimits motor_limits_from_type(const std::string& motor_type)
{
  if (motor_type == "DM4310")
  {
    return { 12.5, 30.0, 10.0 };
  }
  if (motor_type == "DM4310_48V")
  {
    return { 12.5, 50.0, 10.0 };
  }
  if (motor_type == "DM6248P" || motor_type == "DM6428P" || motor_type == "DM3507")
  {
    if (motor_type == "DM3507")
    {
      return { 12.566, 50.0, 5.0 };
    }
    return { 12.566, 20.0, 120.0 };
  }
  if (motor_type == "DM10010L")
  {
    return { 12.5, 25.0, 200.0 };
  }
  if (motor_type == "DM10010")
  {
    return { 12.5, 20.0, 200.0 };
  }
  if (motor_type == "DM4340" || motor_type == "DM4340P" || motor_type == "DM4340_48V")
  {
    return { 12.5, 10.0, 28.0 };
  }
  return {};
}

bool apply_protocol_vmax(const std::string& value, MotorLimits& limits)
{
  if (value.empty()) return true;
  try {
    std::size_t consumed = 0;
    const double vmax = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(vmax) || vmax <= 0.0) return false;
    limits.dq_max = vmax;
    return true;
  } catch (const std::exception&) { return false; }
}

bool parse_damiao_feedback(const CanFrame& frame, const MotorLimits& limits, DamiaoFeedback& feedback)
{
  if (frame.dlc < 6)
  {
    return false;
  }

  const std::uint32_t q_uint = ((static_cast<std::uint32_t>(frame.data[1]) << 8) | frame.data[2]) & 0xFFFF;
  const std::uint32_t dq_uint =
      ((static_cast<std::uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4)) & 0x0FFF;
  const std::uint32_t tau_uint =
      (((static_cast<std::uint32_t>(frame.data[4]) & 0x0F) << 8) | frame.data[5]) & 0x0FFF;

  feedback.error = (frame.data[0] >> 4) & 0x0F;
  feedback.q = uint_to_float(q_uint, -limits.q_max, limits.q_max, 16);
  feedback.dq = uint_to_float(dq_uint, -limits.dq_max, limits.dq_max, 12);
  feedback.tau = uint_to_float(tau_uint, -limits.tau_max, limits.tau_max, 12);
  return true;
}

bool parse_damiao_param_feedback(const CanFrame& frame,
                                 const std::unordered_map<std::uint32_t, std::size_t>& known_can_ids,
                                 DamiaoParamFeedback& feedback)
{
  if (frame.dlc < 8)
  {
    return false;
  }
  feedback.opcode = frame.data[2];
  if (feedback.opcode != 0x33 && feedback.opcode != 0x55 && feedback.opcode != 0xAA)
  {
    return false;
  }

  feedback.slave_id = static_cast<std::uint32_t>(frame.data[0]) |
                      (static_cast<std::uint32_t>(frame.data[1]) << 8);
  if (known_can_ids.find(feedback.slave_id) == known_can_ids.end())
  {
    return false;
  }
  feedback.rid = frame.data[3];
  feedback.value = static_cast<std::uint32_t>(frame.data[4]) |
                   (static_cast<std::uint32_t>(frame.data[5]) << 8) |
                   (static_cast<std::uint32_t>(frame.data[6]) << 16) |
                   (static_cast<std::uint32_t>(frame.data[7]) << 24);
  return true;
}

CanFrame make_control_command(std::uint32_t slave_id, std::uint8_t command)
{
  CanFrame frame;
  frame.id = slave_id;
  frame.dlc = 8;
  frame.data.fill(0xFF);
  frame.data[7] = command;
  return frame;
}

CanFrame make_zero_position_command(std::uint32_t slave_id)
{
  return make_control_command(slave_id, 0xFE);
}

CanFrame make_refresh_status_command(std::uint32_t slave_id)
{
  CanFrame frame;
  frame.id = kParamCanId;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>(slave_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((slave_id >> 8) & 0xFF);
  frame.data[2] = 0xCC;
  return frame;
}

CanFrame make_switch_mode_command(std::uint32_t slave_id, std::uint32_t mode)
{
  CanFrame frame;
  frame.id = kParamCanId;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>(slave_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((slave_id >> 8) & 0xFF);
  frame.data[2] = 0x55;
  frame.data[3] = kCtrlModeRid;
  pack_u32_le(frame.data, 4, mode);
  return frame;
}

CanFrame make_pos_vel_command(std::uint32_t slave_id, double position, double velocity)
{
  CanFrame frame;
  frame.id = slave_id + kPosVelOffset;
  frame.dlc = 8;
  pack_f32_le(frame.data, 0, position);
  pack_f32_le(frame.data, 4, std::abs(velocity));
  return frame;
}

CanFrame make_pos_vel_csp_command(std::uint32_t slave_id, double position, double velocity)
{
  CanFrame frame;
  frame.id = slave_id + kPosVelCspOffset;
  frame.dlc = 8;
  pack_f32_le(frame.data, 0, position);
  pack_f32_le(frame.data, 4, std::abs(velocity));
  return frame;
}

CanFrame make_mit_command(std::uint32_t slave_id, const MotorLimits& limits, double kp, double kd, double position,
                          double velocity, double torque)
{
  const std::uint32_t kp_uint = float_to_uint(kp, 0.0, 500.0, 12);
  const std::uint32_t kd_uint = float_to_uint(kd, 0.0, 5.0, 12);
  const std::uint32_t q_uint = float_to_uint(position, -limits.q_max, limits.q_max, 16);
  const std::uint32_t dq_uint = float_to_uint(velocity, -limits.dq_max, limits.dq_max, 12);
  const std::uint32_t tau_uint = float_to_uint(torque, -limits.tau_max, limits.tau_max, 12);

  CanFrame frame;
  frame.id = slave_id + kMitOffset;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>((q_uint >> 8) & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(q_uint & 0xFF);
  frame.data[2] = static_cast<std::uint8_t>((dq_uint >> 4) & 0xFF);
  frame.data[3] = static_cast<std::uint8_t>(((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F));
  frame.data[4] = static_cast<std::uint8_t>(kp_uint & 0xFF);
  frame.data[5] = static_cast<std::uint8_t>((kd_uint >> 4) & 0xFF);
  frame.data[6] = static_cast<std::uint8_t>(((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F));
  frame.data[7] = static_cast<std::uint8_t>(tau_uint & 0xFF);
  return frame;
}

}  // namespace a6_y_ros2_control

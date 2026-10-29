#ifndef IO__GIMBAL1__GIMBAL1_HPP
#define IO__GIMBAL1__GIMBAL1_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "serial/serial.h"

namespace io
{

struct Gimbal1Feedback
{
  std::array<double, 26> values{};

  double yaw_target_deg = 0.0;
  double yaw_actual_deg = 0.0;
  double yaw_max_speed_deg_s = 0.0;
  double yaw_speed_deg_s = 0.0;
  int yaw_active = 0;
  int yaw_fault = 0;
  int yaw_motor_state = 0;

  double pitch_target_deg = 0.0;
  double pitch_actual_deg = 0.0;
  double pitch_max_speed_deg_s = 0.0;
  double pitch_speed_deg_s = 0.0;
  int pitch_active = 0;
  int pitch_fault = 0;
  int pitch_motor_state = 0;

  std::chrono::steady_clock::time_point received_at;
};

// Independent ASCII interface for relative commands and 26-field CSV
// feedback. It does not use CBoard or the legacy binary Gimbal protocol.
class Gimbal1
{
public:
  explicit Gimbal1(const std::string & config_path);
  ~Gimbal1();

  Gimbal1(const Gimbal1 &) = delete;
  Gimbal1 & operator=(const Gimbal1 &) = delete;

  void enable();
  void disable();
  void move_relative_yaw(double angle_deg, double max_speed_deg_s);
  void move_relative_pitch(double angle_deg, double max_speed_deg_s);

  // Non-blocking poll. Partial serial reads are retained until newline.
  bool poll_feedback(Gimbal1Feedback & feedback, std::string * raw_line = nullptr);

  bool enabled() const;
  const std::string & port() const;
  uint32_t baudrate() const;

  static std::string enable_command();
  static std::string disable_command();
  static std::string relative_yaw_command(double angle_deg, double max_speed_deg_s);
  static std::string relative_pitch_command(double angle_deg, double max_speed_deg_s);
  static Gimbal1Feedback parse_feedback_line(const std::string & line);

private:
  serial::Serial serial_;
  mutable std::mutex mutex_;

  std::string port_;
  uint32_t baudrate_ = 115200;
  uint32_t timeout_ms_ = 20;
  double max_relative_angle_deg_ = 5.0;
  double max_speed_deg_s_ = 30.0;
  bool enabled_ = false;
  std::string rx_buffer_;

  void validate_relative_command(double angle_deg, double max_speed_deg_s) const;
  void write_command_unlocked(const std::string & command);
};

}  // namespace io

#endif  // IO__GIMBAL1__GIMBAL1_HPP

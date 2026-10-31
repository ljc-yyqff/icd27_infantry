#ifndef IO__GIMBAL1__GIMBAL1_HPP
#define IO__GIMBAL1__GIMBAL1_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <fstream>
#include <string>
#include <vector>
#include "serial/serial.h"

namespace io
{
struct Gimbal1Feedback
{
  std::array<double, 9> values{};
  // cmd is the moving planner setpoint, NOT the final commanded target.
  double pitch_setpoint_deg = 0, pitch_actual_deg = 0;
  double yaw_setpoint_deg = 0, yaw_actual_deg = 0;
  double imu_pitch_deg = 0, imu_yaw_deg = 0, tx_dt_ms = 0;
  double pitch_torque_nm = 0, yaw_torque_nm = 0;
  std::chrono::steady_clock::time_point received_at{};
};

struct Gimbal1Status
{
  bool yaw = false;
  int state = -1;  // unknown until a valid #YP/#PP response arrives
  int mode = -1;   // 0=encoder, 1=IMU/spatial; missing remains unknown
};

struct Gimbal1CommandReceipt
{
  std::string command;
  bool ok = false;
  std::string reason;
  int state = -1;
  int mode = -1;
  double target_deg = 0.0;
  std::chrono::steady_clock::time_point received_at{};
};

struct Gimbal1GlobalStatus
{
  bool ok = false;
  std::string reason;
  int yaw_state = -1, yaw_fault = -1, yaw_mode = -1;
  int pitch_state = -1, pitch_fault = -1, pitch_mode = -1;
  int imu_valid = -1;
  std::chrono::steady_clock::time_point received_at{};
};

class Gimbal1
{
public:
  explicit Gimbal1(const std::string & config_path);
  ~Gimbal1();
  Gimbal1(const Gimbal1 &) = delete;
  Gimbal1 & operator=(const Gimbal1 &) = delete;

  // Sends ge only. Motor execution/readiness is owned by the A-board.
  void enable(const std::atomic<bool> * cancel = nullptr);
  void disable();
  // Optional persistent wire trace. Call before enabling; no motion is sent.
  void start_logging(const std::string & path);
  void move_relative_yaw(double angle_deg, double max_speed_deg_s);
  void move_relative_pitch(double angle_deg, double max_speed_deg_s);
  // Drains complete lines, routes status responses, returns newest telemetry.
  // Call regularly even when tracking is paused, to avoid a stale receive queue.
  bool poll_feedback(Gimbal1Feedback & feedback, std::string * raw_line = nullptr);
  std::vector<Gimbal1Feedback> take_feedback_samples();
  bool enabled() const;  // ge requested only; never asserts motor readiness
  bool ready() const;
  std::string status_text() const;
  const std::string & port() const;
  uint32_t baudrate() const;

  static std::string enable_command();
  static std::string disable_command();
  static std::string relative_yaw_command(double angle_deg, double max_speed_deg_s);
  static std::string relative_pitch_command(double angle_deg, double max_speed_deg_s);
  static Gimbal1Feedback parse_feedback_line(const std::string & line);
  static bool parse_status_line(const std::string & line, Gimbal1Status & status);
  static bool parse_command_receipt(const std::string & line, Gimbal1CommandReceipt & receipt);
  static bool parse_global_status(const std::string & line, Gimbal1GlobalStatus & status);

private:
  serial::Serial serial_;
  std::ofstream trace_;
  std::chrono::steady_clock::time_point trace_started_{};
  void trace_unlocked(const std::string & kind, const std::string & message) noexcept;
  std::vector<Gimbal1Feedback> feedback_samples_; // bounded, optional vision pose consumer
  mutable std::mutex mutex_;
  std::string port_, rx_buffer_, raw_feedback_, diagnostic_;
  std::string yaw_response_, pitch_response_, rejected_line_;
  uint32_t baudrate_ = 115200, timeout_ms_ = 20;
  double max_relative_angle_deg_ = 5, max_speed_deg_s_ = 30;
  bool enabled_ = false, have_feedback_ = false;
  Gimbal1Feedback feedback_;
  Gimbal1CommandReceipt receipt_;
  Gimbal1GlobalStatus global_status_;
  using Time = std::chrono::steady_clock::time_point;
  Time last_write_{};

  void validate_relative_command(double angle, double speed) const;
  void write_command_unlocked(const std::string & command);
  bool service_unlocked();
  bool ready_unlocked() const;
  std::string status_text_unlocked() const;
  void move_unlocked(bool yaw, double angle, double speed);
};
}  // namespace io
#endif

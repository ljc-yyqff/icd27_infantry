#include "gimbal1.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

namespace io
{
namespace
{
template <typename T>
T read_or(const YAML::Node & yaml, const char * key, const T & fallback)
{
  return yaml[key] ? yaml[key].as<T>() : fallback;
}

std::string trim(std::string value)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

int integer_field(double value, size_t field_index)
{
  const auto rounded = std::round(value);
  if (std::abs(value - rounded) > 1e-9) {
    throw std::invalid_argument(
      fmt::format("gimbal1 CSV field {} must be an integer", field_index + 1));
  }
  return static_cast<int>(rounded);
}
}  // namespace

Gimbal1::Gimbal1(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);
  if (!yaml["com_port"]) {
    throw std::runtime_error("gimbal1 config is missing 'com_port'");
  }

  port_ = yaml["com_port"].as<std::string>();
  baudrate_ = read_or<uint32_t>(yaml, "baudrate", 115200);
  timeout_ms_ = read_or<uint32_t>(yaml, "serial_timeout_ms", 20);
  max_relative_angle_deg_ = read_or<double>(yaml, "gimbal1_max_relative_angle_deg", 5.0);
  max_speed_deg_s_ = read_or<double>(yaml, "gimbal1_max_speed_deg_s", 30.0);

  if (port_.empty()) throw std::runtime_error("gimbal1 com_port must not be empty");
  if (baudrate_ == 0) throw std::runtime_error("gimbal1 baudrate must be positive");
  if (!std::isfinite(max_relative_angle_deg_) || max_relative_angle_deg_ <= 0.0) {
    throw std::runtime_error("gimbal1_max_relative_angle_deg must be positive");
  }
  if (!std::isfinite(max_speed_deg_s_) || max_speed_deg_s_ <= 0.0) {
    throw std::runtime_error("gimbal1_max_speed_deg_s must be positive");
  }

  serial_.setPort(port_);
  serial_.setBaudrate(baudrate_);
  serial_.setBytesize(serial::eightbits);
  serial_.setParity(serial::parity_none);
  serial_.setStopbits(serial::stopbits_one);
  serial_.setFlowcontrol(serial::flowcontrol_none);
  auto timeout = serial::Timeout::simpleTimeout(timeout_ms_);
  serial_.setTimeout(timeout);
  serial_.open();
}

Gimbal1::~Gimbal1()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!serial_.isOpen()) return;

  // Best-effort fail-safe. Destructors must not throw. If disable() already
  // succeeded, avoid sending the same command a second time.
  if (enabled_) {
    try {
      write_command_unlocked(disable_command());
    } catch (...) {
    }
  }

  try {
    serial_.close();
  } catch (...) {
  }
  enabled_ = false;
}

void Gimbal1::enable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  write_command_unlocked(enable_command());
  enabled_ = true;
}

void Gimbal1::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Always transmit gd, so this method can also be used as an emergency stop.
  write_command_unlocked(disable_command());
  enabled_ = false;
}

void Gimbal1::move_relative_yaw(double angle_deg, double max_speed_deg_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) throw std::runtime_error("gimbal1 is disabled; send 'enable' first");
  validate_relative_command(angle_deg, max_speed_deg_s);
  write_command_unlocked(relative_yaw_command(angle_deg, max_speed_deg_s));
}

void Gimbal1::move_relative_pitch(double angle_deg, double max_speed_deg_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) throw std::runtime_error("gimbal1 is disabled; send 'enable' first");
  validate_relative_command(angle_deg, max_speed_deg_s);
  write_command_unlocked(relative_pitch_command(angle_deg, max_speed_deg_s));
}

bool Gimbal1::poll_feedback(Gimbal1Feedback & feedback, std::string * raw_line)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!serial_.isOpen()) throw std::runtime_error("gimbal1 serial port is not open");

  const auto available = serial_.available();
  if (available > 0) serial_.read(rx_buffer_, available);

  while (true) {
    const auto newline = rx_buffer_.find('\n');
    if (newline == std::string::npos) {
      if (rx_buffer_.size() > 4096) {
        rx_buffer_.clear();
        throw std::runtime_error("gimbal1 feedback exceeded 4096 bytes without a newline");
      }
      return false;
    }

    auto line = rx_buffer_.substr(0, newline);
    rx_buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    line = trim(line);
    if (line.empty()) continue;

    try {
      feedback = parse_feedback_line(line);
    } catch (const std::invalid_argument & e) {
      throw std::invalid_argument(fmt::format("{}; raw='{}'", e.what(), line));
    }
    if (raw_line) *raw_line = line;
    return true;
  }
}

bool Gimbal1::enabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

const std::string & Gimbal1::port() const { return port_; }

uint32_t Gimbal1::baudrate() const { return baudrate_; }

std::string Gimbal1::enable_command() { return "ge\r\n"; }

std::string Gimbal1::disable_command() { return "gd\r\n"; }

std::string Gimbal1::relative_yaw_command(double angle_deg, double max_speed_deg_s)
{
  return fmt::format("yr {:.3f} {:.3f}\r\n", angle_deg, max_speed_deg_s);
}

std::string Gimbal1::relative_pitch_command(double angle_deg, double max_speed_deg_s)
{
  return fmt::format("pr {:.3f} {:.3f}\r\n", angle_deg, max_speed_deg_s);
}

Gimbal1Feedback Gimbal1::parse_feedback_line(const std::string & line)
{
  Gimbal1Feedback feedback;
  std::istringstream input(line);
  std::string token;

  for (size_t i = 0; i < feedback.values.size(); ++i) {
    if (!std::getline(input, token, ',')) {
      throw std::invalid_argument(
        fmt::format("gimbal1 CSV has fewer than 26 fields (stopped at field {})", i + 1));
    }

    token = trim(token);
    if (token.empty()) {
      throw std::invalid_argument(fmt::format("gimbal1 CSV field {} is empty", i + 1));
    }

    size_t parsed = 0;
    double value = 0.0;
    try {
      value = std::stod(token, &parsed);
    } catch (const std::exception &) {
      throw std::invalid_argument(
        fmt::format("gimbal1 CSV field {} is not numeric: '{}'", i + 1, token));
    }
    if (parsed != token.size() || !std::isfinite(value)) {
      throw std::invalid_argument(
        fmt::format("gimbal1 CSV field {} is invalid: '{}'", i + 1, token));
    }
    feedback.values[i] = value;
  }

  if (std::getline(input, token, ',')) {
    throw std::invalid_argument("gimbal1 CSV has more than 26 fields");
  }

  feedback.yaw_target_deg = feedback.values[0];
  feedback.yaw_actual_deg = feedback.values[1];
  feedback.yaw_max_speed_deg_s = feedback.values[2];
  feedback.yaw_speed_deg_s = feedback.values[3];
  feedback.yaw_active = integer_field(feedback.values[4], 4);
  feedback.yaw_fault = integer_field(feedback.values[5], 5);
  feedback.yaw_motor_state = integer_field(feedback.values[6], 6);

  feedback.pitch_target_deg = feedback.values[7];
  feedback.pitch_actual_deg = feedback.values[8];
  feedback.pitch_max_speed_deg_s = feedback.values[9];
  feedback.pitch_speed_deg_s = feedback.values[10];
  feedback.pitch_active = integer_field(feedback.values[11], 11);
  feedback.pitch_fault = integer_field(feedback.values[12], 12);
  feedback.pitch_motor_state = integer_field(feedback.values[13], 13);
  feedback.received_at = std::chrono::steady_clock::now();
  return feedback;
}

void Gimbal1::validate_relative_command(double angle_deg, double max_speed_deg_s) const
{
  if (!std::isfinite(angle_deg) || !std::isfinite(max_speed_deg_s)) {
    throw std::invalid_argument("gimbal1 angle and speed must be finite numbers");
  }
  if (std::abs(angle_deg) > max_relative_angle_deg_) {
    throw std::out_of_range(fmt::format(
      "gimbal1 relative angle {:.3f} exceeds configured limit +/-{:.3f} deg", angle_deg,
      max_relative_angle_deg_));
  }
  if (max_speed_deg_s <= 0.0 || max_speed_deg_s > max_speed_deg_s_) {
    throw std::out_of_range(fmt::format(
      "gimbal1 speed {:.3f} must be in (0, {:.3f}] deg/s", max_speed_deg_s,
      max_speed_deg_s_));
  }
}

void Gimbal1::write_command_unlocked(const std::string & command)
{
  if (!serial_.isOpen()) throw std::runtime_error("gimbal1 serial port is not open");
  const auto written = serial_.write(command);
  if (written != command.size()) {
    throw std::runtime_error(
      fmt::format("gimbal1 incomplete serial write: {}/{} bytes", written, command.size()));
  }
}

}  // namespace io

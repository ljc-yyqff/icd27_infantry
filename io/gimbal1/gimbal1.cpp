#include "gimbal1.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <unordered_map>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

namespace io
{
namespace
{
using Clock = std::chrono::steady_clock;
template <typename T> T read_or(const YAML::Node & n, const char * key, T fallback)
{ return n[key] ? n[key].as<T>() : fallback; }
std::string trim(std::string s)
{
  const auto keep = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), keep));
  s.erase(std::find_if(s.rbegin(), s.rend(), keep).base(), s.end());
  return s;
}
double age(Clock::time_point now, Clock::time_point then)
{ return std::chrono::duration<double>(now - then).count(); }
Clock::time_point after(Clock::time_point now, double seconds)
{ return now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)); }
std::string escaped_line(const std::string & line)
{
  std::string result;
  for (unsigned char c : line.substr(0, 512)) {
    if (c == '\r') result += "\\r";
    else if (c == '\n') result += "\\n";
    else if (std::isprint(c)) result += static_cast<char>(c);
    else result += fmt::format("\\x{:02x}", c);
  }
  if (line.size() > 512) result += "...";
  return result;
}
using Fields = std::unordered_map<std::string, std::string>;
bool parse_fields(const std::string & line, const std::string & prefix, Fields & fields)
{
  std::istringstream input(trim(line));
  std::string token;
  if (!std::getline(input, token, ',') || token != prefix) return false;
  while (std::getline(input, token, ',')) {
    token = trim(token);
    const auto split = token.find('=');
    if (split == std::string::npos) continue;
    const auto key = trim(token.substr(0, split));
    const auto value = trim(token.substr(split + 1));
    if (key.empty() || value.empty() || !fields.emplace(key, value).second) return false;
  }
  return true;
}
bool integer_value(const Fields & fields, const char * key, int & value)
{
  const auto it = fields.find(key);
  if (it == fields.end()) return false;
  size_t parsed = 0;
  try { value = std::stoi(it->second, &parsed); } catch (...) { return false; }
  return parsed == it->second.size();
}
bool double_value(const Fields & fields, const char * key, double & value)
{
  const auto it = fields.find(key);
  if (it == fields.end()) return false;
  size_t parsed = 0;
  try { value = std::stod(it->second, &parsed); } catch (...) { return false; }
  return parsed == it->second.size() && !std::isinf(value);
}
bool string_value(const Fields & fields, const char * key, std::string & value)
{
  const auto it = fields.find(key);
  if (it == fields.end()) return false;
  value = it->second;
  return true;
}
std::string relative_command(const char * axis, double angle, double speed)
{
  if (!std::isfinite(angle) || !std::isfinite(speed) || speed < 1 || speed > 120)
    throw std::invalid_argument("relative angle must be finite; speed must be 1..120 deg/s");
  auto command = fmt::format("{} {:.3f} {:.3f}\r\n", axis, angle, speed);
  if (command.size() - 2 > 31) throw std::length_error("gimbal1 command exceeds 31 characters");
  return command;
}
}

Gimbal1::Gimbal1(const std::string & path)
{
  const auto yaml = YAML::LoadFile(path);
  if (!yaml["com_port"]) throw std::runtime_error("missing com_port");
  port_ = yaml["com_port"].as<std::string>();
  baudrate_ = read_or<uint32_t>(yaml, "baudrate", 115200);
  timeout_ms_ = read_or<uint32_t>(yaml, "serial_timeout_ms", 20);
  max_relative_angle_deg_ = read_or<double>(yaml, "gimbal1_max_relative_angle_deg", 5);
  max_speed_deg_s_ = read_or<double>(yaml, "gimbal1_max_speed_deg_s", 30);
  if (port_.empty() || baudrate_ == 0 || !std::isfinite(max_relative_angle_deg_) ||
      max_relative_angle_deg_ <= 0 || !std::isfinite(max_speed_deg_s_) ||
      max_speed_deg_s_ < 1 || max_speed_deg_s_ > 120)
    throw std::invalid_argument("invalid serial port or command parameter limits");
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
void Gimbal1::start_logging(const std::string & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trace_.open(path, std::ios::out | std::ios::app);
  if (!trace_) throw std::runtime_error("cannot create serial trace: " + path);
  trace_started_ = Clock::now();
  trace_unlocked("OPEN", fmt::format("port={} baud={} motor_state=diagnostic_only", port_, baudrate_));
}
void Gimbal1::trace_unlocked(const std::string & kind, const std::string & message) noexcept
{
  if (!trace_.is_open()) return;
  try {
    const auto wall = std::chrono::system_clock::now();
    const auto stamp = std::chrono::system_clock::to_time_t(wall);
    std::tm utc{};
    gmtime_r(&stamp, &utc);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(wall.time_since_epoch()).count() % 1000;
    trace_ << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill('0')
           << std::setw(3) << millis << "Z +" << std::fixed << std::setprecision(3)
           << age(Clock::now(), trace_started_) << "s " << kind << ' ' << message << '\n';
    trace_.flush();
    if (!trace_) {
      trace_.close();
      std::cerr << "[log] serial trace write failed; trace stopped\n";
    }
  } catch (...) { /* Diagnostics must never prevent gd or change control behavior. */ }
}
Gimbal1::~Gimbal1()
{
  std::lock_guard<std::mutex> lock(mutex_);
  trace_unlocked("CLOSE", "serial close only; no automatic motor command");
  // Closing the serial link does not command motor enable/disable.
  try { if (serial_.isOpen()) serial_.close(); } catch (...) {}
}
void Gimbal1::enable(const std::atomic<bool> * cancel)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cancel && cancel->load()) throw std::runtime_error("enable cancelled by operator");
  write_command_unlocked(enable_command());
  enabled_ = true; // Sent request only; motor state belongs to the A-board.
}
void Gimbal1::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  trace_unlocked("DISABLE", "explicit disable() call (operator d/q)");
  write_command_unlocked(disable_command());
  enabled_ = false;
}
void Gimbal1::move_relative_yaw(double a, double s)
{ std::lock_guard<std::mutex> lock(mutex_); move_unlocked(true, a, s); }
void Gimbal1::move_relative_pitch(double a, double s)
{ std::lock_guard<std::mutex> lock(mutex_); move_unlocked(false, a, s); }
void Gimbal1::move_unlocked(bool yaw, double angle, double speed)
{
  validate_relative_command(angle, speed);
  angle = std::stod(fmt::format("{:.3f}", angle));
  if (angle == 0) return;
  // No feedback/state/arrival gate, target accumulation or automatic retry.
  write_command_unlocked(yaw ? relative_yaw_command(angle, speed) : relative_pitch_command(angle, speed));
}

bool Gimbal1::service_unlocked()
{
  if (!serial_.isOpen()) throw std::runtime_error("gimbal1 serial port is not open");
  const auto bytes = serial_.available();
  if (bytes > 0) serial_.read(rx_buffer_, bytes);
  bool updated = false;
  // Drain complete lines and retain the newest telemetry for diagnostics.
  while (true) {
    const auto end = rx_buffer_.find_first_of("\r\n");
    if (end == std::string::npos) break;
    const auto line = trim(rx_buffer_.substr(0, end));
    rx_buffer_.erase(0, end + 1);
    if (line.empty()) continue;
    trace_unlocked("RX", escaped_line(line));
    try {
      if (line.front() == '#') {
        diagnostic_ = line;
        Gimbal1CommandReceipt command_receipt;
        Gimbal1GlobalStatus global_status;
        Gimbal1Status status;
        if (parse_command_receipt(line, command_receipt)) {
          receipt_ = command_receipt;
          std::cerr << "[board RX] cmd=" << receipt_.command << " ok=" << receipt_.ok
                    << " reason=" << receipt_.reason << " state=" << receipt_.state
                    << " mode=" << receipt_.mode << " target_deg=" << receipt_.target_deg << '\n';
        } else if (parse_global_status(line, global_status)) {
          global_status_ = global_status;
          std::cerr << "[board RX] yaw_state=" << global_status_.yaw_state
                    << " yaw_fault=" << global_status_.yaw_fault
                    << " pitch_state=" << global_status_.pitch_state
                    << " pitch_fault=" << global_status_.pitch_fault
                    << " imu_valid=" << global_status_.imu_valid
                    << " reason=" << global_status_.reason << '\n';
        } else if (parse_status_line(line, status)) {
          if (status.yaw) yaw_response_ = line;
          else pitch_response_ = line;
        } else if (line.rfind("#YP,", 0) == 0 || line.rfind("#PP,", 0) == 0) {
          rejected_line_ = escaped_line(line);
        }
      } else {
        feedback_ = parse_feedback_line(line);
        if (feedback_samples_.size() >= 128) feedback_samples_.erase(feedback_samples_.begin());
        feedback_samples_.push_back(feedback_);
        have_feedback_ = updated = true;
        raw_feedback_ = line;
      }
    } catch (const std::invalid_argument & e) {
      diagnostic_ = e.what();
      rejected_line_ = escaped_line(line);
      trace_unlocked("RX_REJECTED", diagnostic_ + "; raw=" + rejected_line_);
    }
  }
  if (rx_buffer_.size() > 4096) { rx_buffer_.clear(); diagnostic_ = "discarded oversized partial line"; }
  return updated;
}
bool Gimbal1::poll_feedback(Gimbal1Feedback & feedback, std::string * raw_line)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool updated = service_unlocked();
  if (updated) { feedback = feedback_; if (raw_line) *raw_line = raw_feedback_; }
  return updated;
}
std::vector<Gimbal1Feedback> Gimbal1::take_feedback_samples()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Gimbal1Feedback> samples;
  samples.swap(feedback_samples_);
  return samples;
}
bool Gimbal1::ready_unlocked() const
{ return serial_.isOpen(); } // Serial availability only; never asserts motor readiness.
bool Gimbal1::ready() const { std::lock_guard<std::mutex> lock(mutex_); return ready_unlocked(); }
bool Gimbal1::enabled() const { std::lock_guard<std::mutex> lock(mutex_); return enabled_; }
std::string Gimbal1::status_text() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_text_unlocked();
}
std::string Gimbal1::status_text_unlocked() const
{
  return fmt::format("serial_open={} ge_requested={} motor_state=diagnostic_only yaw_state={} pitch_state={} yaw_cmd={} yaw_enc={} pitch_cmd={} pitch_enc={} last_gc={}:{}({}) last_rejected=[{}] {}",
    serial_.isOpen(), enabled_, global_status_.yaw_state, global_status_.pitch_state,
    feedback_.yaw_setpoint_deg, feedback_.yaw_actual_deg, feedback_.pitch_setpoint_deg, feedback_.pitch_actual_deg,
    receipt_.command, receipt_.ok, receipt_.reason, rejected_line_, diagnostic_);
}
const std::string & Gimbal1::port() const { return port_; }
uint32_t Gimbal1::baudrate() const { return baudrate_; }
std::string Gimbal1::enable_command() { return "ge\r\n"; }
std::string Gimbal1::disable_command() { return "gd\r\n"; }
std::string Gimbal1::relative_yaw_command(double a, double s) { return relative_command("yr", a, s); }
std::string Gimbal1::relative_pitch_command(double a, double s) { return relative_command("pr", a, s); }
Gimbal1Feedback Gimbal1::parse_feedback_line(const std::string & line)
{
  Gimbal1Feedback f;
  const auto clean = trim(line);
  if (std::count(clean.begin(), clean.end(), ',') != 8)
    throw std::invalid_argument("gimbal1 telemetry requires exactly 9 CSV fields");
  std::istringstream input(clean);
  std::string token;
  for (auto & value : f.values) {
    std::getline(input, token, ',');
    token = trim(token);
    size_t parsed = 0;
    try { value = std::stod(token, &parsed); }
    catch (...) { throw std::invalid_argument("non-numeric gimbal1 CSV field"); }
    if (parsed != token.size() || std::isinf(value))
      throw std::invalid_argument("invalid gimbal1 CSV field");
  }
  f.pitch_setpoint_deg = f.values[0]; f.pitch_actual_deg = f.values[1];
  f.yaw_setpoint_deg = f.values[2]; f.yaw_actual_deg = f.values[3];
  f.imu_pitch_deg = f.values[4]; f.imu_yaw_deg = f.values[5]; f.tx_dt_ms = f.values[6];
  f.pitch_torque_nm = f.values[7]; f.yaw_torque_nm = f.values[8];
  f.received_at = Clock::now();
  return f;
}
bool Gimbal1::parse_status_line(const std::string & line, Gimbal1Status & status)
{
  std::istringstream input(trim(line));
  std::string token;
  std::getline(input, token, ',');
  if (token != "#YP" && token != "#PP") return false;
  const bool yaw = token == "#YP";
  int state = -1, id = -1, mode = -1;
  bool have_state = false, have_id = false, have_mode = false;
  while (std::getline(input, token, ',')) {
    token = trim(token);
    const auto split = token.find('=');
    if (split == std::string::npos) continue;
    const auto key = trim(token.substr(0, split));
    if (key != "state" && key != "id" && key != "mode") continue;
    const auto value = trim(token.substr(split + 1));
    size_t parsed = 0;
    int number;
    try { number = std::stoi(value, &parsed); } catch (...) { return false; }
    if (parsed != value.size()) return false;
    if (key == "state") { if (have_state) return false; state = number; have_state = true; }
    else if (key == "id") { if (have_id) return false; id = number; have_id = true; }
    else { if (have_mode) return false; mode = number; have_mode = true; }
  }
  if (!have_state || state < 0 || state > 5 || (yaw && (!have_id || id != 1))) return false;
  if (have_mode && mode != 0 && mode != 1) return false;
  status.yaw = yaw; status.state = state; status.mode = mode;
  return true;
}
bool Gimbal1::parse_command_receipt(
  const std::string & line, Gimbal1CommandReceipt & receipt)
{
  Fields fields;
  if (!parse_fields(line, "#GC", fields)) return false;
  int ok = -1;
  if (!string_value(fields, "cmd", receipt.command) ||
      !integer_value(fields, "ok", ok) || (ok != 0 && ok != 1) ||
      !string_value(fields, "reason", receipt.reason) ||
      !integer_value(fields, "state", receipt.state) || receipt.state < 0 ||
      receipt.state > 5 || !integer_value(fields, "mode", receipt.mode) ||
      receipt.mode < 0 || receipt.mode > 1 ||
      !double_value(fields, "target_deg", receipt.target_deg)) {
    return false;
  }
  receipt.ok = ok == 1;
  receipt.received_at = Clock::now();
  return true;
}
bool Gimbal1::parse_global_status(
  const std::string & line, Gimbal1GlobalStatus & status)
{
  Fields fields;
  if (!parse_fields(line, "#GM", fields)) return false;
  int ok = -1;
  if (!integer_value(fields, "ok", ok) || (ok != 0 && ok != 1) ||
      !string_value(fields, "reason", status.reason) ||
      !integer_value(fields, "ys", status.yaw_state) || status.yaw_state < 0 ||
      status.yaw_state > 5 || !integer_value(fields, "yf", status.yaw_fault) ||
      status.yaw_fault < 0 || !integer_value(fields, "ym", status.yaw_mode) ||
      status.yaw_mode < 0 || status.yaw_mode > 1 ||
      !integer_value(fields, "ps", status.pitch_state) || status.pitch_state < 0 ||
      status.pitch_state > 5 || !integer_value(fields, "pf", status.pitch_fault) ||
      status.pitch_fault < 0 || !integer_value(fields, "pm", status.pitch_mode) ||
      status.pitch_mode < 0 || status.pitch_mode > 1 ||
      !integer_value(fields, "valid", status.imu_valid) || status.imu_valid < 0 ||
      status.imu_valid > 1) {
    return false;
  }
  status.ok = ok == 1;
  status.received_at = Clock::now();
  return true;
}
void Gimbal1::validate_relative_command(double angle, double speed) const
{
  if (!std::isfinite(angle) || !std::isfinite(speed)) throw std::invalid_argument("non-finite motion");
  if (std::abs(angle) > max_relative_angle_deg_) throw std::out_of_range("relative angle exceeds limit");
  if (speed < 1 || speed > std::min(120.0, max_speed_deg_s_)) throw std::out_of_range("speed outside limits");
}
void Gimbal1::write_command_unlocked(const std::string & command)
{
  if (!serial_.isOpen()) throw std::runtime_error("gimbal1 serial port is not open");
  if (command.size() < 2 || command.size() - 2 > 31) throw std::length_error("command exceeds 31 characters");
  // Includes status queries, so all outbound lines together stay at or below 50 Hz.
  std::this_thread::sleep_until(after(last_write_, 0.02));
  trace_unlocked("TX_REQUEST", escaped_line(command));
  size_t written = 0;
  try { written = serial_.write(command); }
  catch (const std::exception & e) {
    trace_unlocked("TX_ERROR", escaped_line(command) + "; " + e.what());
    throw;
  }
  last_write_ = Clock::now();
  trace_unlocked("TX_RESULT", fmt::format("bytes={}/{} command={}", written, command.size(), escaped_line(command)));
  if (written != command.size()) throw std::runtime_error("incomplete serial write; do not retry relative command");
}
}  // namespace io

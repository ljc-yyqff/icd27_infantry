#include "io/gimbal1/gimbal1.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int) { stop_requested = 1; }

struct SweepConfig
{
  double speed_deg_s = 10.0;
  double step_deg = 5.0;
  double yaw_half_range_deg = 90.0;
  double pitch_min_deg = 5.0;
  double pitch_center_deg = 25.0;
  double pitch_max_deg = 40.0;
  double enable_settle_s = 0.5;
  double step_settle_s = 0.1;
  double final_hold_s = 10.0;
  double serial_wait_s = 30.0;
};

enum class Axis
{
  YAW,
  PITCH
};

struct Segment
{
  Axis axis;
  double relative_angle_deg;
  std::string description;
};

template <typename T>
T read_or(const YAML::Node & yaml, const char * key, const T & fallback)
{
  return yaml[key] ? yaml[key].as<T>() : fallback;
}

SweepConfig load_sweep_config(const std::string & config_path)
{
  const auto root = YAML::LoadFile(config_path);
  const auto yaml = root["gimbal1_sweep"];
  if (!yaml) throw std::runtime_error("gimbal1 config is missing 'gimbal1_sweep'");

  SweepConfig config;
  config.speed_deg_s = read_or<double>(yaml, "speed_deg_s", config.speed_deg_s);
  config.step_deg = read_or<double>(yaml, "step_deg", config.step_deg);
  config.yaw_half_range_deg =
    read_or<double>(yaml, "yaw_half_range_deg", config.yaw_half_range_deg);
  config.pitch_min_deg = read_or<double>(yaml, "pitch_min_deg", config.pitch_min_deg);
  config.pitch_center_deg = read_or<double>(yaml, "pitch_center_deg", config.pitch_center_deg);
  config.pitch_max_deg = read_or<double>(yaml, "pitch_max_deg", config.pitch_max_deg);
  config.enable_settle_s = read_or<double>(yaml, "enable_settle_s", config.enable_settle_s);
  config.step_settle_s = read_or<double>(yaml, "step_settle_s", config.step_settle_s);
  config.final_hold_s = read_or<double>(yaml, "final_hold_s", config.final_hold_s);
  config.serial_wait_s = read_or<double>(yaml, "serial_wait_s", config.serial_wait_s);

  const std::vector<double> finite_values = {
    config.speed_deg_s,       config.step_deg,         config.yaw_half_range_deg,
    config.pitch_min_deg,     config.pitch_center_deg, config.pitch_max_deg,
    config.enable_settle_s,   config.step_settle_s,    config.final_hold_s,
    config.serial_wait_s};
  if (!std::all_of(finite_values.begin(), finite_values.end(), [](double value) {
        return std::isfinite(value);
      })) {
    throw std::runtime_error("all gimbal1_sweep values must be finite");
  }
  if (config.speed_deg_s < 1.0 || config.speed_deg_s > 120.0 || config.step_deg <= 0.0 || config.step_deg > 5.0 ||
      config.yaw_half_range_deg <= 0.0) {
    throw std::runtime_error(
      "speed and yaw range must be positive; step_deg must be in (0, 5]");
  }
  if (config.pitch_min_deg < -100 || config.pitch_max_deg > 41 ||
      !(config.pitch_min_deg < config.pitch_center_deg &&
        config.pitch_center_deg < config.pitch_max_deg)) {
    throw std::runtime_error("pitch must satisfy min < center < max");
  }
  if (
    config.enable_settle_s < 0.0 || config.step_settle_s < 0.0 ||
    config.final_hold_s < 0.0 || config.serial_wait_s < 0.0) {
    throw std::runtime_error("gimbal1_sweep wait times must not be negative");
  }
  return config;
}

std::vector<Segment> make_segments(const SweepConfig & config)
{
  return {
    {Axis::YAW, config.yaw_half_range_deg, "yaw center -> left (+)"},
    {Axis::YAW, -2.0 * config.yaw_half_range_deg, "yaw left -> right"},
    {Axis::YAW, config.yaw_half_range_deg, "yaw right -> center"},
    {Axis::PITCH, config.pitch_max_deg - config.pitch_center_deg, "pitch center -> upper bound"},
    {Axis::PITCH, config.pitch_min_deg - config.pitch_max_deg, "pitch upper -> lower bound"},
    {Axis::PITCH, config.pitch_center_deg - config.pitch_min_deg, "pitch lower -> center"}};
}

bool wait_interruptible(double seconds, bool dry_run)
{
  if (dry_run) return !stop_requested;

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(seconds));
  while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
    const auto remaining = deadline - std::chrono::steady_clock::now();
    const auto poll_interval =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::milliseconds(100));
    std::this_thread::sleep_for(std::min(remaining, poll_interval));
  }
  return !stop_requested;
}

bool wait_with_feedback(double seconds, io::Gimbal1 * gimbal, bool dry_run)
{
  if (dry_run) return !stop_requested;
  const auto start = std::chrono::steady_clock::now();
  while (!stop_requested && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
    io::Gimbal1Feedback feedback;
    gimbal->poll_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return !stop_requested;
}

std::unique_ptr<io::Gimbal1> open_gimbal(
  const std::string & config_path, double serial_wait_s)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(serial_wait_s));
  std::string last_error = "serial port unavailable";

  do {
    try {
      return std::make_unique<io::Gimbal1>(config_path);
    } catch (const std::exception & e) {
      last_error = e.what();
      std::cerr << "[gimbal1-autorun] waiting for serial port: " << last_error << '\n';
    }
    if (!wait_interruptible(1.0, false)) break;
  } while (std::chrono::steady_clock::now() < deadline);

  throw std::runtime_error("unable to open gimbal serial port: " + last_error);
}

bool run_relative_segment(
  io::Gimbal1 * gimbal, const Segment & segment, const SweepConfig & config, bool dry_run)
{
  std::cout << "[gimbal1-autorun] " << segment.description << ": "
            << fmt::format("{:+.3f} deg", segment.relative_angle_deg) << '\n';

  double remaining = segment.relative_angle_deg;
  while (!stop_requested && std::abs(remaining) > 1e-9) {
    const double magnitude = std::min(std::abs(remaining), config.step_deg);
    const double step = std::copysign(magnitude, remaining);
    const auto command = segment.axis == Axis::YAW
                           ? io::Gimbal1::relative_yaw_command(step, config.speed_deg_s)
                           : io::Gimbal1::relative_pitch_command(step, config.speed_deg_s);

    std::cout << "[gimbal1-autorun] TX "
              << command.substr(0, command.size() - 2) << "\\r\\n\n";
    if (!dry_run) {
      if (segment.axis == Axis::YAW) {
        gimbal->move_relative_yaw(step, config.speed_deg_s);
      } else {
        gimbal->move_relative_pitch(step, config.speed_deg_s);
      }
    }

    remaining -= step;
    if (!wait_with_feedback(magnitude / config.speed_deg_s + config.step_settle_s, gimbal, dry_run)) {
      return false;
    }
  }
  return !stop_requested;
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/gimbal1.yaml";
  const bool dry_run = argc > 2 && std::string(argv[2]) == "--dry-run";

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    const auto config = load_sweep_config(config_path);
    const auto segments = make_segments(config);

    std::cout << "[gimbal1-autorun] mode: " << (dry_run ? "dry-run" : "live") << '\n';
    std::cout << "[gimbal1-autorun] speed: " << config.speed_deg_s
              << " deg/s, step: " << config.step_deg << " deg\n";
    std::cout << "[gimbal1-autorun] pitch relative sweep assumes start at "
              << config.pitch_center_deg << " deg\n";

    std::unique_ptr<io::Gimbal1> gimbal;
    if (!dry_run) gimbal = open_gimbal(config_path, config.serial_wait_s);

    std::cout << "[gimbal1-autorun] TX ge\\r\\n\n";
    if (!dry_run) gimbal->enable();
    if (!wait_with_feedback(config.enable_settle_s, gimbal.get(), dry_run)) return 130;

    for (const auto & segment : segments) {
      if (!run_relative_segment(gimbal.get(), segment, config, dry_run)) return 130;
    }

    std::cout << "[gimbal1-autorun] motion complete; holding for " << config.final_hold_s
              << " seconds\n";
    if (!wait_with_feedback(config.final_hold_s, gimbal.get(), dry_run)) return 130;

    std::cout << "[gimbal1-autorun] TX gd\\r\\n\n";
    if (!dry_run) gimbal->disable();
    std::cout << "[gimbal1-autorun] task complete\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "[gimbal1-autorun] failed: " << e.what() << '\n';
    return 1;
  }
}

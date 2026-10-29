#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal1/gimbal1.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

cv::Point2f armor_center(const auto_aim::Armor & armor)
{
  if (armor.points.empty()) return armor.center;

  cv::Point2f center{0.0F, 0.0F};
  for (const auto & point : armor.points) center += point;
  return center * (1.0F / static_cast<float>(armor.points.size()));
}

const auto_aim::Armor * select_target(
  const std::list<auto_aim::Armor> & armors, const cv::Point2f & optical_center)
{
  const auto_aim::Armor * best = nullptr;
  double best_score = -1.0;

  for (const auto & armor : armors) {
    const auto center = armor_center(armor);
    const auto distance = cv::norm(center - optical_center);
    // Prefer confidence first, then the target nearest the optical axis.
    const auto score = armor.confidence * 100000.0 - distance;
    if (score > best_score) {
      best_score = score;
      best = &armor;
    }
  }
  return best;
}

void draw_crosshair(cv::Mat & image, const cv::Point2f & center, const cv::Scalar & color)
{
  constexpr int half_length = 28;
  cv::line(
    image, {static_cast<int>(center.x) - half_length, static_cast<int>(center.y)},
    {static_cast<int>(center.x) + half_length, static_cast<int>(center.y)}, color, 3);
  cv::line(
    image, {static_cast<int>(center.x), static_cast<int>(center.y) - half_length},
    {static_cast<int>(center.x), static_cast<int>(center.y) + half_length}, color, 3);
}
}  // namespace

const std::string keys =
  "{help h usage ? |                       | show help }"
  "{@config-path   | configs/infantry_daheng.yaml | yaml config path }"
  "{tradition t    | false                 | use the traditional detector instead of YOLO }"
  "{deadband       | 25                    | control deadband in pixels }"
  "{control        | false                 | open gimbal1 serial; starts disabled }"
  "{gain           | 0.15                  | proportional gain from camera error to command }"
  "{min_step       | 0.0                   | minimum non-zero relative angle, degrees }"
  "{max_step       | 0.30                  | maximum relative angle per command, degrees }"
  "{speed          | 5.0                   | command speed, degrees/second }"
  "{control_rate   | 5.0                   | maximum command rate, Hz }"
  "{max_active_seconds | 10.0              | pause after this many seconds; 0 disables timeout }";

int main(int argc, char * argv[])
try {
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto use_tradition = cli.get<bool>("tradition");
  const auto deadband_px = cli.get<double>("deadband");
  const auto control_mode = cli.get<bool>("control");
  const auto gain = cli.get<double>("gain");
  const auto min_step_deg = cli.get<double>("min_step");
  const auto max_step_deg = cli.get<double>("max_step");
  const auto command_speed_deg_s = cli.get<double>("speed");
  const auto control_rate_hz = cli.get<double>("control_rate");
  const auto max_active_seconds = cli.get<double>("max_active_seconds");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  if (!std::isfinite(deadband_px) || deadband_px < 0.0) {
    throw std::invalid_argument("deadband must be a finite non-negative pixel value");
  }
  if (!std::isfinite(gain) || gain <= 0.0 || !std::isfinite(min_step_deg) ||
      min_step_deg < 0.0 || !std::isfinite(max_step_deg) || max_step_deg <= 0.0 ||
      min_step_deg > max_step_deg || !std::isfinite(command_speed_deg_s) ||
      command_speed_deg_s <= 0.0 || !std::isfinite(control_rate_hz) || control_rate_hz <= 0.0 ||
      !std::isfinite(max_active_seconds) || max_active_seconds < 0.0) {
    throw std::invalid_argument(
      "gain, max_step, speed and control_rate must be positive; max_active_seconds must be >= 0; "
      "min_step must satisfy 0 <= min_step <= max_step");
  }

  const auto yaml = YAML::LoadFile(config_path);
  if (!yaml["camera_matrix"]) throw std::runtime_error("config is missing camera_matrix");
  const auto matrix = yaml["camera_matrix"].as<std::vector<double>>();
  if (matrix.size() != 9) throw std::runtime_error("camera_matrix must contain 9 values");

  const double fx = matrix[0];
  const double fy = matrix[4];
  const double cx = matrix[2];
  const double cy = matrix[5];
  if (fx <= 0.0 || fy <= 0.0) throw std::runtime_error("camera focal lengths must be positive");

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, false);
  auto_aim::YOLO yolo(config_path, false);
  std::unique_ptr<io::Gimbal1> gimbal;
  if (control_mode) gimbal = std::make_unique<io::Gimbal1>(config_path);
  tools::Exiter exiter;

  std::cout << (control_mode ? "gimbal1 armor tracking CONTROL mode\n"
                             : "gimbal1 armor tracking preview (DRY-RUN)\n")
            << (control_mode
                  ? "Serial is open, but the gimbal starts DISABLED. Press e to enable tracking.\n"
                  : "No serial port is opened and no gimbal command is transmitted.\n")
            << "Detector: " << (use_tradition ? "traditional" : "YOLO") << '\n'
            << fmt::format(
                 "Optical center=({:.2f}, {:.2f}) fx={:.2f} fy={:.2f}; deadband={:.1f}px\n",
                 cx, cy, fx, fy, deadband_px)
            << "yaw_error: positive means target is to the image right.\n"
            << "pitch_error: positive means target is below the optical center.\n"
            << (control_mode
                  ? "Keys: e=enable/resume, s=pause and hold, d=disable, q=disable and quit.\n"
                  : "Press q to quit.\n");

  int frame_count = 0;
  auto last_frame = std::chrono::steady_clock::now();
  auto last_log = last_frame;
  auto last_command = last_frame;
  auto active_since = last_frame;
  bool tracking_active = false;
  bool send_yaw_next = true;
  int consecutive_target_frames = 0;
  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat image;
    camera.read(image, timestamp);
    if (image.empty()) break;

    std::list<auto_aim::Armor> armors;
    if (use_tradition)
      armors = detector.detect(image, frame_count);
    else
      armors = yolo.detect(image, frame_count);

    const cv::Point2f optical_center{static_cast<float>(cx), static_cast<float>(cy)};
    const auto * target = select_target(armors, optical_center);

    draw_crosshair(image, optical_center, {255, 255, 0});
    tools::draw_text(image, "optical center", optical_center + cv::Point2f(35, -20), {255, 255, 0});

    std::string status = "NO TARGET - no command";
    cv::Scalar status_color{0, 0, 255};
    double error_x = 0.0;
    double error_y = 0.0;
    double yaw_error_deg = 0.0;
    double pitch_error_deg = 0.0;

    if (target != nullptr) {
      ++consecutive_target_frames;
      const auto target_center = armor_center(*target);
      error_x = target_center.x - cx;
      error_y = target_center.y - cy;
      yaw_error_deg = std::atan(error_x / fx) * kRadToDeg;
      pitch_error_deg = std::atan(error_y / fy) * kRadToDeg;

      const bool in_deadband =
        std::abs(error_x) <= deadband_px && std::abs(error_y) <= deadband_px;
      status = in_deadband
                 ? "CENTERED (deadband)"
                 : (control_mode && tracking_active ? "TARGET - controlling"
                                                    : "TARGET - preview only");
      status_color = in_deadband ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 255, 255};

      tools::draw_points(image, target->points, {0, 255, 0}, 4);
      cv::circle(image, target_center, 10, {0, 0, 255}, -1);
      cv::arrowedLine(image, optical_center, target_center, {255, 0, 255}, 4);
      tools::draw_text(
        image,
        fmt::format(
          "{:.2f} {} {} {}", target->confidence, auto_aim::COLORS[target->color],
          auto_aim::ARMOR_NAMES[target->name], auto_aim::ARMOR_TYPES[target->type]),
        target_center + cv::Point2f(15, -20), {0, 255, 0}, 0.8, 2);
    } else {
      consecutive_target_frames = 0;
    }

    std::string mode_text = "DRY-RUN: serial disabled";
    cv::Scalar mode_color{0, 255, 255};
    if (control_mode) {
      if (!gimbal->enabled()) {
        mode_text = "CONTROL: DISABLED - press e";
        mode_color = {0, 0, 255};
      } else if (!tracking_active) {
        mode_text = "CONTROL: PAUSED/HOLD - press e";
        mode_color = {0, 255, 255};
      } else {
        mode_text = "CONTROL: ACTIVE - s pause, d disable";
        mode_color = {0, 255, 0};
      }
    }
    tools::draw_text(image, mode_text, {30, 65}, mode_color, 0.9, 2);
    tools::draw_text(image, status, {30, 105}, status_color, 0.9, 2);
    tools::draw_text(
      image, fmt::format("pixel error: x={:+.1f} y={:+.1f}", error_x, error_y), {30, 145},
      {255, 255, 255}, 0.8, 2);
    tools::draw_text(
      image,
      fmt::format("camera error: yaw={:+.3f} deg pitch={:+.3f} deg", yaw_error_deg, pitch_error_deg),
      {30, 185}, {255, 255, 255}, 0.8, 2);

    const auto now = std::chrono::steady_clock::now();
    const auto frame_dt = std::chrono::duration<double>(now - last_frame).count();
    last_frame = now;
    if (frame_dt > 1e-6) {
      tools::draw_text(
        image, fmt::format("FPS: {:.1f}", 1.0 / frame_dt), {30, 225}, {255, 255, 255}, 0.8, 2);
    }

    if (std::chrono::duration<double>(now - last_log).count() >= 0.25) {
      if (target == nullptr) {
        std::cout << "[preview] no target\n";
      } else {
        std::cout << fmt::format(
          "[preview] center=({:.1f},{:.1f}) px_error=({:+.1f},{:+.1f}) "
          "camera_error=({:+.3f} yaw,{:+.3f} pitch) deg\n",
          armor_center(*target).x, armor_center(*target).y, error_x, error_y, yaw_error_deg,
          pitch_error_deg);
      }
      last_log = now;
    }

    if (
      control_mode && tracking_active && gimbal->enabled() && target != nullptr &&
      consecutive_target_frames >= 5) {
      if (
        max_active_seconds > 0.0 &&
        std::chrono::duration<double>(now - active_since).count() >= max_active_seconds) {
        tracking_active = false;
        std::cout << "[control] automatic pause after " << max_active_seconds
                  << " s; gimbal remains enabled and holding. Press e to resume.\n";
      } else if (
        std::chrono::duration<double>(now - active_since).count() >= 0.5 &&
        std::chrono::duration<double>(now - last_command).count() >= 1.0 / control_rate_hz) {
        double yaw_command = 0.0;
        double pitch_command = 0.0;
        if (std::abs(error_x) > deadband_px) {
          const auto raw = -gain * yaw_error_deg;
          const auto magnitude = std::clamp(std::abs(raw), min_step_deg, max_step_deg);
          yaw_command = std::copysign(magnitude, raw);
        }
        if (std::abs(error_y) > deadband_px) {
          const auto raw = -gain * pitch_error_deg;
          const auto magnitude = std::clamp(std::abs(raw), min_step_deg, max_step_deg);
          pitch_command = std::copysign(magnitude, raw);
        }

        try {
          const bool has_yaw = std::abs(yaw_command) > 1e-6;
          const bool has_pitch = std::abs(pitch_command) > 1e-6;

          // Send exactly one protocol line per control tick. Some A-board
          // receivers parse data per UART-idle event and reject two CRLF
          // commands that arrive in one USB/serial burst.
          if (has_yaw && (!has_pitch || send_yaw_next)) {
            gimbal->move_relative_yaw(yaw_command, command_speed_deg_s);
            std::cout << fmt::format(
              "[control] sent yr {:+.3f} {:.1f}\n", yaw_command, command_speed_deg_s);
            send_yaw_next = false;
          } else if (has_pitch) {
            gimbal->move_relative_pitch(pitch_command, command_speed_deg_s);
            std::cout << fmt::format(
              "[control] sent pr {:+.3f} {:.1f}\n", pitch_command, command_speed_deg_s);
            send_yaw_next = true;
          }
          last_command = now;
        } catch (const std::exception & e) {
          tracking_active = false;
          std::cerr << "[control] command failed; tracking paused: " << e.what() << '\n';
        }
      }
    }

    cv::Mat display;
    cv::resize(image, display, {}, 0.5, 0.5);
    cv::imshow(
      control_mode ? "gimbal1 armor track - CONTROL" : "gimbal1 armor track - DRY RUN", display);
    const auto key = cv::waitKey(1);
    if (key == 'q') {
      if (control_mode && gimbal->enabled()) gimbal->disable();
      break;
    }
    if (control_mode && key == 'e') {
      if (!gimbal->enabled()) gimbal->enable();
      tracking_active = true;
      send_yaw_next = true;
      active_since = std::chrono::steady_clock::now();
      last_command = active_since;
      if (max_active_seconds > 0.0) {
        std::cout << "[control] ge sent; tracking active for at most " << max_active_seconds
                  << " s\n";
      } else {
        std::cout << "[control] ge sent; continuous tracking active; press s to pause\n";
      }
    } else if (control_mode && key == 's') {
      tracking_active = false;
      std::cout << "[control] tracking paused; gimbal remains enabled and holding\n";
    } else if (control_mode && key == 'd') {
      tracking_active = false;
      gimbal->disable();
      std::cout << "[control] gd sent; gimbal disabled\n";
    }
    ++frame_count;
  }

  return 0;
} catch (const std::exception & e) {
  std::cerr << "gimbal1_armor_track: " << e.what() << '\n';
  return 1;
}

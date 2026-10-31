#include "io/gimbal1/gimbal1.hpp"
#include "gimbal1_console.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
void print_help()
{
  std::cout <<
    "Commands:\n"
    "  enable | e                 send ge\\r\\n\n"
    "  disable | d                send gd\\r\\n\n"
    "  yaw | y <angle> <speed>    send relative yaw command yr\n"
    "  pitch | p <angle> <speed>  send relative pitch command pr\n"
    "  feedback | f               show actual yaw/pitch for one second\n"
    "  status                     show axis state, age and raw status replies\n"
    "  help | h                   show this help\n"
    "  quit | q                   disable the gimbal and exit\n"
    "Feedback: actual angles, planner cmd, torque, tx interval (diagnostics only).\n"
    "Full RX/TX diagnostics are written to the printed log path.\n";
}

void require_no_extra_input(std::istringstream & input)
{
  std::string extra;
  if (input >> extra) throw std::invalid_argument("unexpected argument: " + extra);
}

void read_motion_arguments(std::istringstream & input, double & angle, double & speed)
{
  if (!(input >> angle >> speed)) {
    throw std::invalid_argument("expected: <relative-angle-deg> <max-speed-deg-s>");
  }
  require_no_extra_input(input);
}

std::chrono::milliseconds motion_observation_time(double angle_deg, double speed_deg_s)
{
  const auto estimated_ms =
    static_cast<long long>(std::abs(angle_deg) / speed_deg_s * 1000.0 + 1000.0);
  return std::chrono::milliseconds(std::clamp<long long>(estimated_ms, 1500, 5000));
}

void show_actual_angles(io::Gimbal1 & gimbal, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  double last_yaw = std::numeric_limits<double>::quiet_NaN();
  double last_pitch = std::numeric_limits<double>::quiet_NaN();
  size_t valid_count = 0;
  size_t rejected_count = 0;
  auto last_print = std::chrono::steady_clock::time_point{};

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      io::Gimbal1Feedback feedback;
      if (gimbal.poll_feedback(feedback)) {
        valid_count++;
        const bool changed = !std::isfinite(last_yaw) ||
                             std::abs(feedback.yaw_actual_deg - last_yaw) >= 0.001 ||
                             std::abs(feedback.pitch_actual_deg - last_pitch) >= 0.001;
        if (changed || std::chrono::steady_clock::now() - last_print >= std::chrono::milliseconds(200)) {
          std::cout << std::fixed << std::setprecision(3)
                    << "[feedback] yaw_actual=" << feedback.yaw_actual_deg
                    << " pitch_actual=" << feedback.pitch_actual_deg
                    << " yaw_cmd=" << feedback.yaw_setpoint_deg
                    << " pitch_cmd=" << feedback.pitch_setpoint_deg
                    << " yaw_torque=" << feedback.yaw_torque_nm
                    << " pitch_torque=" << feedback.pitch_torque_nm
                    << " tx_dt_ms=" << feedback.tx_dt_ms
                    << " ge_requested=" << gimbal.enabled() << '\n';
          last_print = std::chrono::steady_clock::now();
          last_yaw = feedback.yaw_actual_deg;
          last_pitch = feedback.pitch_actual_deg;
        }
      }
    } catch (const std::invalid_argument &) {
      rejected_count++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (valid_count == 0) {
    std::cerr << "[feedback] no valid 9-field CSV received in this observation window.\n";
  }
  if (rejected_count > 0) {
    std::cerr << "[feedback] ignored " << rejected_count << " malformed CSV line(s).\n";
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/gimbal1.yaml";

  try {
    io::Gimbal1 gimbal(config_path);
    const auto directory = std::filesystem::absolute("logs/gimbal1");
    std::filesystem::create_directories(directory);
    const auto log_path = directory / ("feedback_" + std::to_string(std::time(nullptr)) + "_" + std::to_string(getpid()) + ".log");
    gimbal.start_logging(log_path.string());
    std::cout << "[log] full timestamped serial trace: " << log_path.string() << '\n';
    std::cout << "gimbal1 feedback opened " << gimbal.port() << " at " << gimbal.baudrate()
              << " baud (8N1). It starts DISABLED.\n";
    print_help();

    // Show one initial actual-angle sample, then leave a clean input prompt.
    show_actual_angles(gimbal, std::chrono::milliseconds(300));

    std::string line;
    while (std::cout << "gimbal1_feedback> " << std::flush && gimbal1_read_command(gimbal, line)) {
      std::istringstream input(line);
      std::string command;
      if (!(input >> command)) continue;

      try {
        if (command == "enable" || command == "e") {
          require_no_extra_input(input);
          gimbal.enable();
          std::cout << "ge sent; motor readiness is handled by A-board\n";
          show_actual_angles(gimbal, std::chrono::milliseconds(500));
        } else if (command == "disable" || command == "d") {
          require_no_extra_input(input);
          gimbal.disable();
          std::cout << "sent: gd\\r\\n (disabled)\n";
        } else if (command == "yaw" || command == "y") {
          double angle = 0.0;
          double speed = 0.0;
          read_motion_arguments(input, angle, speed);
          gimbal.move_relative_yaw(angle, speed);
          std::cout << "sent: yr " << angle << ' ' << speed << "\\r\\n\n";
          show_actual_angles(gimbal, motion_observation_time(angle, speed));
        } else if (command == "pitch" || command == "p") {
          double angle = 0.0;
          double speed = 0.0;
          read_motion_arguments(input, angle, speed);
          gimbal.move_relative_pitch(angle, speed);
          std::cout << "sent: pr " << angle << ' ' << speed << "\\r\\n\n";
          show_actual_angles(gimbal, motion_observation_time(angle, speed));
        } else if (command == "feedback" || command == "f") {
          require_no_extra_input(input);
          show_actual_angles(gimbal, std::chrono::seconds(1));
        } else if (command == "status") {
          require_no_extra_input(input);
          io::Gimbal1Feedback feedback;
          gimbal.poll_feedback(feedback);
          std::cout << gimbal.status_text() << '\n';
        } else if (command == "help" || command == "h") {
          require_no_extra_input(input);
          print_help();
        } else if (command == "quit" || command == "q") {
          require_no_extra_input(input);
          gimbal.disable();
          std::cout << "sent: gd\\r\\n; exiting safely.\n";
          return 0;
        } else {
          std::cerr << "Unknown command. Type 'help'.\n";
        }
      } catch (const std::exception & e) {
        std::cerr << "Command rejected: " << e.what() << '\n';
      }
    }

    std::cout << "\nInput closed; closing serial only (no automatic gd).\n";
  } catch (const std::exception & e) {
    std::cerr << "gimbal1 feedback startup failed: " << e.what() << '\n';
    return 1;
  }

  return 0;
}

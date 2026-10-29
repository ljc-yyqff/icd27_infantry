#include "io/gimbal1/gimbal1.hpp"

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t quit = 0;

void on_signal(int) { quit = 1; }
}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/gimbal1.yaml";
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  try {
    io::Gimbal1 gimbal(config_path);
    std::cout << "Monitoring A-board CSV feedback on " << gimbal.port() << " at "
              << gimbal.baudrate() << " baud (8N1). No motion command will be sent.\n"
              << "Press Ctrl+C to stop.\n";

    auto last_feedback = std::chrono::steady_clock::now();
    bool warned_timeout = false;

    while (!quit) {
      io::Gimbal1Feedback feedback;
      std::string raw_line;
      try {
        if (gimbal.poll_feedback(feedback, &raw_line)) {
          last_feedback = feedback.received_at;
          warned_timeout = false;

          std::cout << "CSV[1..26]: " << raw_line << '\n'
                    << std::fixed << std::setprecision(3)
                    << "  yaw   target=" << feedback.yaw_target_deg
                    << " actual=" << feedback.yaw_actual_deg
                    << " speed=" << feedback.yaw_speed_deg_s
                    << " active=" << feedback.yaw_active
                    << " fault=" << feedback.yaw_fault
                    << " motor=" << feedback.yaw_motor_state << '\n'
                    << "  pitch target=" << feedback.pitch_target_deg
                    << " actual=" << feedback.pitch_actual_deg
                    << " speed=" << feedback.pitch_speed_deg_s
                    << " active=" << feedback.pitch_active
                    << " fault=" << feedback.pitch_fault
                    << " motor=" << feedback.pitch_motor_state;

          if (feedback.yaw_active != 1 || feedback.yaw_fault != 0 ||
              feedback.pitch_active != 1 || feedback.pitch_fault != 0) {
            std::cout << "  [NOT READY FOR CALIBRATION]";
          }
          std::cout << '\n'
                    << "  left   target=" << feedback.values[14]
                    << " actual=" << feedback.values[15]
                    << " current_cmd=" << feedback.values[16]
                    << " valid=" << feedback.values[17] << '\n'
                    << "  right  target=" << feedback.values[18]
                    << " actual=" << feedback.values[19]
                    << " current_cmd=" << feedback.values[20]
                    << " valid=" << feedback.values[21] << '\n'
                    << "  feeder target=" << feedback.values[22]
                    << " actual=" << feedback.values[23]
                    << " current_cmd=" << feedback.values[24]
                    << " valid=" << feedback.values[25]
                    << "\n\n" << std::flush;
        }
      } catch (const std::invalid_argument & e) {
        std::cerr << "Rejected malformed CSV line: " << e.what() << '\n';
      }

      const auto silence = std::chrono::steady_clock::now() - last_feedback;
      if (!warned_timeout && silence > std::chrono::seconds(2)) {
        std::cerr << "No valid 26-field CSV feedback received for 2 seconds.\n";
        warned_timeout = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  } catch (const std::exception & e) {
    std::cerr << "gimbal1 feedback monitor failed: " << e.what() << '\n';
    return 1;
  }

  return 0;
}

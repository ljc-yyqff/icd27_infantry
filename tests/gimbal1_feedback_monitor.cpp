#include "io/gimbal1/gimbal1.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
namespace { volatile std::sig_atomic_t quit = 0; void on_signal(int) { quit = 1; } }
int main(int argc, char * argv[])
try {
  std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
  io::Gimbal1 gimbal(argc > 1 ? argv[1] : "configs/gimbal1.yaml");
  std::cout << "Read-only 9-column telemetry monitor: no ge, status queries or motion commands. Ctrl+C exits.\n";
  auto last = std::chrono::steady_clock::now();
  bool warned = false;
  while (!quit) {
    io::Gimbal1Feedback f; std::string raw;
    if (gimbal.poll_feedback(f, &raw)) {
      last = f.received_at; warned = false;
      std::cout << "CSV[1..9]: " << raw << '\n' << std::fixed << std::setprecision(3)
                << "  pitch setpoint=" << f.pitch_setpoint_deg << " enc=" << f.pitch_actual_deg
                << " torque=" << f.pitch_torque_nm << " Nm\n"
                << "  yaw   setpoint=" << f.yaw_setpoint_deg << " enc=" << f.yaw_actual_deg
                << " torque=" << f.yaw_torque_nm << " Nm\n"
                << "  imu pitch=" << f.imu_pitch_deg << " yaw=" << f.imu_yaw_deg
                << " tx_dt=" << f.tx_dt_ms << " ms\n"
                << "  " << gimbal.status_text() << "\n\n";
    }
    if (!warned && std::chrono::steady_clock::now() - last > std::chrono::seconds(2)) {
      std::cerr << "No valid 9-field telemetry for 2 seconds.\n"; warned = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return 0;
} catch (const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }

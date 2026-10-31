#include "io/gimbal1/gimbal1.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
using Clock = std::chrono::steady_clock;
void check(bool ok, const char * name) { if (!ok) throw std::runtime_error(name); }
void rejected(const std::function<void()> & fn) {
  bool threw = false; try { fn(); } catch (const std::exception &) { threw = true; }
  check(threw, "invalid input was accepted");
}
int main()
try {
  check(io::Gimbal1::enable_command() == "ge\r\n", "ge");
  check(io::Gimbal1::disable_command() == "gd\r\n", "gd");
  check(io::Gimbal1::relative_yaw_command(1, 30) == "yr 1.000 30.000\r\n", "yr");
  check(io::Gimbal1::relative_pitch_command(-1, 10) == "pr -1.000 10.000\r\n", "pr");
  rejected([] { io::Gimbal1::relative_yaw_command(1, 0.9); });
  rejected([] { io::Gimbal1::relative_yaw_command(1, 121); });
  rejected([] { io::Gimbal1::relative_yaw_command(NAN, 10); });
  rejected([] { io::Gimbal1::relative_yaw_command(1e30, 10); });
  const auto f = io::Gimbal1::parse_feedback_line(" 12,11,22,21,10,20,20,0.1,-0.2\r\n");
  check(f.pitch_setpoint_deg == 12 && f.pitch_actual_deg == 11 &&
        f.yaw_setpoint_deg == 22 && f.yaw_actual_deg == 21 && f.imu_pitch_deg == 10 &&
        f.imu_yaw_deg == 20 && f.tx_dt_ms == 20 && f.pitch_torque_nm == 0.1 &&
        f.yaw_torque_nm == -0.2, "nine-column mapping");
  for (auto line : {"1,2", "1,2,3,4,5,6,7,8,9,", "1,2,3,4,5,6,7,8,", "1,2,3,4,5,6,7,,9",
       "1,2,3,4,5,6,7,inf,9", "1,2,3,4,5,6,7,8x,9",
       "0,0,0,0,1,0,1,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1", "#PP,state=4"})
    rejected([line] { io::Gimbal1::parse_feedback_line(line); });
  io::Gimbal1Status status;
  check(io::Gimbal1::parse_status_line("#YP,id=1,state=4,unknown=10", status) &&
        status.yaw && status.state == 4, "yaw probe");
  check(io::Gimbal1::parse_status_line("#PP,state=2\r", status) && !status.yaw && status.state == 2 && status.mode == -1, "pitch probe");
  check(io::Gimbal1::parse_status_line("#PP,state=4,mode=1,valid=1", status) &&
        status.state == 4 && status.mode == 1, "space mode retained");
  check(io::Gimbal1::parse_status_line("#YP,id=1,state=4,mode=0", status) &&
        status.yaw && status.mode == 0, "encoder mode retained");
  for (auto line : {"#PP,state=4,mode=0,mode=1", "#PP,state=4,mode=2", "#YP,id=2,state=4", "#YP,state=4", "#PP,state=40", "#PP,state=4x",
       "#PP,state=4,state=5", "#PP,mode=0", "#YCAL,state=4", "1,2,3"})
    check(!io::Gimbal1::parse_status_line(line, status), "invalid probe accepted");

  const auto nan_feedback = io::Gimbal1::parse_feedback_line("nan,nan,nan,nan,1,2,20,0,0");
  check(std::isnan(nan_feedback.pitch_actual_deg) && std::isnan(nan_feedback.yaw_actual_deg),
        "nan telemetry was not retained");
  io::Gimbal1CommandReceipt receipt;
  check(io::Gimbal1::parse_command_receipt(
          "#GC,cmd=pr,ok=1,reason=accepted,state=4,mode=1,target_deg=12.345", receipt) &&
        receipt.command == "pr" && receipt.ok && receipt.mode == 1 &&
        receipt.target_deg == 12.345, "command receipt parse");
  io::Gimbal1GlobalStatus global;
  check(io::Gimbal1::parse_global_status(
          "#GM,id=1,ok=1,reason=accepted,ys=4,yf=0,ym=1,yrq=1,ps=4,pf=0,pm=1,prq=1,"
          "yg=0,pg=0,ya=0,pa=0,pe=0,lo=0,hi=41,sg=-1,valid=1,age=1,sa=0", global) &&
        global.yaw_state == 4 && global.pitch_state == 4 && global.yaw_mode == 1 &&
        global.pitch_mode == 1 && global.imu_valid == 1, "global status parse");

  std::cout << "PASS: ASCII wire format, 9-column telemetry and diagnostic response parsing\n";
  return 0;
} catch (const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }

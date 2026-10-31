#include "io/gimbal1/gimbal1.hpp"
#include "tools/exiter.hpp"
#include "tools/signal_stop_action.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
int main(int argc, char ** argv) try {
  if (argc != 3) return 2;
  const std::string mode = argv[2];
  tools::Exiter exiter;
  io::Gimbal1 gimbal(argv[1]);
  tools::SignalStopAction stop(exiter, mode != "manual", [&] { gimbal.disable(); });
  std::cout << "READY" << std::endl;
  if (mode == "normal") return 0;
  if (mode == "error") throw std::runtime_error("simulated runtime error");
  while (!exiter.exit()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  if (mode == "unwind") throw std::runtime_error("simulated frame timeout during stop");
  return 0;
} catch (const std::exception &) { return 1; }

#pragma once
#include "io/gimbal1/gimbal1.hpp"
#include <poll.h>
#include <unistd.h>
#include <stdexcept>
#include <string>

// Keep telemetry, status replies and command receipts serviced at the prompt.
inline bool gimbal1_read_command(io::Gimbal1 & gimbal, std::string & line)
{
  line.clear();
  while (true) {
    io::Gimbal1Feedback feedback;
    gimbal.poll_feedback(feedback);
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int result = ::poll(&input, 1, 5);
    if (result < 0) throw std::runtime_error("stdin poll failed");
    if (result == 0) continue;
    char c;
    const auto count = ::read(STDIN_FILENO, &c, 1);
    if (count == 0) return !line.empty();
    if (count < 0) throw std::runtime_error("stdin read failed");
    if (c == '\n') return true;
    if (c != '\r') line += c;
  }
}

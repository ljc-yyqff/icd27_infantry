#include "io/gimbal1/gimbal1.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void expect_equal(const std::string & actual, const std::string & expected, const char * name)
{
  if (actual != expected) {
    throw std::runtime_error(
      std::string(name) + " command mismatch: got " + std::to_string(actual.size()) +
      " bytes, expected " + std::to_string(expected.size()));
  }
}
}  // namespace

int main()
{
  expect_equal(io::Gimbal1::enable_command(), "ge\r\n", "enable");
  expect_equal(io::Gimbal1::disable_command(), "gd\r\n", "disable");
  expect_equal(io::Gimbal1::relative_yaw_command(1.0, 30.0), "yr 1.000 30.000\r\n", "yaw");
  expect_equal(
    io::Gimbal1::relative_pitch_command(-1.0, 10.0), "pr -1.000 10.000\r\n", "pitch");

  const auto feedback = io::Gimbal1::parse_feedback_line(
    "-10.699,-10.316,10.000,-0.420,1,0,1,54.620,54.620,10.000,-0.420,0,5,0,"
    "0,0,0,1,0,0,0,1,0,0,0,1");
  if (feedback.yaw_actual_deg != -10.316 || feedback.pitch_actual_deg != 54.620 ||
      feedback.yaw_active != 1 || feedback.yaw_fault != 0 || feedback.pitch_active != 0 ||
      feedback.pitch_fault != 5) {
    throw std::runtime_error("gimbal1 CSV feedback parse mismatch");
  }

  std::cout << "gimbal1 ASCII protocol and 26-field CSV feedback parser OK\n";
  return 0;
}

#pragma once
#include <functional>
#include <iostream>
#include <utility>
#include "tools/exiter.hpp"

namespace tools
{
// An explicit SIGTERM/SIGINT stop command is different from a runtime failure.
// Keep this guard before the resources it references are destroyed.
class SignalStopAction
{
public:
  SignalStopAction(const Exiter & exiter, bool enabled, std::function<void()> action)
  : exiter_(exiter), enabled_(enabled), action_(std::move(action)) {}
  SignalStopAction(const SignalStopAction &) = delete;
  SignalStopAction & operator=(const SignalStopAction &) = delete;
  ~SignalStopAction() noexcept
  {
    if (!enabled_ || !exiter_.exit()) return;
    try { action_(); }
    catch (const std::exception & e) { std::cerr << "[service stop] gd write failed: " << e.what() << '\n'; }
    catch (...) { std::cerr << "[service stop] gd write failed\n"; }
  }
private:
  const Exiter & exiter_;
  bool enabled_;
  std::function<void()> action_;
};
}

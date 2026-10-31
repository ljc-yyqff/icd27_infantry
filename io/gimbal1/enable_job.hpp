#pragma once
#include "gimbal1.hpp"
#include <atomic>
#include <future>
#include <memory>
#include <stdexcept>

namespace io
{
// The UI must not call Gimbal1 while this job owns its serial mutex.
// Destruction cancels and joins before the Gimbal1 instance is destroyed.
class Gimbal1EnableJob
{
public:
  ~Gimbal1EnableJob() { cancel(); if (result_.valid()) result_.wait(); }
  bool pending() const { return result_.valid(); }
  bool finished() const
  { return pending() && result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }
  bool cancelled() const { return cancel_ && cancel_->load(); }
  void cancel() { if (cancel_) cancel_->store(true); }
  void start(Gimbal1 & gimbal)
  {
    if (pending()) throw std::logic_error("enable already in progress");
    cancel_ = std::make_shared<std::atomic<bool>>(false);
    result_ = std::async(std::launch::async, [&gimbal, cancel = cancel_] {
      gimbal.enable(cancel.get());
    });
  }
  void finish() { result_.get(); }
private:
  std::shared_ptr<std::atomic<bool>> cancel_;
  std::future<void> result_;
};
}

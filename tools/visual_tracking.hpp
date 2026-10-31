#ifndef TOOLS_VISUAL_TRACKING_HPP
#define TOOLS_VISUAL_TRACKING_HPP

#include <algorithm>
#include <cmath>
#include <vector>

namespace tools::visual_tracking
{
inline bool fresh_frame(double age_ms, double maximum_ms)
{
  return std::isfinite(age_ms) && age_ms >= 0 && age_ms <= maximum_ms;
}

// Match each small relative increment to its integration interval instead of
// sending every increment at a fixed high speed. Firmware protocol minimum is 1 deg/s.
inline double command_speed(double increment, double integrated_dt, double limit)
{
  if (!std::isfinite(increment) || !std::isfinite(integrated_dt) || integrated_dt <= 0)
    return 1.0;
  return std::clamp(std::abs(increment) / integrated_dt, 1.0, limit);
}

// All state here belongs to image tracking. No motor state or arrival checks.
struct Observation { double x, y, confidence; int identity; };

class TargetTracker
{
public:
  TargetTracker(double jump_px, double tau) : jump_px_(jump_px), tau_(tau) {}
  int update(const std::vector<Observation> & observations, double image_time)
  {
    if (image_time <= last_time_) return -1;
    const double dt = image_time - last_time_;
    if (dt > 0.2) { locked_ = false; pending_count_ = 0; }
    last_time_ = image_time;
    int best = -1;
    double distance = jump_px_;
    if (locked_) {
      for (int i = 0; i < static_cast<int>(observations.size()); ++i) {
        const auto & o = observations[i];
        if (o.identity != identity_) continue;
        const double d = std::hypot(o.x - raw_x_, o.y - raw_y_);
        if (d < distance) { distance = d; best = i; }
      }
    }
    if (best >= 0) {
      const auto & o = observations[best];
      raw_x_ = o.x; raw_y_ = o.y;
      const double alpha = tau_ <= 0 ? 1 : -std::expm1(-dt / tau_);
      x_ += alpha * (o.x - x_); y_ += alpha * (o.y - y_);
      pending_count_ = 0;
      last_match_ = image_time;
      return best;
    }
    // Missing/jumping detections produce no command. Require three consistent
    // observations to acquire a different position/identity; do not follow one outlier.
    if (image_time - last_match_ > 0.2) locked_ = false;
    if (observations.empty()) { pending_count_ = 0; return -1; }
    best = 0;
    for (int i = 1; i < static_cast<int>(observations.size()); ++i)
      if (observations[i].confidence > observations[best].confidence) best = i;
    const auto & o = observations[best];
    if (pending_count_ > 0 && o.identity == pending_identity_ &&
        std::hypot(o.x - pending_x_, o.y - pending_y_) < jump_px_ * 0.5)
      ++pending_count_;
    else pending_count_ = 1;
    pending_x_ = o.x; pending_y_ = o.y; pending_identity_ = o.identity;
    if (pending_count_ < 3) return -1;
    ++generation_;
    identity_ = o.identity; x_ = raw_x_ = o.x; y_ = raw_y_ = o.y;
    last_match_ = image_time; locked_ = true; pending_count_ = 0;
    return best;
  }
  size_t generation() const { return generation_; }
  double x() const { return x_; }
  double y() const { return y_; }
private:
  size_t generation_ = 0;
  double jump_px_, tau_, last_time_ = -1e9, last_match_ = -1e9;
  double x_ = 0, y_ = 0, raw_x_ = 0, raw_y_ = 0, pending_x_ = 0, pending_y_ = 0;
  int identity_ = -1, pending_identity_ = -1, pending_count_ = 0;
  bool locked_ = false;
};

class Axis
{
public:
  Axis(double gain, double stop_px, double start_px, double speed, double max_step)
  : gain_(gain), stop_px_(stop_px), start_px_(start_px), speed_(speed), max_step_(max_step) {}
  double step(double pixels, double degrees, double dt, double feedforward = 0)
  {
    if (!std::isfinite(pixels) || !std::isfinite(degrees) || !std::isfinite(dt) || dt <= 0 || !std::isfinite(feedforward)) {
      reset(); return 0;
    }
    const auto magnitude = std::abs(pixels);
    if (magnitude <= stop_px_) active_ = false;
    else if (magnitude >= start_px_) active_ = true;
    moving_ = active_ || std::abs(feedforward) > 1e-6;
    if (!moving_) return 0;
    const auto velocity = std::clamp(feedforward + (active_ ? gain_ * degrees : 0), -speed_, speed_);
    // Never integrate time spent paused, without a target or stalled in inference.
    return std::clamp(velocity * std::min(dt, 0.1), -max_step_, max_step_);
  }
  void reset() { active_ = moving_ = false; }
  bool active() const { return moving_; }
private:
  double gain_, stop_px_, start_px_, speed_, max_step_;
  bool active_ = false, moving_ = false;
};
}  // namespace tools::visual_tracking
#endif

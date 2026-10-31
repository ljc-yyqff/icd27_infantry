#pragma once
#include <deque>
#include <cmath>
#include <optional>
#include <string>
#include "solver.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
struct TimedPose
{
  double time = 0, yaw = 0, pitch = 0; // seconds and radians, unwrapped yaw
  bool extrapolated = false;
};
class PoseHistory
{
public:
  PoseHistory(double max_gap = .08, double extrapolate = .025)
  : max_gap_(max_gap), extrapolate_(extrapolate) {}
  void push(double time, double yaw, double pitch);
  void clear() { samples_.clear(); }
  std::optional<TimedPose> at(double time) const;
private:
  std::deque<TimedPose> samples_;
  double max_gap_, extrapolate_;
};

// A single armor's Cartesian constant-velocity model, using the project's EKF.
// No vehicle center, armor radius, angular phase or bullet-flight model.
class ArmorMotion
{
public:
  explicit ArmorMotion(double acceleration_sigma = 20) : acceleration_sigma_(acceleration_sigma) {}
  bool update(const Eigen::Vector3d & position, const Eigen::Matrix3d & covariance,
              double time, int identity);
  void reset() { initialized_ = false; count_ = rejected_ = 0; }
  Eigen::Vector3d position(double time) const;
  Eigen::Vector3d velocity() const;
  int count() const { return count_; }
private:
  tools::ExtendedKalmanFilter filter_;
  double time_ = 0, acceleration_sigma_;
  int identity_ = -1, count_ = 0, rejected_ = 0;
  bool initialized_ = false;
  Eigen::Vector3d last_rejected_ = Eigen::Vector3d::Zero();
};
struct ArmorPrediction
{
  bool valid = false, augmentation_valid = false, pose_extrapolated = false;
  std::string reason = "no_observation";
  double yaw_error = 0, pitch_error = 0; // camera right/down, degrees
  double yaw_lead = 0, pitch_lead = 0;
  double yaw_ff = 0, pitch_ff = 0;       // target-only camera angular rates, deg/s
  double error_x = 0, error_y = 0, horizon_ms = 0, reprojection_px = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero();
  double frame_yaw = 0, frame_pitch = 0, current_yaw = 0, current_pitch = 0;
  int motion_samples = 0;
};
// Always retain the observed pixel position loop. The model contributes only
// bounded lead/feedforward; unavailable geometry never stalls basic tracking.
struct CameraCorrection
{
  double yaw, pitch, yaw_ff = 0, pitch_ff = 0;
  bool augmented = false;
};
inline CameraCorrection augment_pixel_correction(double yaw, double pitch, const ArmorPrediction & p)
{
  if (!p.valid || !p.augmentation_valid || !std::isfinite(p.yaw_lead) ||
      !std::isfinite(p.pitch_lead) || !std::isfinite(p.yaw_ff) || !std::isfinite(p.pitch_ff)) return {yaw,pitch,0,0};
  return {yaw+p.yaw_lead,pitch+p.pitch_lead,p.yaw_ff,p.pitch_ff,true};
}

class SingleArmorTracking
{
public:
  explicit SingleArmorTracking(const std::string & config);
  // Time is host receive time, corrected by the configured IMU latency estimate.
  void push_pose(double host_time, double imu_yaw_deg, double imu_pitch_deg);
  bool observe(Armor armor, double host_frame_time, cv::Size image_size = {});
  ArmorPrediction predict(double host_now) const;
  void reset_motion() { motion_.reset(); observation_valid_ = false; consecutive_ = 0; ff_velocity_.setZero(); ff_velocity_time_ = 0; reason_ = "no_observation"; }
  void reset_epoch() { poses_.clear(); reset_motion(); }
  void set_prediction_delay_ms(double delay);
  void set_feedforward_gain(double gain);
  double command_rate_cap() const { return command_rate_cap_; }
  std::string reason() const { return reason_; }
private:
  Solver solver_;
  PoseHistory poses_;
  ArmorMotion motion_;
  Eigen::Matrix3d camera_to_gimbal_, imu_mount_;
  Eigen::Vector3d camera_translation_;
  double fx_, fy_, yaw_sign_ = 1, pitch_sign_ = -1;
  double imu_latency_ = 0, camera_latency_ = 0, extra_delay_ = 0;
  double max_horizon_ = .15, reprojection_limit_ = 4, range_limit_ = 30;
  double sigma_xy_ = .001, sigma_depth_ = .06, ff_gain_ = .8, ff_deadband_ = .5;
  double command_rate_cap_ = 25;
  double reprojection_fraction_ = .04, reprojection_absolute_ = 20;
  double max_lead_deg_ = 2, max_ff_deg_s_ = 30, ff_velocity_tau_ = .05;
  Eigen::Vector3d ff_velocity_ = Eigen::Vector3d::Zero();
  double ff_velocity_time_ = 0;
  cv::Mat camera_matrix_, distortion_;
  TimedPose frame_pose_;
  double frame_time_ = 0, reprojection_ = 0;
  int min_motion_samples_ = 5, consecutive_ = 0;
  bool observation_valid_ = false;
  std::string reason_ = "no_observation";
  Eigen::Matrix3d rotation(const TimedPose & pose) const;
};
} // namespace auto_aim

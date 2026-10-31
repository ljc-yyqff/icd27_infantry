#include "single_armor_tracking.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace
{
constexpr double rad = M_PI / 180.0;
template<class T> T read(const YAML::Node & n, const char * key, T fallback)
{ return n && n[key] ? n[key].as<T>() : fallback; }
}
void PoseHistory::push(double time, double yaw, double pitch)
{
  if (!std::isfinite(time) || !std::isfinite(yaw) || !std::isfinite(pitch)) return;
  if (!samples_.empty() && time <= samples_.back().time) return;
  // A USB burst may contain several telemetry frames with effectively identical
  // receive times. Keep only the newest; never infer angular velocity over microseconds.
  if (!samples_.empty() && time - samples_.back().time < .005) samples_.pop_back();
  if (!samples_.empty()) yaw = samples_.back().yaw + std::remainder(yaw - samples_.back().yaw, 2*M_PI);
  samples_.push_back({time,yaw,pitch,false});
  while (samples_.size() > 2 && time - samples_.front().time > 2) samples_.pop_front();
}
std::optional<TimedPose> PoseHistory::at(double time) const
{
  if (samples_.empty() || !std::isfinite(time) || time < samples_.front().time) return {};
  if (time==samples_.front().time) return samples_.front();
  for (size_t i=1; i<samples_.size(); ++i) {
    const auto & a=samples_[i-1]; const auto & b=samples_[i];
    if (time > b.time) continue;
    if(time==b.time) return b;
    const double dt=b.time-a.time;
    if (dt > max_gap_) return {};
    const double k=(time-a.time)/dt;
    return TimedPose{time,a.yaw+k*(b.yaw-a.yaw),a.pitch+k*(b.pitch-a.pitch),false};
  }
  const auto & b=samples_.back();
  if (time == b.time) return b;
  const double age=time-b.time;
  if (age > extrapolate_ || samples_.size()<2) return {};
  const auto & a=samples_[samples_.size()-2];
  const double dt=b.time-a.time;
  if (dt < .005 || dt > max_gap_) return {};
  const double k=age/dt;
  return TimedPose{time,b.yaw+k*(b.yaw-a.yaw),b.pitch+k*(b.pitch-a.pitch),true};
}
bool ArmorMotion::update(const Eigen::Vector3d & p, const Eigen::Matrix3d & R, double t, int id)
{
  if (!p.allFinite() || !R.allFinite() || !std::isfinite(t)) return false;
  const bool initialize=!initialized_ || id!=identity_ || t-time_>.15;
  if (!initialize && t<=time_) return false;
  if (initialize) {
    Eigen::VectorXd x=Eigen::VectorXd::Zero(6);
    for(int i=0;i<3;++i) x[2*i]=p[i];
    Eigen::MatrixXd P=Eigen::MatrixXd::Identity(6,6);
    for(int i=0;i<3;++i) {
      for(int j=0;j<3;++j) P(2*i,2*j)=R(i,j);
      P(2*i+1,2*i+1)=100;
    }
    filter_=tools::ExtendedKalmanFilter(x,P);
    initialized_=true; identity_=id; time_=t; count_=1; rejected_=0;
    return true;
  }
  const double dt=t-time_;
  Eigen::MatrixXd F=Eigen::MatrixXd::Identity(6,6),Q=Eigen::MatrixXd::Zero(6,6);
  for(int i=0;i<3;++i) {
    const int k=2*i; F(k,k+1)=dt;
    Q(k,k)=std::pow(dt,4)/4; Q(k,k+1)=Q(k+1,k)=std::pow(dt,3)/2; Q(k+1,k+1)=dt*dt;
  }
  filter_.predict(F,Q*acceleration_sigma_*acceleration_sigma_);
  time_=t;
  Eigen::MatrixXd H=Eigen::MatrixXd::Zero(3,6);
  for(int i=0;i<3;++i) H(i,2*i)=1;
  const Eigen::Vector3d residual=p-H*filter_.x;
  const Eigen::Matrix3d S=H*filter_.P*H.transpose()+R;
  if (residual.dot(S.ldlt().solve(residual))>25) {
    rejected_ = rejected_ && (p-last_rejected_).norm()<.3 ? rejected_+1 : 1;
    last_rejected_=p;
    if(rejected_<3) return false;
    reset(); return update(p,R,t,id);
  }
  filter_.update(p,H,R);
  if(!filter_.x.allFinite() || !filter_.P.allFinite()) { reset(); return false; }
  ++count_; rejected_=0;
  return true;
}
Eigen::Vector3d ArmorMotion::position(double time) const
{
  Eigen::Vector3d p;
  for(int i=0;i<3;++i) p[i]=filter_.x[2*i]+filter_.x[2*i+1]*(time-time_);
  return p;
}
Eigen::Vector3d ArmorMotion::velocity() const
{ return {filter_.x[1],filter_.x[3],filter_.x[5]}; }
SingleArmorTracking::SingleArmorTracking(const std::string & config) : solver_(config)
{
  const auto yaml=YAML::LoadFile(config); const auto n=yaml["gimbal1_tracking"];
  auto matrix=yaml["camera_matrix"].as<std::vector<double>>();
  auto rotation=yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto translation=yaml["t_camera2gimbal"].as<std::vector<double>>();
  if(matrix.size()!=9 || rotation.size()!=9 || translation.size()!=3)
    throw std::invalid_argument("invalid single armor calibration dimensions");
  auto mount=yaml["R_gimbal2imubody"].as<std::vector<double>>();
  if(mount.size()!=9) throw std::invalid_argument("invalid IMU mount dimensions");
  imu_mount_=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(mount.data());
  auto distortion=yaml["distort_coeffs"].as<std::vector<double>>();
  camera_matrix_=cv::Mat(3,3,CV_64F,matrix.data()).clone();
  distortion_=cv::Mat(1,5,CV_64F,distortion.data()).clone();
  fx_=matrix[0]; fy_=matrix[4];
  camera_to_gimbal_=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(rotation.data());
  camera_translation_=Eigen::Map<Eigen::Vector3d>(translation.data());
  yaw_sign_=read(n,"imu_yaw_sign",1.0); pitch_sign_=read(n,"imu_pitch_sign",-1.0);
  imu_latency_=read(n,"imu_latency_ms",0.0)/1000;
  camera_latency_=read(n,"camera_latency_ms",0.0)/1000;
  extra_delay_=read(n,"additional_prediction_ms",0.0)/1000;
  max_horizon_=read(n,"max_prediction_ms",150.0)/1000;
  reprojection_limit_=read(n,"max_reprojection_px",4.0);
  reprojection_fraction_=read(n,"max_reprojection_fraction",.04);
  reprojection_absolute_=read(n,"max_reprojection_absolute_px",20.0);
  max_lead_deg_=read(n,"max_lead_deg",2.0);
  max_ff_deg_s_=read(n,"max_feedforward_deg_s",30.0);
  ff_velocity_tau_=read(n,"feedforward_velocity_tau_ms",50.0)/1000;
  range_limit_=read(n,"max_range_m",30.0);
  sigma_xy_=read(n,"measurement_sigma_xy_m",.001);
  sigma_depth_=read(n,"measurement_sigma_depth_m",.06);
  ff_gain_=read(n,"feedforward_gain",.8); ff_deadband_=read(n,"feedforward_deadband_deg_s",.5);
  min_motion_samples_=read(n,"min_motion_samples",5);
  command_rate_cap_=read(n,"command_rate_cap_hz",25.0);
  const double gap=read(n,"pose_max_gap_ms",80.0)/1000;
  const double extrapolate=read(n,"pose_extrapolate_ms",50.0)/1000;
  const double acceleration=read(n,"acceleration_sigma_m_s2",20.0);
  for(double v:{fx_,fy_,max_horizon_,reprojection_limit_,range_limit_,sigma_xy_,sigma_depth_,gap,acceleration,command_rate_cap_,reprojection_fraction_,reprojection_absolute_,max_lead_deg_,max_ff_deg_s_})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("single armor tracking values must be positive");
  for(double v:{imu_latency_,camera_latency_,extra_delay_,extrapolate,ff_gain_,ff_deadband_,ff_velocity_tau_})
    if(!std::isfinite(v)||v<0) throw std::invalid_argument("single armor tracking values must be nonnegative");
  if(reprojection_absolute_<reprojection_limit_ || reprojection_fraction_>.1 || max_lead_deg_>10 || command_rate_cap_>50 || extra_delay_>max_horizon_ || std::abs(yaw_sign_)!=1 || std::abs(pitch_sign_)!=1 || ff_gain_>2 || min_motion_samples_<3 ||
     extrapolate>.05 || max_horizon_>.3 || !camera_to_gimbal_.allFinite() ||
     !camera_translation_.allFinite() || !imu_mount_.allFinite() ||
     (camera_to_gimbal_.transpose()*camera_to_gimbal_-Eigen::Matrix3d::Identity()).norm()>.01 ||
     (imu_mount_.transpose()*imu_mount_-Eigen::Matrix3d::Identity()).norm()>.01 ||
     camera_to_gimbal_.determinant()<.99 || imu_mount_.determinant()<.99) throw std::invalid_argument("invalid two-axis tracking configuration");
  poses_=PoseHistory(gap,extrapolate); motion_=ArmorMotion(acceleration);
}
void SingleArmorTracking::set_prediction_delay_ms(double delay)
{
  if(!std::isfinite(delay)||delay<0||delay/1000>max_horizon_)
    throw std::invalid_argument("prediction delay outside configured horizon");
  extra_delay_=delay/1000;
}
void SingleArmorTracking::set_feedforward_gain(double gain)
{
  if(!std::isfinite(gain)||gain<0||gain>2) throw std::invalid_argument("feedforward_gain must be 0..2");
  ff_gain_=gain;
}
void SingleArmorTracking::push_pose(double t,double yaw,double pitch)
{ poses_.push(t-imu_latency_,yaw*rad,pitch*rad); }
Eigen::Matrix3d SingleArmorTracking::rotation(const TimedPose & p) const
{
  // Fixed roll; standard gimbal axes x forward, y left, z up.
  return (Eigen::AngleAxisd(yaw_sign_*p.yaw,Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch_sign_*p.pitch,Eigen::Vector3d::UnitY())).toRotationMatrix();
}
bool SingleArmorTracking::observe(Armor armor,double host_frame_time,cv::Size image_size)
{
  observation_valid_=false; frame_time_=host_frame_time-camera_latency_;
  const auto fail=[&](const char * reason) { reason_=reason; consecutive_=0; return false; };
  const auto pose=poses_.at(frame_time_);
  if(!pose) return fail("frame_pose_unavailable");
  frame_pose_=*pose;
  if(image_size.width>0 && image_size.height>0)
    for(const auto & p:armor.points)
      if(p.x<0 || p.y<0 || p.x>=image_size.width || p.y>=image_size.height)
        return fail("clipped_armor");
  solver_.set_R_gimbal2world(Eigen::Quaterniond(rotation(*pose)));
  if(!solver_.solve_position(armor,reprojection_)) return fail("pnp_rejected");
  const double short_side=std::min(cv::norm(armor.points[0]-armor.points[3]),
                                  cv::norm(armor.points[1]-armor.points[2]));
  const double limit=std::min(reprojection_absolute_,
    std::max(reprojection_limit_,reprojection_fraction_*short_side));
  if(reprojection_>limit || armor.xyz_in_gimbal.norm()>range_limit_ ||
     armor.xyz_in_gimbal.norm()<.1) return fail("pnp_rejected");
  Eigen::Vector3d camera=camera_to_gimbal_.transpose()*(armor.xyz_in_gimbal-camera_translation_);
  // Preserve the center measurement used by the working pixel loop. PnP supplies
  // depth and rotation compensation, not a replacement center with a geometric bias.
  cv::Point2f center(0,0); for(const auto & p:armor.points) center+=p; center*=.25f;
  std::vector<cv::Point2f> normalized;
  cv::undistortPoints(std::vector<cv::Point2f>{center},normalized,camera_matrix_,distortion_);
  camera=Eigen::Vector3d(normalized[0].x,normalized[0].y,1)*camera.z();
  armor.xyz_in_gimbal=camera_to_gimbal_*camera+camera_translation_;
  armor.xyz_in_world=solver_.R_gimbal2world()*armor.xyz_in_gimbal;
  const Eigen::Matrix3d world_camera=solver_.R_gimbal2world()*camera_to_gimbal_;
  const Eigen::Vector3d ray=camera.normalized();
  // Depth uncertainty is radial, not an independent XYZ perturbation. Otherwise
  // planar PnP depth jitter leaks into false transverse velocity/feedforward.
  const double lateral=std::max(sigma_xy_,camera.norm()*reprojection_/(2*std::min(fx_,fy_)));
  const Eigen::Matrix3d radial=ray*ray.transpose();
  const Eigen::Matrix3d noise=sigma_depth_*sigma_depth_*radial+
    lateral*lateral*(Eigen::Matrix3d::Identity()-radial);
  const int identity=static_cast<int>(armor.color)*100+static_cast<int>(armor.name)*2+static_cast<int>(armor.type);
  if(!motion_.update(armor.xyz_in_world,world_camera*noise*world_camera.transpose(),frame_time_,identity))
    return fail("motion_outlier");
  if(motion_.count()==1 || consecutive_==0) {
    ff_velocity_.setZero();
  } else {
    const double dt=frame_time_-ff_velocity_time_;
    const double alpha=ff_velocity_tau_<=0 ? 1 : -std::expm1(-std::max(0.0,dt)/ff_velocity_tau_);
    ff_velocity_+=alpha*(motion_.velocity()-ff_velocity_);
  }
  ff_velocity_time_=frame_time_;
  consecutive_=motion_.count()==1 ? 1 : consecutive_+1;
  observation_valid_=true; reason_="ok"; return true;
}
ArmorPrediction SingleArmorTracking::predict(double now) const
{
  ArmorPrediction out; out.reason=reason_; out.reprojection_px=reprojection_;
  out.frame_yaw=frame_pose_.yaw/rad; out.frame_pitch=frame_pose_.pitch/rad;
  if(!observation_valid_) return out;
  const auto pose=poses_.at(now);
  if(!pose) { out.reason="current_pose_unavailable"; return out; }
  out.current_yaw=pose->yaw/rad; out.current_pitch=pose->pitch/rad;
  const double horizon=now+extra_delay_-frame_time_;
  if(horizon<0 || horizon>max_horizon_) { out.reason="prediction_expired"; return out; }
  // Use the same IMU mount mapping as Solver, without mutating its observation pose.
  // rotation() is IMU body->absolute; Solver performs the mount change of basis.
  const Eigen::Matrix3d world_gimbal=imu_mount_.transpose()*rotation(*pose)*imu_mount_;
  out.position=motion_.position(now+extra_delay_); out.velocity=ff_velocity_;
  const Eigen::Vector3d camera=camera_to_gimbal_.transpose()*(world_gimbal.transpose()*out.position-camera_translation_);
  const Eigen::Vector3d velocity=camera_to_gimbal_.transpose()*world_gimbal.transpose()*out.velocity;
  if(!camera.allFinite() || !velocity.allFinite() || camera.z()<=.05) {
    out.reason="prediction_behind_camera"; return out;
  }
  out.yaw_error=std::atan2(camera.x(),camera.z())/rad;
  out.pitch_error=std::atan2(camera.y(),camera.z())/rad;
  out.error_x=fx_*camera.x()/camera.z(); out.error_y=fy_*camera.y()/camera.z();
  out.motion_samples=motion_.count();
  const Eigen::Matrix3d frame_gimbal=imu_mount_.transpose()*rotation(frame_pose_)*imu_mount_;
  const Eigen::Vector3d frame_camera=camera_to_gimbal_.transpose()*
    (frame_gimbal.transpose()*motion_.position(frame_time_)-camera_translation_);
  if(frame_camera.z()<=.05 || !frame_camera.allFinite()) {
    out.reason="prediction_behind_camera"; return out;
  }
  if(out.motion_samples>=min_motion_samples_ && consecutive_>=min_motion_samples_) {
    const double ramp=std::min(1.0,(consecutive_-min_motion_samples_+1)/5.0);
    out.yaw_lead=ramp*std::clamp(std::remainder(out.yaw_error-
      std::atan2(frame_camera.x(),frame_camera.z())/rad,360.0),-max_lead_deg_,max_lead_deg_);
    out.pitch_lead=ramp*std::clamp(out.pitch_error-
      std::atan2(frame_camera.y(),frame_camera.z())/rad,-max_lead_deg_,max_lead_deg_);
    out.yaw_ff=(camera.z()*velocity.x()-camera.x()*velocity.z())/(camera.z()*camera.z()+camera.x()*camera.x())/rad;
    out.pitch_ff=(camera.z()*velocity.y()-camera.y()*velocity.z())/(camera.z()*camera.z()+camera.y()*camera.y())/rad;
    out.yaw_ff=std::abs(out.yaw_ff)<ff_deadband_ ? 0 : ramp*std::clamp(ff_gain_*out.yaw_ff,-max_ff_deg_s_,max_ff_deg_s_);
    out.pitch_ff=std::abs(out.pitch_ff)<ff_deadband_ ? 0 : ramp*std::clamp(ff_gain_*out.pitch_ff,-max_ff_deg_s_,max_ff_deg_s_);
    out.augmentation_valid=true;
  }
  out.pose_extrapolated=pose->extrapolated || frame_pose_.extrapolated; out.horizon_ms=horizon*1000;
  out.valid=true; out.reason="ok";
  return out;
}
} // namespace auto_aim

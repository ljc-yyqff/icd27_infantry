#include "tasks/auto_aim/single_armor_tracking.hpp"
#include "tools/visual_tracking.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <random>
#include <filesystem>
#include <fstream>
#include <unistd.h>

using namespace auto_aim;
namespace {
constexpr double rad=M_PI/180;
void check(bool b,const char * what) { if(!b) throw std::runtime_error(what); }
void near(double a,double b,double tolerance,const char * what) { check(std::abs(a-b)<tolerance,what); }
struct Scene {
  Eigen::Matrix3d camera_gimbal,mount;
  Eigen::Vector3d translation;
  cv::Mat K,distortion;
  explicit Scene(const std::string & config) {
    const auto y=YAML::LoadFile(config);
    auto a=y["R_camera2gimbal"].as<std::vector<double>>();
    auto b=y["t_camera2gimbal"].as<std::vector<double>>();
    auto m=y["R_gimbal2imubody"].as<std::vector<double>>();
    camera_gimbal=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(a.data());
    translation=Eigen::Map<Eigen::Vector3d>(b.data());
    mount=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(m.data());
    auto k=y["camera_matrix"].as<std::vector<double>>();
    auto d=y["distort_coeffs"].as<std::vector<double>>();
    K=cv::Mat(3,3,CV_64F,k.data()).clone(); distortion=cv::Mat(1,5,CV_64F,d.data()).clone();
  }
  Eigen::Matrix3d pose(double yaw,double pitch) const {
    const Eigen::Matrix3d imu=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(-pitch,Eigen::Vector3d::UnitY())).toRotationMatrix();
    return mount.transpose()*imu*mount;
  }
  Armor image(const Eigen::Vector3d & position,double yaw,double pitch,ArmorType type=ArmorType::small) const {
    const double w=type==ArmorType::small ? .135 : .230;
    const std::vector<cv::Point3f> corners{{0,float(w/2),.028f},{0,float(-w/2),.028f},
      {0,float(-w/2),-.028f},{0,float(w/2),-.028f}};
    const Eigen::Matrix3d world_gimbal=pose(yaw,pitch);
    const Eigen::Matrix3d armor_world=(Eigen::AngleAxisd(.17,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY())).toRotationMatrix();
    const Eigen::Matrix3d rotation=camera_gimbal.transpose()*world_gimbal.transpose()*armor_world;
    const Eigen::Vector3d t=camera_gimbal.transpose()*(world_gimbal.transpose()*position-translation);
    cv::Mat rotation_cv,rvec,tvec;
    cv::eigen2cv(rotation,rotation_cv); cv::Rodrigues(rotation_cv,rvec); cv::eigen2cv(t,tvec);
    std::vector<cv::Point2f> points;
    cv::projectPoints(corners,rvec,tvec,K,distortion,points);
    Armor a(1,3,.99,cv::Rect{},points);
    a.type=type; a.color=Color::red; a.name=ArmorName::three;
    return a;
  }
};
}
int main(int argc,char ** argv) try {
  check(argc==2,"config path required");
  const std::string config=argv[1];
  PoseHistory history;
  history.push(1,179*rad,.1); history.push(1.02,-179*rad,.12);
  auto p=history.at(1.01); check(bool(p),"pose interpolation missing");
  near(p->yaw,M_PI,1e-9,"yaw interpolation must unwrap across 180 degrees");
  near(p->pitch,.11,1e-9,"pitch interpolation");
  check(history.at(1.03)->extrapolated,"short pose extrapolation is explicit");
  check(!history.at(1.1),"pose extrapolation must be bounded");
  check(!history.at(.99),"no backward extrapolation");
  history.push(1.2,-170*rad,.2);
  check(!history.at(1.1),"large telemetry gaps cannot be interpolated");
  history.push(1.22,std::numeric_limits<double>::quiet_NaN(),0);
  check(!history.at(1.3),"NaN telemetry does not refresh pose");
  PoseHistory burst;
  burst.push(1,0,0); burst.push(1.00001,1,0);
  check(!burst.at(1.001),"a USB burst must not create a microsecond-rate velocity estimate");
  history.clear(); check(!history.at(1.2),"new enable epoch cannot reuse old pose samples");
  std::cout<<"PASS pose timestamps, yaw wrap, gap/extrapolation bounds, invalid/burst telemetry\n";

  ArmorMotion motion;
  const Eigen::Matrix3d R=Eigen::Matrix3d::Identity()*.0001;
  Eigen::Vector3d v(.2,.8,-.3),origin(3,0,1);
  for(int i=0;i<100;++i) check(motion.update(origin+v*(i*.02),R,1+i*.02,1),"CV measurement accepted");
  check((motion.velocity()-v).norm()<.01,"EKF must estimate single armor translation");
  check((motion.position(3.1)-(origin+v*2.1)).norm()<.01,"predict to requested timestamp");
  check(!motion.update(Eigen::Vector3d(20,20,20),R,3,1),"single position outlier must be rejected");
  check(motion.update(origin,R,3.02,2),"new identity initializes a new track");
  check(motion.count()==1 && motion.velocity().norm()==0,"new identity cannot inherit previous velocity");
  check(motion.update(origin,R,4,2) && motion.count()==1,"long image gap clears motion history");
  std::cout<<"PASS original EKF reused with 3D observations, velocity, prediction, outliers and reset\n";

  const Scene scene(config);
  const Eigen::Vector3d initial=scene.pose(.1,.2)*(scene.camera_gimbal*Eigen::Vector3d(.1,-.05,3)+scene.translation);
  for (auto type : {ArmorType::small,ArmorType::big}) {
    SingleArmorTracking tracker(config);
    ArmorPrediction output;
    for(int i=0;i<100;++i) {
      const double t=1+i*.02;
      auto yaw=[](double time) { return .1+.12*std::sin(time-1); };
      auto pitch=[](double time) { return .2+.08*std::sin(time-1); };
      for(double pt:{t-.01,t+.01}) tracker.push_pose(pt,yaw(pt)/rad,pitch(pt)/rad);
      auto armor=scene.image(initial,yaw(t),pitch(t),type);
      check(tracker.observe(armor,t),"stationary world armor observation failed");
      output=tracker.predict(t+.01);
      check(output.valid,"compensated stationary target prediction failed");
    }
    check((output.position-initial).norm()<.02,"camera rotation must not move fixed world armor");
    check(output.velocity.norm()<.05,"camera self-rotation must not become target velocity");
    near(output.yaw_ff,0,.1,"fixed target yaw feedforward must vanish");
    near(output.pitch_ff,0,.1,"fixed target pitch feedforward must vanish");
  }
  std::cout<<"PASS synthetic images: small/big armor PnP and yaw/pitch self-rotation compensation\n";

  SingleArmorTracking moving(config); moving.set_prediction_delay_ms(50);
  const Eigen::Vector3d target_velocity(0,.45,.2);
  ArmorPrediction output;
  double yaw=0,pitch=0,t=0;
  for(int i=0;i<100;++i) {
    t=1+i*.02;
    auto y=[](double time){return .1+.06*(time-1);};
    auto p=[](double time){return .2+.02*(time-1);};
    for(double pt:{t-.01,t+.01}) moving.push_pose(pt,y(pt)/rad,p(pt)/rad);
    yaw=y(t); pitch=p(t);
    check(moving.observe(scene.image(initial+target_velocity*(t-1),yaw,pitch),t),"moving target observation failed");
    output=moving.predict(t+.01); check(output.valid,"moving prediction failed");
  }
  near(output.horizon_ms,60,1e-6,"horizon includes measured host age and configured additional delay");
  check((output.position-(initial+target_velocity*(t-1+.06))).norm()<.02,"future position must include target movement");
  check((output.velocity-target_velocity).norm()<.05,"moving camera must not bias estimated world velocity");
  const Eigen::Matrix3d wg=scene.pose(yaw+.06*.01,pitch+.02*.01);
  const Eigen::Vector3d pc=scene.camera_gimbal.transpose()*(wg.transpose()*output.position-scene.translation);
  const Eigen::Vector3d vc=scene.camera_gimbal.transpose()*wg.transpose()*target_velocity;
  const double expected_yaw=.8*(pc.z()*vc.x()-pc.x()*vc.z())/(pc.z()*pc.z()+pc.x()*pc.x())/rad;
  const double expected_pitch=.8*(pc.z()*vc.y()-pc.y()*vc.z())/(pc.z()*pc.z()+pc.y()*pc.y())/rad;
  near(output.yaw_ff,expected_yaw,.3,"feedforward must contain target-only angular velocity");
  near(output.pitch_ff,expected_pitch,.3,"pitch target-only angular velocity");
  moving.set_feedforward_gain(0);
  check(moving.predict(t+.01).yaw_ff==0,"feedforward can be disabled for comparison");
  moving.push_pose(t+.3,0,0);
  check(!moving.predict(t+.3).valid,"expired observation must not keep commanding");
  std::cout<<"PASS synthetic moving armor: frame-age prediction, velocity feedforward, independent pose compensation\n";

  // Noisy corners + rotating camera, fixed armor: quantify false feedforward.
  SingleArmorTracking noisy(config);
  std::mt19937 rng(27);
  std::normal_distribution<float> noise(0,.3f);
  double false_ff_squared=0;
  for(int i=0;i<150;++i) {
    const double t=5+i*.02;
    auto y=[](double time){return .1+.04*std::sin(time);};
    auto p=[](double time){return .2+.02*std::sin(time);};
    for(double pt:{t-.01,t+.01}) noisy.push_pose(pt,y(pt)/rad,p(pt)/rad);
    auto armor=scene.image(initial,y(t),p(t));
    for(auto & point:armor.points) {point.x+=noise(rng); point.y+=noise(rng);}
    check(noisy.observe(armor,t),"ordinary subpixel corner noise must remain trackable");
    auto out=noisy.predict(t+.01); check(out.valid,"noisy fixed armor should have a valid estimate");
    if(i>30) false_ff_squared+=out.yaw_ff*out.yaw_ff+out.pitch_ff*out.pitch_ff;
  }
  const auto false_ff_rms=std::sqrt(false_ff_squared/119);
  check(false_ff_rms<1.5,"ordinary corner noise must not become high target angular velocity");
  std::cout<<"PASS noisy stationary armor: false feedforward RMS="<<false_ff_rms<<" deg/s\n";

  // Positive latency estimates are subtracted from host timestamps, never added.
  auto modified=YAML::LoadFile(config);
  modified["gimbal1_tracking"]["imu_latency_ms"]=12;
  modified["gimbal1_tracking"]["camera_latency_ms"]=7;
  modified["gimbal1_tracking"]["additional_prediction_ms"]=15;
  const auto temp=std::filesystem::temp_directory_path()/("ict27_tracking_offsets_"+std::to_string(getpid())+".yaml");
  {std::ofstream file(temp); file<<modified;}
  SingleArmorTracking offset(temp.string());
  std::filesystem::remove(temp);
  for(int i=0;i<80;++i) {
    const double t=1+i*.02;
    auto y=[](double time){return .1+.05*(time-1);};
    auto p=[](double time){return .2+.03*(time-1);};
    for(double pt:{t-.01,t+.01}) offset.push_pose(pt+.012,y(pt)/rad,p(pt)/rad);
    check(offset.observe(scene.image(initial+target_velocity*(t-1),y(t),p(t)),t+.007),"latency-adjusted observation");
    output=offset.predict(t+.025); check(output.valid,"latency-adjusted prediction");
    near(output.horizon_ms,40,1e-6,"corrected image time plus measured age plus extra prediction");
    if(i>50) check((output.position-(initial+target_velocity*(t-1+.04))).norm()<.02,
                   "timestamp latency sign must not bias future position");
  }
  offset.reset_epoch();
  check(!offset.predict(3).valid && offset.predict(3).reason=="no_observation", "enable epoch clears motion and pose references");
  std::cout<<"PASS independent host image/IMU latency offsets and additional prediction\n";

  SingleArmorTracking bounded(config); bounded.set_prediction_delay_ms(100);
  const Eigen::Vector3d close=scene.pose(.1,.2)*(scene.camera_gimbal*Eigen::Vector3d(.1,-.05,.5)+scene.translation);
  ArmorPrediction fast;
  for(int i=0;i<20;++i) {
    const double time=1+i*.02;
    for(double pt:{time-.01,time+.01}) bounded.push_pose(pt,.1/rad,.2/rad);
    check(bounded.observe(scene.image(close+Eigen::Vector3d(0,.8*(time-1),0),.1,.2),time),"fast near-field observation");
    fast=bounded.predict(time+.01);
    check(fast.valid,"bounded near-field prediction");
    check(std::abs(fast.yaw_lead)<=2 && std::abs(fast.pitch_lead)<=2,
          "prediction lead remains bounded under high angular speed");
    check(std::abs(fast.yaw_ff)<=30 && std::abs(fast.pitch_ff)<=30,
          "velocity feedforward remains bounded under high angular speed");
  }
  check(fast.augmentation_valid,"consistent fast observations enable augmentation");
  std::cout<<"PASS fast near-field target: bounded prediction lead and feedforward\n";

  SingleArmorTracking missing(config);
  check(!missing.observe(scene.image(initial,.1,.2),1),"missing pose cannot silently fall back to pixels");
  check(missing.predict(1).reason=="frame_pose_unavailable","missing-pose diagnostics");
  missing.push_pose(.99,.1/rad,.2/rad); missing.push_pose(1.01,.1/rad,.2/rad);
  auto bad=scene.image(initial,.1,.2); bad.points.resize(3);
  check(!missing.observe(bad,1) && missing.reason()=="pnp_rejected","invalid corners cannot enter motion filter");
  bad=scene.image(initial,.1,.2); bad.points.assign(4,{100,100});
  check(!missing.observe(bad,1),"degenerate planar pose must be rejected");
  bad=scene.image(initial,.1,.2); bad.points[0].x=std::numeric_limits<float>::quiet_NaN();
  check(!missing.observe(bad,1),"nonfinite corner must be rejected");
  // Actual field-sized quadrilateral: >4px residual is normal at this scale.
  SingleArmorTracking field(config);
  field.push_pose(.99,0,0); field.push_pose(1.01,0,0);
  Armor field_armor(1,3,.99,cv::Rect{},{{1042,1031},{1765,1069},{1768,1345},{1044,1317}});
  field_armor.type=ArmorType::small;
  check(field.observe(field_armor,1,cv::Size(3088,2064)),"field-sized PnP must use scale-aware quality limits");
  check(field.predict(1.01).reprojection_px>4,"fixture covers previously rejected real observations");
  field_armor.points[0].x=-1;
  check(!field.observe(field_armor,1.02,cv::Size(3088,2064)),"clipped model geometry cannot augment control");
  auto fallback=augment_pixel_correction(4,-3,field.predict(1.02));
  check(fallback.yaw==4 && fallback.pitch==-3 && fallback.yaw_ff==0 && fallback.pitch_ff==0,
        "rejected geometry must preserve baseline pixel tracking exactly");
  ArmorPrediction invalid; invalid.valid=true; invalid.augmentation_valid=false;
  check(!augment_pixel_correction(4,-3,invalid).augmented,"warmup leaves baseline intact");
  invalid.augmentation_valid=true; invalid.yaw_lead=.2; invalid.pitch_lead=-.1; invalid.yaw_ff=10;
  const auto combined=augment_pixel_correction(4,-3,invalid);
  near(combined.yaw,4.2,1e-9,"model contributes only its lead, not absolute filtered position");
  near(combined.pitch,-3.1,1e-9,"pitch lead preserves observed position baseline");
  invalid.yaw_lead=std::numeric_limits<double>::quiet_NaN();
  check(!augment_pixel_correction(4,-3,invalid).augmented,"nonfinite augmentation cannot stop baseline");
  tools::visual_tracking::Axis axis(4,10,16,60,3);
  near(axis.step(0,0,.04,-10),-.4,1e-9,"feedforward must continue at the optical center for a moving target");
  near(axis.step(0,0,.04,0),0,1e-9,"stationary target keeps the deadband");
  near(axis.step(100,10,.04,40),2.4,1e-9,"feedback plus feedforward respects rate cap");
  std::cout<<"PASS invalid observations and relative increment adapter (center feedforward, sign, caps)\n";
  return 0;
} catch(const std::exception & e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }

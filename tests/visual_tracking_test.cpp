#include "tools/visual_tracking.hpp"
#include "tools/thread_safe_queue.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
#include <thread>

using tools::visual_tracking::Axis;
using tools::visual_tracking::TargetTracker;
using tools::visual_tracking::Observation;
void check(bool value, const char * message) {
  if (!value) throw std::runtime_error(message);
}
void near(double a, double b, const char * message) { check(std::abs(a-b) < 1e-9, message); }
// Constant visual error: same total correction across rates and alternating-axis schedules.
double total(int hz, bool alternating) {
  Axis axis(2, 10, 16, 30, 10);
  double sum = 0;
  for (int i=0; i<hz; ++i) sum += axis.step(100, 3, 1.0/hz);
  if (alternating) {
    sum = 0;
    for (int i=0; i<hz/2; ++i) sum += axis.step(100, 3, 2.0/hz);
  }
  return sum;
}
int main() try {
  check(tools::visual_tracking::fresh_frame(40,120), "current image is eligible");
  check(!tools::visual_tracking::fresh_frame(121,120), "old image cannot generate commands");
  check(!tools::visual_tracking::fresh_frame(-1,120), "future timestamp cannot generate commands");
  check(!tools::visual_tracking::fresh_frame(std::numeric_limits<double>::quiet_NaN(),120), "invalid timestamp");
  near(tools::visual_tracking::command_speed(.02,.04,30),1,"tiny correction uses protocol minimum speed");
  near(tools::visual_tracking::command_speed(-.2,.05,30),4,"negative increment speed is positive and proportional");
  near(tools::visual_tracking::command_speed(1,.02,30),30,"large correction respects speed cap");
  near(tools::visual_tracking::command_speed(.4,.05,30)*.05,.4,"unsaturated command spans integration interval");
  near(tools::visual_tracking::command_speed(0,0,30),1,"invalid interval fallback");
  for (int hz : {20, 30, 40, 50}) {
    near(total(hz, false), 6, "rate changes must not multiply correction");
    near(total(hz, true), 6, "each axis must integrate its own interval");
  }
  Axis axis(2, 10, 16, 30, 1);
  near(axis.step(15, 1, .05), 0, "noise inside hysteresis must not start motion");
  check(axis.step(17, 1, .05)>0, "leave outer deadband");
  check(axis.step(12, 1, .05)>0, "remain active in hysteresis band");
  near(axis.step(9, 1, .05), 0, "stop at inner boundary");
  near(axis.step(-12, -1, .05), 0, "small sign-changing noise must not restart");
  check(axis.step(-17, -1, .05)<0, "large reverse error can correct immediately");
  near(axis.step(100, 100, 3), 1, "stall must not create a catch-up jump");
  near(axis.step(100, 1, 3), .2, "elapsed time is capped at 100ms");
  near(axis.step(100, 1, std::numeric_limits<double>::quiet_NaN()), 0, "invalid time");
  axis.reset(); near(axis.step(12, 1, .05),0,"reset cannot retain hysteresis");
  Axis pitch(1.2,10,16,30,1), yaw(2,10,16,30,1);
  check(pitch.step(100,2,.05)<yaw.step(100,2,.05), "pitch has independent lower gain");

  TargetTracker tracker(250,.02);
  std::vector<Observation> target{{100,100,.8,3}};
  check(tracker.update(target,1)<0,"acquire frame 1");
  check(tracker.update(target,1.02)<0,"acquire frame 2");
  check(tracker.update(target,1.04)==0,"acquire frame 3");
  std::vector<Observation> distractor{{102,102,.5,3},{700,700,.99,3}};
  check(tracker.update(distractor,1.06)==0,"confidence change must not switch target");
  check(tracker.x()>100 && tracker.x()<102,"light time-based center filtering");
  check(tracker.update({{700,700,.99,3}},1.08)<0,"one-frame outlier must not command");
  check(tracker.update(target,1.10)==0,"recover original target without following outlier");
  check(tracker.update({{700,700,.9,3}},1.12)<0,"jump confirmation 1");
  check(tracker.update({{700,700,.9,3}},1.14)<0,"jump confirmation 2");
  check(tracker.update({{700,700,.9,3}},1.16)==0,"sustained new position reacquires");
  near(tracker.x(),700,"reacquisition must discard old filter position");
  check(tracker.update({},1.18)<0,"missing target cannot command");
  check(tracker.update(target,2)<0,"long gap needs reacquisition");
  check(tracker.update(target,2.02)<0,"long gap confirmation 2");
  check(tracker.update(target,2.04)==0,"long gap confirmation 3");
  check(tracker.update(target,2.04)<0,"duplicate image timestamp cannot command");

  tools::ThreadSafeQueue<int,true> latest(1);
  latest.push(1); latest.push(2); latest.push(3);
  int frame=0;
  check(latest.pop_for(frame,std::chrono::milliseconds(20)) && frame==3,
    "slow consumer must receive latest frame, not oldest queued frame");
  check(!latest.pop_for(frame,std::chrono::milliseconds(20)),"no-frame wait is bounded");
  std::cout << "PASS: rate invariance, alternating axes, deadband hysteresis, dt cap, "
               "target association/reacquisition, center filter, latest-frame queue\n";
  return 0;
} catch (const std::exception & e) { std::cerr<<e.what()<<'\n'; return 1; }

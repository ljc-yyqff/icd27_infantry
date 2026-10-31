#include "io/gimbal1/gimbal1.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <stdexcept>

void check(bool b,const char * msg) { if(!b) throw std::runtime_error(msg); }
int main() try {
  const int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
  check(master>=0 && grantpt(master)==0 && unlockpt(master)==0,"create pseudo terminal");
  const auto file=std::filesystem::temp_directory_path()/("ict27_pose_serial_"+std::to_string(getpid())+".yaml");
  { std::ofstream cfg(file); cfg<<"com_port: '"<<ptsname(master)<<"'\nbaudrate: 115200\n"; }
  {
    io::Gimbal1 gimbal(file.string());
    io::Gimbal1Feedback newest;
    for(int batch=0;batch<12;++batch) {
      std::string lines;
      for(int i=0;i<16;++i) {
        const auto value=std::to_string(batch*16+i);
        lines+="0,0,0,0,"+value+","+value+",20,0,0\r\n";
      }
      check(write(master,lines.data(),lines.size())==static_cast<ssize_t>(lines.size()),"write telemetry burst");
      for(int wait=0;wait<20;++wait) {
        gimbal.poll_feedback(newest);
        if(newest.imu_yaw_deg==batch*16+15) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      check(newest.imu_yaw_deg==batch*16+15,"poll drains full telemetry burst");
    }
    const auto samples=gimbal.take_feedback_samples();
    check(samples.size()==128,"pose sample queue must be bounded");
    check(samples.front().imu_yaw_deg==64 && samples.back().imu_yaw_deg==191,"retain latest samples in wire order");
    for(size_t i=1;i<samples.size();++i)
      check(samples[i].received_at>=samples[i-1].received_at,"retain host receive timestamps");
    check(gimbal.take_feedback_samples().empty(),"each pose sample consumed only once");
    char sent[128];
    check(read(master,sent,sizeof(sent))<0,"pose collection sends no commands");
  }
  close(master); std::filesystem::remove(file);
  std::cout<<"PASS pseudo-serial pose bursts, bounded ordered samples, timestamps, no outgoing command\n";
  return 0;
} catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}

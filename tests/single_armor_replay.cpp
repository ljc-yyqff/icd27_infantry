// Offline replay of a vision CSV and matching wire trace; never opens hardware.
#include "tasks/auto_aim/single_armor_tracking.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>

using Row=std::map<std::string,std::string>;
std::vector<std::string> split(const std::string & s) {
  std::vector<std::string> v;std::istringstream in(s);std::string part;
  while(std::getline(in,part,',')) v.push_back(part);
  return v;
}
double number(const Row & r,const std::string & k) {return std::stod(r.at(k));}
struct Event {double time;bool enable;std::vector<double> values;};
int main(int argc,char ** argv) try {
  if(argc!=4) throw std::runtime_error("usage: single_armor_replay config vision.csv wire.log");
  std::ifstream csv(argv[2]),wire(argv[3]);
  if(!csv||!wire) throw std::runtime_error("cannot open replay input");
  std::string line;std::getline(csv,line);const auto header=split(line);
  std::vector<Row> rows;
  while(std::getline(csv,line)) {
    const auto fields=split(line);if(fields.size()!=header.size()) continue;
    Row r;for(size_t i=0;i<header.size();++i) r[header[i]]=fields[i];rows.push_back(r);
  }
  std::vector<Event> events;std::vector<double> tx_times;
  const std::regex pattern(R"(\+([0-9.]+)s (.*))");std::smatch match;
  while(std::getline(wire,line)) {
    if(!std::regex_search(line,match,pattern)) continue;
    const double t=std::stod(match[1]);const std::string body=match[2];
    if(body.find("TX_RESULT") == 0 && (body.find("command=yr ")!=std::string::npos || body.find("command=pr ")!=std::string::npos))
      tx_times.push_back(t);
    if(body.find("TX_REQUEST ge") == 0) events.push_back({t,true,{}});
    if(body.find("RX ")!=0 || body.find("RX #")==0) continue;
    const auto fields=split(body.substr(3));if(fields.size()!=9) continue;
    std::vector<double> values;try {for(const auto & f:fields) values.push_back(std::stod(f));}catch(...) {continue;}
    events.push_back({t,false,values});
  }
  std::vector<double> offset_samples;size_t tx=0;
  for(const auto & r:rows) if(r.at("axis")!="none") {
    if(tx>=tx_times.size()) break;
    if(offset_samples.size()<20) offset_samples.push_back(number(r,"write_done_t_s")-tx_times[tx]);
    ++tx;
  }
  if(offset_samples.empty()) throw std::runtime_error("no common motion events for time alignment");
  std::sort(offset_samples.begin(),offset_samples.end());const double offset=offset_samples[offset_samples.size()/2];
  auto_aim::SingleArmorTracking tracker(argv[1]);
  size_t event=0, active=0, targets=0, valid=0, augmented=0, base_preserved=0, old_valid=0;
  std::map<std::string,int> reasons;double max_lead=0,max_ff=0;
  for(const auto & r:rows) {
    const double now=number(r,"decision_t_s");
    while(event<events.size() && events[event].time+offset<=now) {
      const auto & e=events[event++];
      if(e.enable) tracker.reset_epoch();
      else tracker.push_pose(e.time+offset,e.values[5],e.values[4]);
    }
    if(r.at("sending")!="1") continue;
    ++active;
    if(r.at("target_valid")!="1" || r.at("fresh")!="1") continue;
    ++targets;old_valid+=r.at("prediction_valid")=="1";
    std::vector<cv::Point2f> points;
    for(int i=0;i<4;++i) points.push_back({float(number(r,"corner"+std::to_string(i)+"_x")),float(number(r,"corner"+std::to_string(i)+"_y"))});
    auto_aim::Armor armor(1,3,.99,cv::Rect{},points);
    armor.type=static_cast<auto_aim::ArmorType>(std::stoi(r.at("armor_type")));
    armor.name=static_cast<auto_aim::ArmorName>(std::stoi(r.at("armor_name")));
    armor.color=static_cast<auto_aim::Color>(std::stoi(r.at("armor_color")));
    tracker.observe(armor,number(r,"frame_t_s"),cv::Size(3088,2064));
    const auto p=tracker.predict(now);++reasons[p.reason];valid+=p.valid;augmented+=p.augmentation_valid;
    const double y=number(r,"yaw_error_deg"),pitch=number(r,"pitch_error_deg");
    const auto c=auto_aim::augment_pixel_correction(y,pitch,p);
    if(!p.valid || !p.augmentation_valid) {
      if(c.yaw!=y || c.pitch!=pitch || c.yaw_ff!=0 || c.pitch_ff!=0)
        throw std::runtime_error("invalid model changed/stalled the baseline pixel loop");
      ++base_preserved;
    }
    max_lead=std::max({max_lead,std::abs(p.yaw_lead),std::abs(p.pitch_lead)});
    max_ff=std::max({max_ff,std::abs(p.yaw_ff),std::abs(p.pitch_ff)});
  }
  std::cout<<"active_frames="<<active<<" target_frames="<<targets<<" previous_model_valid="<<old_valid
    <<" replay_model_valid="<<valid<<" augmentation_ready="<<augmented<<" baseline_preserved="<<base_preserved
    <<" max_lead_deg="<<max_lead<<" max_ff_deg_s="<<max_ff<<'\n';
  for(const auto & pair:reasons) std::cout<<pair.first<<'='<<pair.second<<'\n';
  return 0;
} catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}

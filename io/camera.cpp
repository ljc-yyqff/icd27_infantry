#include "camera.hpp"

#include <stdexcept>

#include "daheng/daheng.hpp"
#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  rotate_180_ = yaml["rotate_180"] ? yaml["rotate_180"].as<bool>() : false;

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else if (camera_name == "daheng") {
    auto gain = tools::read<double>(yaml, "gain");
    auto white_balance_auto =
      yaml["white_balance_auto"] ? yaml["white_balance_auto"].as<std::string>() : "Once";
    auto swap_rb = yaml["swap_rb"] ? yaml["swap_rb"].as<bool>() : false;
    camera_ = std::make_unique<Daheng>(exposure_ms, gain, white_balance_auto, swap_rb);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  if (rotate_180_) cv::rotate(img, img, cv::ROTATE_180);
}

}  // namespace io

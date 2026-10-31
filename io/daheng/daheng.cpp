#include "daheng.hpp"

#include <stdexcept>
#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
namespace
{
bool check_status(GX_STATUS status, const std::string & action)
{
  if (status == GX_STATUS_SUCCESS) return true;
  tools::logger()->warn("{} failed: {:#x}", action, static_cast<int>(status));
  return false;
}
}  // namespace

Daheng::Daheng(double exposure_ms, double gain, const std::string & white_balance_auto, bool swap_rb)
: exposure_us_(exposure_ms * 1e3),
  gain_(gain),
  white_balance_auto_(white_balance_auto),
  swap_rb_(swap_rb),
  handle_(nullptr),
  quit_(false),
  ok_(false),
  queue_(1)
{
  auto status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("GXInitLib failed!");
  }

  // Startup must fail visibly rather than leave the caller waiting forever for a frame.
  try {
    open();
  } catch (...) {
    close();
    GXCloseLib();
    throw;
  }

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng's daemon thread started.");

    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (capture_thread_.joinable()) capture_thread_.join();

      close();
      try_open();
    }

    tools::logger()->info("Daheng's daemon thread stopped.");
  }};
}

Daheng::~Daheng()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  GXCloseLib();
  tools::logger()->info("Daheng destructed.");
}

void Daheng::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  if (!queue_.pop_for(data, 2s)) {
    throw std::runtime_error(
      "Daheng: no camera frame for 2 seconds; check USB connection and camera ownership");
  }

  img = data.img;
  timestamp = data.timestamp;
}

void Daheng::open()
{
  uint32_t device_num = 0;
  auto status = GXUpdateAllDeviceList(&device_num, 1000);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("GXUpdateAllDeviceList failed!");
  }

  if (device_num == 0) {
    throw std::runtime_error("Not found Daheng camera!");
  }

  status = GXOpenDeviceByIndex(1, &handle_);
  if (status != GX_STATUS_SUCCESS) {
    handle_ = nullptr;
    char error_text[1024] = {};
    size_t error_size = sizeof(error_text);
    GX_STATUS error_code = status;
    GXGetLastError(&error_code, error_text, &error_size);
    throw std::runtime_error(
      "GXOpenDeviceByIndex failed (status=" + std::to_string(status) + "): " + error_text +
      "; check whether another camera program/service is using the device");
  }

  GXSetAcqusitionBufferNumber(handle_, 5);
  set_enum_value("AcquisitionMode", "Continuous");
  set_enum_value("TriggerMode", "Off");
  set_enum_value("ExposureAuto", "Off");
  set_enum_value("GainAuto", "Off");
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);

  GX_NODE_ACCESS_MODE white_balance_access;
  status = GXGetNodeAccessMode(handle_, "BalanceWhiteAuto", &white_balance_access);
  if (status == GX_STATUS_SUCCESS && white_balance_access == GX_NODE_ACCESS_MODE_RW) {
    set_enum_value("BalanceWhiteAuto", white_balance_auto_);
  }

  status = GXStreamOn(handle_);
  if (status != GX_STATUS_SUCCESS) {
    close();
    throw std::runtime_error("GXStreamOn failed!");
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng's capture thread started.");

    ok_ = true;
    PGX_FRAME_DATA_EX frame = nullptr;

    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      auto status = GXDQBufEx(handle_, &frame, 100);
      if (status != GX_STATUS_SUCCESS) {
        if (status != GX_STATUS_TIMEOUT) {
          tools::logger()->warn("GXDQBufEx failed: {:#x}", static_cast<int>(status));
          break;
        }
        continue;
      }

      if (frame->nStatus == GX_FRAME_STATUS_SUCCESS) {
        auto timestamp = std::chrono::steady_clock::now();

        try {
          queue_.push({to_bgr(*frame), timestamp});
        } catch (const std::exception & e) {
          tools::logger()->warn("Daheng pixel convert failed: {}", e.what());
        }
      } else {
        tools::logger()->warn("Daheng abnormal frame status: {}", frame->nStatus);
      }

      status = GXQBufEx(handle_, frame);
      if (status != GX_STATUS_SUCCESS) {
        tools::logger()->warn("GXQBufEx failed: {:#x}", static_cast<int>(status));
        break;
      }
    }

    ok_ = false;
    tools::logger()->info("Daheng's capture thread stopped.");
  }};

  tools::logger()->info("Daheng opened.");
}

void Daheng::try_open()
{
  try {
    open();
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void Daheng::close()
{
  if (handle_ == nullptr) return;

  GXStreamOff(handle_);
  auto status = GXCloseDevice(handle_);
  check_status(status, "GXCloseDevice");
  handle_ = nullptr;
}

void Daheng::set_float_value(const std::string & name, double value)
{
  auto status = GXSetFloatValue(handle_, name.c_str(), value);
  check_status(status, "GXSetFloatValue(\"" + name + "\", " + std::to_string(value) + ")");
}

void Daheng::set_enum_value(const std::string & name, const std::string & value)
{
  auto status = GXSetEnumValueByString(handle_, name.c_str(), value.c_str());
  check_status(status, "GXSetEnumValueByString(\"" + name + "\", \"" + value + "\")");
}

cv::Mat Daheng::correct_color(cv::Mat img) const
{
  if (!swap_rb_) return img;

  cv::Mat corrected;
  cv::cvtColor(img, corrected, cv::COLOR_BGR2RGB);
  return corrected;
}

cv::Mat Daheng::to_bgr(const GX_FRAME_DATA_EX & frame) const
{
  auto * img_buf = reinterpret_cast<unsigned char *>(frame.pImgBuf);

  if (img_buf == nullptr) {
    throw std::runtime_error("empty frame buffer");
  }

  const cv::Size size(frame.nWidth, frame.nHeight);
  const auto pixel_format = static_cast<GX_PIXEL_FORMAT_ENTRY>(frame.nPixelFormat);

  if (pixel_format == GX_PIXEL_FORMAT_BGR8) {
    return correct_color(cv::Mat(size, CV_8UC3, img_buf).clone());
  }

  if (pixel_format == GX_PIXEL_FORMAT_RGB8) {
    cv::Mat bgr;
    cv::cvtColor(cv::Mat(size, CV_8UC3, img_buf), bgr, cv::COLOR_RGB2BGR);
    return correct_color(bgr);
  }

  if (pixel_format == GX_PIXEL_FORMAT_MONO8) {
    cv::Mat bgr;
    cv::cvtColor(cv::Mat(size, CV_8UC1, img_buf), bgr, cv::COLOR_GRAY2BGR);
    return correct_color(bgr);
  }

  static const std::unordered_map<GX_PIXEL_FORMAT_ENTRY, cv::ColorConversionCodes> bayer_map = {
    {GX_PIXEL_FORMAT_BAYER_GR8, cv::COLOR_BayerGR2BGR},
    {GX_PIXEL_FORMAT_BAYER_RG8, cv::COLOR_BayerRG2BGR},
    {GX_PIXEL_FORMAT_BAYER_GB8, cv::COLOR_BayerGB2BGR},
    {GX_PIXEL_FORMAT_BAYER_BG8, cv::COLOR_BayerBG2BGR},
  };

  auto iter = bayer_map.find(pixel_format);
  if (iter == bayer_map.end()) {
    throw std::runtime_error("unsupported pixel format: " + std::to_string(frame.nPixelFormat));
  }

  cv::Mat bgr;
  cv::cvtColor(cv::Mat(size, CV_8UC1, img_buf), bgr, iter->second);
  return correct_color(bgr);
}

}  // namespace io

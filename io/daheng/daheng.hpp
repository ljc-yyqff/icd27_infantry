#ifndef IO__DAHENG_HPP
#define IO__DAHENG_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "GxIAPI.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Daheng : public CameraBase
{
public:
  Daheng(double exposure_ms, double gain, const std::string & white_balance_auto, bool swap_rb);
  ~Daheng() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;
  std::string white_balance_auto_;
  bool swap_rb_;
  GX_DEV_HANDLE handle_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  std::atomic<bool> quit_;
  std::atomic<bool> ok_;
  tools::ThreadSafeQueue<CameraData> queue_;

  void open();
  void try_open();
  void close();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, const std::string & value);
  cv::Mat correct_color(cv::Mat img) const;
  cv::Mat to_bgr(const GX_FRAME_DATA_EX & frame) const;
};

}  // namespace io

#endif  // IO__DAHENG_HPP

#include "img_tools.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace tools
{
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  cv::circle(img, point, radius, color, -1);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  std::vector<cv::Point> int_points(points.begin(), points.end());
  draw_points(img, int_points, color, thickness);
}

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

void draw_fps(cv::Mat & img)
{
  static auto last = std::chrono::steady_clock::now();
  static double fps = 0.0;

  const auto now = std::chrono::steady_clock::now();
  const auto dt = std::chrono::duration<double>(now - last).count();
  last = now;

  if (dt > 1e-6) fps = 1.0 / dt;

  char text[32];
  std::snprintf(text, sizeof(text), "FPS: %.1f", fps);

  const double font_scale = 0.8;
  const int thickness = 2;
  int baseline = 0;
  const auto text_size =
    cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
  const cv::Point origin(std::max(10, img.cols - text_size.width - 12), 32);

  cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, {0, 0, 0}, thickness + 2);
  cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, {255, 255, 255}, thickness);
}

}  // namespace tools

#include <fmt/core.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path    | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_only     | 输出文件夹路径   }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");
  std::filesystem::create_directories(output_folder);

  io::Camera camera(config_path);

  tools::logger()->info("默认标定板尺寸为10列7行");
  tools::logger()->info("Press s to save image, q to quit.");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int count = 0;

  while (true) {
    camera.read(img, timestamp);

    auto drawing = img.clone();
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, cv::Size(10, 7), centers_2d);
    cv::drawChessboardCorners(drawing, cv::Size(10, 7), centers_2d, success);
    tools::draw_text(drawing, success ? "detected" : "not detected", {40, 40}, {0, 0, 255});
    cv::resize(drawing, drawing, {}, 0.5, 0.5);

    cv::imshow("Press s to save, q to quit", drawing);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key != 's') continue;

    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    cv::imwrite(img_path, img);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  return 0;
}

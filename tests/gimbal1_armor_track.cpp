#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal1/gimbal1.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/single_armor_tracking.hpp"
#include "tools/exiter.hpp"
#include "tools/signal_stop_action.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/visual_tracking.hpp"

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

cv::Point2f armor_center(const auto_aim::Armor & armor)
{
  if (armor.points.empty()) return armor.center;

  cv::Point2f center{0.0F, 0.0F};
  for (const auto & point : armor.points) center += point;
  return center * (1.0F / static_cast<float>(armor.points.size()));
}

void draw_crosshair(cv::Mat & image, const cv::Point2f & center, const cv::Scalar & color)
{
  constexpr int half_length = 28;
  cv::line(
    image, {static_cast<int>(center.x) - half_length, static_cast<int>(center.y)},
    {static_cast<int>(center.x) + half_length, static_cast<int>(center.y)}, color, 3);
  cv::line(
    image, {static_cast<int>(center.x), static_cast<int>(center.y) - half_length},
    {static_cast<int>(center.x), static_cast<int>(center.y) + half_length}, color, 3);
}
}  // namespace

const std::string keys =
  "{help h usage ? |                       | show help }"
  "{@config-path   | configs/infantry_daheng.yaml | yaml config path }"
  "{tradition t    | false                 | use the traditional detector instead of YOLO }"
  "{deadband       | 10                    | stop correcting inside this pixel radius }"
  "{deadband_exit  | 16                    | resume correction outside this pixel radius }"
  "{control        | false                 | open gimbal1 serial; starts disabled }"
  "{tracking       | pixel                 | pixel, shadow (compute predictions only), or predictive }"
  "{prediction_ms  |                       | additional execution delay estimate, milliseconds }"
  "{feedforward_gain |                     | override target angular velocity feedforward gain, 0..2 }"
  "{headless       | false                 | run without any GUI window or keyboard input }"
  "{auto_start     | false                 | send ge once on first fresh frame and begin tracking }"
  "{gain           |                       | removed; use yaw_gain/pitch_gain in 1/s }"
  "{yaw_gain       | 2.0                   | yaw correction rate gain, per second }"
  "{pitch_gain     | 1.2                   | pitch correction rate gain, per second }"
  "{max_frame_age_ms | 120                 | maximum host frame age when generating commands }"
  "{target_jump_px | 250                   | association distance in original pixels }"
  "{center_tau_ms  | 20                    | center filter time constant; 0 disables smoothing }"
  "{min_step       | 0.0                   | must be 0; no forced minimum increment }"
  "{max_step       | 1.0                  | maximum relative angle per command, degrees }"
  "{speed          | 30.0                   | maximum adaptive command speed, degrees/second }"
  "{control_rate   | 40.0                   | maximum command rate, Hz }"
  "{max_active_seconds | 0.0              | pause after this many seconds; 0 disables timeout }";

int main(int argc, char * argv[])
try {
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto use_tradition = cli.get<bool>("tradition");
  const auto deadband_px = cli.get<double>("deadband");
  const auto control_mode = cli.get<bool>("control");
  const auto headless = cli.get<bool>("headless");
  const auto auto_start = cli.get<bool>("auto_start");
  const auto tracking_mode = cli.get<std::string>("tracking");
  const std::optional<double> prediction_ms = cli.has("prediction_ms") ?
    std::optional<double>(cli.get<double>("prediction_ms")) : std::nullopt;
  const std::optional<double> feedforward_gain = cli.has("feedforward_gain") ?
    std::optional<double>(cli.get<double>("feedforward_gain")) : std::nullopt;
  double yaw_gain = cli.get<double>("yaw_gain");
  double pitch_gain = cli.get<double>("pitch_gain");
  const auto deadband_exit_px = cli.get<double>("deadband_exit");
  const auto max_frame_age_ms = cli.get<double>("max_frame_age_ms");
  const auto target_jump_px = cli.get<double>("target_jump_px");
  const auto center_tau_ms = cli.get<double>("center_tau_ms");
  const auto min_step_deg = cli.get<double>("min_step");
  const auto max_step_deg = cli.get<double>("max_step");
  const auto command_speed_deg_s = cli.get<double>("speed");
  const auto control_rate_hz = cli.get<double>("control_rate");
  const auto max_active_seconds = cli.get<double>("max_active_seconds");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  if (tracking_mode != "pixel" && tracking_mode != "shadow" && tracking_mode != "predictive")
    throw std::invalid_argument("tracking must be pixel, shadow or predictive");
  if (tracking_mode=="pixel" && (prediction_ms || feedforward_gain))
    throw std::invalid_argument("prediction options require --tracking=shadow or --tracking=predictive");
  if (auto_start && !control_mode)
    throw std::invalid_argument("--auto_start requires --control=true");
  if (!std::isfinite(deadband_px) || deadband_px < 0.0) {
    throw std::invalid_argument("deadband must be a finite non-negative pixel value");
  }
  if (cli.has("gain"))
    throw std::invalid_argument("--gain was per-command and has been replaced; omit it or use --yaw_gain=2 --pitch_gain=1.2 (per second)");
  if (!std::isfinite(yaw_gain) || yaw_gain <= 0 || !std::isfinite(pitch_gain) || pitch_gain <= 0 ||
      !std::isfinite(deadband_exit_px) || deadband_exit_px <= deadband_px ||
      !std::isfinite(max_frame_age_ms) || max_frame_age_ms <= 0 ||
      !std::isfinite(target_jump_px) || target_jump_px <= 0 ||
      !std::isfinite(center_tau_ms) || center_tau_ms < 0 || !std::isfinite(min_step_deg) ||
      min_step_deg != 0.0 || !std::isfinite(max_step_deg) || max_step_deg <= 0.0 ||
      min_step_deg > max_step_deg || !std::isfinite(command_speed_deg_s) ||
      (command_speed_deg_s < 1.0 || command_speed_deg_s > 120.0) || !std::isfinite(control_rate_hz) ||
      control_rate_hz <= 0.0 || control_rate_hz > 50.0 ||
      !std::isfinite(max_active_seconds) || max_active_seconds < 0.0) {
    throw std::invalid_argument(
      "axis gains/max_step/age/jump must be positive; deadband_exit > deadband; center_tau_ms >= 0; "
      "speed: 1..120 deg/s; control_rate: (0,50] Hz; max_active_seconds >= 0; min_step must be 0");
  }

  const auto yaml = YAML::LoadFile(config_path);
  if (!yaml["camera_matrix"]) throw std::runtime_error("config is missing camera_matrix");
  const auto matrix = yaml["camera_matrix"].as<std::vector<double>>();
  if (matrix.size() != 9) throw std::runtime_error("camera_matrix must contain 9 values");

  const double yaw_sign = yaml["gimbal1_yaw_direction"] ? yaml["gimbal1_yaw_direction"].as<double>() : -1;
  const double pitch_sign = yaml["gimbal1_pitch_direction"] ? yaml["gimbal1_pitch_direction"].as<double>() : -1;
  if (std::abs(yaw_sign) != 1 || std::abs(pitch_sign) != 1)
    throw std::invalid_argument("gimbal1 direction must be +1 or -1");
  const double fx = matrix[0];
  const double fy = matrix[4];
  const double cx = matrix[2];
  const double cy = matrix[5];
  if (fx <= 0.0 || fy <= 0.0) throw std::runtime_error("camera focal lengths must be positive");

  std::unique_ptr<auto_aim::SingleArmorTracking> motion_tracker;
  if (tracking_mode != "pixel") {
    motion_tracker = std::make_unique<auto_aim::SingleArmorTracking>(config_path);
    if (prediction_ms) motion_tracker->set_prediction_delay_ms(*prediction_ms);
    if (feedforward_gain) motion_tracker->set_feedforward_gain(*feedforward_gain);
    tools::logger()->info("[prediction] mode={}; two-axis IMU compensation assumes fixed roll; latency offsets must be calibrated", tracking_mode);
  }
  const double effective_control_rate = motion_tracker ?
    std::min(control_rate_hz,motion_tracker->command_rate_cap()) : control_rate_hz;
  if (motion_tracker) tools::logger()->info(
    "[prediction] requested rate={} Hz, effective cap={} Hz; reserve telemetry slots for current command-receipt firmware",
    control_rate_hz,effective_control_rate);
  tools::Exiter exiter;
  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, false);
  auto_aim::YOLO yolo(config_path, false);
  std::unique_ptr<io::Gimbal1> gimbal;
  // The existing boot-service command opts into manual signal-stop semantics.
  // A fault, target loss, stale telemetry or normal EOF does not trigger this.
  tools::SignalStopAction service_stop(exiter, control_mode && headless && auto_start, [&] {
    if (gimbal) {
      gimbal->disable();
      tools::logger()->info("[service stop] explicit SIGTERM/SIGINT: gd sent once");
    }
  });
  std::ofstream vision_log;
  if (control_mode) {
    gimbal = std::make_unique<io::Gimbal1>(config_path);
    const auto folder = std::filesystem::absolute("logs/gimbal1");
    std::filesystem::create_directories(folder);
    const auto path = folder / ("tracking_" + std::to_string(std::time(nullptr)) + "_" + std::to_string(getpid()) + ".log");
    gimbal->start_logging(path.string());
    auto vision_path = path;
    vision_path.replace_extension(".vision.csv");
    vision_log.open(vision_path);
    if (!vision_log) throw std::runtime_error("cannot open vision tracking log");
    vision_log << "frame,frame_t_s,decision_t_s,age_ms,raw_x,raw_y,filtered_x,filtered_y,"
                  "yaw_error_deg,pitch_error_deg,target_valid,fresh,sending,axis,increment_deg,axis_dt_s,"
                  "pitch_cmd,pitch_enc,yaw_cmd,yaw_enc,feedback_t_s,feedback_present,write_done_t_s,command_speed_deg_s,tracking_mode,prediction_valid,prediction_reason,"
                  "pose_extrapolated,horizon_ms,pnp_rms_px,world_x,world_y,world_z,velocity_x,velocity_y,velocity_z,"
                  "predicted_yaw_error,predicted_pitch_error,yaw_feedforward,pitch_feedforward,motion_samples,"
                  "frame_imu_yaw,frame_imu_pitch,current_imu_yaw,current_imu_pitch,"
                  "corner0_x,corner0_y,corner1_x,corner1_y,corner2_x,corner2_y,corner3_x,corner3_y,"
                  "armor_type,armor_name,armor_color,armor_confidence,augmentation_valid,yaw_lead_deg,pitch_lead_deg\n";
    vision_log << std::fixed << std::setprecision(6);
    tools::logger()->info("[vision trace] {}", vision_path.string());
    tools::logger()->info("[serial trace] {}", path.string());
    tools::logger()->info("[serial] port={} baud={} (8N1); motor responses are diagnostics only", gimbal->port(), gimbal->baudrate());
  }
  if (headless) std::cout << std::unitbuf;
  std::cout << (control_mode ? "gimbal1 armor tracking CONTROL mode\n"
                             : "gimbal1 armor tracking preview (DRY-RUN)\n")
            << (control_mode
                  ? (auto_start ? "Serial is open. Auto-start will send ge once on the first fresh frame.\n"
                                : "Serial is open. Press e to send ge and start tracking; motor state is not managed here.\n")
                  : "No serial port is opened and no gimbal command is transmitted.\n")
            << "Detector: " << (use_tradition ? "traditional" : "YOLO") << '\n'
            << fmt::format(
                 "Optical center=({:.2f}, {:.2f}) fx={:.2f} fy={:.2f}; deadband={:.1f}px\n",
                 cx, cy, fx, fy, deadband_px)
            << "yaw_error: positive means target is to the image right.\n"
            << "pitch_error: positive means target is below the optical center.\n"
            << (headless ? "Headless mode: stop with SIGTERM/SIGINT.\n" : control_mode
                  ? "Keys: e=enable/resume, s=stop new increments, d=disable, q=disable and quit.\n"
                  : "Press q to quit.\n");

  int frame_count = 0;
  auto last_frame = std::chrono::steady_clock::now();
  auto last_log = last_frame;
  auto last_prediction_log = last_frame;
  size_t target_generation = 0;
  auto last_command = last_frame;
  auto active_since = last_frame;
  bool tracking_active = false;
  bool auto_start_sent = false;
  bool send_yaw_next = true;
  tools::visual_tracking::TargetTracker target_tracker(target_jump_px, center_tau_ms / 1000.0);
  tools::visual_tracking::Axis yaw_axis(yaw_gain, deadband_px, deadband_exit_px, command_speed_deg_s, max_step_deg);
  tools::visual_tracking::Axis pitch_axis(pitch_gain, deadband_px, deadband_exit_px, command_speed_deg_s, max_step_deg);
  auto last_yaw = last_frame;
  auto last_pitch = last_frame;
  tools::logger()->info("[tracking] yaw_gain={} pitch_gain={} /s, deadband={}/{} px, rate<={} Hz, max_age={} ms",
    yaw_gain, pitch_gain, deadband_px, deadband_exit_px, effective_control_rate, max_frame_age_ms);
  std::chrono::steady_clock::time_point timestamp;

  io::Gimbal1Feedback serial_feedback;
  const auto poll_serial = [&] {
    if (!control_mode) return;
    try {
      gimbal->poll_feedback(serial_feedback);
      const auto samples = gimbal->take_feedback_samples();
      if (motion_tracker) for (const auto & sample : samples)
        motion_tracker->push_pose(std::chrono::duration<double>(sample.received_at.time_since_epoch()).count(),
                                 sample.imu_yaw_deg, sample.imu_pitch_deg);
    } catch (const std::exception & e) {
      tools::logger()->warn("[serial diagnostic] {}", e.what());
    }
  };
  while (!exiter.exit()) {
    poll_serial();
    cv::Mat image;
    camera.read(image, timestamp);
    if (exiter.exit() || image.empty()) break;

    std::list<auto_aim::Armor> armors;
    if (use_tradition)
      armors = detector.detect(image, frame_count);
    else
      armors = yolo.detect(image, frame_count);

    if (exiter.exit()) break;
    if (motion_tracker) poll_serial(); // Collect pose samples bracketing this image after inference.
    const cv::Point2f optical_center{static_cast<float>(cx), static_cast<float>(cy)};
    const auto detection_done = std::chrono::steady_clock::now();
    const double frame_age_ms = std::chrono::duration<double, std::milli>(detection_done - timestamp).count();
    const bool frame_fresh = tools::visual_tracking::fresh_frame(frame_age_ms, max_frame_age_ms);
    std::vector<tools::visual_tracking::Observation> observations;
    std::vector<const auto_aim::Armor *> candidates;
    for (const auto & armor : armors) {
      const auto center = armor_center(armor);
      if (!std::isfinite(center.x) || !std::isfinite(center.y)) continue;
      observations.push_back({center.x, center.y, armor.confidence,
        static_cast<int>(armor.color) * 100 + static_cast<int>(armor.name)});
      candidates.push_back(&armor);
    }
    const auto selected = target_tracker.update(frame_fresh ? observations :
      std::vector<tools::visual_tracking::Observation>{},
      std::chrono::duration<double>(timestamp.time_since_epoch()).count());
    const auto * target = selected >= 0 ? candidates[selected] : nullptr;
    auto_aim::ArmorPrediction prediction;
    if (motion_tracker) {
      if (target_generation != target_tracker.generation()) {
        motion_tracker->reset_motion();
        target_generation = target_tracker.generation();
      }
      prediction.reason = frame_fresh ? "no_target" : "stale_frame";
      if (target && frame_fresh) {
        motion_tracker->observe(*target, std::chrono::duration<double>(timestamp.time_since_epoch()).count(),image.size());
        prediction = motion_tracker->predict(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
      }
    }
    const cv::Point2f filtered_center{static_cast<float>(target_tracker.x()), static_cast<float>(target_tracker.y())};
    cv::Point2f raw_center{0, 0};
    // Preserve the rejected candidate in diagnostics too, to expose center jumps.
    if (!observations.empty()) {
      const auto best_raw = std::max_element(observations.begin(), observations.end(),
        [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
      raw_center = {static_cast<float>(best_raw->x), static_cast<float>(best_raw->y)};
    }

    draw_crosshair(image, optical_center, {255, 255, 0});
    tools::draw_text(image, "optical center", optical_center + cv::Point2f(35, -20), {255, 255, 0});

    std::string status = "NO TARGET - no command";
    cv::Scalar status_color{0, 0, 255};
    double error_x = 0.0;
    double error_y = 0.0;
    double yaw_error_deg = 0.0;
    double pitch_error_deg = 0.0;

    if (target != nullptr) {
      raw_center = armor_center(*target);
      const auto target_center = filtered_center;
      error_x = target_center.x - cx;
      error_y = target_center.y - cy;
      yaw_error_deg = std::atan(error_x / fx) * kRadToDeg;
      pitch_error_deg = std::atan(error_y / fy) * kRadToDeg;

      const bool in_deadband =
        std::abs(error_x) <= deadband_px && std::abs(error_y) <= deadband_px;
      status = in_deadband
                 ? "CENTERED (deadband)"
                 : (control_mode && tracking_active ? "TARGET - controlling"
                                                    : "TARGET - preview only");
      if (tracking_mode=="predictive" && !prediction.valid)
        status = "TARGET - pixel base; model unavailable";
      status_color = in_deadband ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 255, 255};

      tools::draw_points(image, target->points, {0, 255, 0}, 4);
      cv::circle(image, target_center, 10, {0, 0, 255}, -1);
      cv::arrowedLine(image, optical_center, target_center, {255, 0, 255}, 4);
      tools::draw_text(
        image,
        fmt::format(
          "{:.2f} {} {} {}", target->confidence, auto_aim::COLORS[target->color],
          auto_aim::ARMOR_NAMES[target->name], auto_aim::ARMOR_TYPES[target->type]),
        target_center + cv::Point2f(15, -20), {0, 255, 0}, 0.8, 2);
    }

    std::string mode_text = "DRY-RUN: serial disabled";
    cv::Scalar mode_color{0, 255, 255};
    if (control_mode) {
      mode_text = tracking_active ? "SENDING: s pause, d withdraw torque, q quit" : "PAUSED: e starts sending";
      if (tracking_active && tracking_mode=="predictive" && !prediction.valid)
        mode_text = "PIXEL BASE: " + prediction.reason;
      mode_color = tracking_active ? cv::Scalar{0,255,0} : cv::Scalar{0,255,255};
    }
    tools::draw_text(image, mode_text, {30, 65}, mode_color, 0.9, 2);
    tools::draw_text(image, status, {30, 105}, status_color, 0.9, 2);
    tools::draw_text(
      image, fmt::format("pixel error: x={:+.1f} y={:+.1f}", error_x, error_y), {30, 145},
      {255, 255, 255}, 0.8, 2);
    tools::draw_text(
      image,
      fmt::format("camera error: yaw={:+.3f} deg pitch={:+.3f} deg", yaw_error_deg, pitch_error_deg),
      {30, 185}, {255, 255, 255}, 0.8, 2);

    const auto now = std::chrono::steady_clock::now();
    const double decision_age_ms = std::chrono::duration<double, std::milli>(now - timestamp).count();
    const bool fresh_at_decision = frame_fresh && tools::visual_tracking::fresh_frame(decision_age_ms, max_frame_age_ms);
    const bool usable = target != nullptr && fresh_at_decision;
    const auto correction = tracking_mode=="predictive" ?
      auto_aim::augment_pixel_correction(yaw_error_deg,pitch_error_deg,prediction) :
      auto_aim::CameraCorrection{yaw_error_deg,pitch_error_deg,0,0};
    const double control_yaw=correction.yaw, control_pitch=correction.pitch;
    const double control_error_x=correction.augmented ? fx*std::tan(control_yaw/kRadToDeg) : error_x;
    const double control_error_y=correction.augmented ? fy*std::tan(control_pitch/kRadToDeg) : error_y;
    const double yaw_ff=correction.yaw_ff, pitch_ff=correction.pitch_ff;
    if (motion_tracker && std::chrono::duration<double>(now-last_prediction_log).count()>=1) {
      tools::logger()->info("[prediction] mode={} valid={} overlay={} reason={} horizon_ms={:.1f} pnp_rms={:.2f} samples={} ff=({:+.2f},{:+.2f})",
        tracking_mode, prediction.valid, prediction.augmentation_valid, prediction.reason, prediction.horizon_ms,
        prediction.reprojection_px, prediction.motion_samples, prediction.yaw_ff, prediction.pitch_ff);
      last_prediction_log=now;
    }
    if (motion_tracker) tools::draw_text(image,
      fmt::format("{}: {} ff=({:+.1f},{:+.1f}) horizon={:.1f}ms", tracking_mode, prediction.reason,
        prediction.yaw_ff, prediction.pitch_ff, prediction.horizon_ms),
      {30, 385}, prediction.valid ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), .7, 2);
    if (auto_start && !auto_start_sent && fresh_at_decision && !exiter.exit()) {
      gimbal->enable(); // Explicit --auto_start request; no motor readiness/arrival gate.
      if (motion_tracker) motion_tracker->reset_epoch();
      tracking_active = true;
      auto_start_sent = true;
      send_yaw_next = true;
      active_since = last_command = last_yaw = last_pitch = std::chrono::steady_clock::now();
      yaw_axis.reset(); pitch_axis.reset();
      tools::logger()->info("[auto-start] ge sent once; visual tracking active; board response is diagnostic only");
    }
    if (!fresh_at_decision) {
      status = "STALE FRAME - no command";
      tools::draw_text(image, status, {30, 305}, {0, 0, 255}, 0.8, 2);
    }
    if (tracking_active && max_active_seconds > 0 &&
        std::chrono::duration<double>(now - active_since).count() >= max_active_seconds) {
      tracking_active = false;
      std::cout << "[control] time limit reached; no new increments; pending motion may continue.\n";
    }
    const auto frame_dt = std::chrono::duration<double>(now - last_frame).count();
    last_frame = now;
    if (frame_dt > 1e-6) {
      tools::draw_text(
        image, fmt::format("FPS: {:.1f}", 1.0 / frame_dt), {30, 225}, {255, 255, 255}, 0.8, 2);
    }

    if (std::chrono::duration<double>(now - last_log).count() >= 0.25) {
      if (target == nullptr) {
        std::cout << "[preview] no target\n";
      } else {
        std::cout << fmt::format(
          "[preview] center=({:.1f},{:.1f}) px_error=({:+.1f},{:+.1f}) "
          "camera_error=({:+.3f} yaw,{:+.3f} pitch) deg\n",
          armor_center(*target).x, armor_center(*target).y, error_x, error_y, yaw_error_deg,
          pitch_error_deg);
      }
      last_log = now;
    }

    std::string sent_axis = "none";
    double sent_increment = 0, sent_dt = 0, write_done_t = 0, sent_speed = 0;
    if (!usable || !tracking_active) {
      yaw_axis.reset(); pitch_axis.reset();
      last_yaw = last_pitch = now;
    } else if (control_mode) {
      const double yaw_dt = std::chrono::duration<double>(now - last_yaw).count();
      const double pitch_dt = std::chrono::duration<double>(now - last_pitch).count();
      const double yaw_command = yaw_axis.step(control_error_x, yaw_sign * control_yaw, yaw_dt, yaw_sign * yaw_ff);
      const double pitch_command = pitch_axis.step(control_error_y, pitch_sign * control_pitch, pitch_dt, pitch_sign * pitch_ff);
      if (!yaw_axis.active()) last_yaw = now;
      if (!pitch_axis.active()) last_pitch = now;
      if (std::chrono::duration<double>(now - last_command).count() >= 1.0 / effective_control_rate &&
          tools::visual_tracking::fresh_frame(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - timestamp).count(), max_frame_age_ms)) {
        try {
          const bool has_yaw = std::abs(yaw_command) >= 0.0005;
          const bool has_pitch = std::abs(pitch_command) >= 0.0005;
          // One line per tick, <=50 Hz total. Integrate each axis's own elapsed time.
          if (has_yaw && (!has_pitch || send_yaw_next)) {
            sent_dt = std::min(yaw_dt, 0.1);
            sent_speed = tools::visual_tracking::command_speed(yaw_command, sent_dt, command_speed_deg_s);
            gimbal->move_relative_yaw(yaw_command, sent_speed);
            last_yaw = now; send_yaw_next = false;
            sent_axis = "yr"; sent_increment = yaw_command; sent_dt = std::min(yaw_dt, 0.1);
          } else if (has_pitch) {
            sent_dt = std::min(pitch_dt, 0.1);
            sent_speed = tools::visual_tracking::command_speed(pitch_command, sent_dt, command_speed_deg_s);
            gimbal->move_relative_pitch(pitch_command, sent_speed);
            last_pitch = now; send_yaw_next = true;
            sent_axis = "pr"; sent_increment = pitch_command; sent_dt = std::min(pitch_dt, 0.1);
          }
          if (has_yaw || has_pitch) {
            last_command = std::chrono::steady_clock::now();
            write_done_t = std::chrono::duration<double>(last_command.time_since_epoch()).count();
          }
        } catch (const std::exception & e) {
          tracking_active = false;
          if (headless) throw; // Let systemd report/restart a failed serial connection.
          tools::logger()->error("[control] command failed; tracking paused: {}", e.what());
        }
      }
    }
    if (sent_axis != "none")
      tools::logger()->info("[control] sent {} {:+.3f} {:.1f} age_ms={:.1f} axis_dt={:.4f}",
        sent_axis, sent_increment, sent_speed, decision_age_ms, sent_dt);
    if (vision_log) {
      vision_log << frame_count << ','
        << std::chrono::duration<double>(timestamp.time_since_epoch()).count() << ','
        << std::chrono::duration<double>(now.time_since_epoch()).count() << ',' << decision_age_ms << ','
        << raw_center.x << ',' << raw_center.y << ',' << filtered_center.x << ',' << filtered_center.y << ','
        << yaw_error_deg << ',' << pitch_error_deg << ',' << (target != nullptr) << ',' << fresh_at_decision << ','
        << tracking_active << ',' << sent_axis << ',' << sent_increment << ',' << sent_dt << ','
        << serial_feedback.pitch_setpoint_deg << ',' << serial_feedback.pitch_actual_deg << ','
        << serial_feedback.yaw_setpoint_deg << ',' << serial_feedback.yaw_actual_deg << ','
        << std::chrono::duration<double>(serial_feedback.received_at.time_since_epoch()).count() << ','
        << (serial_feedback.received_at != std::chrono::steady_clock::time_point{}) << ',' << write_done_t << ',' << sent_speed << ','
        << tracking_mode << ',' << prediction.valid << ',' << prediction.reason << ',' << prediction.pose_extrapolated << ','
        << prediction.horizon_ms << ',' << prediction.reprojection_px << ','
        << prediction.position.x() << ',' << prediction.position.y() << ',' << prediction.position.z() << ','
        << prediction.velocity.x() << ',' << prediction.velocity.y() << ',' << prediction.velocity.z() << ','
        << prediction.yaw_error << ',' << prediction.pitch_error << ',' << prediction.yaw_ff << ',' << prediction.pitch_ff << ','
        << prediction.motion_samples << ',' << prediction.frame_yaw << ',' << prediction.frame_pitch << ','
        << prediction.current_yaw << ',' << prediction.current_pitch;
      for (size_t corner=0; corner<4; ++corner) {
        const auto point = target && target->points.size()==4 ? target->points[corner] : cv::Point2f(0,0);
        vision_log << ',' << point.x << ',' << point.y;
      }
      vision_log << ',' << (target ? static_cast<int>(target->type) : -1)
        << ',' << (target ? static_cast<int>(target->name) : -1)
        << ',' << (target ? static_cast<int>(target->color) : -1)
        << ',' << (target ? target->confidence : 0) << ',' << prediction.augmentation_valid
        << ',' << prediction.yaw_lead << ',' << prediction.pitch_lead << '\n';
      if (frame_count % 50 == 0) vision_log.flush();
    }
    tools::draw_text(image, fmt::format("frame age: {:.1f} ms", decision_age_ms),
      {30, 345}, {255, 255, 255}, 0.8, 2);

    if (control_mode) {
      tools::draw_text(image, (motion_tracker ? "IMU for vision; motor state diagnostic: " : "Telemetry (display only): ") + gimbal->status_text().substr(0,110),
        {30,265}, {255,255,255}, 0.6, 1);
    }
    int key = -1;
    if (!headless) {
      cv::Mat display;
      cv::resize(image, display, {}, 0.5, 0.5);
      cv::imshow(
        control_mode ? "gimbal1 armor track - CONTROL" : "gimbal1 armor track - DRY RUN", display);
      key = cv::waitKey(1);
    }
    if (key == 'q') {
      if (control_mode) gimbal->disable(); // Explicit operator command only.
      break;
    }
    if (control_mode && key == 'e') {
      try {
        gimbal->enable(); // Explicit operator request; no readiness waiting.
        if (motion_tracker) motion_tracker->reset_epoch();
        tracking_active = true;
        send_yaw_next = true;
        active_since = last_command = std::chrono::steady_clock::now();
        last_yaw = last_pitch = last_command;
        yaw_axis.reset(); pitch_axis.reset();
        tools::logger()->info("[serial TX] ge\\r\\n written; awaiting board response for diagnostics (not proof of enable)");
      } catch (const std::exception & e) {
        tools::logger()->error("[serial write] {}", e.what());
      }
    } else if (control_mode && key == 's') {
      tracking_active = false; // Pause visual output only; sends no motor command.
    } else if (control_mode && key == 'd') {
      tracking_active = false;
      gimbal->disable(); // Explicit operator command only.
    }
    ++frame_count;
  }

  return 0;
} catch (const std::exception & e) {
  std::cerr << "gimbal1_armor_track: " << e.what() << '\n';
  return 1;
}

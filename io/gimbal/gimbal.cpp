#include "gimbal.hpp"

#include <algorithm>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto baudrate = yaml["baudrate"] ? yaml["baudrate"].as<uint32_t>() : 115200;
  auto timeout_ms = yaml["serial_timeout_ms"] ? yaml["serial_timeout_ms"].as<uint32_t>() : 20;
  reconnect_error_count_ =
    yaml["serial_reconnect_error_count"] ? yaml["serial_reconnect_error_count"].as<int>() : 50;
  reconnect_error_count_ = std::max(1, reconnect_error_count_);
  offline_timeout_ms_ =
    yaml["serial_offline_timeout_ms"] ? yaml["serial_offline_timeout_ms"].as<int>() : 100;
  offline_timeout_ms_ = std::max(1, offline_timeout_ms_);

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    auto timeout = serial::Timeout::simpleTimeout(timeout_ms);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  tools::logger()->info(
    "[Gimbal] Serial opened: {}, {} baud, 8N1, timeout {} ms.", com_port, baudrate,
    timeout_ms);

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> serial_lock(serial_mutex_);
  if (serial_.isOpen()) serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !received_ ||
    std::chrono::steady_clock::now() - last_rx_time_ >
      std::chrono::milliseconds(offline_timeout_ms_)) {
    return GimbalMode::IDLE;
  }
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool Gimbal::online() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return received_ &&
         std::chrono::steady_clock::now() - last_rx_time_ <=
           std::chrono::milliseconds(offline_timeout_ms_);
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  if (!online()) {
    VisionToGimbal = {};
    VisionToGimbal.mode = GIMBAL_TX_MODE_NO_CONTROL;
  }

  VisionToGimbal.head[0] = GIMBAL_PROTOCOL_SOF_0;
  VisionToGimbal.head[1] = GIMBAL_PROTOCOL_SOF_1;
  if (VisionToGimbal.mode > GIMBAL_TX_MODE_CONTROL_AND_FIRE) {
    VisionToGimbal.mode = GIMBAL_TX_MODE_NO_CONTROL;
  }
  VisionToGimbal.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&VisionToGimbal),
    sizeof(VisionToGimbal) - sizeof(VisionToGimbal.crc16));

  try {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    serial_.write(reinterpret_cast<uint8_t *>(&VisionToGimbal), sizeof(VisionToGimbal));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  VisionToGimbal command{};
  command.mode = control ? (fire ? GIMBAL_TX_MODE_CONTROL_AND_FIRE : GIMBAL_TX_MODE_CONTROL)
                         : GIMBAL_TX_MODE_NO_CONTROL;
  command.yaw = yaw;
  command.yaw_vel = yaw_vel;
  command.yaw_acc = yaw_acc;
  command.pitch = pitch;
  command.pitch_vel = pitch_vel;
  command.pitch_acc = pitch_acc;
  send(command);
}

bool Gimbal::read_exact(uint8_t * buffer, size_t size)
{
  size_t offset = 0;
  try {
    while (!quit_ && offset < size) {
      auto count = serial_.read(buffer + offset, size - offset);
      if (count == 0) return false;
      offset += count;
    }
  } catch (const std::exception & e) {
    return false;
  }
  return offset == size;
}

bool Gimbal::read_header()
{
  bool got_first_byte = false;
  uint8_t byte = 0;

  while (!quit_) {
    if (!read_exact(&byte, 1)) return false;

    if (!got_first_byte) {
      got_first_byte = byte == GIMBAL_PROTOCOL_SOF_0;
      continue;
    }

    if (byte == GIMBAL_PROTOCOL_SOF_1) return true;
    got_first_byte = byte == GIMBAL_PROTOCOL_SOF_0;
  }
  return false;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count >= reconnect_error_count_) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read_header()) {
      error_count++;
      continue;
    }

    rx_data_.head[0] = GIMBAL_PROTOCOL_SOF_0;
    rx_data_.head[1] = GIMBAL_PROTOCOL_SOF_1;

    auto t = std::chrono::steady_clock::now();

    if (!read_exact(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    std::lock_guard<std::mutex> lock(mutex_);

    received_ = true;
    last_rx_time_ = t;

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }

    queue_.push({q, t});
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        if (serial_.isOpen()) serial_.close();
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        serial_.open();
      }
      queue_.clear();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        received_ = false;
        mode_ = GimbalMode::IDLE;
      }
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io

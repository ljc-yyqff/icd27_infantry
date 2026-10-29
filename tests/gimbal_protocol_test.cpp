#include <fmt/core.h>

#include <cstdio>

#include "io/gimbal/gimbal_protocol.h"
#include "tools/crc.hpp"

bool check(bool condition, const char * message)
{
  if (!condition) fmt::print(stderr, "gimbal protocol test failed: {}\n", message);
  return condition;
}

int main()
{
  GimbalToVisionPacket rx{};
  rx.head[0] = GIMBAL_PROTOCOL_SOF_0;
  rx.head[1] = GIMBAL_PROTOCOL_SOF_1;
  rx.mode = GIMBAL_RX_MODE_AUTO_AIM;
  rx.q[0] = 1.0F;
  rx.bullet_speed = 22.0F;
  rx.crc16 = gimbal_protocol_crc16(
    reinterpret_cast<const uint8_t *>(&rx), sizeof(rx) - sizeof(rx.crc16));

  if (!check(
        rx.crc16 == tools::get_crc16(
                      reinterpret_cast<const uint8_t *>(&rx),
                      sizeof(rx) - sizeof(rx.crc16)),
        "A-board CRC differs from NUC CRC"))
    return 1;
  if (!check(
        gimbal_protocol_check_crc16(reinterpret_cast<const uint8_t *>(&rx), sizeof(rx)),
        "A-board packet rejected by shared CRC"))
    return 1;
  if (!check(
        tools::check_crc16(reinterpret_cast<const uint8_t *>(&rx), sizeof(rx)),
        "A-board packet rejected by NUC CRC"))
    return 1;

  VisionToGimbalPacket tx{};
  tx.head[0] = GIMBAL_PROTOCOL_SOF_0;
  tx.head[1] = GIMBAL_PROTOCOL_SOF_1;
  tx.mode = GIMBAL_TX_MODE_CONTROL;
  tx.yaw = 0.25F;
  tx.pitch = -0.10F;
  tx.crc16 = gimbal_protocol_crc16(
    reinterpret_cast<const uint8_t *>(&tx), sizeof(tx) - sizeof(tx.crc16));

  if (!check(
        gimbal_protocol_check_crc16(reinterpret_cast<const uint8_t *>(&tx), sizeof(tx)),
        "NUC packet rejected by shared CRC"))
    return 1;
  if (!check(
        tools::check_crc16(reinterpret_cast<const uint8_t *>(&tx), sizeof(tx)),
        "NUC packet rejected by NUC CRC"))
    return 1;

  auto corrupted = tx;
  reinterpret_cast<uint8_t *>(&corrupted)[5] ^= 0x01;
  if (!check(
        !gimbal_protocol_check_crc16(
          reinterpret_cast<const uint8_t *>(&corrupted), sizeof(corrupted)),
        "corrupted packet was accepted"))
    return 1;

  fmt::print(
    "gimbal protocol OK: A-board->NUC={} bytes, NUC->A-board={} bytes\n", sizeof(rx),
    sizeof(tx));
  return 0;
}

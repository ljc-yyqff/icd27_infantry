#ifndef IO_GIMBAL_GIMBAL_PROTOCOL_H_
#define IO_GIMBAL_GIMBAL_PROTOCOL_H_

#include <stdint.h>

/*
 * NUC <-> RoboMaster A-board serial protocol.
 *
 * Wire format requirements:
 * - little-endian
 * - IEEE-754 32-bit float
 * - packed structs (no alignment padding)
 * - CRC16 covers every byte except the final crc16 field
 */

#define GIMBAL_PROTOCOL_SOF_0 ((uint8_t)'S')
#define GIMBAL_PROTOCOL_SOF_1 ((uint8_t)'P')

#define GIMBAL_RX_MODE_IDLE ((uint8_t)0)
#define GIMBAL_RX_MODE_AUTO_AIM ((uint8_t)1)
#define GIMBAL_RX_MODE_SMALL_BUFF ((uint8_t)2)
#define GIMBAL_RX_MODE_BIG_BUFF ((uint8_t)3)

#define GIMBAL_TX_MODE_NO_CONTROL ((uint8_t)0)
#define GIMBAL_TX_MODE_CONTROL ((uint8_t)1)
#define GIMBAL_TX_MODE_CONTROL_AND_FIRE ((uint8_t)2)

#if defined(__GNUC__)
#define GIMBAL_PACKED __attribute__((packed))
#else
#define GIMBAL_PACKED
#pragma pack(push, 1)
#endif

/* A-board -> NUC, 43 bytes. Angles use rad; angular rates use rad/s. */
typedef struct GIMBAL_PACKED
{
  uint8_t head[2];
  uint8_t mode;
  float q[4]; /* w, x, y, z */
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed; /* m/s */
  uint16_t bullet_count;
  uint16_t crc16;
} GimbalToVisionPacket;

/* NUC -> A-board, 29 bytes. Angles use rad; angular rates use rad/s. */
typedef struct GIMBAL_PACKED
{
  uint8_t head[2];
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
} VisionToGimbalPacket;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

#define GIMBAL_TO_VISION_PACKET_SIZE ((uint16_t)43)
#define VISION_TO_GIMBAL_PACKET_SIZE ((uint16_t)29)

/*
 * Reflected CRC-16, polynomial 0x8408, initial value 0xffff, no final xor.
 * This bitwise implementation matches tools/get_crc16() on the NUC side and
 * is intentionally table-free so it can be copied into an STM32 project.
 */
static inline uint16_t gimbal_protocol_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc = 0xffff;
  uint32_t i;
  uint8_t bit;

  for (i = 0; i < len; ++i) {
    crc ^= data[i];
    for (bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

static inline uint8_t gimbal_protocol_check_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t expected;
  if (len < 2U) return 0U;
  expected = (uint16_t)data[len - 2U] | ((uint16_t)data[len - 1U] << 8U);
  return (uint8_t)(gimbal_protocol_crc16(data, len - 2U) == expected);
}

#if defined(__cplusplus)
static_assert(sizeof(float) == 4, "The protocol requires a 32-bit float");
static_assert(sizeof(GimbalToVisionPacket) == GIMBAL_TO_VISION_PACKET_SIZE, "Bad RX packet size");
static_assert(sizeof(VisionToGimbalPacket) == VISION_TO_GIMBAL_PACKET_SIZE, "Bad TX packet size");
#else
_Static_assert(sizeof(float) == 4, "The protocol requires a 32-bit float");
_Static_assert(sizeof(GimbalToVisionPacket) == GIMBAL_TO_VISION_PACKET_SIZE, "Bad RX packet size");
_Static_assert(sizeof(VisionToGimbalPacket) == VISION_TO_GIMBAL_PACKET_SIZE, "Bad TX packet size");
#endif

#endif  // IO_GIMBAL_GIMBAL_PROTOCOL_H_

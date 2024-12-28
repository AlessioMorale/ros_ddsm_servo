#ifndef DDS210_DRIVER_LIBRARY_PROTOCOL_HPP
#define DDS210_DRIVER_LIBRARY_PROTOCOL_HPP

#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <stdint.h>
namespace ddsm210_driver::protocol {

enum class DDSM210_mode : uint8_t {
  MODE_OPEN_LOOP = 0,
  MODE_VELOCITY = 2,
  MODE_POSITION = 3,
};

enum class DDSM210_command : uint8_t {
  CMD_DRIVE = 0x64,
  CMD_DRIVE_RESPONSE = 0x64,
  CMD_FEEDBACK = 0x74,
  CMD_FEEDBACK_RESPONSE = 0x74,
  CMD_MODE_SWITCH = 0xA0,
  CMD_MODE_SWITCH_RESPONSE = 0xA0,
  CMD_SET_ID = 0x55,
  CMD_SET_ID_RESPONSE = 0x64,
  CMD_GET_MODE = 0x75,
  CMD_GET_MODE_RESPONSE = 0x75,
};

enum class DDSM210_brake : uint8_t {
  NO_BRAKE = 0,
  BRAKE = 0xFF,
};

enum class DDSM210_set_id_key : uint8_t {
  SET_ID_KEY = 0x53,
};
static constexpr uint8_t DDSM210_PACKET_SIZE = 10;
typedef struct {
  uint8_t : 1;
  uint8_t overcurrent : 1;
  uint8_t : 2;
  uint8_t overtemperature : 1;
  uint8_t : 3;
} DDSM210_error_code_t;

typedef union __attribute__((packed)) {
  struct __attribute__((packed)) {
    uint8_t id;
    DDSM210_command cmd;
    union __attribute__((__packed__)) {
      struct __attribute__((__packed__)) {
        uint16_t target;
        uint8_t _res;
        uint8_t _res2;
        uint8_t acceleration_time;
        DDSM210_brake brake;
        // uint8_t _res3;
      } drive;
      struct __attribute__((__packed__)) {
        int16_t velocity;
        int16_t current;
        uint8_t acceleration_time;
        uint8_t temperature;
        DDSM210_error_code_t error_code;
      } drive_response;
      struct __attribute__((packed)) {
        uint32_t mileage;
        uint16_t position;
        DDSM210_error_code_t error_code;
      } feedback_response;
      struct __attribute__((packed)) {
        DDSM210_mode mode;
        uint8_t _res[6];
      } mode_switch;
      struct __attribute__((packed)) {
        DDSM210_mode mode;
        uint8_t _res[6];
      } mode_switch_response;
      struct __attribute__((packed)) {
        DDSM210_set_id_key key;
        uint8_t id;
        uint8_t _res[5];
      } set_id;
      struct __attribute__((packed)) {
        DDSM210_mode mode;
        uint8_t _res[6];
      } get_mode_response;
    } packet;
    uint8_t checksum;
  } data;
  uint8_t raw[DDSM210_PACKET_SIZE];
} DDSM210_packet_t;

class utils {
  utils() = delete;
  ~utils() = delete;

public:
  static constexpr float ACCELERATION_TIME_SCALE = 10.0f;
  static constexpr float POSITION_SCALE = 32767.0f / 360.0f;
  static constexpr float VELOCITY_SCALE = 10.0f;
  static constexpr float CURRENT_SCALE = 10000.0f;
  static constexpr float TEMPERATURE_SCALE = 1.0f;

  static constexpr uint32_t COMMANDS_DELAY_US = 4000;
  static constexpr unsigned int SET_ID_RESEND_COUNT = 5;

  static float convert(int16_t value, float scale) {
    return static_cast<float>(decode(value)) / scale;
  }
  static int16_t convert(float value, float scale) {
    return encode(static_cast<int16_t>(value * scale));
  }

  static float convert_u8(uint8_t value, float scale) {
    return static_cast<float>(value) / scale;
  }
  static uint8_t convert_u8(float value, float scale) {
    return static_cast<uint8_t>(value * scale);
  }
  static uint8_t calculate_checksum(const DDSM210_packet_t &packet) {
    static_assert(DDSM210_PACKET_SIZE == sizeof(DDSM210_packet_t::data), "Packet size is invalid");
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(packet.raw) - 1; i++) {
      // checksum = crc8_update(checksum, packet.raw[i]);
      checksum = crc8_table[checksum ^ packet.raw[i]];
    }
    return checksum;
  }
  // CRC-8/MAXIM
  static uint8_t crc8_update(uint8_t crc, uint8_t data) {
    uint8_t i;
    crc = crc ^ data;
    for (i = 0; i < 8; ++i) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8c;
      } else {
        crc >>= 1;
      }
    }
    return crc;
  }

  static void fill_crc(protocol::DDSM210_packet_t &data) {
    data.data.checksum = calculate_checksum(data);
  }

  static bool validate_packet(const DDSM210_packet_t &packet) {
    return calculate_checksum(packet) == packet.data.checksum;
  }

  static void prepare_packet(uint8_t id, protocol::DDSM210_command cmd,
                             protocol::DDSM210_packet_t &packet) {
    std::memset(&packet, 0, sizeof(packet));
    packet.data.id = id;
    packet.data.cmd = cmd;
  }

private:
  static int16_t decode(int16_t packet_value) {
    return static_cast<int16_t>(ntohs(static_cast<uint16_t>(packet_value)));
  }
  static int16_t encode(int16_t value) {
    return static_cast<int16_t>(htons(static_cast<uint16_t>(value)));
  }

  static constexpr uint8_t crc8_table[256] = {
      0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F,
      0x41, 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60,
      0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80,
      0xDE, 0x3C, 0x62, 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E,
      0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38,
      0x66, 0xE5, 0xBB, 0x59, 0x07, 0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47,
      0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A, 0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7,
      0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
      0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51,
      0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD, 0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E,
      0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50, 0xAF, 0xF1, 0x13, 0x4D, 0xCE,
      0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0,
      0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73, 0xCA, 0x94, 0x76,
      0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B, 0x57, 0x09,
      0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, 0xE9,
      0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
      0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B,
      0x35};
};
} // namespace ddsm210_driver::protocol
#endif // DDS210_DRIVER_LIBRARY_PROTOCOL_HPP
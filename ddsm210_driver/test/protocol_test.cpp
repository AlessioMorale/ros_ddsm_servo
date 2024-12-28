#include "ddsm210_driver/protocol.hpp"
#include <cstdint>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace ddsm210_driver::protocol;

class ProtocolTest : public ::testing::Test {
protected:
  DDSM210_packet_t packet;

  void SetUp() override {
    std::memset(&packet, 0, sizeof(packet));
  }
};

TEST_F(ProtocolTest, CalculateChecksum) {
  utils::prepare_packet(1, DDSM210_command::CMD_DRIVE, packet);
  uint8_t checksum = utils::calculate_checksum(packet);
  EXPECT_EQ(checksum, 0x50); 
}

TEST_F(ProtocolTest, ValidatePacket) {
  utils::prepare_packet(1, DDSM210_command::CMD_MODE_SWITCH, packet);
  packet.data.packet.mode_switch.mode = DDSM210_mode::MODE_VELOCITY;
  auto calculated_checksum = utils::calculate_checksum(packet);
  EXPECT_EQ(calculated_checksum, 0xe4);
  packet.data.checksum = 0xe4;
  bool is_valid = utils::validate_packet(packet);
  EXPECT_TRUE(is_valid);
}

TEST_F(ProtocolTest, PreparePacket) {
  utils::prepare_packet(1, DDSM210_command::CMD_DRIVE, packet);

  EXPECT_EQ(packet.data.id, 1);
  EXPECT_EQ(packet.data.cmd, DDSM210_command::CMD_DRIVE);
  EXPECT_EQ(packet.data.checksum, 0);
}

TEST_F(ProtocolTest, ConvertInt16) {
  int16_t value = 1000;
  int16_t encoded_value = static_cast<int16_t>(htons(static_cast<uint16_t>(value)));
  float scale = 10.0f;
  float converted = utils::convert(encoded_value, scale);
  EXPECT_FLOAT_EQ(converted, 100.0f);

  int16_t back_converted = utils::convert(converted, scale);
  EXPECT_EQ(back_converted, encoded_value);
}

TEST_F(ProtocolTest, ConvertUint8) {
  uint8_t value = 100;
  float scale = 10.0f;
  float converted = utils::convert_u8(value, scale);
  EXPECT_FLOAT_EQ(converted, 10.0f);

  uint8_t back_converted = utils::convert_u8(converted, scale);
  EXPECT_EQ(back_converted, value);
}

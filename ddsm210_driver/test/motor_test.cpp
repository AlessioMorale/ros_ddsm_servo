#include "ddsm210_driver/comm/port.hpp"
#include "ddsm210_driver/motors.hpp"
#include "ddsm210_driver/protocol.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <cstdint>
#include <list>
#include <memory>
#include <queue>

using namespace ddsm210_driver;

class MockSerialPort : public comm::Port {
public:
  MockSerialPort(std::queue<protocol::DDSM210_packet_t>* sent_packet_queue) : Port(), sent_packets{sent_packet_queue} {
  }

  virtual void set_receive_callback(comm::receive_callback_t callback) override {
    receive_callback_ = callback;
  }

  void send(const std::vector<uint8_t> &data) override {
    protocol::DDSM210_packet_t packet;
    std::memcpy(&packet, data.data(), sizeof(packet));
    uint8_t id = packet.data.id;
    protocol::DDSM210_command cmd = packet.data.cmd;
    if(enable_replies){
      if (cmd == protocol::DDSM210_command::CMD_SET_ID) {
        send_set_id_response(id);
      }
    }
    sent_packets->push(packet);
  }

  MOCK_METHOD(void, close,(),(override));

  void receive(const std::vector<uint8_t> &data) {
    receive_callback_(data);
  }
  void send_set_id_response(uint8_t id){
    protocol::DDSM210_packet_t resp_packet;
    protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_SET_ID_RESPONSE, resp_packet); 
    protocol::utils::fill_crc(resp_packet);
    auto data = std::vector<uint8_t>(resp_packet.raw, resp_packet.raw + sizeof(resp_packet.raw));
    receive(data);
  }

  void send_drive_response(uint8_t id, int16_t velocity, int16_t current, uint8_t acceleration_time, uint8_t temperature, bool overcurrent, bool overtemperature){
    protocol::DDSM210_packet_t resp_packet;
    protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_DRIVE_RESPONSE, resp_packet);
    resp_packet.data.packet.drive_response.velocity =  static_cast<int16_t>(htons(static_cast<uint16_t>(velocity)));
    resp_packet.data.packet.drive_response.current = static_cast<int16_t>(htons(static_cast<uint16_t>(current)));
    resp_packet.data.packet.drive_response.acceleration_time = acceleration_time;
    resp_packet.data.packet.drive_response.temperature = temperature;
    resp_packet.data.packet.drive_response.error_code.overcurrent = overcurrent;
    resp_packet.data.packet.drive_response.error_code.overtemperature = overtemperature;
    protocol::utils::fill_crc(resp_packet);
    auto data = std::vector<uint8_t>(resp_packet.raw, resp_packet.raw + sizeof(resp_packet.raw));
    receive(data);
  }
  std::queue<protocol::DDSM210_packet_t>* sent_packets;
  bool enable_replies = false;

private:
  comm::receive_callback_t receive_callback_;
};

class MotorTest : public ::testing::Test {
protected:
  void SetUp() override {
    auto port = std::make_unique<MockSerialPort>(&sent_packets);
    mock_port_ = port.get();
    clear_sent_packets();
    motor_ = std::make_unique<Motors>(motor_ids, std::move(port));

    motor_->register_feedback_callback((motor_feedback_callback)[this](const Motor_feedback_t &feedback) {
      feedback_queue_.push(feedback);
    });
    CheckFailSafe();
  }
  
  void clear_sent_packets(){
    while(!sent_packets.empty()){
      sent_packets.pop();
    }
  }
  void CheckFailSafe(){
    std::list<protocol::DDSM210_packet_t> packets;

    while (!sent_packets.empty()) {
      packets.push_back(sent_packets.front());
      sent_packets.pop();
    }

    for (auto i : motor_ids) {
      auto mode_switch = std::find_if(packets.begin(), packets.end(), [i](protocol::DDSM210_packet_t packet) {
        return packet.data.id == i && packet.data.cmd == protocol::DDSM210_command::CMD_MODE_SWITCH;
      });
      EXPECT_NE(mode_switch, packets.end());
      EXPECT_EQ(mode_switch->data.packet.mode_switch.mode, protocol::DDSM210_mode::MODE_OPEN_LOOP);

      auto drive = std::find_if(packets.begin(), packets.end(), [i](protocol::DDSM210_packet_t packet) {
        return packet.data.id == i && packet.data.cmd == protocol::DDSM210_command::CMD_DRIVE;
      });
      EXPECT_NE(drive, packets.end());
      EXPECT_EQ(drive->data.packet.drive.target, 0);
    }
  }
  void TearDown() override {
    EXPECT_CALL(*mock_port_, close());
    clear_sent_packets();
    mock_port_ = nullptr;
    motor_.reset();
    CheckFailSafe();
  }
  
  std::vector<uint8_t> motor_ids {1, 2, 3};
  std::queue<protocol::DDSM210_packet_t> sent_packets;
  std::queue<Motor_feedback_t> feedback_queue_;
  MockSerialPort *mock_port_;
  std::unique_ptr<Motors> motor_;
};

TEST_F(MotorTest, TestSetMode) {
  motor_->set_mode(1, command_mode::MODE_OPENLOOP);
}

TEST_F(MotorTest, TestSetTarget) {
  motor_->set_target(1, 100.0f, 0.5f, true);
}

TEST_F(MotorTest, TestSetId) {
  mock_port_->enable_replies = true;
  bool result = motor_->set_id(1);
  EXPECT_TRUE(result);
}

TEST_F(MotorTest, TestReceiveCallbackInvalidPacket) {
  std::vector<uint8_t> invalid_data(5, 0);
  mock_port_->receive(invalid_data);
  // No expectations, just ensure no crash
}

TEST_F(MotorTest, TestReceiveCallbackValidPacket) {
  while(!feedback_queue_.empty()){
    feedback_queue_.pop();
  }
  mock_port_->send_drive_response(42, -10, 20, 30, 50, true, false);

  EXPECT_EQ(feedback_queue_.size(), 1);
  Motor_feedback_t feedback = feedback_queue_.front();

  EXPECT_EQ(feedback.id, 42);
  EXPECT_EQ(feedback.velocity, -1.0f);
  EXPECT_EQ(feedback.current, 0.002f);
  EXPECT_EQ(feedback.acceleration, 3.0f);
  EXPECT_EQ(feedback.temperature, 50.0f);
  EXPECT_EQ(feedback.overcurrent, true);
  EXPECT_EQ(feedback.overtemperature, false);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

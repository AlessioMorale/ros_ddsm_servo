#include "ddsm210_driver/motor.hpp"
#include "ddsm210_driver/protocol.hpp"
#include <cstdint>
#include <unistd.h>
#include <iostream>

namespace ddsm210_driver {
Motor::Motor(std::list<uint8_t> ids, std::unique_ptr<comm::Port> port, bool auto_fail_safe) : port_(std::move(port)), auto_fail_safe_(auto_fail_safe) {
  port_->set_receive_callback(std::bind(&Motor::receive_callback, this, std::placeholders::_1));

  for (auto id : ids) {
    motor_modes_[id] = command_mode::MODE_INVALID;
  }
  if(auto_fail_safe_){
    set_fail_safe();
  }
}

void Motor::receive_callback(std::vector<uint8_t> data) {
  std::cerr << "recvd" << std::endl;

  if (data.size() != sizeof(protocol::DDSM210_packet_t)) {
    std::cerr << "Invalid packet size " << data.size() << std::endl;
    return;

  }
  protocol::DDSM210_packet_t packet;

  std::memcpy(&packet, data.data(), sizeof(packet));

  if (!protocol::utils::validate_packet(packet)) {
    std::cerr << "Invalid packet received" <<  std::endl;
    return;
  }

  auto id = packet.data.id;
  last_id_received_ = id;

  switch (packet.data.cmd) {
  case protocol::DDSM210_command::CMD_DRIVE_RESPONSE: {
    // Note: this covers both CMD_DRIVE_RESPONSE and CMD_SET_ID_RESPONSE, they share the same id
    // :(
    Motor_feedback_t feedback;

    feedback.id = id;

    feedback.velocity = protocol::utils::convert(packet.data.packet.drive_response.velocity,
                                                 protocol::utils::VELOCITY_SCALE);
    feedback.current = protocol::utils::convert(packet.data.packet.drive_response.current,
                                                protocol::utils::CURRENT_SCALE);
    feedback.acceleration =
        protocol::utils::convert_u8(packet.data.packet.drive_response.acceleration_time,
                                    protocol::utils::ACCELERATION_TIME_SCALE);
    feedback.temperature = protocol::utils::convert_u8(
        packet.data.packet.drive_response.temperature, protocol::utils::TEMPERATURE_SCALE);
    feedback.overcurrent = packet.data.packet.drive_response.error_code.overcurrent;
    feedback.overtemperature = packet.data.packet.drive_response.error_code.overtemperature;
    if(feedback_callback_){
      feedback_callback_(feedback);
    }
    break;
  }

  case protocol::DDSM210_command::CMD_MODE_SWITCH_RESPONSE: {
    static std::map<protocol::DDSM210_mode, command_mode> mode_map = {
        {protocol::DDSM210_mode::MODE_OPEN_LOOP, command_mode::MODE_OPENLOOP},
        {protocol::DDSM210_mode::MODE_POSITION, command_mode::MODE_POSITION},
        {protocol::DDSM210_mode::MODE_VELOCITY, command_mode::MODE_VELOCITY},
    };
    auto mode = packet.data.packet.mode_switch_response.mode;

    if (mode_map.find(mode) != mode_map.end()) {
      motor_modes_[id] = mode_map[mode];
    } else {
      motor_modes_[id] = command_mode::MODE_INVALID;
    }
    break;
  }

  default:
    break;
  }
}

void Motor::set_fail_safe() {
  for (auto motor : motor_modes_) {
    auto id = motor.first;
    set_mode(id, protocol::DDSM210_mode::MODE_OPEN_LOOP);
    usleep(protocol::utils::COMMANDS_DELAY_US);
    // Set the motor target to 0 to stop any motion
    set_target(id, 0.0f);
    usleep(protocol::utils::COMMANDS_DELAY_US);
  }
}

void Motor::set_mode(uint8_t id, protocol::DDSM210_mode mode) {
  protocol::DDSM210_packet_t packet;
  protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_MODE_SWITCH, packet);
  packet.data.packet.mode_switch.mode = mode;
  send_packet(packet);
}

void Motor::send_packet(protocol::DDSM210_packet_t &packet) {
  protocol::utils::fill_crc(packet);
  auto data = std::vector<uint8_t>(packet.raw, packet.raw + sizeof(packet.raw));
  port_->send(data);
}

void Motor::set_target(uint8_t id, float target, float acceleration_time, bool brake) {
  protocol::DDSM210_packet_t packet;
  protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_DRIVE, packet);
  packet.data.packet.drive.target =
      protocol::utils::convert(target, protocol::utils::VELOCITY_SCALE);
  packet.data.packet.drive.acceleration_time =
      protocol::utils::convert_u8(acceleration_time, protocol::utils::ACCELERATION_TIME_SCALE);
  packet.data.packet.drive.brake =
      brake ? protocol::DDSM210_brake::BRAKE : protocol::DDSM210_brake::NO_BRAKE;
  send_packet(packet);
}

bool Motor::set_id(uint8_t id) {
  protocol::DDSM210_packet_t packet;
  protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_SET_ID, packet);
  packet.data.packet.set_id.id = id;
  packet.data.packet.set_id.key = protocol::DDSM210_set_id_key::SET_ID_KEY;
  last_id_received_ = std::nullopt;
  for (uint i = 0; i < protocol::utils::SET_ID_RESEND_COUNT; i++) {
    send_packet(packet);
    usleep(protocol::utils::COMMANDS_DELAY_US);
  }

  // wait some time to get a response
  usleep(10 * protocol::utils::COMMANDS_DELAY_US);
  return last_id_received_.has_value() && last_id_received_.value() == id;
}

void Motor::register_feedback_callback(motor_feedback_callback callback) {
  feedback_callback_ = callback;
}

Motor::~Motor() {
  if(auto_fail_safe_){
    set_fail_safe();
  }
  port_->close();
}

} // namespace ddsm210_driver

// Copyright 2025 Alessio Morale <alessiomorale-at-gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "ddsm210_driver/motors.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>

#include "ddsm210_driver/protocol.hpp"

namespace ddsm210_driver
{
Motors::Motors(std::vector<uint8_t> ids, std::unique_ptr<comm::Port> port, bool auto_fail_safe)
: port_(std::move(port)), auto_fail_safe_(auto_fail_safe)
{
  port_->set_receive_callback(std::bind(&Motors::receive_callback, this, std::placeholders::_1));

  for (auto id : ids) {
    motor_modes_[id] = command_mode::MODE_INVALID;
  }
  if (auto_fail_safe_) {
    set_fail_safe();
  }
}

void Motors::receive_callback(std::vector<uint8_t> data)
{
  // Serial reads are a byte stream and don't align with packet boundaries: a single read may
  // contain a partial packet, several packets, or the tail of one packet plus the start of the
  // next. Accumulate bytes into the fixed-size buffer and resync on the checksum instead of
  // assuming size() == one packet.
  constexpr size_t packet_size = sizeof(protocol::DDSM210_packet_t);
  rx_byte_count_.fetch_add(data.size(), std::memory_order_relaxed);

  size_t offset = 0;
  while (offset < data.size()) {
    if (rx_len_ == rx_buffer_.size()) {
      // Buffer full without a valid frame in it: drop the oldest byte to make room and resync.
      std::memmove(rx_buffer_.data(), rx_buffer_.data() + 1, --rx_len_);
      rx_discarded_count_.fetch_add(1, std::memory_order_relaxed);
    }
    size_t chunk = std::min(rx_buffer_.size() - rx_len_, data.size() - offset);
    std::memcpy(rx_buffer_.data() + rx_len_, data.data() + offset, chunk);
    rx_len_ += chunk;
    offset += chunk;

    while (rx_len_ >= packet_size) {
      protocol::DDSM210_packet_t packet;
      std::memcpy(&packet, rx_buffer_.data(), packet_size);

      if (protocol::utils::validate_packet(packet)) {
        process_packet(packet);
        rx_len_ -= packet_size;
        std::memmove(rx_buffer_.data(), rx_buffer_.data() + packet_size, rx_len_);
      } else {
        // Not a valid frame at this offset: drop one byte and retry to resynchronize.
        std::memmove(rx_buffer_.data(), rx_buffer_.data() + 1, --rx_len_);
        rx_discarded_count_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void Motors::process_packet(const protocol::DDSM210_packet_t & packet)
{
  auto id = packet.data.id;
  last_id_received_ = id;

  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    if (awaited_id_ == id && awaited_cmd_ == packet.data.cmd) {
      response_received_ = true;
      response_cv_.notify_one();
    }
  }

  switch (packet.data.cmd) {
    case protocol::DDSM210_command::CMD_DRIVE_RESPONSE: {
      // Note: this covers both CMD_DRIVE_RESPONSE and CMD_SET_ID_RESPONSE, they share the same id
      // :(
      Motor_feedback_t feedback;

      feedback.id = id;

      feedback.velocity = protocol::utils::convert(
        packet.data.packet.drive_response.velocity, protocol::utils::VELOCITY_SCALE);
      feedback.current = protocol::utils::convert(
        packet.data.packet.drive_response.current, protocol::utils::CURRENT_SCALE);
      feedback.acceleration = protocol::utils::convert_u8(
        packet.data.packet.drive_response.acceleration_time,
        protocol::utils::ACCELERATION_TIME_SCALE);
      feedback.temperature = protocol::utils::convert_u8(
        packet.data.packet.drive_response.temperature, protocol::utils::TEMPERATURE_SCALE);
      feedback.overcurrent = packet.data.packet.drive_response.error_code.overcurrent;
      feedback.overtemperature = packet.data.packet.drive_response.error_code.overtemperature;
      if (feedback_callback_) {
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

void Motors::set_fail_safe()
{
  for (auto motor : motor_modes_) {
    auto id = motor.first;
    for (int i = 0; i < 10; i++) {
      if (motor_modes_[id] == command_mode::MODE_OPENLOOP) {
        break;
      }
      set_mode(id, command_mode::MODE_OPENLOOP);
      usleep(protocol::utils::COMMANDS_DELAY_US);
    }
    // Set the motor target to 0 to stop any motion
    set_target(id, 0.0f, 0.5, false);
    usleep(protocol::utils::COMMANDS_DELAY_US);
  }
}

void Motors::set_mode(uint8_t id, command_mode mode)
{
  static std::map<command_mode, protocol::DDSM210_mode> mode_map = {
    {command_mode::MODE_OPENLOOP, protocol::DDSM210_mode::MODE_OPEN_LOOP},
    {command_mode::MODE_POSITION, protocol::DDSM210_mode::MODE_POSITION},
    {command_mode::MODE_VELOCITY, protocol::DDSM210_mode::MODE_VELOCITY},
  };
  if (motor_modes_[id] == mode) {
    return;
  }
  auto m = mode_map.find(mode);

  if (m == mode_map.end()) {
    throw std::runtime_error("Invalid mode");
    return;
  }
  protocol::DDSM210_packet_t packet;
  protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_MODE_SWITCH, packet);
  packet.data.packet.mode_switch.mode = m->second;
  send_packet_and_wait_response(packet, "mode switch");
}

void Motors::send_packet(protocol::DDSM210_packet_t & packet)
{
  protocol::utils::fill_crc(packet);
  auto data = std::vector<uint8_t>(packet.raw, packet.raw + sizeof(packet.raw));
  port_->send(data);
}

bool Motors::send_packet_and_wait_response(protocol::DDSM210_packet_t & packet, const char * label)
{
  // All motor TX lines are wired together onto a single RX line, so a motor's response can only
  // be reliably captured if no other motor is talking at the same time. Waiting here serializes
  // the request/response cycle and keeps the bus from being driven by more than one motor.
  uint8_t id = packet.data.id;
  uint64_t bytes_before = rx_byte_count_.load(std::memory_order_relaxed);
  uint64_t discarded_before = rx_discarded_count_.load(std::memory_order_relaxed);

  std::unique_lock<std::mutex> lock(response_mutex_);
  awaited_id_ = id;
  awaited_cmd_ = packet.data.cmd;
  response_received_ = false;
  lock.unlock();

  send_packet(packet);

  lock.lock();
  bool received = response_cv_.wait_for(
    lock, std::chrono::microseconds(protocol::utils::RESPONSE_TIMEOUT_US),
    [this] { return response_received_; });
  awaited_id_.reset();
  awaited_cmd_.reset();
  lock.unlock();

  if (!received) {
    // Bytes seen but discarded => something was on the wire but couldn't be framed (corruption /
    // collision). Zero bytes seen => the motor never replied at all (dead/disconnected/wrong id).
    uint64_t bytes_seen = rx_byte_count_.load(std::memory_order_relaxed) - bytes_before;
    uint64_t bytes_discarded =
      rx_discarded_count_.load(std::memory_order_relaxed) - discarded_before;
    std::cerr << "Timeout waiting for " << label << " response from motor " << static_cast<int>(id)
               << " (" << bytes_seen << " bytes seen, " << bytes_discarded << " discarded)"
               << std::endl;
  }
  return received;
}

void Motors::set_target(uint8_t id, float target, float acceleration_time, bool brake)
{
  protocol::DDSM210_packet_t packet;
  protocol::utils::prepare_packet(id, protocol::DDSM210_command::CMD_DRIVE, packet);
  packet.data.packet.drive.target =
    protocol::utils::convert(target, protocol::utils::VELOCITY_SCALE);
  packet.data.packet.drive.acceleration_time =
    protocol::utils::convert_u8(acceleration_time, protocol::utils::ACCELERATION_TIME_SCALE);
  packet.data.packet.drive.brake =
    brake ? protocol::DDSM210_brake::BRAKE : protocol::DDSM210_brake::NO_BRAKE;
  send_packet_and_wait_response(packet, "drive");
}

bool Motors::set_id(uint8_t id) { return set_id(id, id); }

// CMD_SET_ID must be addressed to the broadcast id; `new_id` is carried in the payload.
bool Motors::set_id(uint8_t /*id*/, uint8_t new_id)
{
  protocol::DDSM210_packet_t packet;
  protocol::utils::prepare_packet(
    protocol::DDSM210_SET_ID_BROADCAST_ADDRESS, protocol::DDSM210_command::CMD_SET_ID, packet);
  packet.data.packet.set_id.id = new_id;
  packet.data.packet.set_id.key = protocol::DDSM210_set_id_key::SET_ID_KEY;
  last_id_received_ = std::nullopt;
  for (uint i = 0; i < protocol::utils::SET_ID_RESEND_COUNT; i++) {
    send_packet(packet);
    usleep(protocol::utils::COMMANDS_DELAY_US);
  }

  // wait some time to get a response
  usleep(10 * protocol::utils::COMMANDS_DELAY_US);
  return last_id_received_.has_value() && last_id_received_.value() == new_id;
}

void Motors::register_feedback_callback(motor_feedback_callback callback)
{
  feedback_callback_ = callback;
}

Motors::~Motors()
{
  if (auto_fail_safe_) {
    set_fail_safe();
  }
  port_->close();
}

}  // namespace ddsm210_driver

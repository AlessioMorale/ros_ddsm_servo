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

#ifndef DDSM210_DRIVER__MOTORS_HPP_
#define DDSM210_DRIVER__MOTORS_HPP_
#include <stdint.h>
#include <sys/types.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "ddsm210_driver/comm/port.hpp"
#include "ddsm210_driver/protocol.hpp"

namespace ddsm210_driver
{
enum class command_mode
{
  MODE_INVALID = -1,
  MODE_OPENLOOP = 0,
  MODE_POSITION,
  MODE_VELOCITY
};

typedef struct
{
  uint8_t id;
  float velocity;
  float current;
  float acceleration;
  float temperature;
  bool overcurrent;
  bool overtemperature;
} Motor_feedback_t;

using motor_feedback_callback = std::function<void(const Motor_feedback_t &)>;
class Motors
{
public:
  Motors(std::vector<uint8_t> ids, std::unique_ptr<comm::Port> port, bool auto_fail_safe = true);
  ~Motors();
  bool set_id(uint8_t id);
  bool set_id(uint8_t id, uint8_t new_id);
  void drive(
    uint8_t id, command_mode mode, float setpoint, float acceleration_time, bool brake = false);
  void register_feedback_callback(motor_feedback_callback callback);
  void set_mode(uint8_t id, command_mode mode);
  void set_target(uint8_t id, float target, float acceleration_time = 0, bool brake = false);
  void set_fail_safe();

private:
  void receive_callback(std::vector<uint8_t> data);
  void process_packet(const protocol::DDSM210_packet_t & packet);
  void send_packet(protocol::DDSM210_packet_t & data);
  // Sends the packet and blocks (up to RESPONSE_TIMEOUT_US) until the matching response for
  // packet's id/cmd is received. Returns false and logs diagnostics on timeout.
  bool send_packet_and_wait_response(protocol::DDSM210_packet_t & packet, const char * label);
  std::unique_ptr<comm::Port> port_;
  bool auto_fail_safe_;
  std::function<void(const Motor_feedback_t &)> feedback_callback_;
  std::map<uint8_t, command_mode> motor_modes_;
  std::optional<uint8_t> last_id_received_ = std::nullopt;
  // Serial reads don't align with packet boundaries, so incoming bytes are accumulated here
  // until a full, checksum-valid frame can be extracted. Fixed-size to avoid heap churn/growth
  // on the receive path; oldest bytes are dropped if it ever fills up without a valid frame.
  static constexpr size_t RX_BUFFER_CAPACITY = sizeof(protocol::DDSM210_packet_t) * 4;
  std::array<uint8_t, RX_BUFFER_CAPACITY> rx_buffer_{};
  size_t rx_len_ = 0;
  // Diagnostics: total bytes seen on the wire and bytes discarded while resyncing/full, so a
  // timeout can be reported as "silence" vs "garbled/unsolicited traffic".
  std::atomic<uint64_t> rx_byte_count_{0};
  std::atomic<uint64_t> rx_discarded_count_{0};

  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  std::optional<uint8_t> awaited_id_;
  std::optional<protocol::DDSM210_command> awaited_cmd_;
  bool response_received_ = false;
};
};  // namespace ddsm210_driver
#endif  // DDSM210_DRIVER__MOTORS_HPP_

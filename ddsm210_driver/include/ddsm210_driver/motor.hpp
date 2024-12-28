#ifndef DDS210_DRIVER_HPP
#define DDS210_DRIVER_HPP
#include "ddsm210_driver/protocol.hpp"
#include "ddsm210_driver/comm/port.hpp"
#include <cstdint>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <stdint.h>
#include <sys/types.h>

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

using motor_feedback_callback = std::function<void(const Motor_feedback_t&)>;
class Motor
{
public:
  Motor(std::list<uint8_t> ids, std::unique_ptr<comm::Port> port, bool auto_fail_safe = true);
  ~Motor();
  bool set_id(uint8_t id);
  void drive(uint8_t id, command_mode mode, float setpoint, float acceleration_time, bool brake = false);
  void register_feedback_callback(motor_feedback_callback callback);
  void set_mode(uint8_t id, protocol::DDSM210_mode mode);
  void set_target(uint8_t id, float target, float acceleration_time = 0, bool brake = false);
  void set_fail_safe();
private:
  void receive_callback(std::vector<uint8_t> data);
  void send_packet(protocol::DDSM210_packet_t& data);
  std::unique_ptr<comm::Port> port_;
  bool auto_fail_safe_;
  std::function<void(const Motor_feedback_t&)> feedback_callback_;
  std::map<uint8_t, command_mode> motor_modes_;
  std::optional<uint8_t> last_id_received_ = std::nullopt;
};
};  // namespace ddsm210_driver
#endif  // DDS210_DRIVER_HPP
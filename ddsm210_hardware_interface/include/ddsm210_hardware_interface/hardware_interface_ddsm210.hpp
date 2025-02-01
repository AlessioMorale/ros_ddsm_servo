// Copyright (c) 2024, Alessio Morale
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DDSM210_HARDWARE_INTERFACE__HARDWARE_INTERFACE_DDSM210_HPP_
#define DDSM210_HARDWARE_INTERFACE__HARDWARE_INTERFACE_DDSM210_HPP_

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ddsm210_driver/motors.hpp"
#include "ddsm210_hardware_interface/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
namespace ddsm210_hardware_interface
{

class MotorError : public std::runtime_error
{
public:
  explicit MotorError(const std::string & msg) : std::runtime_error(msg) {}
};

class HardwareInterfaceDDSM210 : public hardware_interface::SystemInterface
{
public:
  HardwareInterfaceDDSM210();
  ~HardwareInterfaceDDSM210();
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  enum class MotorState
  {
    UNKNOWN,
    INITIALIZED,
    ACTIVE,
    INACTIVE,
    ERROR
  };

  size_t extract_joint_index(const std::string & interface_name);
  void emergency_stop(const std::string & reason);
  void stop_motors();
  void safety_monitor();
  bool is_motor_operational(size_t index) const;
  void motor_feedback_callback(const ddsm210_driver::Motor_feedback_t &);
  rclcpp::Logger logger_{rclcpp::get_logger("HardwareInterfaceDDSM210")};
  size_t motor_count_;
  std::vector<double> velocity_commands_;
  std::vector<double> effort_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<std::string> active_command_interfaces_;
  std::string serial_port_{};

  std::vector<MotorState> motor_states_;
  std::vector<uint8_t> motor_ids_;
  std::atomic<bool> is_emergency_stopped_;
  std::atomic<bool> is_initialized_;
  std::atomic<bool> stop_safety_monitor_{false};
  std::atomic<bool> is_system_running_{false};
  std::mutex interface_mutex_;
  std::thread safety_thread_;

  rclcpp::Time last_read_time_;
  rclcpp::Time last_write_time_;

  const double communication_timeout_;  // seconds
  std::unique_ptr<ddsm210_driver::Motors> motors_driver_;
};

}  // namespace ddsm210_hardware_interface

#endif  // DDSM210_HARDWARE_INTERFACE__HARDWARE_INTERFACE_DDSM210_HPP_

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

#include "ddsm210_hardware_interface/hardware_interface_ddsm210.hpp"

#include <sys/types.h>

#include <cstdlib>
#include <exception>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "ddsm210_driver/comm/serial_port.hpp"
#include "ddsm210_driver/motors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ddsm210_hardware_interface
{
const char SERIAL_PORT_PARAMETER_NAME[]{"device"};
HardwareInterfaceDDSM210::HardwareInterfaceDDSM210()
: motor_count_(0),
  velocity_commands_{},
  effort_commands_{},
  velocity_states_{},
  effort_states_{},
  motor_feedbacks_{},
  active_command_interfaces_{},
  is_emergency_stopped_(false),
  is_initialized_(false),
  communication_timeout_(1.0)  // seconds
{
  safety_thread_ = std::thread(&HardwareInterfaceDDSM210::safety_monitor, this);
}

HardwareInterfaceDDSM210::~HardwareInterfaceDDSM210()
{
  try {
    emergency_stop("Destructor called - shutting down safely");
    if (safety_thread_.joinable()) {
      stop_safety_monitor_ = true;
      safety_thread_.join();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error during shutdown: %s", e.what());
  }
}

hardware_interface::CallbackReturn HardwareInterfaceDDSM210::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);
  try {
    RCLCPP_INFO(logger_, "Motor interface configured");
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Configuration error: %s", e.what());
    emergency_stop("Failed to configure motors");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn HardwareInterfaceDDSM210::on_init(
  const hardware_interface::HardwareInfo & info)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  try {
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
      throw MotorError("Failed to initialize base system interface");
    }

    motor_count_ = info_.joints.size();
    if (motor_count_ == 0) {
      throw MotorError("No joints specified in hardware info");
    }

    // open the serial port
    serial_port_ = info.hardware_parameters.at(SERIAL_PORT_PARAMETER_NAME);
    RCLCPP_INFO(logger_, "Using serial port %s", serial_port_.c_str());

    auto serial_port_handle = std::make_unique<ddsm210_driver::comm::SerialPort>();

    if (!serial_port_handle->open(serial_port_, 115200)) {
      throw MotorError("Failed to open serial port");
    }

    velocity_commands_.resize(motor_count_, std::numeric_limits<double>::quiet_NaN());
    effort_commands_.resize(motor_count_, std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(motor_count_, std::numeric_limits<double>::quiet_NaN());
    effort_states_.resize(motor_count_, std::numeric_limits<double>::quiet_NaN());
    motor_states_.resize(motor_count_, MotorState::UNKNOWN);
    motor_ids_.resize(motor_count_, 0);
    motor_feedbacks_.resize(motor_count_);
    // Initialize command interface states
    active_command_interfaces_.resize(motor_count_);

    // Configure interfaces and verify parameters for each motor
    for (uint i = 0; i < info_.joints.size(); i++) {
      if (
        info_.joints[i].command_interfaces.size() != 2 ||
        info_.joints[i].state_interfaces.size() != 2) {
        throw MotorError("Joint " + std::to_string(i) + " missing required interfaces");
      }

      auto motor_id_param = info_.joints[i].parameters.find("motor_id");
      if (motor_id_param == info_.joints[i].parameters.end()) {
        throw MotorError("Joint " + std::to_string(i) + " missing motor_id parameter");
      }

      try {
        motor_ids_[i] = static_cast<uint8_t>(std::stoul(motor_id_param->second));
        if (motor_ids_[i] == 0) {
          throw std::exception();
        }
      } catch (const std::exception & e) {
        throw MotorError(
          "Joint " + std::to_string(i) + " has invalid motor_id parameter " +
          motor_id_param->second);
      }
      // Set default command interface to velocity
      active_command_interfaces_[i] = hardware_interface::HW_IF_VELOCITY;
      // Initialize motor state
      motor_states_[i] = MotorState::INITIALIZED;
      RCLCPP_INFO(
        logger_, "Setting up joint %s with motor ID %d", info_.joints[i].name.c_str(),
        motor_ids_[i]);
    }

    motors_driver_ =
      std::make_unique<ddsm210_driver::Motors>(motor_ids_, std::move(serial_port_handle), false);
    motors_driver_->register_feedback_callback(
      [this](const ddsm210_driver::Motor_feedback_t & feedback) {
        motor_feedback_callback(feedback);
      });

    is_initialized_ = true;
    last_read_time_ = rclcpp::Clock().now();
    last_write_time_ = last_read_time_;

    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Initialization error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> HardwareInterfaceDDSM210::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < motor_count_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HardwareInterfaceDDSM210::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < motor_count_; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HardwareInterfaceDDSM210::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  try {
    if (!is_initialized_) {
      throw MotorError("Cannot activate uninitialized interface");
    }

    if (is_emergency_stopped_) {
      throw MotorError("Cannot activate while emergency stopped");
    }

    // Initialize motor states and commands
    for (uint i = 0; i < motor_count_; i++) {
      if (!is_motor_operational(i)) {
        throw MotorError("Motor " + std::to_string(i) + " is not operational");
      }

      velocity_commands_[i] = 0.0;
      effort_commands_[i] = 0.0;
      velocity_states_[i] = 0.0;
      effort_states_[i] = 0.0;
      motor_states_[i] = MotorState::ACTIVE;
    }

    RCLCPP_INFO(logger_, "Motor interface activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Activation error: %s", e.what());
    emergency_stop("Failed to activate motors");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn HardwareInterfaceDDSM210::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  try {
    // Safely stop all motors
    stop_motors();
    for (uint i = 0; i < motor_count_; i++) {
      motor_states_[i] = MotorState::INACTIVE;
    }
    is_system_running_ = false;
    RCLCPP_INFO(logger_, "Motor interface deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Deactivation error: %s", e.what());
    emergency_stop("Failed to deactivate motors safely");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::return_type HardwareInterfaceDDSM210::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  if (is_emergency_stopped_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    for (uint i = 0; i < motor_count_; i++) {
      if (!is_motor_operational(i)) {
        throw MotorError("Motor " + std::to_string(i) + " is not responding");
      }

      velocity_states_[i] = motor_feedbacks_[i].velocity;
      effort_states_[i] = motor_feedbacks_[i].current;
    }

    last_read_time_ = rclcpp::Clock().now();
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Read error: %s", e.what());
    emergency_stop("Failed to read motor states");
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type HardwareInterfaceDDSM210::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);
  static rclcpp::Clock log_clock;
  if (is_emergency_stopped_) {
    return hardware_interface::return_type::ERROR;
  }
  is_system_running_ = true;

  try {
    for (uint i = 0; i < motor_count_; i++) {
      if (!is_motor_operational(i)) {
        throw MotorError("Motor " + std::to_string(i) + " is not operational");
      }
      float command = 0.0;
      ddsm210_driver::command_mode motor_mode{ddsm210_driver::command_mode::MODE_VELOCITY};
      if (active_command_interfaces_[i] == hardware_interface::HW_IF_EFFORT) {
        command = static_cast<float>(effort_commands_[i]);
        motor_mode = ddsm210_driver::command_mode::MODE_OPENLOOP;
      } else {
        command = static_cast<float>(velocity_commands_[i]);
      }
      motors_driver_->set_mode(motor_ids_[i], motor_mode);
      motors_driver_->set_target(motor_ids_[i], command);

      RCLCPP_INFO_THROTTLE(
        logger_, log_clock, 30000, "Motor %d: velocity=%.2f, effort=%.2f", i, velocity_commands_[i],
        effort_commands_[i]);
    }

    last_write_time_ = rclcpp::Clock().now();
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Write error: %s", e.what());
    emergency_stop("Failed to write motor commands");
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type HardwareInterfaceDDSM210::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Update active command interfaces
  std::lock_guard<std::mutex> lock(interface_mutex_);
  for (const auto & interface : stop_interfaces) {
    // Extract joint index from interface name (assuming format "joint<N>/<interface_type>")
    size_t joint_idx = extract_joint_index(interface);
    if (joint_idx >= motor_count_) continue;

    // Reset commands for stopped interface
    if (interface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
      velocity_commands_[joint_idx] = 0.0;
    } else if (interface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
      effort_commands_[joint_idx] = 0.0;
    }
  }

  for (const auto & interface : start_interfaces) {
    size_t joint_idx = extract_joint_index(interface);
    if (joint_idx >= motor_count_) continue;

    // Update active command interface
    if (interface == hardware_interface::HW_IF_VELOCITY) {
      active_command_interfaces_[joint_idx] = hardware_interface::HW_IF_VELOCITY;
    } else if (interface == hardware_interface::HW_IF_EFFORT) {
      active_command_interfaces_[joint_idx] = hardware_interface::HW_IF_EFFORT;
    }
    RCLCPP_INFO(
      logger_, "Switched joint %zu to %s control", joint_idx,
      active_command_interfaces_[joint_idx].c_str());
  }

  return hardware_interface::return_type::OK;
}

void HardwareInterfaceDDSM210::motor_feedback_callback(
  const ddsm210_driver::Motor_feedback_t & feedback)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);
  static rclcpp::Clock log_clock;

  size_t motor_index = 0;
  for (size_t i = 0; i < motor_ids_.size(); i++) {
    if (motor_ids_[i] == feedback.id) {
      motor_index = i;
      break;
    }
  }

  if (motor_index >= motor_count_) {
    RCLCPP_ERROR(logger_, "Received feedback for unknown motor ID %d", feedback.id);
    return;
  }

  // Update motor states
  motor_feedbacks_[motor_index] = feedback;
  RCLCPP_INFO_THROTTLE(
    logger_, log_clock, 10000, "Feedback for %zu: velocity=%.2f, effort=%.2f", motor_index,
    velocity_states_[motor_index], effort_states_[motor_index]);
}

void HardwareInterfaceDDSM210::emergency_stop(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  if (!is_emergency_stopped_) {
    is_emergency_stopped_ = true;
    RCLCPP_ERROR(logger_, "EMERGENCY STOP triggered: %s", reason.c_str());

    try {
      // Immediately stop all motors
      stop_motors();
      for (uint i = 0; i < motor_count_; i++) {
        motor_states_[i] = MotorState::ERROR;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error during emergency stop: %s", e.what());
    }
  }
}

void HardwareInterfaceDDSM210::stop_motors()
{
  try {
    for (uint i = 0; i < motor_count_; i++) {
      velocity_commands_[i] = 0.0;
      effort_commands_[i] = 0.0;
    }

    motors_driver_->set_fail_safe();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error stopping motors: %s", e.what());
    throw;
  }
}

void HardwareInterfaceDDSM210::safety_monitor()
{
  const auto check_period = std::chrono::milliseconds(100);  // 10Hz safety checks

  while (!stop_safety_monitor_) {
    try {
      if (!is_system_running_) {
        std::this_thread::sleep_for(check_period);
        continue;
      }
      // Check communication timeouts
      auto now = rclcpp::Clock().now();
      if ((now - last_read_time_).seconds() > communication_timeout_) {
        emergency_stop("Communication timeout - no recent reads");
      }
      if ((now - last_write_time_).seconds() > communication_timeout_) {
        emergency_stop("Communication timeout - no recent writes");
      }

      // Check motor states
      for (uint i = 0; i < motor_count_; i++) {
        if (!is_motor_operational(i)) {
          emergency_stop("Motor " + std::to_string(i) + " is not responding");
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Safety monitor error: %s", e.what());
    }

    std::this_thread::sleep_for(check_period);
  }
}

bool HardwareInterfaceDDSM210::is_motor_operational(size_t index) const
{
  if (index >= motor_count_) return false;
  return motor_states_[index] == MotorState::ACTIVE ||
         motor_states_[index] == MotorState::INITIALIZED;
}

// Helper method to extract joint index from interface name
size_t HardwareInterfaceDDSM210::extract_joint_index(const std::string & interface_name)
{
  size_t joint_pos = interface_name.find("joint");
  size_t separator_pos = interface_name.find("/");
  if (joint_pos == std::string::npos || separator_pos == std::string::npos) {
    return motor_count_;  // Return invalid index
  }

  std::string index_str = interface_name.substr(joint_pos + 5, separator_pos - (joint_pos + 5));
  try {
    return std::stoul(index_str);
  } catch (...) {
    return motor_count_;  // Return invalid index
  }
}

}  // namespace ddsm210_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ddsm210_hardware_interface::HardwareInterfaceDDSM210, hardware_interface::SystemInterface)

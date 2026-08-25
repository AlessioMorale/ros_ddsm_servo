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

#include <unistd.h>

#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "boost/asio/io_context.hpp"
#include "ddsm210_driver/comm/serial_port.hpp"
#include "ddsm210_driver/motors.hpp"

namespace
{
struct ModeInfo
{
  ddsm210_driver::command_mode mode;
  const char * name;
};

constexpr ModeInfo kModes[] = {
  {ddsm210_driver::command_mode::MODE_OPENLOOP, "open-loop (duty cycle)"},
  {ddsm210_driver::command_mode::MODE_POSITION, "position"},
  {ddsm210_driver::command_mode::MODE_VELOCITY, "velocity (RPM)"},
};

std::string mode_name(ddsm210_driver::command_mode mode)
{
  for (const auto & m : kModes) {
    if (m.mode == mode) {
      return m.name;
    }
  }
  return "unknown";
}

void print_mode_help()
{
  std::cerr << "      <mode> is one of:" << std::endl;
  for (const auto & m : kModes) {
    std::cerr << "        " << static_cast<int>(m.mode) << " = " << m.name << std::endl;
  }
}
}  // namespace

int usage(char * argv[])
{
  std::cerr << "Usage: " << argv[0] << " <serial_port> <id> <action> [<value>]" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Arguments:" << std::endl;
  std::cerr << "  <serial_port>  Path to the serial device, e.g. /dev/ttyAMA0" << std::endl;
  std::cerr << "  <id>           Motor bus address to talk to (0-255)" << std::endl;
  std::cerr << "  <action>       One of: set_mode, set_target, set_id" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Actions:" << std::endl;
  std::cerr << "  set_mode <mode>" << std::endl;
  std::cerr << "      Switch the motor at <id> to the given control mode." << std::endl;
  print_mode_help();
  std::cerr << std::endl;
  std::cerr << "  set_target <target>" << std::endl;
  std::cerr << "      Send a target setpoint to the motor at <id>." << std::endl;
  std::cerr << "      <target> meaning depends on the motor's current mode:" << std::endl;
  std::cerr << "        velocity mode:  target speed in RPM" << std::endl;
  std::cerr << "        position mode:  target position" << std::endl;
  std::cerr << "        open-loop mode: target duty cycle" << std::endl;
  std::cerr << std::endl;
  std::cerr << "  set_id [<new_id>]" << std::endl;
  std::cerr << "      Assign a new bus address to the motor currently addressed as <id>."
            << std::endl;
  std::cerr << "      <new_id> is the address to assign (0-255); defaults to <id> if omitted,"
            << std::endl;
  std::cerr << "      which only re-confirms the motor's current address." << std::endl;
  std::cerr << std::endl;
  std::cerr << "Examples:" << std::endl;
  std::cerr << "  " << argv[0] << " /dev/ttyAMA0 1 set_mode 2" << std::endl;
  std::cerr << "  " << argv[0] << " /dev/ttyAMA0 1 set_target 120" << std::endl;
  std::cerr << "  " << argv[0] << " /dev/ttyAMA0 1 set_id 16" << std::endl;
  return 1;
}
int main(int argc, char * argv[])
{
  if (argc < 4) {
    return usage(argv);
  }

  std::string serial_port = argv[1];
  uint8_t id = std::atoi(argv[2]);
  std::string action = argv[3];
  int value = 0;

  std::cout << "Serial port:    " << serial_port << std::endl;
  std::cout << "Motor ID:       " << static_cast<int>(id) << std::endl;
  std::cout << "Action:         " << action << std::endl;
  if (argc == 5) {
    value = std::atoi(argv[4]);
    if (action == "set_mode") {
      const auto mode = static_cast<ddsm210_driver::command_mode>(value);
      std::cout << "Control mode:   " << value << " (" << mode_name(mode) << ")" << std::endl;
    } else if (action == "set_target") {
      std::cout << "Target:         " << value << std::endl;
    } else if (action == "set_id") {
      std::cout << "New ID:         " << value << std::endl;
    } else {
      std::cout << "Value:          " << value << std::endl;
    }
  }
  auto ret = 0;
  auto port = std::make_unique<ddsm210_driver::comm::SerialPort>();
  if (!port->open(serial_port, 115200)) {
    std::cerr << "Error: failed to open serial port '" << serial_port << "'" << std::endl;
    ret = 1;
  } else {
    std::cout << "Serial port opened" << std::endl;
  }

  if (ret == 0) {
    auto motor =
      std::make_unique<ddsm210_driver::Motors>(std::vector<uint8_t>{id}, std::move(port), false);
    motor->register_feedback_callback([](const ddsm210_driver::Motor_feedback_t & feedback) {
      std::cout << "Feedback:" << std::endl;
      std::cout << "  ID:              " << static_cast<int>(feedback.id) << std::endl;
      std::cout << "  Velocity (RPM):  " << feedback.velocity << std::endl;
      std::cout << "  Current (A):     " << feedback.current << std::endl;
      std::cout << "  Acceleration:    " << feedback.acceleration << std::endl;
      std::cout << "  Temperature (C): " << feedback.temperature << std::endl;
      std::cout << "  Overcurrent:     " << feedback.overcurrent << std::endl;
      std::cout << "  Overtemperature: " << feedback.overtemperature << std::endl;
    });
    try {
      if (action == "set_mode") {
        if (argc != 5) {
          std::cerr << "Error: set_mode requires <mode>" << std::endl;
          return usage(argv);
        }
        motor->set_mode(id, static_cast<ddsm210_driver::command_mode>(value));
      } else if (action == "set_target") {
        if (argc != 5) {
          std::cerr << "Error: set_target requires <target>" << std::endl;
          return usage(argv);
        }
        motor->set_target(id, value, 3);
      } else if (action == "set_id") {
        // <id> is the motor's current bus address; the value argument is the new id to assign.
        const uint8_t new_id = argc == 5 ? static_cast<uint8_t>(value) : id;
        const bool ok = motor->set_id(id, new_id);
        std::cout << "Set ID to " << static_cast<int>(new_id) << ": "
                  << (ok ? "success" : "failed") << std::endl;
        if (!ok) {
          ret = 1;
        }
      } else {
        std::cerr << "Error: unknown action '" << action << "'" << std::endl;
        ret = usage(argv);
      }
    } catch (const std::exception & e) {
      std::cerr << "Error: " << e.what() << std::endl;
      ret = 1;
    }
    usleep(1000000);
    motor.reset();
  }
  return ret;
}


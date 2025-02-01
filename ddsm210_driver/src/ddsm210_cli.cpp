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

int usage(char * argv[])
{
  std::cerr << "Usage: " << argv[0] << " <serial_port> <id> <action> [<value>]" << std::endl;
  std::cerr << "Actions:" << std::endl;
  std::cerr << "  set_mode <mode>" << std::endl;
  std::cerr << "  set_target <target>" << std::endl;
  std::cerr << "  set_id" << std::endl;
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

  std::cout << "Serial Port: " << serial_port << std::endl;
  std::cout << "ID: " << static_cast<int>(id) << std::endl;
  std::cout << "Action: " << action << std::endl;
  if (argc == 5) {
    value = std::atoi(argv[4]);
    std::cout << "Value: " << value << std::endl;
  }
  auto ret = 0;
  auto port = std::make_unique<ddsm210_driver::comm::SerialPort>();
  if (!port->open(serial_port, 115200)) {
    std::cerr << "Failed to open serial port" << std::endl;
    ret = 1;
  } else {
    std::cout << "Serial port opened" << std::endl;
  }

  if (ret == 0) {
    auto motor =
      std::make_unique<ddsm210_driver::Motors>(std::vector<uint8_t>{id}, std::move(port), false);
    motor->register_feedback_callback([](const ddsm210_driver::Motor_feedback_t & feedback) {
      std::cout << "Feedback: " << std::endl;
      std::cout << "  ID: " << static_cast<int>(feedback.id) << std::endl;
      std::cout << "  Velocity: " << feedback.velocity << std::endl;
      std::cout << "  Current: " << feedback.current << std::endl;
      std::cout << "  Acceleration: " << feedback.acceleration << std::endl;
      std::cout << "  Temperature: " << feedback.temperature << std::endl;
      std::cout << "  Overcurrent: " << feedback.overcurrent << std::endl;
      std::cout << "  Overtemperature: " << feedback.overtemperature << std::endl;
    });
    if (action == "set_mode") {
      if (argc != 5) {
        ret = usage(argv);
      }
      motor->set_mode(id, static_cast<ddsm210_driver::command_mode>(value));
    } else if (action == "set_target") {
      if (argc != 5) {
        ret = usage(argv);
      }
      motor->set_target(id, value, 3);
    } else if (action == "set_id") {
    } else {
      ret = usage(argv);
    }
    usleep(1000000);
    motor.reset();
  }
  return ret;
}

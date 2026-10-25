// Copyright (c) 2026, Alessio Morale
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

#ifndef DDSM210_HARDWARE_INTERFACE__UNITS_HPP_
#define DDSM210_HARDWARE_INTERFACE__UNITS_HPP_

#include <algorithm>
#include <cmath>

namespace ddsm210_hardware_interface::units
{
// ros2_control velocity interfaces are in rad/s; the DDSM210 driver speaks rpm (the protocol
// unit is 0.1 rpm, ddsm210_driver::Motors takes and reports rpm).
constexpr double PI = 3.14159265358979323846;
constexpr double RPM_PER_RAD_S = 60.0 / (2.0 * PI);

// No-load speed of the DDSM210 is 210 +/- 11 rpm; the protocol field is +/-2100 (0.1 rpm).
constexpr double MAX_RPM = 210.0;

constexpr double rad_s_to_rpm(double rad_s) { return rad_s * RPM_PER_RAD_S; }
constexpr double rpm_to_rad_s(double rpm) { return rpm / RPM_PER_RAD_S; }

// Velocity command (rad/s) -> driver target (rpm), limited to what the motor can do. A NaN
// command (interface not written yet) stops the motor instead of reaching the integer cast.
inline double command_rad_s_to_rpm(double rad_s)
{
  if (std::isnan(rad_s)) {
    return 0.0;
  }
  return std::clamp(rad_s_to_rpm(rad_s), -MAX_RPM, MAX_RPM);
}

}  // namespace ddsm210_hardware_interface::units

#endif  // DDSM210_HARDWARE_INTERFACE__UNITS_HPP_

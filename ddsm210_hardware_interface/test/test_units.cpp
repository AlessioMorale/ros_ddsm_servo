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

#include <gtest/gtest.h>

#include <cmath>
#include <arpa/inet.h>

#include <cstdint>
#include <limits>

#include "ddsm210_driver/protocol.hpp"
#include "ddsm210_hardware_interface/units.hpp"

using namespace ddsm210_hardware_interface;

TEST(Units, one_revolution_per_second_is_sixty_rpm)
{
  EXPECT_NEAR(units::rad_s_to_rpm(2.0 * units::PI), 60.0, 1e-9);
  EXPECT_NEAR(units::rpm_to_rad_s(60.0), 2.0 * units::PI, 1e-9);
}

TEST(Units, round_trip)
{
  for (double rad_s : {-20.0, -1.0, 0.0, 0.5, 5.5, 21.0}) {
    EXPECT_NEAR(units::rpm_to_rad_s(units::rad_s_to_rpm(rad_s)), rad_s, 1e-12);
  }
}

TEST(Units, command_is_limited_to_the_motor_range)
{
  EXPECT_NEAR(units::command_rad_s_to_rpm(5.0), 47.746, 1e-3);
  EXPECT_DOUBLE_EQ(units::command_rad_s_to_rpm(100.0), 210.0);
  EXPECT_DOUBLE_EQ(units::command_rad_s_to_rpm(-100.0), -210.0);
}

TEST(Units, nan_command_stops_the_motor)
{
  EXPECT_DOUBLE_EQ(units::command_rad_s_to_rpm(std::numeric_limits<double>::quiet_NaN()), 0.0);
}

// A commanded wheel speed must come back as the same wheel speed through the whole wire path:
// rad/s -> rpm -> 0.1 rpm integer, big-endian -> and back (what the motor would echo).
TEST(Units, command_to_wire_and_back_is_within_protocol_resolution)
{
  using ddsm210_driver::protocol::utils;
  for (double rad_s : {-21.0, -5.5, -0.1, 0.0, 0.1, 5.5, 21.0}) {
    const float rpm = static_cast<float>(units::command_rad_s_to_rpm(rad_s));
    const int16_t wire = utils::convert(rpm, utils::VELOCITY_SCALE);
    const double back = units::rpm_to_rad_s(static_cast<double>(utils::convert(wire, utils::VELOCITY_SCALE)));
    // 0.1 rpm resolution, truncated: at most 0.1 rpm = 0.0105 rad/s
    EXPECT_NEAR(back, rad_s, 0.0105) << rad_s << " rad/s";
  }
  // 5.5 rad/s is 52.5 rpm = 525 in protocol units
  const int16_t wire = utils::convert(static_cast<float>(units::rad_s_to_rpm(5.5)), utils::VELOCITY_SCALE);
  EXPECT_EQ(static_cast<int16_t>(ntohs(static_cast<uint16_t>(wire))), 525);
}

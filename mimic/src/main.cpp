// Copyright 2026 Malia Labor and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstddef>
#include <cstdint>

#include <array>

#include <libhal-actuator/mx_64.hpp>
#include <libhal-actuator/rx_64.hpp>
#include <libhal-expander/tca9548a.hpp>
#include <libhal-sensor/as5600.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

enum class angle_select : uint8_t
{
  spin = 0,
  shoulder_angle = 1,
  elbow_angle = 2,
  wrist_angle = 3
};

hal::degrees process_angle(hal::degrees p_angle, hal::degrees p_servo_center)
{
  hal::degrees const sensor_center = 270.0f;
  // sensor cycles back to 0, but limit to only 180 degrees
  if (p_angle < 15.0f) {
    p_angle = 360.0f;
  }
  p_angle = std::clamp(p_angle, 180.0f, 360.0f);
  hal::degrees angle_diff = sensor_center - p_angle;
  return p_servo_center - angle_diff;
}

void application();

int main()
{
  application();
}

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto const clock = resources::clock();
  auto const console = resources::console();
  auto const i2c = resources::i2c();
  auto const uart = resources::uart2();

  hal::print(*console, "Mimic Application Starting...\n");

  hal::actuator::rx_64::config wrist_config = {
    .baud_rate = 57600, .id = 3, .min_angle = 60, .max_angle = 240
  };
  hal::actuator::mx_64::config elbow_config = {
    .baud_rate = 57600, .id = 0, .min_angle = 90, .max_angle = 270
  };
  hal::actuator::rx_64::config shoulder_rotate_config = {
    .baud_rate = 57600, .id = 4, .min_angle = 0, .max_angle = 300
  };
  hal::actuator::rx_64::config shoulder_lead_config = {
    .baud_rate = 57600, .id = 1, .min_angle = 60, .max_angle = 240
  };
  hal::actuator::rx_64::config shoulder_opose_config = {
    .baud_rate = 57600, .id = 2, .min_angle = 60, .max_angle = 240
  };

  auto wrist_servo = hal::actuator::rx_64(uart, wrist_config, clock);
  auto elbow_servo = hal::actuator::mx_64(uart, elbow_config, clock);
  auto spin_servo = hal::actuator::rx_64(uart, shoulder_rotate_config, clock);
  auto shoulder_lead_servo =
    hal::actuator::rx_64(uart, shoulder_lead_config, clock);
  auto shoulder_opose_servo =
    hal::actuator::rx_64(uart, shoulder_opose_config, clock);

  wrist_servo.torque_limit(50.0f);
  elbow_servo.torque_limit(50.0f);
  spin_servo.torque_limit(50.0f);
  shoulder_lead_servo.torque_limit(50.0f);
  shoulder_opose_servo.torque_limit(50.0f);

  wrist_servo.led(false);
  elbow_servo.led(false);
  spin_servo.led(false);
  shoulder_lead_servo.led(false);
  shoulder_opose_servo.led(false);

  auto i2c_mux = hal::expander::tca9548a(*i2c);
  std::bitset<8> init_ports{ 0x00 };
  init_ports.set(0);
  i2c_mux.set_ports(init_ports);
  hal::sensor::as5600 hall_sensor(i2c);

  std::array<hal::degrees, 4> mimic_angles;

  while (true) {
    using namespace std::literals;
    using namespace hal;
    for (size_t i = 0; i < mimic_angles.size(); i++) {
      std::bitset<8> ports{ 0x00 };
      ports.set(i);
      i2c_mux.set_ports(ports);

      hal::delay(*clock, 10ms);
      auto magnet_status = hall_sensor.magnet_status();
      if (magnet_status.detected) {
        mimic_angles[i] = hall_sensor.raw_angle();
      }
      // hal::delay(*clock, 10ms);
    }

    auto shoulder_angle =
      process_angle(mimic_angles[(u8)angle_select::shoulder_angle], 150.0f);

    auto elbow_angle =
      process_angle(mimic_angles[(u8)angle_select::elbow_angle], 180.0f);

    auto wrist_angle =
      process_angle(mimic_angles[(u8)angle_select::wrist_angle], 150.0f);

    hal::print<64>(
      *console, "Spin: %.2f    ", mimic_angles[(u8)angle_select::spin]);
    hal::print<64>(*console, "Shoulder: %.2f    ", shoulder_angle);
    hal::print<64>(*console, "Elbow: %.2f    ", elbow_angle);
    hal::print<64>(*console, "Wrist: %.2f \n", wrist_angle);

    spin_servo.position(mimic_angles[(u8)angle_select::spin]);

    shoulder_lead_servo.sync_position(shoulder_angle, shoulder_opose_servo);
    elbow_servo.position(elbow_angle);
    wrist_servo.position(wrist_angle);
  }
}

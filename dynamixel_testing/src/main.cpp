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

#include <cinttypes>
#include <cstddef>
#include <cstdint>

#include <array>
#include <utility>

#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>
#include <libhal-expander/tca9548a.hpp>
#include <libhal-sensor/as5600.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <dynamixel.hpp>
#include <resource_list.hpp>

int main()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  initialize_platform();

  auto const clock = resources::clock();
  auto const console = resources::console();
  auto const status_led = resources::status_led();
  auto const uart = resources::uart2();
  hal::print(*console, "Dynamixel Servo Protocol 1: Motor Test Starting...\n");

  [[maybe_unused]] constexpr hal::actuator::dynamixel_servo_protocol_1::config
    wrist_config = {
      .servo = hal::actuator::dynamixel_servo::rx,
      .baud_rate = 57600,
      .id = 3,
      .min_angle = 60,
      .max_angle = 240,
    };

  [[maybe_unused]] constexpr hal::actuator::dynamixel_servo_protocol_1::config
    elbow_config = {
      .servo = hal::actuator::dynamixel_servo::mx,
      .baud_rate = 57600,
      .id = 0,
      .min_angle = 90,
      .max_angle = 270,
    };

  hal::optional_ptr<hal::actuator::dynamixel_servo_protocol_1> servo;
  hal::optional_ptr<hal::actuator::dynamixel_servo_protocol_1> servo2;

  while (true) {
    try {
      servo = hal::make_strong_ptr<hal::actuator::dynamixel_servo_protocol_1>(
        resources::driver_allocator(), uart, clock, elbow_config);
      hal::print(*console, "Servo constructed [1]!\n");
      hal::delay(*clock, 10ms);

      servo->torque_limit(100.0f);
      hal::print(*console, "Servo torque limit set [2]!\n");
      hal::delay(*clock, 10ms);

      servo->torque_enable(true);
      hal::print(*console, "Servo torque enabled [3]!\n");
      hal::delay(*clock, 10ms);

      servo->speed(60.0f);
      hal::print(*console, "Servo speed set [4]!\n");
      hal::delay(*clock, 10ms);

      servo->led(true);
      hal::print(*console, "Servo led set [5]!\n");
      hal::delay(*clock, 10ms);

      servo->pid_p_gain(64);
      hal::print(*console, "Servo PID set to 8 [6]!\n");
      hal::delay(*clock, 10ms);

      hal::print(*console, "Servo initialized!\n");

      // ======

      servo2 = hal::make_strong_ptr<hal::actuator::dynamixel_servo_protocol_1>(
        resources::driver_allocator(), uart, clock, wrist_config);
      hal::print(*console, "Servo2 constructed [1]!\n");
      hal::delay(*clock, 10ms);

      servo2->torque_limit(100.0f);
      hal::print(*console, "Servo2 torque limit set [2]!\n");
      hal::delay(*clock, 10ms);

      servo2->torque_enable(true);
      hal::print(*console, "Servo2 torque enabled [3]!\n");
      hal::delay(*clock, 10ms);

      servo2->speed(30.0f);
      hal::print(*console, "Servo2 speed set [4]!\n");
      hal::delay(*clock, 10ms);

      servo2->led(true);
      hal::print(*console, "Servo2 led set [5]!\n");
      hal::delay(*clock, 10ms);
      hal::print(*console, "Servo2 initialized!\n");

      break;
    } catch (hal::io_error const&) {
      hal::print(*console, "❌ Servo init FAILED due to io_error!\n");
    } catch (hal::timed_out const&) {
      hal::print(*console, "❌ Servo init FAILED due to timed_out!\n");
    }
  }

  int timeout_counter = 0;
  int io_error_counter = 0;
  int action_counter = 0;

  auto print_status = [&]() {
    hal::print<64>(*console,
                   "[(%d) W:%" PRIu8 " | TM:%d | IO:%d ]\n",
                   action_counter,
                   servo->last_error_code(),
                   timeout_counter,
                   io_error_counter);
  };

  hal::print(*console, "Setting up servo position...\n");
  servo->queue_position(180.0f);
  hal::delay(*clock, 10ms);
  servo2->queue_position(180.0f);
  hal::delay(*clock, 10ms);
  servo->execute_action();
  hal::delay(*clock, 10ms);
  servo2->execute_action();
  hal::print(*console, "Holding for 3 seconds...\n");
  hal::delay(*clock, 3s);

  constexpr auto degree_change = 5.0f;
  while (true) {
    int select = 0;
    for (hal::degrees deg = 120.0f; deg < 220.0f; deg += degree_change) {
      select++;
      hal::delay(*clock, 500ms);
      try {
        action_counter++;
        print_status();
        hal::print<64>(*console, "deg = %d\n", static_cast<int>(deg));

        // if (select & 1) {
        servo->queue_position(deg);
        hal::delay(*clock, 10ms);
        servo->execute_action();
        hal::delay(*clock, 10ms);
        // } else {
        // servo2->queue_position(deg);
        // hal::delay(*clock, 10ms);
        // servo2->execute_action();
        // hal::delay(*clock, 10ms);
        // }
      } catch (hal::timed_out const&) {
        hal::print(*console, "❌  ⏰\n");
        timeout_counter++;
        deg -= degree_change;
      } catch (hal::io_error const&) {
        hal::print(*console, "❌  📡\n");
        io_error_counter++;
        deg -= degree_change;
      } catch (...) {
        hal::print(*console, "⁉️\n");
      }
    }
    for (hal::degrees deg = 220.0f; deg > 120.0f; deg -= degree_change) {
      select++;
      hal::delay(*clock, 500ms);
      try {
        action_counter++;
        print_status();
        hal::print<64>(*console, "deg = %d\n", static_cast<int>(deg));

        // if (select & 1) {
        servo->queue_position(deg);
        hal::delay(*clock, 10ms);
        servo->execute_action();
        hal::delay(*clock, 10ms);
        // } else {
        // servo2->queue_position(deg);
        // hal::delay(*clock, 10ms);
        // servo2->execute_action();
        // hal::delay(*clock, 10ms);
        // }

      } catch (hal::timed_out const&) {
        hal::print(*console, "❌  ⏰\n");
        timeout_counter++;
        deg += degree_change;
      } catch (hal::io_error const&) {
        hal::print(*console, "❌  📡\n");
        io_error_counter++;
        deg += degree_change;
      } catch (...) {
        hal::print(*console, "⁉️\n");
      }
    }
  }
}

extern "C"
{
  [[gnu::noreturn]] void _exit(int p_status)
  {
    [[maybe_unused]] int volatile status = p_status;
    while (true) {
      continue;
    }
  }
}

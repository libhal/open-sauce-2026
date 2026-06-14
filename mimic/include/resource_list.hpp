// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#pragma once

#include <memory_resource>

#include <libhal/can.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace resources {
std::pmr::polymorphic_allocator<> driver_allocator();
void reset();
void sleep(hal::time_duration p_duration);
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::serial> uart2();
hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::output_pin> status_led();
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel();
hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency();
hal::v5::strong_ptr<hal::i2c> i2c();
hal::v5::strong_ptr<hal::input_pin> pump_button();
hal::v5::strong_ptr<hal::output_pin> pump_direction();
hal::v5::strong_ptr<hal::pwm16_channel> pump_power();
hal::v5::strong_ptr<hal::pwm_group_manager> pump_power_frequency();
}  // namespace resources

// Application function is implemented by one of the .cpp files.
void initialize_platform();
void application();

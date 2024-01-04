/**
 * @file wheel_controller.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief WheelController class implementation that initializes the controller
 * and instantiates a Wheel. It controls and sets the logic as requested from
 * CanBus message.
 * @version 0.1
 * @date 2021-10-10
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 * @license This project is released under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "wheel_controller.h" // for WheelController
#include "constants.h"        // for internal::*
#include "gpio.h"             // for HAL_GPIO*
#include "stm32f1xx_hal.h"    // for HAL_GetTick
#include "stm32f1xx_ll_tim.h" // for LL_TIM_OC_SetCompareCH3
#include "tim.h"              // for htim1

#include <cmath>   // for std::abs
#include <cstdint> // for int Data Types

bool WheelController::Init(const Wheel &wheel) {
  wheel_ = &wheel;
  timeout_ = HAL_GetTick();

  Stop(false);
  Drive(false);

  return true;
}

void WheelController::WheelSignalIrqHandler() {
  if (is_reverse_) {
    if (signal_counter_ == INT64_MIN) {
      signal_counter_ = 0;
    } else {
      signal_counter_--;
    }
  } else {
    if (signal_counter_ == INT64_MAX) {
      signal_counter_ = 0;
    } else {
      signal_counter_++;
    }
  }
}

bool WheelController::CalculateWheelOdometry() {
  std::call_once(first_odometry_tick_, [this]() {
    old_time_ = HAL_GetTick();
    old_signal_counter_ = signal_counter_;
  });

  uint64_t current_time = HAL_GetTick();
  time_taken_ = current_time - old_time_;

  int64_t current_signal_counter = signal_counter_;
  delta_signal_counter_ =
      std::abs(current_signal_counter - old_signal_counter_);

  int sign = is_reverse_ ? -1 : 1;

  double position = (internal::wheel::kCircumferenceMeters *
                     static_cast<double>(current_signal_counter)) /
                    internal::wheel::kPulsePerRevolution;
  wheel_feedback_status_.Position(position);

  double rpm =
      (internal::wheel::kPulsePerRevolution * delta_signal_counter_) /
      (internal::wheel::kCircumferenceMillimeters * time_taken_ / 6000.0);
  wheel_feedback_status_.Rpm(rpm);

  double velocity = sign *
                    (internal::wheel::kCircumferenceMillimeters /
                     internal::wheel::kPulsePerRevolution) *
                    delta_signal_counter_ / time_taken_;
  wheel_feedback_status_.Velocity(velocity);

  if (velocity <= -internal::kMinVelocityToEffort ||
      velocity >= internal::kMinVelocityToEffort) {
    wheel_feedback_status_.Effort(internal::wheel::kPowerInWatt /
                                  std::abs(velocity));
  }

  old_time_ = current_time;
  old_signal_counter_ = current_signal_counter;
  return true;
}

void WheelController::UpdateTimeout() { timeout_ = HAL_GetTick(); }

bool WheelController::TimeoutCheck() const {
  uint64_t current_timeout = HAL_GetTick();
  if (current_timeout - timeout_ >= internal::kTimeoutMillis) {
    timeout_ = current_timeout;
    return true;
  }

  return false;
}

void WheelController::SetDirection(const bool &direction) const {
  is_reverse_ = !direction;

  // Active Low: Direction clockwise
  if (direction) {
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
  }
}

void WheelController::SetSpeed(const uint8_t &speed) const {
  if (speed == 0) {
    Drive(false);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  } else {
    Drive(true);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  }

  LL_TIM_OC_SetCompareCH3(TIM1, speed);
}

void WheelController::SetSpeedAndDirection(const uint8_t &speed,
                                           const bool &direction) const {
  SetDirection(direction);
  SetSpeed(speed);
}

WheelController::WheelStatus WheelController::WheelFeedbackStatus() const {
  const auto wheel_feedback_status = wheel_feedback_status_;

  // Reset all the values after the feedback is requested except the position
  wheel_feedback_status_.Effort(0);
  wheel_feedback_status_.Rpm(0);
  wheel_feedback_status_.Velocity(0.0);

  return wheel_feedback_status;
}

void WheelController::Brake(const bool &brake) const {
  // Active High: Brake applied
  if (brake) {
    HAL_GPIO_WritePin(MOTOR_BRK_GPIO_Port, MOTOR_BRK_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(MOTOR_BRK_GPIO_Port, MOTOR_BRK_Pin, GPIO_PIN_RESET);
  }
}

void WheelController::Stop(const bool &stop) const {
  // Active Low: Drive disabled
  if (stop) {
    HAL_GPIO_WritePin(MOTOR_STOP_GPIO_Port, MOTOR_STOP_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(MOTOR_STOP_GPIO_Port, MOTOR_STOP_Pin, GPIO_PIN_SET);
  }
}

void WheelController::Drive(const bool &drive) const { Brake(!drive); }

/**
 * @file wheel_controller.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Declaration of the WheelController class that controls a Wheel object
 * according to messages received over CanBus.
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics
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
#ifndef CAN_MOTOR_CONTROLLER_CORE_INC_WHEEL_CONTROLLER_H_
#define CAN_MOTOR_CONTROLLER_CORE_INC_WHEEL_CONTROLLER_H_
#include "stl_helper_functions.h" // for std::call_once and std::once_flag
#include "wheel.h"                // for Wheel

#include <cstdint> // for int Data Types

/**
 * @brief Controls a Wheel object according to messages received over CanBus.
 */
class WheelController {
public:
  /**
   * @brief Struct that contains the status of the Wheel.
   */
  using WheelStatus = struct WheelStatus {
  public:
    /**
     * @brief Gets the effort value associated with the WheelStatus.
     */
    uint16_t Effort() const { return effort; }

    /**
     * @brief Sets the effort value associated with the WheelStatus.
     *
     * @param effort The effort value to set.
     */
    void Effort(const uint16_t &effort) { this->effort = effort; }

    /**
     * @brief Gets the position value associated with the WheelStatus.
     */
    double Position() const { return position; }

    /**
     * @brief Sets the position value associated with the WheelStatus.
     *
     * @param position The position value to set.
     */
    void Position(const double &position) { this->position = position; }

    /**
     * @brief Gets the RPM value associated with the WheelStatus.
     */
    uint16_t Rpm() const { return rpm; }

    /**
     * @brief Sets the RPM value associated with the WheelStatus.
     *
     * @param rpm The RPM value to set.
     */
    void Rpm(const uint16_t &rpm) { this->rpm = rpm; }

    /**
     * @brief Gets the velocity value associated with the WheelStatus.
     */
    double Velocity() const { return velocity; }

    /**
     * @brief Sets the velocity value associated with the WheelStatus.
     *
     * @param velocity The velocity value to set.
     */
    void Velocity(const double &velocity) { this->velocity = velocity; }

  private:
    uint16_t effort{0};
    double position{0.0};
    uint16_t rpm{0};
    double velocity{0.0};
  };

  /**
   * @brief Construct a new Wheel Controller object
   */
  WheelController() = default;

  /**
   * @brief Initializes the wheel controller with the given wheel.
   *
   * @param wheel The wheel to be controlled.
   * @return true if initialization was successful, false otherwise.
   */
  bool Init(const Wheel &wheel);

  /**
   * @brief Handles the wheel signal IRQ.
   */
  void WheelSignalIrqHandler();

  /**
   * @brief Calculates the wheel odometry.
   *
   * @return true if odometry was calculated, false otherwise.
   */
  bool CalculateWheelOdometry();

  /**
   * @brief Updates the timeout value.
   */
  void UpdateTimeout();

  /**
   * @brief Checks if the timeout has occurred.
   *
   * @return true if the timeout has occurred, false otherwise.
   */
  bool TimeoutCheck() const;

  /**
   * @brief Sets the direction of the wheel.
   *
   * @param direction The direction of the wheel (true for forward, false for
   * backward).
   */
  void SetDirection(const bool &direction) const;

  /**
   * @brief Sets the speed of the wheel.
   *
   * @param speed The speed of the wheel (in PWM 0-255).
   */
  void SetSpeed(const uint8_t &speed) const;

  /**
   * @brief Sets the speed and direction of the wheel.
   *
   * @param speed The speed of the wheel (in PWM 0-255).
   * @param direction The direction of the wheel (true for forward, false for
   * backward).
   */
  void SetSpeedAndDirection(const uint8_t &speed, const bool &direction) const;

  /**
   * @brief Gets the current status of the wheel.
   *
   * @return The current status of the wheel.
   */
  WheelStatus WheelFeedbackStatus() const;

  /**
   * @brief Drives the wheel.
   *
   * @param drive true to drive the wheel, false to stop it.
   */
  void Drive(const bool &drive) const;

  /**
   * @brief Destroys the Wheel Controller object.
   */
  ~WheelController() = default;

private:
  /**
   * @brief Applies the brake to the wheel.
   *
   * @param brake true to apply the brake, false to release it.
   */
  void Brake(const bool &brake) const;

  /**
   * @brief Disables the drive of the wheel.
   *
   * @param kStop true to disable the drive, false to enable it.
   */
  void Stop(const bool &stop) const;

  mutable WheelStatus wheel_feedback_status_{};
  const Wheel *wheel_{nullptr};
  mutable volatile bool is_reverse_{false};
  mutable uint64_t timeout_{0};
  uint64_t old_time_{0};
  uint64_t time_taken_{0};
  volatile int64_t signal_counter_{0};
  int64_t old_signal_counter_{0};
  int64_t delta_signal_counter_{0};
  std::once_flag first_odometry_tick_;
};
#endif // CAN_MOTOR_CONTROLLER_CORE_INC_WHEEL_CONTROLLER_H_

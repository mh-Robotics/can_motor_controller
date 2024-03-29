/**
 * @file wheel.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Wheel class declaration that initializes the wheel and its properties.
 * @version 0.1
 * @date 2021-10-09
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
#ifndef CAN_MOTOR_CONTROLLER_INC_WHEEL_H_
#define CAN_MOTOR_CONTROLLER_INC_WHEEL_H_
#include "constants.h" // for k*

#include <cstdint> // for int Data Types

/**
 * @brief A class representing a wheel
 */
class Wheel {
  /**
   * @brief An enum representing the different wheels
   */
  enum class Wheel_Enum : uint8_t {
    kBegin = 0,
    kFrontLeft = kBegin,
    kFrontRight,
    kBackLeft,
    kBackRight,
    kUnspecified,
    kEnd
  };

  /**
   * @brief An enum representing the different CAN IDs
   */
  enum class CanId : uint8_t {
    kBegin = 0,
    kCommandWheelFrontLeft = kBegin,
    kCommandWheelFrontRight,
    kCommandWheelBackLeft,
    kCommandWheelBackRight,
    kFeedbackWheelFrontLeft,
    kFeedbackWheelFrontRight,
    kFeedbackWheelBackLeft,
    kFeedbackWheelBackRight,
    kUnspecified,
    kEnd
  };

public:
  /**
   * @brief An enum representing the different commands
   */
  enum class Command : uint8_t {
    kBegin = 0,
    kWheelOff = kBegin,
    kWheelOn,
    kWheelStatus,
    kWheelStatusUpdate,
    kWheelSpeed,
    kUnspecified,
    kEnd
  };

  /**
   * @brief A structure representing the properties of a wheel
   */
  using properties = struct Properties {
  public:
    /**
     * @brief Get the receive ID of the wheel
     *
     * @return The receive ID of the wheel
     */
    uint8_t ReceiveId() const { return command_id; }

    /**
     * @brief Set the receive ID of the wheel
     *
     * @param command_id The receive ID of the wheel
     */
    void ReceiveId(const uint8_t &command_id) { this->command_id = command_id; }

    /**
     * @brief Get the transmit ID of the wheel
     *
     * @return The transmit ID of the wheel
     */
    uint8_t TransmitId() const { return feedback_id; }

    /**
     * @brief Set the transmit ID of the wheel
     *
     * @param feedback_id The transmit ID of the wheel
     */
    void TransmitId(const uint8_t &feedback_id) {
      this->feedback_id = feedback_id;
    }

  private:
    /**
     * @brief The CAN ID for sending commands to the wheel
     */
    uint8_t command_id{0x00};

    /**
     * @brief The CAN ID for receiving feedback from the wheel
     */
    uint8_t feedback_id{0x00};

    /**
     * @brief Whether the wheel is reversed
     */
    bool reverse{false};
  };

  /**
   * @brief Construct a new Wheel object
   */
  Wheel() = default;

  /**
   * @brief Initialize the Wheel object with given pin configuration
   *
   * @param pinConfiguration The pin configuration to be used for the wheel
   * @return true Initialization successful
   * @return false Initialization failed
   */
  bool Init();

  /**
   * @brief Returns the properties object for this wheel
   *
   * @return properties The properties object for this wheel
   */
  properties Properties() const;

  /**
   * @brief Destroy the Wheel object
   */
  ~Wheel() = default;

private:
  /**
   * @brief Sets the specific properties for the wheel
   *
   * @param wheelEnum The specific wheel to set properties for
   * @return true Setting properties successful
   * @return false Setting properties failed
   */
  bool EnumToCanId(const Wheel::Wheel_Enum &wheelEnum);

  /**
   * @brief Determines which wheel this object belongs to
   *
   * @return Wheel_Enum The enum value of the wheel this object belongs to
   */
  Wheel_Enum DetermineWheel();

  /**
   * @brief The properties object for this wheel
   */
  properties properties_;
};
#endif // CAN_MOTOR_CONTROLLER_INC_WHEEL_H_

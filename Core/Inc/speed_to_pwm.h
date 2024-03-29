/**
 * @file speed_to_pwm.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief A class for mapping linear velocity to PWM value.
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
#ifndef CAN_MOTOR_CONTROLLER_CORE_INC_SPEED_TO_PWM_H_
#define CAN_MOTOR_CONTROLLER_CORE_INC_SPEED_TO_PWM_H_
#include "constants.h"            // for k*
#include "stl_helper_functions.h" // for std::prev

#include <cmath>   // for std::abs
#include <cstdint> // for int Data Types
#include <map>     // for std::map

/**
 * @brief The SpeedToPWM class provides a lookup table for mapping speed values
 * to PWM values. @todo(jimmyhalimi): This class needs to update to PID Control.
 */
class SpeedToPWM {
public:
  /**
   * @brief Constructor to initialize the lookup table with speed to PWM
   * mappings.
   */
  SpeedToPWM() {
    // Initialize the lookup table with speed to PWM mappings
    // Set manually to determine the speed using the already implemeted formulas
    map_[0.05] = 1;
    map_[0.17] = 5;
    map_[0.28] = 10;
    map_[0.51] = 20;
    map_[0.68] = 30;
    map_[1.00] = 50;
    map_[1.48] = 70;
    map_[1.89] = 90;
    map_[2.17] = 105;
    map_[2.40] = 120;
    map_[2.67] = 135;
    map_[2.92] = 150;
    map_[3.10] = 165;
    map_[3.38] = 180;
    map_[3.79] = 205;
    map_[4.00] = 220;
    map_[4.17] = 235;
    map_[internal::wheel::kMaxSpeed] = 255;
  }

  /**
   * @brief Get the PWM value that corresponds to the given speed value.
   *
   * @param speed The speed value to get the PWM value for.
   * @return The PWM value that corresponds to the given speed value.
   */
  uint8_t GetPWM(double speed) const {
    // Ensure that speed is within the range of the map
    if (speed < 0.0) {
      speed = std::abs(speed);
    } else if (speed > internal::wheel::kMaxSpeed) {
      speed = internal::wheel::kMaxSpeed;
    }

    if (speed == 0) {
      return 0;
    }

    // Find the closest speed value in the map
    auto it = map_.lower_bound(speed);
    if (it == map_.begin()) {
      // If speed is less than the first value in the map, return the first PWM
      // value
      return it->second;
    } else if (it == map_.end()) {
      // If speed is greater than the last value in the map, return the last PWM
      // value
      return std::prev(it)->second;
    } else {
      // Interpolate between the two closest speed values in the map
      double prev_speed = std::prev(it)->first;
      double next_speed = it->first;
      int prev_pwm = std::prev(it)->second;
      int next_pwm = it->second;
      double alpha = (speed - prev_speed) / (next_speed - prev_speed);
      return (1.0 - alpha) * prev_pwm + alpha * next_pwm;
    }
  }

private:
  std::map<double, int> map_;
};
#endif // CAN_MOTOR_CONTROLLER_CORE_INC_SPEED_TO_PWM_H_

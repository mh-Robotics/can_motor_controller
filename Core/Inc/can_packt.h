/**
 * @file can_packt.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief A class for packing and unpacking compressed CAN messages.
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
#ifndef CAN_MOTOR_CONTROLLER_CORE_INC_CAN_PACKT_H_
#define CAN_MOTOR_CONTROLLER_CORE_INC_CAN_PACKT_H_
#include "can.h"              // for can_frame_t
#include "wheel_controller.h" // for WheelController::WheelStatus

#include <cstdint> // for int Data Types
#include <cstring> // for std::memcpy

/**
 * @brief Defines a bit-field struct to represent the compressed motor status
 * data
 */
using CompressedWheelStatus =
    struct __attribute__((packed)) CompressedWheelStatus {
  uint16_t effort : 10;  /**< Effort (10 bits), Range 0 to 1023 */
  int32_t position : 32; /**< Position (1 sign bit + 31 bits for magnitude),
                            Range -2147483.648 to 2147483.647 */
  uint16_t rpm : 10;     /**< RPM (10 bits), Range 0 to 1023  */
  int16_t velocity : 12; /**< Velocity (12 bits), Range -20.48 to 20.47 */
};

/**
 * @brief A class for packing and unpacking compressed CAN messages
 */
class CanPackt {
public:
  /**
   * @brief Construct a new Can Packt object
   *
   * @param transmit_id the ID of the transmitter node
   * @param receive_id the ID of the receiver node
   */
  CanPackt(uint8_t transmit_id, uint8_t receive_id)
      : transmit_id_(transmit_id), receive_id_(receive_id){};

  /**
   * @brief Packs a compressed wheel status data structure into a CAN frame
   *
   * @tparam inType the input data type (must be a WheelStatus)
   * @tparam outType the output data type (must be a can_frame_t)
   * @param data the input data to pack
   * @return the packed CAN frame
   */
  template <typename inType, typename outType>
  outType PackCompressed(const inType &wheel_status) {
    static_assert(IS_CAN_DLC(sizeof(inType)),
                  "Struct is larger than CAN message data field size");

    can_frame_t canFrame;
    canFrame.tx_header.StdId = transmit_id_;
    canFrame.tx_header.ExtId = 0x00;
    canFrame.tx_header.IDE = CAN_ID_STD;
    canFrame.tx_header.RTR = CAN_RTR_DATA;
    canFrame.tx_header.DLC = CAN_MAX_DLEN;
    canFrame.tx_header.TransmitGlobalTime = DISABLE;

    std::memcpy(canFrame.buffer, &wheel_status, sizeof(inType));

    return canFrame;
  }

  /**
   * @brief Unpacks a compressed wheel status CAN frame into a wheel status data
   * structure
   *
   * @tparam inType the input data type (must be a can_frame_t)
   * @tparam outType the output data type (must be a WheelStatus)
   * @param msg the CAN frame to unpack
   * @return the unpacked wheel status data structure
   */
  template <typename inType, typename outType>
  outType UnpackCompressed(const inType &can_frame) {
    static_assert(IS_CAN_DLC(sizeof(outType)),
                  "Struct is larger than CAN message data field size");

    outType data;
    std::memcpy(&data, can_frame.buffer, sizeof(outType));

    return data;
  }

private:
  uint8_t transmit_id_{0x00}; /**< The ID of the transmitter node */
  uint8_t receive_id_{0x00};  /**< The ID of the receiver node */
};

/**
 * @brief Specialization of the PackCompressed template function for packing a
 * WheelStatus into a can_frame_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param wheel_status The input data to pack.
 * @return The packed data as a CAN frame.
 */
template <>
inline can_frame_t
CanPackt::PackCompressed<WheelController::WheelStatus, can_frame_t>(
    const WheelController::WheelStatus &wheel_status) {
  static_assert(IS_CAN_DLC(sizeof(CompressedWheelStatus)),
                "Struct is larger than CAN message data field size");

  can_frame_t canFrame;
  canFrame.tx_header.StdId = transmit_id_;
  canFrame.tx_header.ExtId = 0x00;
  canFrame.tx_header.IDE = CAN_ID_STD;
  canFrame.tx_header.RTR = CAN_RTR_DATA;
  canFrame.tx_header.DLC = CAN_MAX_DLEN;
  canFrame.tx_header.TransmitGlobalTime = DISABLE;

  // Compress the motor status data into a bit-field struct
  CompressedWheelStatus compressed_status;
  compressed_status.effort = wheel_status.Effort() & 0x3FF;
  compressed_status.position =
      static_cast<int32_t>(wheel_status.Position() * 1000);
  compressed_status.rpm = wheel_status.Rpm() & 0x3FF;
  compressed_status.velocity =
      static_cast<int16_t>(wheel_status.Velocity() * 100) & 0xFFF;

  // Copy the compressed data into the CAN frame
  std::memcpy(canFrame.buffer, &compressed_status,
              sizeof(CompressedWheelStatus));

  return canFrame;
}

/**
 * @brief Specialization of the PackCompressed template function for unpacking a
 * can_frame_t into a WheelStatus
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param can_frame_t The input data to pack.
 * @return The packed data as a wheel status data structure.
 */
template <>
inline WheelController::WheelStatus
CanPackt::UnpackCompressed<can_frame_t, WheelController::WheelStatus>(
    const can_frame_t &can_frame) {
  static_assert(IS_CAN_DLC(sizeof(can_frame_t::buffer)),
                "Struct is larger than CAN message data field size");

  WheelController::WheelStatus wheel_status;

  // Extract the compressed data from the CAN frame
  CompressedWheelStatus compressed_status;
  std::memcpy(&compressed_status, can_frame.buffer,
              sizeof(CompressedWheelStatus));

  // Unpack the compressed data into the motor status struct
  wheel_status.Effort(compressed_status.effort);
  wheel_status.Position(static_cast<double>(compressed_status.position) / 1000);
  wheel_status.Rpm(compressed_status.rpm);
  wheel_status.Velocity(static_cast<double>(compressed_status.velocity) / 100);

  return wheel_status;
}
#endif // CAN_MOTOR_CONTROLLER_CORE_INC_CAN_PACKT_H_

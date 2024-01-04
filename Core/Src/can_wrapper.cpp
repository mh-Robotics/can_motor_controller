/**
 * @file can_wrapper.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
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
#include "can_wrapper.h"      // for CanWrapper
#include "can_packt.h"        // for CanPackt
#include "speed_to_pwm.h"     // for SpeedToPWM
#include "wheel_controller.h" // for WheelController::WheelStatus

#include <cstdint> // for int Data Types

bool CanWrapper::Init(const uint8_t &transmit_id, const uint8_t &receive_id) {
  canpressor_ = new CanPackt(transmit_id, receive_id);
  speedmapper_ = new SpeedToPWM();
  return Setup(receive_id);
}

bool CanWrapper::Setup(const int &receive_id) {
  CAN_FilterTypeDef sFilterConfig;

  // Configure the filter to accept messages with the specified receive_id
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = receive_id << 5;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0x03 << 5;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
    return false;
  }

  // Start the CAN module
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
    return false;
  }

  // Enable FIFO 0 message pending interrupt
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    Error_Handler();
    return false;
  }

  return true;
}

bool CanWrapper::CommandHandler() {
  return HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_msg_.rx_header,
                              can_msg_.buffer) == HAL_OK;
}

void CanWrapper::FeedbackHandler(
    const WheelController::WheelStatus &wheel_status) {
  can_frame_t can_frame =
      canpressor_->PackCompressed<WheelController::WheelStatus, can_frame_t>(
          wheel_status);

  // // DEBUG Only when we want to have another device send this message to bus
  // can_frame.tx_header.StdId = 0x00;
  // uint8_t buffer[] = {0x00, 0x60, 0x70, 0x2f,
  //                     0x0c, 0x40, 0x22, 0x00}; //  min values
  // uint8_t buffer[] = {0xff, 0xdf, 0x8f, 0xd0,
  //                     0xf3, 0xff, 0x1d, 0x00}; // max values
  // uint8_t buffer[] = {0x00, 0x00, 0x00, 0x00,
  //                     0x00, 0x80, 0x0c, 0x00}; // velocity 2.5
  // memcpy(can_frame.buffer, buffer, sizeof(buffer));

  // // DEBUG Only when we want to test loopback msg.
  // can_frame_t can_frame = CanMessage();

  // DEBUG Only when we want to test loopback msg going through unpack and pack.
  //  can_frame_t can_frame =
  //      canpressor_->PackCompressed<WheelController::WheelStatus,
  //      can_frame_t>(
  //          WheelCommandStatus());

  if (HAL_CAN_AddTxMessage(&hcan, &can_frame.tx_header, can_frame.buffer,
                           &can_frame.mailbox) != HAL_OK) {
    Error_Handler();
    return;
  }
}

// @todo(jimmyhalimi): Since no mutex, we need to use volatile for this
can_frame_t CanWrapper::CanMessage() const { return can_msg_; }

WheelController::WheelStatus CanWrapper::WheelCommandStatus() const {
  return canpressor_
      ->UnpackCompressed<can_frame_t, WheelController::WheelStatus>(
          CanMessage());
}

uint8_t CanWrapper::SpeedPwm() const {
  return speedmapper_->GetPWM(WheelCommandStatus().Velocity());
}

bool CanWrapper::Direction() const {
  return WheelCommandStatus().Velocity() >= 0;
}

bool CanWrapper::cleanCanMessage() {
  uint8_t buffer[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(can_msg_.buffer, buffer, sizeof(buffer));

  return true;
}

/**
 * @file    vehicle_can.h
 * @brief   Sets up CAN3, broadcasts BMS status to the VCU and handles received vehicle-bus frames.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_VEHICLE_CAN_H
#define BMS_VEHICLE_CAN_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Vehicle CAN message IDs (11-bit). Layout in can/CAN_MESSAGES.md.
static constexpr uint32_t BMS_STATUS_ID      = 0x310;
static constexpr uint32_t BMS_TEMP_ID        = 0x311;
static constexpr uint32_t BMS_CELL_REQUEST_ID = 0x312;
static constexpr uint32_t BMS_CELL_DATA_ID   = 0x313;
static constexpr uint32_t BMS_SET_SOC_ID     = 0x314;  // bytes 0-1: SOC uint16, 0.1 %/bit
static constexpr uint32_t BMS_CURRENT_CAL_ID = 0x315;  // byte 0: point (1/2), bytes 1-2: actual A int16, 0.1 A/bit

// 0x310 and 0x311 go out this often in every state, including FAULT.
static constexpr uint32_t BMS_STATUS_BROADCAST_MS = 500;

extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> vehicleCan;

extern volatile bool hasSeenVehicleCan;  // any non-IMD frame during INIT

void vehicleCanSetup();
void vehicleCanServiceStatusBroadcast();
void vehicleCanSendStatus(bool isOnChargerCan);
void vehicleCanReceive(const CAN_message_t &frame);
void vehicleCanHandleCommandFrame(const CAN_message_t &frame, bool isFromChargerCan);

#endif  // BMS_VEHICLE_CAN_H

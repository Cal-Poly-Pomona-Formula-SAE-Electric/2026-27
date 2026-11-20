/**
 * @file    vehicle_can.h
 * @brief   Starts CAN3, drains received frames to their module and sends SOC/calibration commands to the BMS.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * CAN IDs and byte layouts follow can/CAN_MESSAGES.md (BMS frames are big-endian).
 */
#ifndef TELEMETRY_VEHICLE_CAN_H
#define TELEMETRY_VEHICLE_CAN_H

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// ---------------- Message IDs ----------------
static constexpr uint32_t BMS_STATUS_ID       = 0x310;  // 11-bit, every 500 ms: flags, cell V min/max/avg, SOC
static constexpr uint32_t BMS_TEMP_ID         = 0x311;  // 11-bit, every 500 ms: temps, fault code, pack current
static constexpr uint32_t BMS_CELL_REQUEST_ID = 0x312;  // telemetry -> BMS: request a cell dump
static constexpr uint32_t BMS_CELL_DATA_ID    = 0x313;  // BMS -> telemetry: cell dump response
static constexpr uint32_t BMS_SET_SOC_ID      = 0x314;  // bytes 0-1: SOC uint16, 0.1 %/bit
static constexpr uint32_t BMS_CURRENT_CAL_ID  = 0x315;  // byte 0: point (1/2), bytes 1-2: actual A int16, 0.1 A/bit
static constexpr uint32_t CHARGER_STATUS_ID   = 0x18FF50E5UL;  // 29-bit extended, every 1 s

// ---------------- Frame scaling shared by the BMS frames ----------------
static constexpr uint32_t CAN_STANDARD_FRAME = 0;     // CanFrame.extd value for an 11-bit ID
static constexpr int      CAN_TEMP_OFFSET_C  = 40;    // temperature bytes carry C + 40
static constexpr uint8_t  CAN_TEMP_NO_SENSOR = 0xFF;  // temperature byte for a missing sensor

void canSetup();
void canReceiveFrames();
void canSendSoc(float socPct);
void canSendCurrentCalPoint(uint8_t calPoint, float currentA);

#endif  // TELEMETRY_VEHICLE_CAN_H

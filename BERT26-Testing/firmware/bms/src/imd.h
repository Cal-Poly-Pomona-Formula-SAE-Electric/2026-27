/**
 * @file    imd.h
 * @brief   Polls and decodes the Bender iso175 HV voltage used for the precharge check and the pack voltage cross-check.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_IMD_H
#define BMS_IMD_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// IMD frame IDs on the vehicle CAN (11-bit).
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;  // cyclic status
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;
static constexpr uint32_t IMD_REQUEST_ID      = 0x22;
static constexpr uint8_t  IMD_INDEX_HV_VOLTAGE = 0x5E;

extern volatile bool  isImdVoltageValid;
extern volatile float imdVoltageV;

void imdRequestHvVoltage();
void imdHandleResponseFrame(const CAN_message_t &frame);
void imdInvalidateReading();

#endif  // BMS_IMD_H

/**
 * @file    imd.cpp
 * @brief   Polls and decodes the Bender iso175 HV voltage used for the precharge check and the pack voltage cross-check.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "imd.h"

#include "can_bytes.h"
#include "vehicle_can.h"

static float imdDecodeVoltage(uint16_t raw);

volatile bool  isImdVoltageValid = false;
volatile float imdVoltageV       = 0.0f;

// iso175 "HV voltage" encoding: 0.05 V per bit around a 32128 offset.
static constexpr float    IMD_VOLTAGE_RAW_OFFSET  = 32128.0f;
static constexpr float    IMD_VOLTS_PER_BIT       = 0.05f;
static constexpr uint16_t IMD_VOLTAGE_RAW_INVALID = 0xFFFF;

static float imdDecodeVoltage(uint16_t raw) {
  return ((float)raw - IMD_VOLTAGE_RAW_OFFSET) * IMD_VOLTS_PER_BIT;
}

// Ask the IMD for its HV voltage measurement; the answer arrives as 0x23.
void imdRequestHvVoltage() {
  CAN_message_t request{};
  request.id = IMD_REQUEST_ID; request.len = 8; request.buf[0] = IMD_INDEX_HV_VOLTAGE;
  for (int i = 1; i < 8; i++) request.buf[i] = 0xFF;
  vehicleCan.write(request);
}

void imdHandleResponseFrame(const CAN_message_t &frame) {
  if (frame.buf[0] != IMD_INDEX_HV_VOLTAGE) return;
  uint16_t raw = canReadU16Le(frame.buf[1], frame.buf[2]);
  if (raw == IMD_VOLTAGE_RAW_INVALID) { isImdVoltageValid = false; }
  else { imdVoltageV = imdDecodeVoltage(raw); isImdVoltageValid = true; }
}

// Discard a stale reading so a fresh poll is required before it is trusted.
void imdInvalidateReading() {
  isImdVoltageValid = false;
  imdVoltageV = 0.0f;
}

/**
 * @file    live_signals.h
 * @brief   Holds the latest BMS status/temperature and charger values decoded from CAN for the web page.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef TELEMETRY_LIVE_SIGNALS_H
#define TELEMETRY_LIVE_SIGNALS_H

#include <Arduino.h>

// Index into liveSignalValues / liveSignalNames. Order sets the /data JSON key order.
enum LiveSignal {
  SIG_BMS_FLAGS,       // 0x310 byte 0  — bit0=fault, bit1=contactors, bit2=charge
  SIG_CELL_V_MIN,      // 0x310 bytes 1-2  mV → V
  SIG_CELL_V_MAX,      // 0x310 bytes 3-4  mV → V
  SIG_CELL_V_AVG,      // 0x310 bytes 5-6  mV → V
  SIG_SOC,             // 0x310 byte 7   %
  SIG_TEMP_MIN,        // 0x311 byte 0   °C (-1 = no sensor)
  SIG_TEMP_MAX,        // 0x311 byte 1   °C
  SIG_TEMP_AVG,        // 0x311 byte 2   °C
  SIG_FAULT_CODE,      // 0x311 byte 3   (see CAN_MESSAGES.md for codes)
  SIG_PACK_CURRENT,    // 0x311 bytes 4-5  A (negative = regen/charging)
  SIG_CHARGER_VOLTAGE, // 0x18FF50E5 bytes 0-1  V
  SIG_CHARGER_CURRENT, // 0x18FF50E5 bytes 2-3  A
  SIG_CHARGER_FLAGS,   // 0x18FF50E5 byte 4  — bit0=HW fault, bit1=temp, bit2=AC, bit3=no AC, bit4=timeout
  SIGNAL_COUNT
};

extern const char* liveSignalNames[SIGNAL_COUNT];  // JSON keys the web page reads
extern double liveSignalValues[SIGNAL_COUNT];      // units per LiveSignal comment

void liveSignalsReset();
void liveSignalsHandleBmsStatusFrame(uint8_t* frameData, int frameLength);
void liveSignalsHandleBmsTempFrame(uint8_t* frameData, int frameLength);
void liveSignalsHandleChargerFrame(uint8_t* frameData, int frameLength);

#endif  // TELEMETRY_LIVE_SIGNALS_H

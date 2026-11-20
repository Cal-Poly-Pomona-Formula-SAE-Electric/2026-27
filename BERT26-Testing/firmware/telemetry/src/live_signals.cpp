/**
 * @file    live_signals.cpp
 * @brief   Holds the latest BMS status/temperature and charger values decoded from CAN for the web page.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "live_signals.h"

#include <string.h>

#include "vehicle_can.h"

// Names must match the keys main page JavaScript reads (d.SOC, d.PackCurrent, ...).
const char* liveSignalNames[SIGNAL_COUNT] = {
  "BmsFlags",
  "CellVMin",
  "CellVMax",
  "CellVAvg",
  "SOC",
  "TempMin",
  "TempMax",
  "TempAvg",
  "FaultCode",
  "PackCurrent",
  "ChargerVoltage",
  "ChargerCurrent",
  "ChargerFlags"
};

double liveSignalValues[SIGNAL_COUNT];

// ---------------- Frame layouts ----------------
static constexpr int    BMS_STATUS_LENGTH_BYTES      = 8;
static constexpr int    BMS_TEMP_MIN_LENGTH_BYTES    = 6;
static constexpr int    CHARGER_STATUS_MIN_LENGTH_BYTES = 5;
static constexpr double V_PER_MV                     = 0.001;  // 0x310 cell voltages are sent in mV
// The page shows any temperature below -0.5 as N/A, so -1 marks a missing sensor.
static constexpr int    TEMP_NO_SENSOR_C             = -1;
// Pack current: int16 with 3000 bias, 0.1 A/bit  →  A = (raw - 3000) × 0.1
static constexpr int    PACK_CURRENT_BIAS_UNITS      = 3000;
static constexpr double PACK_CURRENT_A_PER_UNIT      = 0.1;
static constexpr double CHARGER_V_PER_UNIT           = 0.1;
static constexpr double CHARGER_A_PER_UNIT           = 0.1;

void liveSignalsReset() {
  memset(liveSignalValues, 0, sizeof(liveSignalValues));
}

// ---------------- 0x310 BMS status ----------------
// Flags, min/max/avg cell voltage, SOC.
void liveSignalsHandleBmsStatusFrame(uint8_t* frameData, int frameLength) {
  if (frameLength < BMS_STATUS_LENGTH_BYTES) return;
  liveSignalValues[SIG_BMS_FLAGS]  = frameData[0];
  liveSignalValues[SIG_CELL_V_MIN] = (uint16_t)(frameData[1] << 8 | frameData[2]) * V_PER_MV;
  liveSignalValues[SIG_CELL_V_MAX] = (uint16_t)(frameData[3] << 8 | frameData[4]) * V_PER_MV;
  liveSignalValues[SIG_CELL_V_AVG] = (uint16_t)(frameData[5] << 8 | frameData[6]) * V_PER_MV;
  liveSignalValues[SIG_SOC]        = frameData[7];
}

// ---------------- 0x311 BMS temperatures ----------------
// Temps, fault code, pack current.
void liveSignalsHandleBmsTempFrame(uint8_t* frameData, int frameLength) {
  if (frameLength < BMS_TEMP_MIN_LENGTH_BYTES) return;
  liveSignalValues[SIG_TEMP_MIN]     = (frameData[0] != CAN_TEMP_NO_SENSOR) ? (int)frameData[0] - CAN_TEMP_OFFSET_C : TEMP_NO_SENSOR_C;
  liveSignalValues[SIG_TEMP_MAX]     = (frameData[1] != CAN_TEMP_NO_SENSOR) ? (int)frameData[1] - CAN_TEMP_OFFSET_C : TEMP_NO_SENSOR_C;
  liveSignalValues[SIG_TEMP_AVG]     = (frameData[2] != CAN_TEMP_NO_SENSOR) ? (int)frameData[2] - CAN_TEMP_OFFSET_C : TEMP_NO_SENSOR_C;
  liveSignalValues[SIG_FAULT_CODE]   = frameData[3];
  liveSignalValues[SIG_PACK_CURRENT] = ((int16_t)((uint16_t)frameData[4] << 8 | frameData[5]) - PACK_CURRENT_BIAS_UNITS) * PACK_CURRENT_A_PER_UNIT;
}

// ---------------- 0x18FF50E5 charger status ----------------
void liveSignalsHandleChargerFrame(uint8_t* frameData, int frameLength) {
  if (frameLength < CHARGER_STATUS_MIN_LENGTH_BYTES) return;
  liveSignalValues[SIG_CHARGER_VOLTAGE] = (uint16_t)(frameData[0] << 8 | frameData[1]) * CHARGER_V_PER_UNIT;
  liveSignalValues[SIG_CHARGER_CURRENT] = (uint16_t)(frameData[2] << 8 | frameData[3]) * CHARGER_A_PER_UNIT;
  liveSignalValues[SIG_CHARGER_FLAGS]   = frameData[4];
}

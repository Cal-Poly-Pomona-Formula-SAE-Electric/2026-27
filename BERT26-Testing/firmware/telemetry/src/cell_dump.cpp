/**
 * @file    cell_dump.cpp
 * @brief   Asks the BMS for every cell voltage or temperature (0x312) and stores the 0x313 replies.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * Voltage and temperature dumps are requested in turn every 5 s, so each one
 * refreshes every 10 s. An incomplete dump is asked for again after 1 s.
 */
#include "cell_dump.h"

#include <string.h>
#include <ESP32-TWAI-CAN.hpp>

#include "vehicle_can.h"

uint16_t cellVoltagesMv[PACK_CELL_COUNT];
int8_t   cellTempsC[PACK_CELL_COUNT];
bool     areCellVoltagesValid = false;
bool     areCellTempsValid    = false;

static uint8_t  activeDumpType = CELL_DUMP_IDLE;  // which dump the 0x313 frames now belong to
static uint32_t dumpStartMs    = 0;               // millis() when activeDumpType was last (re)requested

// ---------------- Timing ----------------
static constexpr uint32_t CELL_DUMP_INTERVAL_MS = 5000;  // alternate voltage/temp request every 5 s
// A full 32-frame voltage dump takes about 160 ms (1 frame/5 ms), so 1 s
// without the last frame means frames were dropped.
static constexpr uint32_t CELL_DUMP_RETRY_MS = 1000;

// ---------------- 0x312 request / 0x313 response layout ----------------
static constexpr uint8_t CELL_REQUEST_LENGTH_BYTES   = 1;
static constexpr int     CELL_DUMP_MIN_LENGTH_BYTES  = 2;   // frame index + at least one data byte
static constexpr int     DUMP_DATA_START_BYTE        = 1;   // byte 0 is the frame index
// Voltage dump: 32 frames × 3 cells = 96 cells, bytes 1-6 = 3 × uint16 mV.
static constexpr uint8_t VOLTAGE_DUMP_FRAME_COUNT    = 32;
static constexpr int     VOLTAGE_CELLS_PER_FRAME     = 3;
static constexpr int     BYTES_PER_CELL_VOLTAGE      = 2;
static constexpr int     VOLTAGE_DUMP_LENGTH_BYTES   = 7;
// Temp dump: 14 frames × 7 cells = 98 slots (last 2 = padding), bytes 1-7 = 7 × uint8 (°C + 40 offset).
static constexpr uint8_t TEMP_DUMP_FRAME_COUNT       = 14;
static constexpr int     TEMP_CELLS_PER_FRAME        = 7;

void cellDumpReset() {
  // Every byte 0xFF makes each uint16 0xFFFF = CELL_VOLTAGE_NOT_RECEIVED_MV.
  memset(cellVoltagesMv, 0xFF, sizeof(cellVoltagesMv));
  memset(cellTempsC, CELL_TEMP_NOT_RECEIVED_C, sizeof(cellTempsC));
}

// ---------------- Request schedule ----------------
void cellDumpService() {
  static uint32_t lastDumpRequestMs = 0;
  static uint8_t  nextDumpType      = CELL_DUMP_VOLTAGES;
  uint32_t nowMs = millis();
  if (nowMs - lastDumpRequestMs >= CELL_DUMP_INTERVAL_MS) {
    lastDumpRequestMs = nowMs;
    cellDumpRequest(nextDumpType);
    nextDumpType = (nextDumpType == CELL_DUMP_VOLTAGES) ? CELL_DUMP_TEMPERATURES : CELL_DUMP_VOLTAGES;
  }

  // Retry an incomplete dump (e.g. dropped CAN frames) instead of waiting for
  // the next scheduled alternation.
  bool isDumpPending = (activeDumpType == CELL_DUMP_VOLTAGES && !areCellVoltagesValid) ||
                       (activeDumpType == CELL_DUMP_TEMPERATURES && !areCellTempsValid);
  if (isDumpPending && (nowMs - dumpStartMs >= CELL_DUMP_RETRY_MS)) {
    cellDumpRequest(activeDumpType);
  }
}

// Sends 0x312 with byte 0 = dumpType (CELL_DUMP_VOLTAGES or CELL_DUMP_TEMPERATURES).
void cellDumpRequest(uint8_t dumpType) {
  activeDumpType = dumpType;
  dumpStartMs = millis();
  CanFrame requestFrame = {};
  requestFrame.identifier       = BMS_CELL_REQUEST_ID;
  requestFrame.extd             = CAN_STANDARD_FRAME;
  requestFrame.data_length_code = CELL_REQUEST_LENGTH_BYTES;
  requestFrame.data[0]          = dumpType;
  ESP32Can.writeFrame(requestFrame);
}

// ---------------- 0x313 response ----------------
// The frame does not say which dump it belongs to, so activeDumpType decides.
void cellDumpHandleFrame(uint8_t* frameData, int frameLength) {
  if (frameLength < CELL_DUMP_MIN_LENGTH_BYTES) return;
  if (activeDumpType == CELL_DUMP_VOLTAGES) {
    if (frameLength < VOLTAGE_DUMP_LENGTH_BYTES) return;
    uint8_t frameIndex = frameData[0];
    if (frameIndex >= VOLTAGE_DUMP_FRAME_COUNT) return;
    int firstCell = frameIndex * VOLTAGE_CELLS_PER_FRAME;
    for (int cellOffset = 0; cellOffset < VOLTAGE_CELLS_PER_FRAME && (firstCell + cellOffset) < PACK_CELL_COUNT; cellOffset++) {
      int highByte = DUMP_DATA_START_BYTE + cellOffset * BYTES_PER_CELL_VOLTAGE;
      cellVoltagesMv[firstCell + cellOffset] = (uint16_t)frameData[highByte] << 8 | frameData[highByte + 1];
    }
    if (frameIndex == VOLTAGE_DUMP_FRAME_COUNT - 1) areCellVoltagesValid = true;
  } else if (activeDumpType == CELL_DUMP_TEMPERATURES) {
    uint8_t frameIndex = frameData[0];
    if (frameIndex >= TEMP_DUMP_FRAME_COUNT) return;
    int firstCell = frameIndex * TEMP_CELLS_PER_FRAME;
    for (int cellOffset = 0; cellOffset < TEMP_CELLS_PER_FRAME && (firstCell + cellOffset) < PACK_CELL_COUNT; cellOffset++) {
      uint8_t tempRaw = frameData[DUMP_DATA_START_BYTE + cellOffset];
      cellTempsC[firstCell + cellOffset] = (tempRaw != CAN_TEMP_NO_SENSOR) ? (int8_t)(tempRaw - CAN_TEMP_OFFSET_C) : CELL_TEMP_NOT_RECEIVED_C;
    }
    if (frameIndex == TEMP_DUMP_FRAME_COUNT - 1) areCellTempsValid = true;
  }
}

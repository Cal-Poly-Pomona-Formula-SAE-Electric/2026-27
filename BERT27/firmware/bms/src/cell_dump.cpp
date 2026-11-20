/**
 * @file    cell_dump.cpp
 * @brief   Streams every cell voltage or temperature over CAN in 0x313 frames when the VCU asks.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "cell_dump.h"

#include <math.h>
#include <FlexCAN_T4.h>

#include "can_bytes.h"
#include "cell_monitor.h"
#include "vehicle_can.h"
#include "charger_can.h"

// One frame every CELL_DUMP_GAP_MS until the dump is complete.
static constexpr uint32_t CELL_DUMP_GAP_MS = 5;
// Voltages: 3 cells per frame as uint16 mV (6 bytes), last byte padding -> 32 frames.
static constexpr uint8_t  VOLTAGE_CELLS_PER_FRAME = 3;
// Temperatures: 7 cells per frame as uint8 (C + 40) -> 14 frames.
static constexpr uint8_t  TEMP_CELLS_PER_FRAME    = 7;
static constexpr float    MV_PER_V            = 1000.0f;
static constexpr uint16_t CELL_MV_INVALID     = 0xFFFF;
static constexpr float    CAN_TEMP_OFFSET_C   = 40.0f;
static constexpr uint8_t  CAN_TEMP_INVALID    = 0xFF;
static constexpr int      CAN_TEMP_MAX_VALID  = 254;

static uint8_t cellDumpType   = CELL_DUMP_IDLE;
static uint8_t cellDumpFrameIndex = 0;
static bool    isCellDumpOnChargerCan = false;
static elapsedMillis cellDumpTimerMs;

void cellDumpStart(uint8_t dumpType, bool isOnChargerCan) {
  cellDumpType   = dumpType;
  cellDumpFrameIndex = 0;
  isCellDumpOnChargerCan = isOnChargerCan;
  cellDumpTimerMs = CELL_DUMP_GAP_MS;  // fire immediately
}

// Sends one cell data frame per call once the gap has elapsed.
void cellDumpService() {
  if (cellDumpType == CELL_DUMP_IDLE) return;
  if (cellDumpTimerMs < CELL_DUMP_GAP_MS) return;
  cellDumpTimerMs = 0;

  CAN_message_t dataFrame{};
  dataFrame.id = BMS_CELL_DATA_ID; dataFrame.len = 8;
  dataFrame.buf[0] = cellDumpFrameIndex;

  if (cellDumpType == CELL_DUMP_VOLTAGES) {
    uint16_t firstCell = (uint16_t)cellDumpFrameIndex * VOLTAGE_CELLS_PER_FRAME;
    for (uint8_t i = 0; i < VOLTAGE_CELLS_PER_FRAME; i++) {
      uint16_t cellIndex = firstCell + i;
      uint16_t cellMv = (cellIndex < PACK_CELLS && !isnan(cellVoltagesV[cellIndex]))
                        ? (uint16_t)(cellVoltagesV[cellIndex] * MV_PER_V) : CELL_MV_INVALID;
      canWriteU16Be(&dataFrame.buf[1 + i * 2], cellMv);
    }
    dataFrame.buf[7] = 0xFF;
    uint16_t totalFrames = (PACK_CELLS + VOLTAGE_CELLS_PER_FRAME - 1) / VOLTAGE_CELLS_PER_FRAME;  // 32
    cellDumpFrameIndex++;
    if (cellDumpFrameIndex >= totalFrames) cellDumpType = CELL_DUMP_IDLE;

  } else {
    uint16_t firstCell = (uint16_t)cellDumpFrameIndex * TEMP_CELLS_PER_FRAME;
    for (uint8_t i = 0; i < TEMP_CELLS_PER_FRAME; i++) {
      uint16_t cellIndex = firstCell + i;
      dataFrame.buf[1 + i] = (cellIndex < PACK_CELLS && !isnan(cellTempsC[cellIndex]))
                             ? (uint8_t)constrain((int)(cellTempsC[cellIndex] + CAN_TEMP_OFFSET_C), 0, CAN_TEMP_MAX_VALID)
                             : CAN_TEMP_INVALID;
    }
    uint16_t totalFrames = (PACK_CELLS + TEMP_CELLS_PER_FRAME - 1) / TEMP_CELLS_PER_FRAME;  // 14
    cellDumpFrameIndex++;
    if (cellDumpFrameIndex >= totalFrames) cellDumpType = CELL_DUMP_IDLE;
  }

  if (isCellDumpOnChargerCan) chargerCan.write(dataFrame); else vehicleCan.write(dataFrame);
}

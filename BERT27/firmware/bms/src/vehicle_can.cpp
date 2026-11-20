/**
 * @file    vehicle_can.cpp
 * @brief   Sets up CAN3, broadcasts BMS status to the VCU and handles received vehicle-bus frames.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "vehicle_can.h"

#include <math.h>

#include "can_bytes.h"
#include "cell_monitor.h"
#include "current_sensor.h"
#include "soc.h"
#include "imd.h"
#include "faults.h"
#include "state_machine.h"
#include "charger_can.h"
#include "cell_dump.h"

// CAN3 on Teensy 4.1 pins 31 (TX) and 30 (RX), set up by FlexCAN_T4::begin().
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> vehicleCan;

static constexpr uint32_t VEHICLE_CAN_BAUD_BPS = 500000;

volatile bool hasSeenVehicleCan = false;

static elapsedMillis statusBroadcastTimerMs;

// 0x310 flag bits.
static constexpr uint8_t STATUS_FLAG_FAULT             = 0x01;
static constexpr uint8_t STATUS_FLAG_CONTACTORS_CLOSED = 0x02;
static constexpr uint8_t STATUS_FLAG_CHARGE_MODE       = 0x04;
// Frame scaling.
static constexpr float   MV_PER_V               = 1000.0f;
static constexpr float   CAN_TEMP_OFFSET_C      = 40.0f;   // byte = C + 40
static constexpr uint8_t CAN_TEMP_INVALID       = 0xFF;
static constexpr float   CAN_CURRENT_UNITS_PER_A = 10.0f;
static constexpr float   CAN_CURRENT_BIAS        = 3000.0f;  // 3000 = 0 A
static constexpr float   CAN_SOC_PCT_PER_UNIT    = 0.1f;
static constexpr float   CAN_CAL_AMPS_PER_UNIT   = 0.1f;

void vehicleCanSetup() {
  vehicleCan.begin();
  vehicleCan.setBaudRate(VEHICLE_CAN_BAUD_BPS);
  vehicleCan.setMaxMB(16);
  vehicleCan.enableFIFO();
  vehicleCan.enableFIFOInterrupt();
  vehicleCan.onReceive(vehicleCanReceive);
}

// ---------------- Status broadcast ----------------
void vehicleCanServiceStatusBroadcast() {
  if (statusBroadcastTimerMs >= BMS_STATUS_BROADCAST_MS) {
    statusBroadcastTimerMs = 0;
    vehicleCanSendStatus(false);
    if (bmsMode == MODE_CHARGE) vehicleCanSendStatus(true);
  }
}

// Sends 0x310 (flags, cell voltage stats, SOC) and 0x311 (cell temp stats,
// fault code, pack current) on the chosen bus.
void vehicleCanSendStatus(bool isOnChargerCan) {
  uint16_t validVoltageCount = cellMonitorCalcStats(cellVoltagesV, PACK_CELLS, cellVoltageStats);
  uint16_t validTempCount    = cellMonitorCalcStats(cellTempsC, PACK_CELLS, cellTempStats);

  CAN_message_t statusFrame{};
  statusFrame.id = BMS_STATUS_ID; statusFrame.len = 8;
  statusFrame.buf[0] = (activeFaultCode != FAULT_NONE ? STATUS_FLAG_FAULT : 0x00)
                     | (areContactorsClosed          ? STATUS_FLAG_CONTACTORS_CLOSED : 0x00)
                     | (bmsMode == MODE_CHARGE       ? STATUS_FLAG_CHARGE_MODE : 0x00);
  uint16_t minCellMv = validVoltageCount ? (uint16_t)(cellVoltageStats.minValue * MV_PER_V) : 0;
  uint16_t maxCellMv = validVoltageCount ? (uint16_t)(cellVoltageStats.maxValue * MV_PER_V) : 0;
  uint16_t avgCellMv = validVoltageCount ? (uint16_t)(cellVoltageStats.average  * MV_PER_V) : 0;
  canWriteU16Be(&statusFrame.buf[1], minCellMv);
  canWriteU16Be(&statusFrame.buf[3], maxCellMv);
  canWriteU16Be(&statusFrame.buf[5], avgCellMv);
  statusFrame.buf[7] = (uint8_t)constrain((int)socPct, 0, 100);
  if (isOnChargerCan) chargerCan.write(statusFrame); else vehicleCan.write(statusFrame);

  CAN_message_t tempFrame{};
  tempFrame.id = BMS_TEMP_ID; tempFrame.len = 8;
  tempFrame.buf[0] = validTempCount ? (uint8_t)constrain((int)(cellTempStats.minValue + CAN_TEMP_OFFSET_C), 0, 255) : CAN_TEMP_INVALID;
  tempFrame.buf[1] = validTempCount ? (uint8_t)constrain((int)(cellTempStats.maxValue + CAN_TEMP_OFFSET_C), 0, 255) : CAN_TEMP_INVALID;
  tempFrame.buf[2] = validTempCount ? (uint8_t)constrain((int)(cellTempStats.average  + CAN_TEMP_OFFSET_C), 0, 255) : CAN_TEMP_INVALID;
  tempFrame.buf[3] = (uint8_t)activeFaultCode;
  int16_t currentRaw = (int16_t)(packCurrentA * CAN_CURRENT_UNITS_PER_A + CAN_CURRENT_BIAS);
  canWriteU16Be(&tempFrame.buf[4], (uint16_t)currentRaw);
  tempFrame.buf[6] = 0xFF; tempFrame.buf[7] = 0xFF;
  if (isOnChargerCan) chargerCan.write(tempFrame); else vehicleCan.write(tempFrame);
}

// ---------------- Receive ----------------
void vehicleCanReceive(const CAN_message_t &frame) {
  if (frame.id == IMD_RESPONSE_ID) {
    imdHandleResponseFrame(frame);
    return;
  }
  // Mode detection: any non-IMD frame on the vehicle CAN means discharge.
  if (bmsState == STATE_INIT && frame.id != IMD_INFO_GENERAL_ID && frame.id != IMD_RESPONSE_ID)
    hasSeenVehicleCan = true;

  vehicleCanHandleCommandFrame(frame, false);
}

// Commands the BMS accepts on either bus. The cell dump answers on the bus the
// request came from.
void vehicleCanHandleCommandFrame(const CAN_message_t &frame, bool isFromChargerCan) {
  if (frame.id == BMS_CELL_REQUEST_ID && frame.len >= 1) {
    cellDumpStart(frame.buf[0], isFromChargerCan);  // 1 = voltages, 2 = temps
  }
  // SOC set: bytes 0-1 = SOC as uint16, 0.1 %/bit (e.g. 725 = 72.5 %)
  if (frame.id == BMS_SET_SOC_ID && frame.len >= 2) {
    socSet(canReadU16Be(frame.buf[0], frame.buf[1]) * CAN_SOC_PCT_PER_UNIT);
  }
  // Current cal point: byte 0 = point (1/2), bytes 1-2 = actual current int16, 0.1 A/bit
  if (frame.id == BMS_CURRENT_CAL_ID && frame.len >= 3) {
    int16_t actualRawA = (int16_t)canReadU16Be(frame.buf[1], frame.buf[2]);
    currentSensorRecordCalPoint(frame.buf[0], actualRawA * CAN_CAL_AMPS_PER_UNIT);
  }
}

/**
 * @file    bms_link.cpp
 * @brief   BMS status and temperature frames, cell data requests, BMS state.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "bms_link.h"

#include "safety_limits.h"
#include "can_bytes.h"
#include "vehicle_can.h"
#include "faults.h"
#include "regen.h"
#include "rtd.h"
#include "data_log.h"
#include "can_trace.h"
#include "fault_text.h"
#include "dashboard.h"

bool isRegenBmsPairValid = false;

static constexpr uint32_t CELL_LOG_REQUEST_PERIOD_MS = 1000;
// A 96-cell voltage dump takes 32 frames * 5 ms in the BMS firmware.
static constexpr uint32_t CELL_VOLTAGE_DUMP_WAIT_MS = 250;

float packVoltageV = 0.0f;
float packCurrentA = 0.0f;

uint8_t stateOfChargePct = 0;
uint8_t batteryTempC = 0;
uint16_t minCellVoltageMv = 0;
uint16_t maxCellVoltageMv = 0;
uint16_t avgCellVoltageMv = 0;
bool hasCellVoltageStats = false;
int16_t minCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t maxCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t avgCellTempC = CELL_TEMP_UNKNOWN_C;
bool hasCellTempStats = false;

bool isBmsActive = false;
bool hasBmsFault = true;
bool isBmsTimedOut = true;
bool hasBmsStatus = false;
uint8_t bmsFlags = 0;
uint8_t bms2026FaultCode = 0;
bool hasBms2026FaultCode = false;
uint32_t lastBmsRxMs = 0;
bool hasBmsCurrent = false;
uint32_t lastBmsCellVoltageRxMs = 0;
uint32_t lastBmsCurrentRxMs = 0;

uint32_t lastCellLogRequestMs = 0;
uint32_t cellVoltageRequestMs = 0;
bool isCellTemperatureRequestPending = false;

// Request one complete 96-cell voltage dump and one temperature dump each
// second. The delay keeps the two BMS response sequences from overlapping.
void bmsServiceCellDataRequests(uint32_t nowMs) {
  if (!isSdLogActive || !hasBmsStatus || isBmsTimedOut) return;

  if (isCellTemperatureRequestPending) {
    if (nowMs - cellVoltageRequestMs < CELL_VOLTAGE_DUMP_WAIT_MS) return;

    CAN_message_t request{};
    request.id = BMS_CELL_REQUEST_2026_ID;
    request.len = 1;
    request.buf[0] = CELL_DATA_REQUEST_TEMPERATURES;
    canSendFrame(request);
    isCellTemperatureRequestPending = false;
    return;
  }

  if (nowMs - lastCellLogRequestMs < CELL_LOG_REQUEST_PERIOD_MS) return;

  CAN_message_t request{};
  request.id = BMS_CELL_REQUEST_2026_ID;
  request.len = 1;
  request.buf[0] = CELL_DATA_REQUEST_VOLTAGES;
  canSendFrame(request);
  lastCellLogRequestMs = nowMs;
  cellVoltageRequestMs = nowMs;
  isCellTemperatureRequestPending = true;
}

// Contactors open is a normal pre-RTD condition, not a fault: it should drop/
// block RTD (back to neutral) without lighting the fault indicator.
bool bmsAreContactorsOpen() {
  return hasBmsStatus && !(bmsFlags & 0x02);
}

// ---------------- CAN receive handlers ----------------
void bmsHandleStatusFrame(const CAN_message_t &frame) {
  if (frame.len < 8) {
    return;
  }

  bool wasActive = isBmsActive;
  bool previousFault = hasBmsFault;
  bool isFaultDetectionArmed = faultsIsBmsImdDetectionArmed();

  uint8_t flags = frame.buf[0];
  minCellVoltageMv = canReadU16Be(frame.buf[1], frame.buf[2]);
  maxCellVoltageMv = canReadU16Be(frame.buf[3], frame.buf[4]);
  avgCellVoltageMv = canReadU16Be(frame.buf[5], frame.buf[6]);
  hasCellVoltageStats = true;

  uint32_t nowMs = millis();
  hasBmsStatus    = true;
  isBmsTimedOut      = false;
  lastBmsRxMs      = nowMs;
  lastBmsCellVoltageRxMs = nowMs;
  // Remain fail-safe until the following 0x311 frame supplies the matching
  // current measurement and the pair passes validation.
  isRegenBmsPairValid = false;
  bmsFlags         = flags;
  hasBmsFault         = (flags & 0x01) != 0;
  isBmsActive        = !hasBmsFault;
  stateOfChargePct = frame.buf[7];
  packVoltageV      = (avgCellVoltageMv / 1000.0f) * 96.0f;  // 96 cells in series

  if (isBmsActive && !wasActive) {
    Serial.println("[BMS] Active");
  }

  if (isFaultDetectionArmed && hasBmsFault != previousFault) {
    Serial.print("[BMS] 2026 flags=0x");
    Serial.print(flags, HEX);
    Serial.print(" fault=");
    Serial.println(hasBmsFault ? "YES" : "NO");
    if (hasBmsFault) {
      dashShowFaultBanner((hasBms2026FaultCode && bms2026FaultCode != 0)
                          ? faultTextGetBmsTemperatureCode(bms2026FaultCode)
                          : "BMS Fault");
    }
  }

  if (faultsIsAnyActive() || bmsAreContactorsOpen()) {
    rtdDrop(rtdGetDropCauseText());
  }

  isScreenDirty = true;
}

void bmsHandleTempFrame(const CAN_message_t &frame) {
  if (frame.len < 6) {
    return;
  }

  // byte 1 = max cell temp encoded as (°C + 40); 0xFF means no valid reading
  uint8_t previousFaultCode = hasBms2026FaultCode ? bms2026FaultCode : 0;
  bms2026FaultCode = frame.buf[3];
  hasBms2026FaultCode = true;
  if (faultsIsBmsImdDetectionArmed() && bms2026FaultCode != 0 &&
      bms2026FaultCode != previousFaultCode) {
    dashShowFaultBanner(faultTextGetBmsTemperatureCode(bms2026FaultCode));
  }

  hasCellTempStats = (frame.buf[0] != 0xFF && frame.buf[1] != 0xFF && frame.buf[2] != 0xFF);
  if (hasCellTempStats) {
    minCellTempC = (int16_t)frame.buf[0] - 40;
    maxCellTempC = (int16_t)frame.buf[1] - 40;
    avgCellTempC = (int16_t)frame.buf[2] - 40;
  } else {
    minCellTempC = CELL_TEMP_UNKNOWN_C;
    maxCellTempC = CELL_TEMP_UNKNOWN_C;
    avgCellTempC = CELL_TEMP_UNKNOWN_C;
  }

  if (frame.buf[1] != 0xFF) {
    batteryTempC = (uint8_t)constrain((int)frame.buf[1] - 40, 0, 255);
  }

  // bytes 4-5 = pack current, big-endian, raw = A * 10 + 3000
  int16_t bmsCurrentRaw = (int16_t)(((uint16_t)frame.buf[4] << 8) | frame.buf[5]);
  packCurrentA = ((float)bmsCurrentRaw - 3000.0f) / 10.0f;
  hasBmsCurrent = true;
  uint32_t nowMs = millis();
  lastBmsCurrentRxMs = nowMs;

  // The BMS sends 0x310 immediately before 0x311. Pair only adjacent/fresh
  // frames so a current sample is never used to correct an unrelated voltage.
  isRegenBmsPairValid = false;
  if (hasCellVoltageStats &&
      (uint32_t)(nowMs - lastBmsCellVoltageRxMs) <=
          REGEN_BMS_PAIR_MAX_SKEW_MS) {
    isRegenBmsPairValid = regenPushVoltageSample(nowMs);
  }

  isScreenDirty = true;
}

/**
 * @file    inverter.cpp
 * @brief   PM100DX status frames, CAN broadcast enable and automatic fault clear.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "inverter.h"

#include "safety_limits.h"
#include "can_bytes.h"
#include "bms_link.h"
#include "vehicle_can.h"
#include "rtd.h"
#include "fault_text.h"
#include "dashboard.h"

void inverterSendBroadcastEnable();
void inverterSendFaultClear();

static constexpr uint32_t PM100_PARAM_COMMAND_ID  = 0x0C1;

// EEPROM parameter 148 = "CAN Active Messages Lo Word"; bit 0x0200 enables the
// 0x0A9 Internal Voltages broadcast that carries the GLV (12V) voltage.
static constexpr uint16_t PM100_PARAM_CAN_ACTIVE_MSGS = 148;
static constexpr uint32_t PM100_BROADCAST_ENABLE_DELAY_MS = 5000;
static constexpr uint32_t PM100_BROADCAST_ENABLE_RETRY_MS = 5000;
static constexpr uint8_t PM100_BROADCAST_ENABLE_MAX_ATTEMPTS = 3;
// Parameter 20 = "Fault Clear"; writing 0 tells the inverter to clear latched
// fault codes. Sent automatically when the inverter is faulted but the BMS
// reports contactors closed and the DC bus is back at pack voltage, so a
// transient fault (e.g. under-voltage from an HV cycle or pack sag) doesn't
// require power-cycling the inverter. 240 V = 96 series cells at the 2.5 V
// minimum, so any bus above that means the pack is genuinely back on the bus.
static constexpr uint16_t PM100_PARAM_FAULT_CLEAR = 20;

float inverterDcBusVoltageV = 0.0f;
float inverterDcBusCurrentA = 0.0f;
bool hasInverterVoltageInfo = false;
bool hasInverterCurrentInfo = false;
// GLV (12V system) voltage reported by the PM100 Internal Voltages frame.
float glvVoltageV = 0.0f;
bool hasGlvVoltage = false;
// If the inverter is talking but 0x0A9 never arrives, its broadcast mask has
// Internal Voltages disabled; we write parameter 148 once to turn it back on.
bool hasPm100Traffic = false;
uint32_t firstPm100FrameMs = 0;
uint8_t pm100BroadcastEnableAttempts = 0;
uint32_t pm100BroadcastEnableLastTxMs = 0;
float powerKw = 0.0f;
int16_t inverterTempTenthsC = 0;
int16_t pm100MotorTempTenthsC = 0;
bool hasInverterTemp = false;
bool hasPm100MotorTemp = false;
uint32_t pm100PostFaults = 0;
uint32_t pm100RunFaults = 0;
bool hasPm100FaultStatus = false;
bool hasPm100Fault = false;
// Automatic fault-clear bookkeeping; attempts reset when the fault clears or
// the contactors open, so each HV cycle gets a fresh set of retries.
uint8_t pm100FaultClearAttempts = 0;
uint32_t pm100FaultClearLastTxMs = 0;
int16_t motorRpm = 0;

void inverterHandleTemperatures1Frame(const CAN_message_t &frame) {
  if (frame.len < 8) {
    return;
  }

  int16_t moduleATempRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t moduleBTempRaw = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t moduleCTempRaw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t gateDriverTempRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  int16_t hottestTempRaw = moduleATempRaw;
  if (moduleBTempRaw > hottestTempRaw) hottestTempRaw = moduleBTempRaw;
  if (moduleCTempRaw > hottestTempRaw) hottestTempRaw = moduleCTempRaw;
  if (gateDriverTempRaw > hottestTempRaw) hottestTempRaw = gateDriverTempRaw;

  inverterTempTenthsC = hottestTempRaw;
  hasInverterTemp = true;
  isScreenDirty = true;
}

void inverterHandleTemperatures3Frame(const CAN_message_t &frame) {
  if (frame.len < 6) {
    return;
  }

  pm100MotorTempTenthsC = canReadS16Le(frame.buf[4], frame.buf[5]);
  hasPm100MotorTemp = true;
  isScreenDirty = true;
}

void inverterHandleRpmFrame(const CAN_message_t &frame) {
  if (frame.len < 4) {
    return;
  }

  motorRpm = canReadS16Le(frame.buf[2], frame.buf[3]);
  isScreenDirty = true;
}

void inverterHandleCurrentInfoFrame(const CAN_message_t &frame) {
  if (frame.len < 8) {
    return;
  }

  inverterDcBusCurrentA = (float)canReadS16Le(frame.buf[6], frame.buf[7]) / 10.0f;
  hasInverterCurrentInfo = true;
  isScreenDirty = true;
}

void inverterHandleVoltageInfoFrame(const CAN_message_t &frame) {
  if (frame.len < 2) {
    return;
  }

  inverterDcBusVoltageV = (float)canReadS16Le(frame.buf[0], frame.buf[1]) / 10.0f;
  hasInverterVoltageInfo = true;
  isScreenDirty = true;
}

void inverterHandleInternalVoltagesFrame(const CAN_message_t &frame) {
  if (frame.len < 8) {
    return;
  }

  // Bytes 6-7 = 12V System voltage, "Low Voltage" format (volts * 100).
  glvVoltageV = (float)canReadS16Le(frame.buf[6], frame.buf[7]) / 100.0f;
  hasGlvVoltage = true;
  isScreenDirty = true;
}

void inverterHandleParamResponseFrame(const CAN_message_t &frame) {
  if (frame.len < 3) {
    return;
  }

  uint16_t address = canReadU16Le(frame.buf[0], frame.buf[1]);
  if (address == PM100_PARAM_CAN_ACTIVE_MSGS) {
    Serial.print("[PM100] CAN Active Messages parameter write ");
    Serial.println(frame.buf[2] ? "succeeded" : "FAILED");
  } else if (address == PM100_PARAM_FAULT_CLEAR) {
    Serial.print("[PM100] Fault clear command ");
    Serial.println(frame.buf[2] ? "accepted" : "REJECTED");
  }
}

// The 0x0A9 Internal Voltages broadcast (GLV voltage source) can be disabled
// in the inverter's EEPROM broadcast mask. If the inverter is alive but 0x0A9
// stays silent, write parameter 148 to enable all broadcast messages.
// EEPROM parameters are only writable while the motor is not running.
void inverterServiceBroadcastEnable() {
  if (hasGlvVoltage || !hasPm100Traffic || isReadyToDriveLatched) {
    return;
  }

  if (pm100BroadcastEnableAttempts >= PM100_BROADCAST_ENABLE_MAX_ATTEMPTS) {
    return;
  }

  uint32_t nowMs = millis();
  if (nowMs - firstPm100FrameMs < PM100_BROADCAST_ENABLE_DELAY_MS) {
    return;
  }

  if (pm100BroadcastEnableAttempts > 0 &&
      nowMs - pm100BroadcastEnableLastTxMs < PM100_BROADCAST_ENABLE_RETRY_MS) {
    return;
  }

  pm100BroadcastEnableAttempts++;
  pm100BroadcastEnableLastTxMs = nowMs;

  Serial.print("[PM100] No 0x0A9 Internal Voltages frames; enabling broadcasts, attempt ");
  Serial.println(pm100BroadcastEnableAttempts);
  inverterSendBroadcastEnable();
}

void inverterSendBroadcastEnable() {
  CAN_message_t txFrame = {};
  txFrame.id = PM100_PARAM_COMMAND_ID;
  txFrame.len = 8;
  txFrame.flags.extended = 0;
  txFrame.buf[0] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS & 0xFF);
  txFrame.buf[1] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS >> 8);
  txFrame.buf[2] = 1;     // write
  txFrame.buf[3] = 0;     // reserved
  txFrame.buf[4] = 0xFF;  // CAN Active Messages Lo Word: enable all broadcasts
  txFrame.buf[5] = 0xFF;
  txFrame.buf[6] = 0xFF;  // Hi Word: keep command/BMS/OBD2 mailboxes enabled
  txFrame.buf[7] = 0xFF;
  canSendFrame(txFrame);
}

// Automatically clears latched PM100 faults over CAN once the conditions that
// caused them are gone: the BMS must report contactors closed and the inverter
// must see pack voltage on its DC bus again. Without this, an under-voltage
// fault latched during an HV cycle or pack sag blocks torque until someone
// power-cycles the inverter. Fault Clear is a command parameter (not an EEPROM
// write), so it is safe to send any time; a fault whose cause is still present
// simply latches again.
void inverterServiceFaultClear() {
  if (!hasPm100FaultStatus || !hasPm100Fault) {
    return;
  }

  if (!hasBmsStatus || bmsAreContactorsOpen()) {
    pm100FaultClearAttempts = 0;
    return;
  }

  if (!hasInverterVoltageInfo ||
      inverterDcBusVoltageV < PM100_FAULT_CLEAR_MIN_BUS_V) {
    return;
  }

  if (pm100FaultClearAttempts >= PM100_FAULT_CLEAR_MAX_ATTEMPTS) {
    return;
  }

  uint32_t nowMs = millis();
  if (pm100FaultClearAttempts > 0 &&
      nowMs - pm100FaultClearLastTxMs < PM100_FAULT_CLEAR_RETRY_MS) {
    return;
  }

  pm100FaultClearAttempts++;
  pm100FaultClearLastTxMs = nowMs;

  Serial.print("[PM100] Fault latched with contactors closed and bus at ");
  Serial.print(inverterDcBusVoltageV, 1);
  Serial.print(" V; sending fault clear, attempt ");
  Serial.println(pm100FaultClearAttempts);
  inverterSendFaultClear();
}

void inverterSendFaultClear() {
  CAN_message_t txFrame = {};
  txFrame.id = PM100_PARAM_COMMAND_ID;
  txFrame.len = 8;
  txFrame.flags.extended = 0;
  txFrame.buf[0] = (uint8_t)(PM100_PARAM_FAULT_CLEAR & 0xFF);
  txFrame.buf[1] = (uint8_t)(PM100_PARAM_FAULT_CLEAR >> 8);
  txFrame.buf[2] = 1;  // write
  txFrame.buf[3] = 0;  // reserved
  txFrame.buf[4] = 0;  // data 0 = clear all faults
  txFrame.buf[5] = 0;
  txFrame.buf[6] = 0;
  txFrame.buf[7] = 0;
  canSendFrame(txFrame);
}

void inverterHandleFaultCodesFrame(const CAN_message_t &frame) {
  if (frame.len < 8) {
    return;
  }

  bool previousFault = hasPm100FaultStatus && hasPm100Fault;
  pm100PostFaults = canReadU32Le(frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3]);
  pm100RunFaults = canReadU32Le(frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
  hasPm100FaultStatus = true;
  hasPm100Fault = (pm100PostFaults != 0 || pm100RunFaults != 0);
  if (hasPm100Fault && !previousFault) {
    dashShowFaultBanner(faultTextGetFirstPm100(pm100PostFaults, pm100RunFaults));
  }
  if (!hasPm100Fault) {
    if (previousFault && pm100FaultClearAttempts > 0) {
      Serial.println("[PM100] Faults cleared");
    }
    pm100FaultClearAttempts = 0;
  }
  isScreenDirty = true;
}

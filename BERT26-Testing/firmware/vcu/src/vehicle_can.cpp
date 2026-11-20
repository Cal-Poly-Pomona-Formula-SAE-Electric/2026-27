/**
 * @file    vehicle_can.cpp
 * @brief   Sets up CAN3, sends frames and dispatches received frames to their module.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "vehicle_can.h"

#include "energy_meter.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "data_log.h"
#include "can_trace.h"

void canDispatchReceivedFrame(const CAN_message_t &frame);
void canLogFrameToSerial(const char *direction, const CAN_message_t &frame);

// ---------------- CAN ----------------
// GPIO 23 is used by the RTD button, so this sketch uses the Teensy 4.1 CAN3
// peripheral on pins 30=CAN RX and 31=CAN TX. These pins connect to the TXD/RXD
// side of an external CAN transceiver module; the module's CANH/CANL side
// connects to the vehicle CAN bus.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> vehicleCan;

static constexpr uint32_t CAN_BAUD_BPS = 500000;
static constexpr bool IS_CAN_LOG_SERIAL_ENABLED = true;

static constexpr uint32_t PM100_PARAM_RESPONSE_ID = 0x0C2;

// ---- Formula SAE competition Energy Meter (EM_User_Manual_2026) ----
// The Energy Meter shares this 500 kbit/s vehicle bus. It broadcasts three
// 11-bit standard frames; all signals are little-endian. The VCU only listens
// and decodes Voltage, the module-temperature summary, and the status bits for
// the dashboard.
static constexpr uint32_t EM_MEASUREMENT_ID = 0x10D;  // 20 ms:  Current[A], Voltage[V] floats
static constexpr uint32_t EM_STATUS_ID      = 0x40D;  // 100 ms: status bits + Energy[Whr] float
static constexpr uint32_t EM_TEMPERATURE_ID = 0x60D;  // 250 ms: muxed module temperatures

// ---------------- CAN transport ----------------
void canSetup() {
  vehicleCan.begin();
  vehicleCan.setBaudRate(CAN_BAUD_BPS);
  vehicleCan.setMaxMB(16);
  vehicleCan.enableFIFO();

  Serial.println("[CAN] CAN3 started at 500 kbit/s on pins 30=RX, 31=TX");
}

void canSendFrame(CAN_message_t &frame) {
  noInterrupts();
  vehicleCan.write(frame);
  interrupts();
  canLogFrame("TX", frame);
}

void canReceiveFrames() {
  CAN_message_t rxFrame;
  while (vehicleCan.read(rxFrame)) {
    canLogFrame("RX", rxFrame);
    canDispatchReceivedFrame(rxFrame);
  }
}

void canDispatchReceivedFrame(const CAN_message_t &frame) {
  if (!hasPm100Traffic && frame.id >= PM100_TEMPERATURE_1_FRAME_ID &&
      frame.id <= PM100_TORQUE_CAPABILITY_FRAME_ID) {
    hasPm100Traffic = true;
    firstPm100FrameMs = millis();
  }

  switch (frame.id) {
    case IMD_INFO_GENERAL_ID:
    case IMD_RESPONSE_ID:
      imdHandleFrame(frame);
      break;

    case PM100_TEMPERATURE_1_FRAME_ID:
      inverterHandleTemperatures1Frame(frame);
      break;

    case PM100_TEMPERATURE_3_FRAME_ID:
      inverterHandleTemperatures3Frame(frame);
      break;

    case PM100_RPM_FRAME_ID:
      inverterHandleRpmFrame(frame);
      break;

    case PM100_CURRENT_INFO_FRAME_ID:
      inverterHandleCurrentInfoFrame(frame);
      break;

    case PM100_VOLTAGE_INFO_FRAME_ID:
      inverterHandleVoltageInfoFrame(frame);
      break;

    case PM100_INTERNAL_VOLTAGES_FRAME_ID:
      inverterHandleInternalVoltagesFrame(frame);
      break;

    case PM100_FAULT_CODES_FRAME_ID:
      inverterHandleFaultCodesFrame(frame);
      break;

    case PM100_PARAM_RESPONSE_ID:
      inverterHandleParamResponseFrame(frame);
      break;

    case BMS_STATUS_2026_ID:
      bmsHandleStatusFrame(frame);
      break;

    case BMS_TEMPERATURE_2026_ID:
      bmsHandleTempFrame(frame);
      break;

    case BMS_CELL_RESPONSE_2026_ID:
      logWriteCellDataResponse(frame);
      break;

    case EM_MEASUREMENT_ID:
      emHandleMeasurementFrame(frame);
      break;

    case EM_STATUS_ID:
      emHandleStatusFrame(frame);
      break;

    case EM_TEMPERATURE_ID:
      emHandleTemperatureFrame(frame);
      break;

    default:
      break;
  }
}

// ---------------- CAN logging ----------------
void canLogFrame(const char *direction, const CAN_message_t &frame) {
  traceUpdateState(direction, frame);

  // Future SD-card logging should mirror this raw record and the decoded record here.
  if (IS_CAN_LOG_SERIAL_ENABLED) {
    canLogFrameToSerial(direction, frame);
    traceLogToSerial(direction, frame);
  }
}

void canLogFrameToSerial(const char *direction, const CAN_message_t &frame) {
  Serial.print("[CAN ");
  Serial.print(direction);
  Serial.print("] t=");
  Serial.print(millis());
  Serial.print(" id=0x");
  Serial.print(frame.id, HEX);
  Serial.print(frame.flags.extended ? " ext" : " std");
  Serial.print(" len=");
  Serial.print(frame.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < frame.len; i++) {
    tracePrintCanHexByte(frame.buf[i]);
    if (i + 1 < frame.len) {
      Serial.print(' ');
    }
  }

  Serial.println();
}

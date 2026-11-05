/**
 * @file    imd.cpp
 * @brief   Bender iso175 IMD frames and the warning/alarm poll.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.6.1.7
 */
#include "imd.h"

#include "can_bytes.h"
#include "vehicle_can.h"
#include "faults.h"
#include "tssi.h"
#include "can_trace_imd.h"
#include "fault_text.h"
#include "dashboard.h"

bool hasImdFault = true;
uint16_t imdWarnAlarms = 0;
uint16_t imdRIsoKohm = 0xFFFF;
uint8_t imdRIsoStatus = 0;
uint8_t imdDeviceActivity = 0;
uint32_t lastImdRxMs = 0;
bool isImdCanTimedOut = true;

void imdHandleFrame(const CAN_message_t &frame) {
  bool wasTimedOut = isImdCanTimedOut;
  uint16_t previousWarnings = imdWarnAlarms;
  bool previousFault = hasImdFault;
  bool isFaultDetectionArmed = faultsIsBmsImdDetectionArmed();

  if (frame.id == IMD_INFO_GENERAL_ID && frame.len >= 5) {
    imdRIsoKohm = canReadU16Le(frame.buf[0], frame.buf[1]);
    imdRIsoStatus = frame.buf[2];
    imdWarnAlarms = (imdWarnAlarms & 0xFF00u) | frame.buf[4];
    imdDeviceActivity = (frame.len >= 7) ? frame.buf[6] : IMD_SNV_BYTE;
    lastImdRxMs = millis();
    isImdCanTimedOut = false;
    hasImdFault = tssiHasFaultInWarnBits(imdWarnAlarms);
  } else if (frame.id == IMD_RESPONSE_ID && frame.len >= 3 && frame.buf[0] == IDX_WARN_ALARMS) {
    imdWarnAlarms = canReadU16Le(frame.buf[1], frame.buf[2]);
    lastImdRxMs = millis();
    isImdCanTimedOut = false;
    hasImdFault = tssiHasFaultInWarnBits(imdWarnAlarms);
  } else {
    return;
  }

  if (wasTimedOut) {
    Serial.println("[TSSI] IMD CAN restored");
  }

  if (isFaultDetectionArmed && (imdWarnAlarms != previousWarnings || hasImdFault != previousFault)) {
    Serial.print("[TSSI] IMD warn=0x");
    Serial.print(imdWarnAlarms, HEX);
    tssiPrintWarnBits(imdWarnAlarms);
    Serial.print(" => ");
    Serial.println(hasImdFault ? "FAULT, red blink" : "OK, green on");
    if (hasImdFault && !previousFault) {
      dashShowFaultBanner(faultTextGetFirstTssi(imdWarnAlarms));
    }
  }
}

void imdRequestWarnings() {
  CAN_message_t txFrame = {};
  txFrame.id = IMD_REQUEST_ID;
  txFrame.len = 8;
  txFrame.flags.extended = 0;
  txFrame.buf[0] = IDX_WARN_ALARMS;
  for (int i = 1; i < 8; i++) {
    txFrame.buf[i] = 0xFF;
  }
  canSendFrame(txFrame);
}

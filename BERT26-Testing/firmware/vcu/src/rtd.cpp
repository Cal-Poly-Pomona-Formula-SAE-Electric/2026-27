/**
 * @file    rtd.cpp
 * @brief   Runs the ready-to-drive state machine: RTD button, entry conditions, drive mode and drop causes.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "rtd.h"

#include "pins.h"
#include "safety_limits.h"
#include "buzzer.h"
#include "imd.h"
#include "bms_link.h"
#include "faults.h"
#include "pedals.h"
#include "can_trace.h"
#include "fault_text.h"
#include "dashboard.h"

bool rtdAreInputsReady();
bool rtdIsReadyForButton();
void rtdLogBlockedReason();
void rtdHandleButtonPressed();
void rtdSetState(RtdState newState);
const char *rtdGetStateName(RtdState state);

static constexpr uint32_t RTD_BLOCKED_LOG_PERIOD_MS = 1000;

// ---------------- Runtime state ----------------
char driveMode = 'N';  // RTD state machine sets D only while ready to drive.

RtdState rtdState = RTD_STATE_FAULT_BLOCKED;
// Compatibility flag used by the display and PM100 gate; kept in sync with rtdState.
bool isReadyToDriveLatched = false;

bool wasReadyButtonPressed = false;

// EV.10.2: ready-to-drive sound, 1-3 seconds. Solid continuous tone for the
// first 1.25 s (satisfies the >=1 s continuous requirement), then a short
// playful flourish. Step on+off times sum to 2000 ms total.
static const ToneStep RTD_PATTERN[] = {
  {2093, 1250,  0},  // C7 - continuous main sound (1.25 s)
  {2637,  120, 40},  // E7
  {3136,  120, 40},  // G7
  {2637,  120, 40},  // E7
  {3136,  270,  0}   // G7 - final flourish
};

// ---------------- Ready to drive state machine ----------------
void rtdService() {
  rtdRefreshStateFromInputs();

  bool isPressed = rtdReadButtonPressed();

  if (isPressed && !wasReadyButtonPressed) {
    rtdHandleButtonPressed();
    isScreenDirty = true;
  }

  wasReadyButtonPressed = isPressed;
}

bool rtdReadButtonPressed() {
  return digitalRead(PIN_READY_BUTTON) == HIGH;
}

bool rtdAreInputsReady() {
  if (!faultsIsBmsImdDetectionArmed()) {
    return false;
  }

  // APPS implausibility intentionally does not gate RTD; it zeroes the torque
  // command instead (see pedalsUpdate/inverterCmdUpdateState).
  return !faultsIsAnyActive() && !bmsAreContactorsOpen();
}

bool rtdIsReadyForButton() {
  return rtdState == RTD_STATE_READY_FOR_BUTTON;
}

void rtdRefreshStateFromInputs() {
  if (rtdState == RTD_STATE_READY_TO_DRIVE) {
    if (!rtdAreInputsReady()) {
      rtdDrop(rtdGetDropCauseText());
    }
    return;
  }

  bool areInputsReady = rtdAreInputsReady();
  if (!areInputsReady) {
    rtdLogBlockedReason();
  }

  rtdSetState(areInputsReady ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);
}

// Periodically prints why RTD_STATE_FAULT_BLOCKED hasn't cleared, e.g. so a
// stuck contactor/fault/IMD flag after an HV power cycle shows up in the
// serial log instead of just leaving the dashboard stuck on "N".
void rtdLogBlockedReason() {
  static uint32_t lastLogMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastLogMs < RTD_BLOCKED_LOG_PERIOD_MS) {
    return;
  }
  lastLogMs = nowMs;

  if (!faultsIsBmsImdDetectionArmed()) {
    Serial.println("[RTD] Blocked: startup fault-detection delay active");
    return;
  }

  Serial.print("[RTD] Blocked:");
  if (hasImdFault)            Serial.print(" imdFault");
  if (hasBmsFault)            Serial.print(" bmsFault");
  if (bmsAreContactorsOpen())    Serial.print(" contactorsOpen");
  if (isImdCanTimedOut)      Serial.print(" imdCanTimedOut");
  if (isBmsTimedOut)         Serial.print(" bmsTimedOut");
  Serial.print(" bmsFlags=0x");
  tracePrintCanHexByte(bmsFlags);
  Serial.print(" imdWarnAlarms=0x");
  Serial.print(imdWarnAlarms, HEX);
  Serial.println();
}

void rtdHandleButtonPressed() {
  Serial.print("[RTD] Button pressed, state=");
  Serial.println(rtdGetStateName(rtdState));

  switch (rtdState) {
    case RTD_STATE_READY_FOR_BUTTON:
      if (IS_BRAKE_REQUIRED_FOR_RTD && !isBraking) {
        Serial.println("[RTD] Not latched; brake pedal must be pressed");
        break;
      }
      rtdSetState(RTD_STATE_READY_TO_DRIVE);
      buzzerPlayPattern(RTD_PATTERN, sizeof(RTD_PATTERN) / sizeof(RTD_PATTERN[0]), false);
      Serial.println("[RTD] Ready to drive latched");
      break;

    case RTD_STATE_READY_TO_DRIVE:
      rtdDrop("RTD button pressed");
      break;

    case RTD_STATE_FAULT_BLOCKED:
    default:
      rtdSetState(RTD_STATE_FAULT_BLOCKED);
      Serial.println(faultsIsAnyActive() ? "[RTD] Not latched; anyFault is active"
                                       : "[RTD] Not latched; contactors open");
      break;
  }
}

void rtdSetState(RtdState newState) {
  char newDriveMode = (newState == RTD_STATE_READY_TO_DRIVE) ? 'D' : 'N';

  if (rtdState == newState) {
    isReadyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
    if (driveMode != newDriveMode) {
      rtdSetDriveMode(newDriveMode);
      isScreenDirty = true;
    }
    return;
  }

  rtdState = newState;
  isReadyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
  rtdSetDriveMode(newDriveMode);
  isScreenDirty = true;
}

void rtdDrop(const char *reason) {
  bool wasLatched = (rtdState == RTD_STATE_READY_TO_DRIVE) || isReadyToDriveLatched;

  rtdSetState(rtdAreInputsReady() ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);

  if (wasLatched && reason != nullptr) {
    Serial.print("[RTD] Dropped; ");
    Serial.println(reason);
    if (strcmp(reason, "RTD button pressed") != 0) {
      dashShowFaultBanner(reason);
    }
  }
}

// Names the specific condition currently blocking RTD, for serial logs and
// the dashboard fault banner.
const char *rtdGetDropCauseText() {
  if (isBmsTimedOut && hasBmsStatus) return "BMS Timeout";
  if (isImdCanTimedOut) return "IMD Timeout";
  if (hasBmsFault) {
    return (hasBms2026FaultCode && bms2026FaultCode != 0)
               ? faultTextGetBmsTemperatureCode(bms2026FaultCode)
               : "BMS Fault";
  }
  if (hasImdFault) return faultTextGetFirstTssi(imdWarnAlarms);
  if (bmsAreContactorsOpen()) return "Contactors Open";
  return "Fault";
}

const char *rtdGetStateName(RtdState state) {
  switch (state) {
    case RTD_STATE_FAULT_BLOCKED:
      return "FAULT_BLOCKED";
    case RTD_STATE_READY_FOR_BUTTON:
      return "READY_FOR_BUTTON";
    case RTD_STATE_READY_TO_DRIVE:
      return "READY_TO_DRIVE";
    default:
      return "UNKNOWN";
  }
}

void rtdSetDriveMode(char newDriveMode) {
  if (newDriveMode != 'D' && newDriveMode != 'N') {
    return;
  }

  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  isScreenDirty = true;
}

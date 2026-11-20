/**
 * @file    tssi.cpp
 * @brief   Drives the TSSI lights: green when safe, red flashing on an IMD or BMS fault.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "tssi.h"

#include "pins.h"
#include "safety_limits.h"
#include "imd.h"
#include "bms_link.h"
#include "faults.h"
#include "rtd.h"
#include "dashboard.h"

void tssiUpdateLed(bool isFaultActive, uint32_t nowMs);
void tssiSetGreenOn();
void tssiSetRedOn();
void tssiSetRedOff();
void tssiSetAllOff();

static constexpr uint32_t IMD_GET_REQ_PERIOD_MS    = 500;

uint32_t tssiBlinkTimerMs = 0;
bool isTssiRedOn = false;
// Once the TSSI has blinked red for any fault it stays latched in the red-blink
// state until the board is power-cycled (this flag clears on reset).
bool isTssiFaultLatched = false;
uint32_t lastImdGetReqMs = 0;

bool tssiHasFaultInWarnBits(uint16_t warnBits) {
  return (warnBits & IMD_FAULT_BITS_MASK) != 0;
}

void tssiSetup() {
  pinMode(PIN_TSSI_LED_RED, OUTPUT);
  pinMode(PIN_TSSI_LED_GREEN, OUTPUT);
  tssiSetAllOff();

  hasImdFault = true;
  isImdCanTimedOut = true;
  lastImdRxMs = millis();
  tssiBlinkTimerMs = millis();
  lastImdGetReqMs = 0;
}

void tssiService() {
  uint32_t nowMs = millis();
  bool isFaultDetectionArmed = faultsIsBmsImdDetectionArmed();

  if (isFaultDetectionArmed && nowMs - lastImdRxMs > IMD_HEARTBEAT_TIMEOUT_MS) {
    if (!isImdCanTimedOut) {
      Serial.println("[TSSI] IMD heartbeat lost, forcing FAULT");
      dashShowFaultBanner("IMD Timeout");
    }
    isImdCanTimedOut = true;
    hasImdFault = true;
  }

  if ((faultsIsAnyActive() || bmsAreContactorsOpen()) && isReadyToDriveLatched) {
    rtdDrop(rtdGetDropCauseText());
  }

  if (nowMs - lastImdGetReqMs >= IMD_GET_REQ_PERIOD_MS) {
    lastImdGetReqMs = nowMs;
    imdRequestWarnings();
  }

  // Latch the red blink the first time a fault is seen; once latched the TSSI
  // stays red until power cycle, even if the underlying fault clears.
  if (isFaultDetectionArmed && faultsIsAnyActive()) {
    isTssiFaultLatched = true;
  }

  if (isTssiFaultLatched) {
    tssiUpdateLed(true, nowMs);
  } else if (!isFaultDetectionArmed) {
    tssiSetGreenOn();
  } else {
    tssiUpdateLed(faultsIsAnyActive(), nowMs);
  }
}

void tssiUpdateLed(bool isFaultActive, uint32_t nowMs) {
  if (isFaultActive) {
    uint32_t elapsedMs = nowMs - tssiBlinkTimerMs;
    if (elapsedMs >= TSSI_BLINK_PERIOD_MS) {
      tssiBlinkTimerMs += TSSI_BLINK_PERIOD_MS;
      elapsedMs = nowMs - tssiBlinkTimerMs;
    }

    if (elapsedMs < TSSI_BLINK_ON_MS) {
      if (!isTssiRedOn) {
        tssiSetRedOn();
      }
    } else if (isTssiRedOn) {
      tssiSetRedOff();
    }
  } else {
    tssiSetGreenOn();
  }
}

void tssiSetGreenOn() {
  digitalWrite(PIN_TSSI_LED_RED, LOW);
  digitalWrite(PIN_TSSI_LED_GREEN, HIGH);
  isTssiRedOn = false;
}

void tssiSetRedOn() {
  digitalWrite(PIN_TSSI_LED_GREEN, LOW);
  digitalWrite(PIN_TSSI_LED_RED, HIGH);
  isTssiRedOn = true;
}

void tssiSetRedOff() {
  digitalWrite(PIN_TSSI_LED_RED, LOW);
  isTssiRedOn = false;
}

void tssiSetAllOff() {
  digitalWrite(PIN_TSSI_LED_GREEN, LOW);
  digitalWrite(PIN_TSSI_LED_RED, LOW);
  isTssiRedOn = false;
}

void tssiPrintWarnBits(uint16_t bits) {
  if (bits & (1u << 0)) Serial.print(" [DeviceError]");
  if (bits & (1u << 1)) Serial.print(" [HV+ConnFail]");
  if (bits & (1u << 2)) Serial.print(" [HV-ConnFail]");
  if (bits & (1u << 3)) Serial.print(" [EarthConnFail]");
  if (bits & (1u << 4)) Serial.print(" [IsoAlarm]");
  if (bits & (1u << 5)) Serial.print(" [IsoWarning]");
  if (bits & (1u << 6)) Serial.print(" [IsoOutdated]");
  if (bits & (1u << 7)) Serial.print(" [UnbalanceAlarm]");
  if (bits & (1u << 8)) Serial.print(" [Undervoltage]");
  if (bits & (1u << 9)) Serial.print(" [UnsafeToStart]");
  if (bits & (1u << 10)) Serial.print(" [EarthliftOpen]");
}

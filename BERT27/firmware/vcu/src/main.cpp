#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pins.h"
#include "safety_limits.h"
#include "calibration.h"
#include "vehicle_speed.h"
#include "buzzer.h"
#include "fans.h"
#include "imu.h"
#include "energy_meter.h"
#include "bms_link.h"
#include "inverter.h"
#include "vehicle_can.h"
#include "faults.h"
#include "power_derate.h"
#include "pedals.h"
#include "inverter_command.h"
#include "rtd.h"
#include "tssi.h"
#include "data_log.h"
#include "serial_console.h"
#include "dashboard.h"

/*
  BERT26 VCU integrated sketch

  Functions:
    - Drives the ST7796/XPT2046 touchscreen dashboard.
    - Plays a buzzer chirp when BMS status becomes active.
    - Plays a different buzzer pattern when the RTD button on GPIO 23 is pressed.
    - Reads the dual analog accelerator pedal sensor.
    - Sends Cascadia PM100DX torque commands at 100 Hz over CAN.
    - Logs float telemetry to separate measured (MEASnnnn.BIN), derived
      (DERInnnn.BIN), and per-cell (CELLnnnn.BIN) files on the built-in SD card.
*/

// ---------------- Setup ----------------
void setup() {
  startupFaultDelayStartMs = millis();

  Serial.begin(115200);

  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  fansSetup();

  pinMode(PIN_READY_BUTTON, INPUT);
  wasReadyButtonPressed = rtdReadButtonPressed();
  rtdRefreshStateFromInputs();

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(PIN_APPS_1, INPUT);
  pinMode(PIN_APPS_2, INPUT);
  pinMode(PIN_BSE_1, INPUT);
  pinMode(PIN_BSE_2, INPUT);
  pinMode(PIN_HV_CURRENT_SENSE, INPUT);

  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  digitalWrite(PIN_BRAKE_LIGHT, LOW);

  // Pulldowns so a disconnected status line reads LOW and shows red.
  pinMode(PIN_BSPD_STATUS, INPUT_PULLDOWN);
  pinMode(PIN_IMD_STATUS, INPUT_PULLDOWN);
  pinMode(PIN_BMS_STATUS, INPUT_PULLDOWN);

  tssiSetup();
  canSetup();
  inverterCmdSetupTimer();

  while (!Serial && millis() < 1500) {}

  Serial.println("BERT26 VCU starting");
  Serial.print("[Startup] BMS/IMD fault detection delayed for ");
  Serial.print(STARTUP_BMS_IMD_FAULT_DELAY_MS);
  Serial.println(" ms");

  imuSetup();

  logSetup();

  dashSetup();

  dashDraw();
}

// ---------------- Main loop ----------------
void loop() {
  consoleHandleInput();
  fansUpdateDutyFromTemps();
  fansService();
  canReceiveFrames();
  pedalsUpdateBrakePressureSensors();
  pedalsUpdate();
  digitalWrite(PIN_BRAKE_LIGHT, isBraking ? HIGH : LOW);
  pedalsLogAppsTelemetry();
  rtdService();
  buzzerService();
  tssiService();
  dashServiceFaultBanner();
  dashHandleTouch();

  uint32_t nowMs = millis();
  if (faultsIsBmsImdDetectionArmed() && hasBmsStatus && (nowMs - lastBmsRxMs > BMS_TIMEOUT_MS)) {
    if (!isBmsTimedOut) {
      Serial.println("[BMS] Status timeout"); //,disabling torque
      dashShowFaultBanner("BMS Timeout");
      isScreenDirty = true;
    }
    isBmsTimedOut = true;
    isBmsActive = false;
    hasBmsFault = true;
    rtdDrop("BMS Timeout");
  }

  emServiceTimeout(nowMs);

  if (hasInverterVoltageInfo && hasInverterCurrentInfo) {
    powerKw = (inverterDcBusVoltageV * inverterDcBusCurrentA) / 1000.0f;
  } else {
    powerKw = 0.0f;
  }
  speedMph = fabsf((float)motorRpm) * MPH_PER_RPM;
  speedKmh = speedMph * KMH_PER_MPH;
  speedUpdateCumulativeDistance(nowMs);
  derateService(nowMs);
  inverterCmdUpdateState();
  inverterCmdServiceTxLogging();
  inverterServiceBroadcastEnable();
  inverterServiceFaultClear();
  bmsServiceCellDataRequests(nowMs);
  logService();

  // Rewrite the entire screen every 2 minutes to wipe any accumulated SPI
  // glitches that the per-cell incremental redraw would otherwise leave behind.
  static uint32_t lastFullRedrawMs = 0;
  if (nowMs - lastFullRedrawMs >= DASHBOARD_FULL_REDRAW_MS) {
    lastFullRedrawMs = nowMs;
    isDashFullRedrawNeeded = true;
    isScreenDirty = true;
  }

  static uint32_t lastDrawMs = 0;
  if (isScreenDirty || nowMs - lastDrawMs >= SCREEN_PERIOD_MS) {
    lastDrawMs = nowMs;
    isScreenDirty = false;
    dashDraw();
  }
}


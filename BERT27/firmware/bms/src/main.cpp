/**
 * @file    main.cpp
 * @brief   Starts every BMS module and runs the main loop.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include <Arduino.h>
#include "pins.h"
#include "calibration.h"
#include "bq79616.h"
#include "cell_monitor.h"
#include "current_sensor.h"
#include "shutdown_circuit.h"
#include "faults.h"
#include "state_machine.h"
#include "charger_can.h"
#include "vehicle_can.h"
#include "cell_dump.h"
#include "serial_console.h"

/*
  BERT26 BMS integrated sketch

  Functions:
    - Reads 96 cell voltages and temperatures from a six-device BQ79616 stack.
    - Tracks pack current and state of charge, saved to EEPROM.
    - Runs precharge and closes the AIR+ once the shutdown circuit closes.
    - Opens the contactors and the shutdown circuit on any cell or pack fault.
    - Broadcasts 0x310/0x311 status to the VCU and dumps cell data on request.
    - Controls the HK-MF charger over CAN1 when charger traffic is seen at start-up.
*/

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("[BMS] Full firmware — Teensy 4.1");

  delay(500);  // power-up settle before touching pins or attaching interrupts

  // SAFETY: contactor drives and the status line start LOW (open / fault).
  pinMode(PIN_BMS_STATUS,      OUTPUT); digitalWrite(PIN_BMS_STATUS,      LOW);
  pinMode(PIN_PRECHARGE_RELAY, OUTPUT); digitalWrite(PIN_PRECHARGE_RELAY, LOW);
  pinMode(PIN_POS_AIR,         OUTPUT); digitalWrite(PIN_POS_AIR,         LOW);

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(PIN_PACK_VOLTAGE_SENSE, INPUT);
  pinMode(PIN_PACK_CURRENT_SENSE, INPUT);
  sdcSetup();

  chargerCanSetup();
  vehicleCanSetup();

  currentSensorLoadCalibration();
  tempReadTimerMs = TEMP_INTERVAL_MS;  // first temperature read on loop entry
  stateMachineStartInitTimer();  // before bqInit so the INIT window includes BQ start-up

  if (!bqInitWithRetries()) {
    faultsEnter(FAULT_BQ_INIT);
    return;
  }

  cellMonitorReadInitialVoltages();

  Serial.println("[BMS] Waiting for 3s INIT window (mode detection active)...");
}

// ---------------- Main loop ----------------
void loop() {
  vehicleCan.events();
  chargerCan.events();

  cellDumpService();
  cellMonitorService();
  consoleHandleInput();
  vehicleCanServiceStatusBroadcast();
  stateMachineService();
}

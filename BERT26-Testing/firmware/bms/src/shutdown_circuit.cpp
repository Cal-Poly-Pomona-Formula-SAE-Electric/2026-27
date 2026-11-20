/**
 * @file    shutdown_circuit.cpp
 * @brief   Watches the shutdown circuit input and opens the contactors when it drops.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.7.2
 */
#include "shutdown_circuit.h"

#include "pins.h"
#include "soc.h"
#include "state_machine.h"

static void sdcEdgeIsr();

volatile bool isSdcHigh     = false;
volatile bool hasSdcChanged = false;

void sdcSetup() {
  pinMode(PIN_SDC_SENSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SDC_SENSE), sdcEdgeIsr, CHANGE);
}

static void sdcEdgeIsr() {
  isSdcHigh     = (bool)digitalRead(PIN_SDC_SENSE);
  hasSdcChanged = true;
}

// SDC fell LOW: a clean shutdown, NOT a fault. The BQ monitors shut themselves
// down through COMM_TIMEOUT_CONF once polling stops.
void sdcHandleShutdown() {
  // SAFETY: open both contactor drives before anything else.
  digitalWrite(PIN_PRECHARGE_RELAY, LOW);
  digitalWrite(PIN_POS_AIR,         LOW);
  areContactorsClosed = false;
  socSave();
  bmsState = STATE_WAIT_SDC;
  Serial.println("[BMS] SDC shutdown — waiting for next close");
}

// Called at the end of every state handler that can be running with the
// contactors closed: a falling edge drops straight back to WAIT_SDC.
void sdcServiceFallingEdge() {
  if (hasSdcChanged && !isSdcHigh) { hasSdcChanged = false; sdcHandleShutdown(); }
}

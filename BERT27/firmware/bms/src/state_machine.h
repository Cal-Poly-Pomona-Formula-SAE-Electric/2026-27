/**
 * @file    state_machine.h
 * @brief   BMS state machine: init, wait for SDC, precharge, close AIR+, running, charging, fault.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.5.6, EV.7.3
 */
#ifndef BMS_STATE_MACHINE_H
#define BMS_STATE_MACHINE_H

#include <Arduino.h>

// Decided at the end of INIT from which CAN bus shows traffic.
enum BmsMode { MODE_UNKNOWN, MODE_DISCHARGE, MODE_CHARGE };

enum BmsState {
  STATE_INIT,
  STATE_WAIT_SDC,
  STATE_PRECHARGE,
  STATE_CLOSE_POSITIVE,
  STATE_CHARGE_SETTLE,
  STATE_RUNNING,
  STATE_CHARGE_ACTIVE,
  STATE_CHARGE_DONE,
  STATE_FAULT
};

extern BmsMode  bmsMode;
extern BmsState bmsState;
extern bool     areContactorsClosed;

void stateMachineStartInitTimer();
void stateMachineService();

#endif  // BMS_STATE_MACHINE_H

/**
 * @file    faults.h
 * @brief   Checks cell faults and latches the BMS into FAULT (contactors open, shutdown circuit opened).
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.7.3
 */
#ifndef BMS_FAULTS_H
#define BMS_FAULTS_H

#include <Arduino.h>

// Numeric values go out in 0x311 byte 3; keep the order fixed.
enum FaultCode {
  FAULT_NONE = 0,
  FAULT_CELL_UV,
  FAULT_CELL_OV,
  FAULT_PACK_UV,
  FAULT_PACK_OV,
  FAULT_OVER_TEMP,
  FAULT_SPREAD,
  FAULT_PRECHARGE_TIMEOUT,
  FAULT_PRECHARGE_FAST,
  FAULT_PACK_VOLT_SENSE,
  FAULT_CHARGER_COMMS,
  FAULT_BQ_INIT,
  FAULT_BQ_COMMS,
  FAULT_CHARGE_TIMEOUT
};

extern FaultCode activeFaultCode;
extern float spreadLimitV;  // computed at init from DCIR and max pack current

FaultCode faultsCheckCells();
void faultsEnter(FaultCode code);
const char *faultsGetName(FaultCode code);

#endif  // BMS_FAULTS_H

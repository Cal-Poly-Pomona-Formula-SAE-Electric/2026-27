/**
 * @file    faults.cpp
 * @brief   Checks cell faults and latches the BMS into FAULT (contactors open, shutdown circuit opened).
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.7.3
 */
#include "faults.h"

#include <math.h>

#include "pins.h"
#include "safety_limits.h"
#include "cell_monitor.h"
#include "soc.h"
#include "state_machine.h"
#include "charger_can.h"

FaultCode activeFaultCode = FAULT_NONE;
float spreadLimitV = 0.81f;

// Channels 0..7 sit behind the mux that can misread; see FIRST_8_OVERTEMP_DELAY_MS.
static constexpr uint16_t FIRST_MUX_CHANNEL_COUNT = 8;

static const char *FAULT_NAMES[] = {
    "none", "cell_UV", "cell_OV", "pack_UV", "pack_OV",
    "over_temp", "spread", "precharge_timeout",
    "precharge_fast", "pack_volt_sense", "charger_comms", "BQ_init", "BQ_comms",
    "charge_timeout"};

const char *faultsGetName(FaultCode code) {
  return FAULT_NAMES[(int)code];
}

// ---------------- Cell checks ----------------
// EV.7.3.3: the AMS must open the shutdown circuit on cell voltage or
// temperature outside the allowed range.
FaultCode faultsCheckCells() {
  static bool isFirst8OverTemp = false;
  static uint32_t first8OverTempStartMs = 0;
  float packSumV = 0.0f;
  for (uint16_t i = 0; i < PACK_CELLS; i++) {
    if (isnan(cellVoltagesV[i])) continue;
    if (cellVoltagesV[i] < CELL_MIN_V) return FAULT_CELL_UV;
    if (cellVoltagesV[i] > CELL_MAX_V) return FAULT_CELL_OV;
    packSumV += cellVoltagesV[i];
  }
  if (packSumV < PACK_MIN_V) return FAULT_PACK_UV;
  if (bmsState == STATE_CHARGE_ACTIVE && packSumV > PACK_CHARGE_OV_V) return FAULT_PACK_OV;
  if (!isnan(cellVoltageStats.spread) && cellVoltageStats.spread > spreadLimitV) return FAULT_SPREAD;

  // Over-temperature on the first 8 channels must persist before it counts.
  bool isFirst8OverTempNow = false;
  for (uint16_t i = 0; i < PACK_CELLS; i++) {
    if (isnan(cellTempsC[i]) || cellTempsC[i] < CELL_TEMP_MAX_C) continue;
    if (i < FIRST_MUX_CHANNEL_COUNT) isFirst8OverTempNow = true;
    else return FAULT_OVER_TEMP;
  }
  if (isFirst8OverTempNow) {
    if (!isFirst8OverTemp) {
      isFirst8OverTemp = true;
      first8OverTempStartMs = millis();
    } else if ((uint32_t)(millis() - first8OverTempStartMs) >= FIRST_8_OVERTEMP_DELAY_MS) {
      return FAULT_OVER_TEMP;
    }
  } else {
    isFirst8OverTemp = false;
  }
  return FAULT_NONE;
}

// ---------------- Latch ----------------
// Latched until power cycle. Opens both contactor drives and the shutdown circuit.
void faultsEnter(FaultCode code) {
  // SAFETY: outputs go safe before any bookkeeping.
  digitalWrite(PIN_PRECHARGE_RELAY, LOW);
  digitalWrite(PIN_POS_AIR,         LOW);
  digitalWrite(PIN_BMS_STATUS,      LOW);  // EV.7.3.3: open the shutdown circuit
  areContactorsClosed = false;
  activeFaultCode = code;
  bmsState = STATE_FAULT;

  if (bmsMode == MODE_CHARGE) {
    chargerCanSendFaultStop();
  }

  socSave();

  Serial.printf("[FAULT] %s — power cycle to reset\n", faultsGetName(code));
}

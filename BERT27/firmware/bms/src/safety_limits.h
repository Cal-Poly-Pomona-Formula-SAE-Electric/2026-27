/**
 * @file    safety_limits.h
 * @brief   Cell and pack limits, precharge timing, pack voltage cross-check and charger timeouts. Changes need a second reviewer.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.5.6.2, EV.7.3
 *
 * Not calibration: these values are not changed after tech inspection (IN.16.2.3).
 */
#ifndef BMS_SAFETY_LIMITS_H
#define BMS_SAFETY_LIMITS_H

#include <Arduino.h>
#include "pack_config.h"

// ---------------- Cell limits ----------------
static constexpr float CELL_MIN_V      = 2.50f;  // per cell, under-voltage cutoff
static constexpr float CELL_MAX_V      = 4.20f;  // per cell, over-voltage cutoff (both modes)
static constexpr float CELL_TEMP_MAX_C = 60.0f;  // discharge and charge
// Preserve the original 402 V / 96 channel pack fault point of 4.1875 V per cell.
static constexpr float CELL_PACK_OV_V  = 402.0f / 96.0f;

// ---------------- Pack limits ----------------
static constexpr float PACK_MIN_V       = CELLS_SERIES * CELL_MIN_V;      // 237.5 V
static constexpr float PACK_CHARGE_OV_V = CELLS_SERIES * CELL_PACK_OV_V;  // 397.813 V; below 95 x 4.20 V

// ---------------- Over-temperature debounce ----------------
// The first 8 channels sit behind a mux that can read wrong for a while, so
// an over-temperature on them must persist this long before it faults.
static constexpr uint32_t FIRST_8_OVERTEMP_DELAY_MS = 30000;

// ---------------- Precharge ----------------
// EV.5.6.2: RC = 528 ms, t90 = 1216 ms, max(2 x 1216, 1216 + 500) = 2432 ms.
static constexpr uint32_t PRECHARGE_TIMEOUT_MS   = 10000;
static constexpr uint32_t PRECHARGE_MIN_MS       = 200;    // discharge only; faster means a bypassed resistor
static constexpr float    PRECHARGE_DONE_RATIO    = 0.90f;  // inverter side at 90 % of battery side
static constexpr float    PRECHARGE_MIN_BATTERY_V = 10.0f;  // below this no battery is seen
// Pack voltage cross-check (IMD vs. isolation amp) is held off this long after precharge.
static constexpr uint32_t PACK_VOLT_CHECK_HOLDOFF_MS = 4000;

// ---------------- Pack voltage sense cross-check ----------------
static constexpr float   IMD_DISAGREE_ABS_V    = 100.0f;  // absolute threshold
static constexpr float   IMD_DISAGREE_FRACTION = 0.40f;   // of the isolation amp reading
static constexpr uint8_t IMD_DISAGREE_MAX_COUNT = 5;      // consecutive hits before faulting

// ---------------- Charger ----------------
static constexpr uint32_t CHARGER_TIMEOUT_MS = 5000;     // no charger frame for this long faults
// Max CV taper time; safety cutoff if current never falls below CHARGER_DONE_A.
static constexpr uint32_t CV_TIMEOUT_MS      = 5400000;  // 90 min

// ---------------- BQ79616 start-up ----------------
static constexpr uint8_t  BQ_INIT_MAX_ATTEMPTS  = 10;
static constexpr uint32_t BQ_INIT_RETRY_WAIT_MS = 200;

#endif  // BMS_SAFETY_LIMITS_H

/**
 * @file    pack_config.h
 * @brief   Pack layout (Molicel P50B 95s4p plus one bypassed channel), capacity, DCIR and charge targets.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * Rule and safety limits are in safety_limits.h, not here.
 */
#ifndef BMS_PACK_CONFIG_H
#define BMS_PACK_CONFIG_H

#include <Arduino.h>

// ---------------- Cell layout ----------------
static constexpr uint8_t CELLS_SERIES        = 95;
static constexpr uint8_t CELLS_PARALLEL      = 4;
static constexpr uint8_t CELL_CHANNELS       = 96;   // BQ/CAN layout remains 6 x 16
// Zero-based channel index of the bypassed busbar position: pack cell 92 = IC6 Cell12.
static constexpr uint8_t BYPASSED_CELL_INDEX = 91;

// ---------------- Capacity and resistance ----------------
static constexpr float PACK_CAPACITY_AH = 20.0f;   // 4 x 5 Ah
static constexpr float CELL_NOMINAL_V   = 3.6f;
static constexpr float PACK_NOMINAL_V   = CELLS_SERIES * CELL_NOMINAL_V;  // 342.0 V
// Spread limit = DCIR x I_cell_max = 0.014 ohm x (80 kW / 342 V / 4P) = ~0.82 V
static constexpr float CELL_DCIR_OHM    = 0.014f;
static constexpr float PACK_POWER_MAX_W = 80000.0f;

// ---------------- Charge targets ----------------
// Preserve the original 400 V / 96 channel target of 4.1667 V per cell.
static constexpr float CELL_CHARGE_TARGET_V = 400.0f / 96.0f;
static constexpr float PACK_CHARGE_MAX_V    = CELLS_SERIES * CELL_CHARGE_TARGET_V;  // 395.833 V

// ---------------- Charger (HK-MF OBC, Elcon protocol) ----------------
static constexpr float CHARGER_REQUEST_V = PACK_CHARGE_MAX_V;  // 395.8 V requested (0.1 V protocol resolution)
static constexpr float CHARGER_REQUEST_A = 10.0f;              // max for the 312 V platform
static constexpr float CHARGER_DONE_A    = 0.5f;               // current-taper termination
// The charger is treated as being on its CV plateau once its output voltage
// is within this margin of the pack charge target.
static constexpr float CHARGER_CV_PLATEAU_MARGIN_V = 5.0f;

#endif  // BMS_PACK_CONFIG_H

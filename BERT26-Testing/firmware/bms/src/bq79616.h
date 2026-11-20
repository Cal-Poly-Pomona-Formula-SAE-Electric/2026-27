/**
 * @file    bq79616.h
 * @brief   TI BQ79600/BQ79616 stack driver: UART frames, CRC, wake, auto-address, ADC and GPIO setup, cell voltage and NTC temperature reads.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * Talks to the BQ79600 bridge over Serial1 at 1 Mbaud. The stack is six
 * BQ79616 monitors, 16 channels each.
 */
#ifndef BMS_BQ79616_H
#define BMS_BQ79616_H

#include <Arduino.h>
#include "pack_config.h"

// ---------------- Stack layout ----------------
static const uint8_t  BQ_STACK_DEVICE_COUNT = 6;
static const uint8_t  BQ_CELLS_PER_DEVICE   = 16;
static const uint16_t PACK_CELLS = (uint16_t)BQ_STACK_DEVICE_COUNT * BQ_CELLS_PER_DEVICE;  // 96
static_assert(PACK_CELLS == CELL_CHANNELS, "BQ channel count must match the CAN cell layout");
static_assert(BYPASSED_CELL_INDEX < PACK_CELLS, "Bypassed cell index is outside the BQ channel array");
static constexpr uint8_t BYPASSED_BQ_DEVICE = (BYPASSED_CELL_INDEX / BQ_CELLS_PER_DEVICE) + 1u;  // ID 6
static constexpr uint8_t BYPASSED_BQ_CELL   = (BYPASSED_CELL_INDEX % BQ_CELLS_PER_DEVICE) + 1u;  // Cell 12

bool bqInit();
bool bqInitWithRetries();
void bqHardwareReset();
void bqAdcInit();
bool bqReadVoltages(uint8_t deviceId, float voltagesV[BQ_CELLS_PER_DEVICE]);
bool bqReadTemps(uint8_t deviceId, float tempsC[BQ_CELLS_PER_DEVICE]);

#endif  // BMS_BQ79616_H

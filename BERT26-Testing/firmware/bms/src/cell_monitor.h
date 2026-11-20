/**
 * @file    cell_monitor.h
 * @brief   Reads every cell voltage and temperature, masks the bypassed channel and keeps min/max/avg statistics.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_CELL_MONITOR_H
#define BMS_CELL_MONITOR_H

#include <Arduino.h>
#include "bq79616.h"

struct CellStats {
  float minValue;
  float maxValue;
  float spread;
  float average;
  float stdDev;
};

// Voltages are read every loop; temperatures every TEMP_INTERVAL_MS.
static constexpr uint32_t TEMP_INTERVAL_MS = 10000;

extern float cellVoltagesV[PACK_CELLS];
extern float cellTempsC[PACK_CELLS];
extern CellStats cellVoltageStats;
extern CellStats cellTempStats;
extern elapsedMillis tempReadTimerMs;

void cellMonitorReadInitialVoltages();
void cellMonitorMaskBypassedVoltage();
uint16_t cellMonitorCalcStats(const float *values, uint16_t count, CellStats &stats);
void cellMonitorService();

#endif  // BMS_CELL_MONITOR_H

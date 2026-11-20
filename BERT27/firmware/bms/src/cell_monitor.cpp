/**
 * @file    cell_monitor.cpp
 * @brief   Reads every cell voltage and temperature, masks the bypassed channel and keeps min/max/avg statistics.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "cell_monitor.h"

#include <math.h>

#include "pack_config.h"
#include "current_sensor.h"
#include "soc.h"
#include "faults.h"

static void cellMonitorPrintCells();

float cellVoltagesV[PACK_CELLS];
float cellTempsC[PACK_CELLS];
static float bypassDropV = NAN;  // raw VC12 measurement retained for diagnostics
CellStats cellVoltageStats;
CellStats cellTempStats;
elapsedMillis tempReadTimerMs;

// Module 1 is the last device touched by the voltage read burst. Its ADC needs
// this long to leave cell-voltage mode before the GPIO/NTC mux reads start,
// otherwise the first cell reads ~45 C from residual charge.
static constexpr uint32_t TEMP_READ_SETTLE_MS = 100;

// Baseline voltage read for OCV-based SOC. bqInit does not read voltages, so
// this fills the array once before the INIT window.
void cellMonitorReadInitialVoltages() {
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++)
    bqReadVoltages(stackIndex + 1, cellVoltagesV + stackIndex * BQ_CELLS_PER_DEVICE);
  cellMonitorMaskBypassedVoltage();
}

void cellMonitorMaskBypassedVoltage() {
  // Preserve the busbar measurement for diagnostics, then remove it from all
  // cell statistics, UV/OV checks, pack sums, SOC initialization, and CAN data.
  bypassDropV = cellVoltagesV[BYPASSED_CELL_INDEX];
  cellVoltagesV[BYPASSED_CELL_INDEX] = NAN;
}

// Returns the number of valid (non-NaN) values. NaN entries are skipped.
uint16_t cellMonitorCalcStats(const float *values, uint16_t count, CellStats &stats) {
  stats = {NAN, NAN, NAN, NAN, NAN};
  uint16_t validCount = 0; double sum = 0;
  for (uint16_t i = 0; i < count; i++) {
    if (isnan(values[i])) continue;
    if (validCount == 0 || values[i] < stats.minValue) stats.minValue = values[i];
    if (validCount == 0 || values[i] > stats.maxValue) stats.maxValue = values[i];
    sum += values[i]; validCount++;
  }
  if (!validCount) return 0;
  stats.average = (float)(sum / validCount);
  stats.spread = stats.maxValue - stats.minValue;
  double sumSquares = 0;
  for (uint16_t i = 0; i < count; i++) {
    if (isnan(values[i])) continue;
    double delta = values[i] - stats.average; sumSquares += delta * delta;
  }
  stats.stdDev = (float)sqrt(sumSquares / validCount);
  return validCount;
}

static void cellMonitorPrintCells() {
  Serial.printf("[CELLS] SOC=%.1f%%  PackCurrent=%.1fA\n", socPct, packCurrentA);
  Serial.printf("[CELLS] Volt  min=%.3fV  max=%.3fV  avg=%.3fV  spread=%.3fV\n",
                cellVoltageStats.minValue, cellVoltageStats.maxValue, cellVoltageStats.average, cellVoltageStats.spread);
  Serial.printf("[CELLS] Busbar index %u (IC%u Cell%u) drop=%.4fV\n",
                BYPASSED_CELL_INDEX, BYPASSED_BQ_DEVICE,
                BYPASSED_BQ_CELL, bypassDropV);
  Serial.printf("[CELLS] Temp  min=%.1fC  max=%.1fC  avg=%.1fC\n",
                cellTempStats.minValue, cellTempStats.maxValue, cellTempStats.average);
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    Serial.printf("[CELLS] IC%u V:", stackIndex + 1);
    for (uint8_t cell = 0; cell < BQ_CELLS_PER_DEVICE; cell++)
      Serial.printf(" %.3f", cellVoltagesV[stackIndex * BQ_CELLS_PER_DEVICE + cell]);
    Serial.println();
    Serial.printf("[CELLS] IC%u T:", stackIndex + 1);
    for (uint8_t cell = 0; cell < BQ_CELLS_PER_DEVICE; cell++) {
      if (isnan(cellTempsC[stackIndex * BQ_CELLS_PER_DEVICE + cell])) Serial.print(" ---");
      else Serial.printf(" %.1f", cellTempsC[stackIndex * BQ_CELLS_PER_DEVICE + cell]);
    }
    Serial.println();
  }
}

// Voltages every call, temperatures every TEMP_INTERVAL_MS.
void cellMonitorService() {
  bool isAllVoltagesOk = true;
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    if (!bqReadVoltages(stackIndex + 1, cellVoltagesV + stackIndex * BQ_CELLS_PER_DEVICE)) {
      Serial.printf("[BQ] voltage read failed: id%d\n", stackIndex + 1);
      isAllVoltagesOk = false;
    }
  }
  if (!isAllVoltagesOk) { faultsEnter(FAULT_BQ_COMMS); return; }
  cellMonitorMaskBypassedVoltage();
  uint16_t validVoltageCount = cellMonitorCalcStats(cellVoltagesV, PACK_CELLS, cellVoltageStats);
  if (validVoltageCount != CELLS_SERIES) {
    Serial.printf("[BQ] expected %u valid cell voltages, got %u\n", CELLS_SERIES, validVoltageCount);
    faultsEnter(FAULT_BQ_COMMS);
    return;
  }

  if (tempReadTimerMs >= TEMP_INTERVAL_MS) {
    tempReadTimerMs = 0;
    bool isAllTempsOk = true;
    delay(TEMP_READ_SETTLE_MS);
    for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++)
      if (!bqReadTemps(stackIndex + 1, cellTempsC + stackIndex * BQ_CELLS_PER_DEVICE)) isAllTempsOk = false;
    if (!isAllTempsOk) { faultsEnter(FAULT_BQ_COMMS); return; }
    cellMonitorCalcStats(cellTempsC, PACK_CELLS, cellTempStats);
    bqAdcInit();  // back to cell-voltage mode after the GPIO reads
    cellMonitorPrintCells();
  }
}

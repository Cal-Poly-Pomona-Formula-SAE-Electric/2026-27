/**
 * @file    soc.cpp
 * @brief   State of charge: OCV lookup at start-up, coulomb counting, EEPROM save and load.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "soc.h"

#include <EEPROM.h>
#include <math.h>

#include "pack_config.h"
#include "eeprom_layout.h"
#include "cell_monitor.h"
#include "current_sensor.h"

static float socFromOcv(float cellV);

static constexpr float SOC_DEFAULT_PCT = 50.0f;  // used when no EEPROM value and no cell reading
static constexpr float SECONDS_PER_HOUR = 3600.0f;

float socPct = SOC_DEFAULT_PCT;

// ---------------- Open-circuit voltage lookup ----------------
// Piecewise-linear OCV curve for the Molicel P50B, per cell.
static float socFromOcv(float cellV) {
  static const float OCV_V[]   = {2.50f, 3.00f, 3.25f, 3.40f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float OCV_SOC_PCT[] = { 0.0f,  5.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f};
  static const int OCV_POINT_COUNT = 12;
  if (cellV <= OCV_V[0]) return 0.0f;
  if (cellV >= OCV_V[OCV_POINT_COUNT - 1]) return 100.0f;
  for (int i = 0; i < OCV_POINT_COUNT - 1; i++) {
    if (cellV <= OCV_V[i + 1]) {
      float fraction = (cellV - OCV_V[i]) / (OCV_V[i + 1] - OCV_V[i]);
      return OCV_SOC_PCT[i] + fraction * (OCV_SOC_PCT[i + 1] - OCV_SOC_PCT[i]);
    }
  }
  return 100.0f;
}

// ---------------- Start-up, save, set ----------------
void socInit() {
  float storedPct = 0.0f;
  EEPROM.get(EEPROM_SOC_ADDR, storedPct);
  if (!isnan(storedPct) && storedPct >= 0.0f && storedPct <= 100.0f) {
    socPct = storedPct;
    Serial.printf("[SOC] Loaded from EEPROM: %.1f%%\n", socPct);
  } else {
    // Fall back to an OCV estimate from the average cell voltage.
    uint16_t validCount = cellMonitorCalcStats(cellVoltagesV, PACK_CELLS, cellVoltageStats);
    socPct = (validCount > 0) ? socFromOcv(cellVoltageStats.average) : SOC_DEFAULT_PCT;
    Serial.printf("[SOC] OCV estimate: %.1f%%\n", socPct);
  }
}

void socSave() {
  EEPROM.put(EEPROM_SOC_ADDR, socPct);
}

void socSet(float newSocPct) {
  socPct = fmaxf(0.0f, fminf(100.0f, newSocPct));
  socSave();
  Serial.printf("[SOC] Manually set to %.1f%%\n", socPct);
}

// ---------------- Coulomb counting ----------------
void socUpdate(float elapsedS) {
  // Positive current = discharge (SOC drops), negative = charge/regen (SOC rises)
  socPct -= (packCurrentA * elapsedS) / (SECONDS_PER_HOUR * PACK_CAPACITY_AH) * 100.0f;
  socPct = fmaxf(0.0f, fminf(100.0f, socPct));
}

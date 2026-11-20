/**
 * @file    main.cpp
 * @brief   Bench sketch: reads cell voltages (every loop) and NTC temperatures
 *          (every 10 s) from a BQ79600 + N x BQ79616 stack and prints pack stats.
 *
 * Board:   BMS (Teensy 4.1). The original was written for a Teensy 4.0; the
 *          pins it uses (Serial1 on pin 1, LED on 13) are the same on both.
 * Original Creator: BERT26 software (bq79616_parametricFinal_LimitedPolling.ino)
 * Worked on by: BERT27 restructure
 *
 * Change the stack size with BQ_STACK_DEVICE_COUNT in bq79616.h.
 *
 * Output:
 *   [vcell:idX] V: <C1..C16>
 *   [temp:idX]  T: <T1..T16>
 *   [pack:volt] min max spread avg std
 *   [pack:temp] min max spread avg std
 *   [timing] loop=Xms
 */
#include <Arduino.h>
#include <math.h>

#include "bq79616.h"

struct PackStats { float minValue, maxValue, spread, average, stdDev; };

static const uint32_t TEMP_INTERVAL_MS = 10000;  // temperature read period
// Each loop waits this long before polling, which limits how often the stack is read.
static const uint32_t LOOP_DELAY_MS    = 5000;
static const uint32_t USB_WAIT_MS      = 8000;   // give a USB monitor time to attach, then run without one

static const int PIN_LED = 13;

// Most recent readings, kept between loops. [device * 16 + cell]
static float cellVoltagesV[PACK_CELLS];
static float cellTempsC[PACK_CELLS];  // NAN = not yet read

// ---------------- Stats ----------------
// Min, max, spread, mean and standard deviation over the readings that are not NAN.
// Returns how many readings were used.
static uint16_t packCalcStats(const float *values, uint16_t length, PackStats &stats) {
  stats = {NAN, NAN, NAN, NAN, NAN};
  uint16_t validCount = 0;
  double sum = 0;
  for (uint16_t i = 0; i < length; i++) {
    if (isnan(values[i])) continue;
    if (validCount == 0 || values[i] < stats.minValue) stats.minValue = values[i];
    if (validCount == 0 || values[i] > stats.maxValue) stats.maxValue = values[i];
    sum += values[i];
    validCount++;
  }
  if (!validCount) return 0;
  stats.average = (float)(sum / validCount);
  stats.spread = stats.maxValue - stats.minValue;
  double sumSquares = 0;
  for (uint16_t i = 0; i < length; i++) {
    if (isnan(values[i])) continue;
    double deviation = values[i] - stats.average;
    sumSquares += deviation * deviation;
  }
  stats.stdDev = (float)sqrt(sumSquares / validCount);
  return validCount;
}

// ---------------- Printing ----------------
static void packPrintVoltages(uint8_t deviceId, const float voltagesV[BQ_CELLS_PER_DEVICE]) {
  Serial.printf("[vcell:id%d] V:", deviceId);
  for (int i = 0; i < BQ_CELLS_PER_DEVICE; i++) Serial.printf(" %.5f", voltagesV[i]);
  Serial.println();
}

static void packPrintTemps(uint8_t deviceId, const float tempsC[BQ_CELLS_PER_DEVICE]) {
  Serial.printf("[temp:id%d]  T:", deviceId);
  for (int i = 0; i < BQ_CELLS_PER_DEVICE; i++) {
    if (isnan(tempsC[i])) Serial.print(" ---");
    else                  Serial.printf(" %.2f", tempsC[i]);
  }
  Serial.println();
}

static void packPrintStats() {
  PackStats voltageStats, tempStats;
  uint16_t validVoltageCount = packCalcStats(cellVoltagesV, PACK_CELLS, voltageStats);
  uint16_t validTempCount    = packCalcStats(cellTempsC, PACK_CELLS, tempStats);
  Serial.println("--- Pack ---");
  if (validVoltageCount)
    Serial.printf("[pack:volt] min=%.5f max=%.5f spread=%.5f avg=%.5f std=%.5f V (%u)\n",
                  voltageStats.minValue, voltageStats.maxValue, voltageStats.spread,
                  voltageStats.average, voltageStats.stdDev, validVoltageCount);
  if (validTempCount)
    Serial.printf("[pack:temp] min=%.2f max=%.2f spread=%.2f avg=%.2f std=%.2f C (%u)\n",
                  tempStats.minValue, tempStats.maxValue, tempStats.spread,
                  tempStats.average, tempStats.stdDev, validTempCount);
}

// ---------------- Setup ----------------
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  uint32_t startMs = millis();
  while (!Serial && millis() - startMs < USB_WAIT_MS) {}
  Serial.printf("\n[boot] BQ79600 + %d x BQ79616  (%u cells)\n",
                BQ_STACK_DEVICE_COUNT, PACK_CELLS);
  bqInit();

  // NAN so the stats skip temperatures until the first real read.
  for (uint16_t i = 0; i < PACK_CELLS; i++) cellTempsC[i] = NAN;

  Serial.println("[init] done");
}

// ---------------- Main loop ----------------
void loop() {
  static uint32_t lastTempReadMs = 0;
  uint32_t nowMs = millis();
  uint32_t loopStartMs = nowMs;
  delay(LOOP_DELAY_MS);

  // Voltages: every iteration.
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    uint8_t deviceId = stackIndex + 1;
    float *deviceVoltagesV = cellVoltagesV + stackIndex * BQ_CELLS_PER_DEVICE;
    if (bqReadVoltages(deviceId, deviceVoltagesV)) {
      packPrintVoltages(deviceId, deviceVoltagesV);
    } else {
      Serial.printf("[vcell:id%d] FAIL\n", deviceId);
    }
  }

  // Temperatures: every TEMP_INTERVAL_MS.
  if (nowMs - lastTempReadMs >= TEMP_INTERVAL_MS) {
    lastTempReadMs = nowMs;
    for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
      uint8_t deviceId = stackIndex + 1;
      float *deviceTempsC = cellTempsC + stackIndex * BQ_CELLS_PER_DEVICE;
      bqReadTemps(deviceId, deviceTempsC);
      packPrintTemps(deviceId, deviceTempsC);
    }
    // The mux sweep rewrites the GPIO config, so re-arm the ADC for normal
    // voltage reading.
    bqAdcInit();
  }

  // Pack stats: every iteration, from the latest data.
  packPrintStats();

  Serial.printf("[timing] loop=%lums\n", millis() - loopStartMs);
}

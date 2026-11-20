/**
 * @file    current_sensor.cpp
 * @brief   Reads pack current from the LEM DHAB S/124 and stores its two-point calibration in EEPROM.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "current_sensor.h"

#include <EEPROM.h>
#include <math.h>

#include "pins.h"
#include "calibration.h"
#include "eeprom_layout.h"

static float currentSensorReadRawA();
static void currentSensorSaveCalibration();

float packCurrentA = 0.0f;

// Two-point linear fit applied to the raw sensor reading.
static float currentCalGain    = 1.0f;
static float currentCalOffsetA = 0.0f;
static float calPoint1RawA = 0.0f, calPoint1ActualA = 0.0f;
static float calPoint2RawA = 0.0f, calPoint2ActualA = 0.0f;
static bool  isCalPoint1Set = false, isCalPoint2Set = false;

// Gains outside this range mean the EEPROM holds garbage, not a calibration.
static constexpr float CAL_GAIN_MIN = 0.01f;
static constexpr float CAL_GAIN_MAX = 100.0f;

// ---------------- Reading ----------------
static float currentSensorReadRawA() {
  float adcV    = analogRead(PIN_PACK_CURRENT_SENSE) * (ADC_FULL_SCALE_V / ADC_MAX_COUNT);
  float sensorV = adcV / CURRENT_SENSOR_DIVIDER_GAIN;
  return (sensorV - CURRENT_SENSOR_ZERO_V) / CURRENT_SENSOR_VOLTS_PER_AMP;
}

float currentSensorReadA() {
  return currentCalGain * currentSensorReadRawA() + currentCalOffsetA;
}

// ---------------- Calibration ----------------
static void currentSensorSaveCalibration() {
  EEPROM.put(EEPROM_CAL_GAIN_ADDR, currentCalGain);
  EEPROM.put(EEPROM_CAL_OFFSET_ADDR, currentCalOffsetA);
}

void currentSensorLoadCalibration() {
  float storedGain = 0.0f, storedOffsetA = 0.0f;
  EEPROM.get(EEPROM_CAL_GAIN_ADDR, storedGain);
  EEPROM.get(EEPROM_CAL_OFFSET_ADDR, storedOffsetA);
  if (!isnan(storedGain) && !isnan(storedOffsetA) && storedGain > CAL_GAIN_MIN && storedGain < CAL_GAIN_MAX) {
    currentCalGain = storedGain; currentCalOffsetA = storedOffsetA;
    Serial.printf("[CAL] Loaded: gain=%.4f  offset=%.4fA\n", currentCalGain, currentCalOffsetA);
  } else {
    Serial.println("[CAL] No valid calibration in EEPROM, using defaults (gain=1 offset=0)");
  }
}

// Apply a known current, then send its true value as point 1; repeat with a
// different current as point 2. Once both are set the fit is applied and saved.
void currentSensorRecordCalPoint(uint8_t pointNumber, float actualA) {
  float rawA = currentSensorReadRawA();
  if (pointNumber == 1) {
    calPoint1RawA = rawA; calPoint1ActualA = actualA; isCalPoint1Set = true;
    Serial.printf("[CAL] Point 1: raw=%.3fA  actual=%.3fA\n", rawA, actualA);
  } else {
    calPoint2RawA = rawA; calPoint2ActualA = actualA; isCalPoint2Set = true;
    Serial.printf("[CAL] Point 2: raw=%.3fA  actual=%.3fA\n", rawA, actualA);
  }
  if (isCalPoint1Set && isCalPoint2Set) {
    float rawSpanA = calPoint2RawA - calPoint1RawA;
    if (fabsf(rawSpanA) < CURRENT_CAL_MIN_POINT_SPACING_A) {
      Serial.println("[CAL] ERROR: cal points too close — apply different currents and retry");
      return;
    }
    currentCalGain    = (calPoint2ActualA - calPoint1ActualA) / rawSpanA;
    currentCalOffsetA = calPoint1ActualA - currentCalGain * calPoint1RawA;
    currentSensorSaveCalibration();
    Serial.printf("[CAL] Applied: gain=%.4f  offset=%.4fA\n", currentCalGain, currentCalOffsetA);
  }
}

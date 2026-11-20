/**
 * @file    pack_voltage.cpp
 * @brief   Reads the battery-side pack voltage from the isolation amplifier.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "pack_voltage.h"

#include "pins.h"
#include "calibration.h"

static float packVoltageAdcToVolts(int adcCounts);

static float packVoltageAdcToVolts(int adcCounts) {
  return ((float)adcCounts - PACK_VOLTAGE_ADC_OFFSET_COUNTS) / PACK_VOLTAGE_ADC_COUNTS_PER_VOLT;
}

// Average of PACK_VOLTAGE_SAMPLES raw readings, then converted.
float packVoltageReadV() {
  long sum = 0;
  for (int i = 0; i < PACK_VOLTAGE_SAMPLES; i++) sum += analogRead(PIN_PACK_VOLTAGE_SENSE);
  return packVoltageAdcToVolts((int)(sum / PACK_VOLTAGE_SAMPLES));
}

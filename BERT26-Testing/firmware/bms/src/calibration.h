/**
 * @file    calibration.h
 * @brief   Sensor conversion values measured on the car: isolation amp fit, current sensor chain, ADC setup.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * Calibration changes are allowed after tech inspection (IN.16.2.3).
 * Rule and safety limits are in safety_limits.h, not here.
 */
#ifndef BMS_CALIBRATION_H
#define BMS_CALIBRATION_H

#include <Arduino.h>

// ---------------- ADC ----------------
static constexpr int   ADC_RESOLUTION_BITS = 12;
static constexpr float ADC_MAX_COUNT       = 4095.0f;
static constexpr float ADC_FULL_SCALE_V    = 3.3f;

// ---------------- Pack voltage (battery-side isolation amp) ----------------
// Linear fit of the amp output: V = (ADC - 1.83) / 5.2957
static constexpr float PACK_VOLTAGE_ADC_OFFSET_COUNTS   = 1.83f;
static constexpr float PACK_VOLTAGE_ADC_COUNTS_PER_VOLT = 5.2957f;
static constexpr int   PACK_VOLTAGE_SAMPLES             = 16;  // averaged per reading

// ---------------- Pack current (LEM DHAB S/124, channel 2, +/-500 A) ----------------
//   Uout = S x IP + UO  (S = 4 mV/A, UO = 2.5 V at UC = 5 V)
//   Voltage divider gain = 0.6875 before the Teensy pin.
static constexpr float CURRENT_SENSOR_DIVIDER_GAIN     = 0.6875f;
static constexpr float CURRENT_SENSOR_ZERO_V           = 2.5f;
static constexpr float CURRENT_SENSOR_VOLTS_PER_AMP    = 0.004f;
// Two calibration points closer than this cannot give a usable gain.
static constexpr float CURRENT_CAL_MIN_POINT_SPACING_A = 0.5f;

#endif  // BMS_CALIBRATION_H

/**
 * @file    pedals.h
 * @brief   Reads the accelerator and brake pedals and runs the APPS plausibility and BSPC checks.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_PEDALS_H
#define VCU_PEDALS_H

#include <Arduino.h>

static const int ADC_RESOLUTION_BITS = 12;

extern float apps1Pct;
extern float apps2Pct;
extern float pedalPct;
extern float desiredTorqueNm;
extern bool isAppsPlausible;
extern const char *appsFaultText;
extern bool isBspcActive;
extern int apps1RawLast;
extern int apps2RawLast;
extern int bse1RawLast;
extern int bse2RawLast;
extern float brakePressurePct;
extern bool isBraking;
extern bool isHardBraking;

void pedalsUpdate();
void pedalsLogAppsTelemetry();
void pedalsUpdateBrakePressureSensors();

#endif  // VCU_PEDALS_H

/**
 * @file    fans.h
 * @brief   Sets the cooling fan PWM from motor/inverter temperature.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_FANS_H
#define VCU_FANS_H

#include <Arduino.h>

// Fan duty in percent. The serial console can set these by hand
// (isFanManualOverride = true) instead of the temperature curve.
extern int fan1DutyPct;
extern int fan2DutyPct;
extern bool isFanManualOverride;

void fansSetup();
void fansUpdateDutyFromTemps();
void fansService();

#endif  // VCU_FANS_H

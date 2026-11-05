/**
 * @file    fault_text.h
 * @brief   Builds the fault names shown on the dashboard from the TSSI, BMS and PM100 fault bits.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_FAULT_TEXT_H
#define VCU_FAULT_TEXT_H

#include <Arduino.h>

const char *faultTextGetForDashboard();
const char *faultTextGetFirstTssi(uint16_t bits);
const char *faultTextGetFirstPm100(uint32_t postFaults, uint32_t runFaults);
const char *faultTextGetBmsTemperatureCode(uint8_t faultCode);
const char *faultTextGetPm100Post(uint8_t bit);
const char *faultTextGetPm100Run(uint8_t bit);

#endif  // VCU_FAULT_TEXT_H

/**
 * @file    pack_voltage.h
 * @brief   Reads the battery-side pack voltage from the isolation amplifier.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_PACK_VOLTAGE_H
#define BMS_PACK_VOLTAGE_H

#include <Arduino.h>

float packVoltageReadV();

#endif  // BMS_PACK_VOLTAGE_H

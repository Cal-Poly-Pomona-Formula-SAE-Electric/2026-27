/**
 * @file    soc.h
 * @brief   State of charge: OCV lookup at start-up, coulomb counting, EEPROM save and load.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_SOC_H
#define BMS_SOC_H

#include <Arduino.h>

extern float socPct;

void socInit();
void socSave();
void socSet(float newSocPct);
void socUpdate(float elapsedS);

#endif  // BMS_SOC_H

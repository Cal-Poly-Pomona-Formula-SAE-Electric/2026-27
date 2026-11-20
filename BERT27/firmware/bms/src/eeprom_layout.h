/**
 * @file    eeprom_layout.h
 * @brief   Where each saved value lives in the Teensy EEPROM, so modules never overlap.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_EEPROM_LAYOUT_H
#define BMS_EEPROM_LAYOUT_H

// Each entry is a 4-byte float.
static constexpr int EEPROM_SOC_ADDR        = 0;  // state of charge, %
static constexpr int EEPROM_CAL_GAIN_ADDR   = 4;  // current sensor gain
static constexpr int EEPROM_CAL_OFFSET_ADDR = 8;  // current sensor offset, A

#endif  // BMS_EEPROM_LAYOUT_H

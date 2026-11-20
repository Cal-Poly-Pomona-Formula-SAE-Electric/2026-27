/**
 * @file    cell_dump.h
 * @brief   Streams every cell voltage or temperature over CAN in 0x313 frames when the VCU asks.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_CELL_DUMP_H
#define BMS_CELL_DUMP_H

#include <Arduino.h>

// Request byte 0 of 0x312.
static constexpr uint8_t CELL_DUMP_IDLE         = 0;
static constexpr uint8_t CELL_DUMP_VOLTAGES     = 1;
static constexpr uint8_t CELL_DUMP_TEMPERATURES = 2;

void cellDumpStart(uint8_t dumpType, bool isOnChargerCan);
void cellDumpService();

#endif  // BMS_CELL_DUMP_H

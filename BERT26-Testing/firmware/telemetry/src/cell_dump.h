/**
 * @file    cell_dump.h
 * @brief   Asks the BMS for every cell voltage or temperature (0x312) and stores the 0x313 replies.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef TELEMETRY_CELL_DUMP_H
#define TELEMETRY_CELL_DUMP_H

#include <Arduino.h>

static constexpr int PACK_CELL_COUNT = 96;  // 6 ICs × 16 cells

// Request byte 0 of 0x312; same values as the BMS cell_dump module.
static constexpr uint8_t CELL_DUMP_IDLE         = 0;
static constexpr uint8_t CELL_DUMP_VOLTAGES     = 1;
static constexpr uint8_t CELL_DUMP_TEMPERATURES = 2;

// Marks a cell not received yet (or invalid). The page shows these as "invalid" / "no sensor".
static constexpr uint16_t CELL_VOLTAGE_NOT_RECEIVED_MV = 0xFFFF;
static constexpr int8_t   CELL_TEMP_NOT_RECEIVED_C     = INT8_MIN;

extern uint16_t cellVoltagesMv[PACK_CELL_COUNT];
extern int8_t   cellTempsC[PACK_CELL_COUNT];
extern bool     areCellVoltagesValid;  // set once a full voltage dump has arrived
extern bool     areCellTempsValid;     // set once a full temperature dump has arrived

void cellDumpReset();
void cellDumpService();
void cellDumpRequest(uint8_t dumpType);
void cellDumpHandleFrame(uint8_t* frameData, int frameLength);

#endif  // TELEMETRY_CELL_DUMP_H

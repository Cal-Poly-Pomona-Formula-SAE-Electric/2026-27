/**
 * @file    bms_link.h
 * @brief   BMS status and temperature frames, cell data requests, BMS state.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_BMS_LINK_H
#define VCU_BMS_LINK_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint32_t BMS_CELL_REQUEST_2026_ID  = 0x312;
static constexpr int16_t CELL_TEMP_UNKNOWN_C = -1000;

extern bool isRegenBmsPairValid;
extern float packVoltageV;
extern float packCurrentA;
extern uint8_t stateOfChargePct;
extern uint8_t batteryTempC;
extern uint16_t minCellVoltageMv;
extern uint16_t maxCellVoltageMv;
extern uint16_t avgCellVoltageMv;
extern bool hasCellVoltageStats;
extern int16_t minCellTempC;
extern int16_t maxCellTempC;
extern int16_t avgCellTempC;
extern bool hasCellTempStats;
extern bool isBmsActive;
extern bool hasBmsFault;
extern bool isBmsTimedOut;
extern bool hasBmsStatus;
extern uint8_t bmsFlags;
extern uint8_t bms2026FaultCode;
extern bool hasBms2026FaultCode;
extern uint32_t lastBmsRxMs;
extern bool hasBmsCurrent;
extern uint32_t lastBmsCellVoltageRxMs;
extern uint32_t lastBmsCurrentRxMs;

void bmsServiceCellDataRequests(uint32_t nowMs);
bool bmsAreContactorsOpen();
void bmsHandleStatusFrame(const CAN_message_t &frame);
void bmsHandleTempFrame(const CAN_message_t &frame);

#endif  // VCU_BMS_LINK_H

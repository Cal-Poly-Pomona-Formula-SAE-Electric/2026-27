/**
 * @file    energy_meter.h
 * @brief   Decodes the FSAE Energy Meter frames for the dashboard.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_ENERGY_METER_H
#define VCU_ENERGY_METER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Decoded Energy Meter values the dashboard and log read.
extern float emVoltageV;
extern bool hasEmMeasurement;
extern bool hasEmStatus;
extern uint8_t emSensorCount;
extern uint8_t emMinTempRaw;
extern uint8_t emMaxTempRaw;
extern bool hasEmTempSummary;
extern bool hasEmTemps;
extern bool isEmTimedOut;

void emHandleMeasurementFrame(const CAN_message_t &frame);
void emHandleStatusFrame(const CAN_message_t &frame);
void emHandleTemperatureFrame(const CAN_message_t &frame);
void emServiceTimeout(uint32_t nowMs);
const char *emGetStatusSummaryText(uint16_t &colorOut);

#endif  // VCU_ENERGY_METER_H

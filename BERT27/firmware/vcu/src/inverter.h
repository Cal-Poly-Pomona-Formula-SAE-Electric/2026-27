/**
 * @file    inverter.h
 * @brief   PM100DX status frames, CAN broadcast enable and automatic fault clear.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_INVERTER_H
#define VCU_INVERTER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

extern float inverterDcBusVoltageV;
extern float inverterDcBusCurrentA;
extern bool hasInverterVoltageInfo;
extern bool hasInverterCurrentInfo;
extern float glvVoltageV;
extern bool hasGlvVoltage;
extern bool hasPm100Traffic;
extern uint32_t firstPm100FrameMs;
extern float powerKw;
extern int16_t inverterTempTenthsC;
extern int16_t pm100MotorTempTenthsC;
extern bool hasInverterTemp;
extern bool hasPm100MotorTemp;
extern uint32_t pm100PostFaults;
extern uint32_t pm100RunFaults;
extern bool hasPm100FaultStatus;
extern bool hasPm100Fault;
extern int16_t motorRpm;

void inverterHandleTemperatures1Frame(const CAN_message_t &frame);
void inverterHandleTemperatures3Frame(const CAN_message_t &frame);
void inverterHandleRpmFrame(const CAN_message_t &frame);
void inverterHandleCurrentInfoFrame(const CAN_message_t &frame);
void inverterHandleVoltageInfoFrame(const CAN_message_t &frame);
void inverterHandleInternalVoltagesFrame(const CAN_message_t &frame);
void inverterHandleParamResponseFrame(const CAN_message_t &frame);
void inverterServiceBroadcastEnable();
void inverterServiceFaultClear();
void inverterHandleFaultCodesFrame(const CAN_message_t &frame);

#endif  // VCU_INVERTER_H

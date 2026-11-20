/**
 * @file    imd.h
 * @brief   Bender iso175 IMD frames and the warning/alarm poll.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.6.1.7
 */
#ifndef VCU_IMD_H
#define VCU_IMD_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint32_t IMD_REQUEST_ID      = 0x22;
static constexpr uint8_t IDX_WARN_ALARMS      = 0x6C;

// IMD state decoded from 0x37 and 0x23. hasImdFault drives the TSSI and RTD.
extern bool hasImdFault;
extern uint16_t imdWarnAlarms;
extern uint16_t imdRIsoKohm;
extern uint8_t imdRIsoStatus;
extern uint32_t lastImdRxMs;
extern bool isImdCanTimedOut;

void imdHandleFrame(const CAN_message_t &frame);
void imdRequestWarnings();

#endif  // VCU_IMD_H

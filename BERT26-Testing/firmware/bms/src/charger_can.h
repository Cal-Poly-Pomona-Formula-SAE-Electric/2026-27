/**
 * @file    charger_can.h
 * @brief   Sets up CAN1 and talks to the HK-MF charger (Elcon protocol, 250 kbit/s, extended IDs).
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_CHARGER_CAN_H
#define BMS_CHARGER_CAN_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Charger IDs (29-bit extended, Elcon/J1939)
static constexpr uint32_t CHARGER_COMMAND_ID   = 0x1806E5F4;  // BMS -> charger
static constexpr uint32_t CHARGER_BROADCAST_ID = 0x18FF50E5;  // charger -> BMS

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> chargerCan;

extern volatile bool     hasSeenChargerCan;  // any charger-bus frame during INIT
extern volatile float    chargerOutputV;
extern volatile float    chargerOutputA;
extern volatile uint8_t  chargerStatus;
extern volatile uint32_t chargerLastRxMs;

void chargerCanSetup();
void chargerCanSendCommand(bool isStop);
void chargerCanSendFaultStop();
void chargerCanReceive(const CAN_message_t &frame);

#endif  // BMS_CHARGER_CAN_H

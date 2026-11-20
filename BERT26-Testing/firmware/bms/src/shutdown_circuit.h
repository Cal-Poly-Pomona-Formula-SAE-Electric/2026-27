/**
 * @file    shutdown_circuit.h
 * @brief   Watches the shutdown circuit input and opens the contactors when it drops.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.7.2
 */
#ifndef BMS_SHUTDOWN_CIRCUIT_H
#define BMS_SHUTDOWN_CIRCUIT_H

#include <Arduino.h>

extern volatile bool isSdcHigh;      // true = SDC closed (negative contactor closed)
extern volatile bool hasSdcChanged;  // set on any edge, cleared by the state handlers

void sdcSetup();
void sdcHandleShutdown();
void sdcServiceFallingEdge();

#endif  // BMS_SHUTDOWN_CIRCUIT_H

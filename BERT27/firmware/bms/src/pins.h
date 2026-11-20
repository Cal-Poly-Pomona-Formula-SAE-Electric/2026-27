/**
 * @file    pins.h
 * @brief   Every Teensy 4.1 pin the BMS uses, in one place.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_PINS_H
#define BMS_PINS_H

// ---------------- BQ79600 bridge ----------------
// Serial1 TX. Held LOW for a timed pulse to wake or reset the BQ79600 bridge.
static const int PIN_BQ_UART_TX = 1;

// ---------------- Analog inputs ----------------
static const int PIN_PACK_VOLTAGE_SENSE = 14;  // A0, battery-side isolation amplifier
static const int PIN_PACK_CURRENT_SENSE = 16;  // A2, LEM DHAB S/124 channel 2 (+/-500 A)

// ---------------- Shutdown circuit ----------------
// Rising edge = negative contactor closed, falling edge = shutdown.
static const int PIN_SDC_SENSE = 20;
// Output to the shutdown circuit: HIGH = nominal, LOW = BMS fault opens the loop.
static const int PIN_BMS_STATUS = 41;

// ---------------- Contactor outputs (low-side switches) ----------------
static const int PIN_PRECHARGE_RELAY = 24;
static const int PIN_POS_AIR         = 25;

// ---------------- CAN ----------------
// The CAN pins are fixed by the Teensy 4.1 peripherals and configured by
// FlexCAN_T4::begin(): CAN1 (charger) TX=22 RX=23, CAN3 (vehicle/IMD) TX=31 RX=30.

#endif  // BMS_PINS_H

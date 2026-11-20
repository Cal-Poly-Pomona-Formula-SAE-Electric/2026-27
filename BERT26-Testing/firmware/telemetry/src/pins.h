/**
 * @file    pins.h
 * @brief   Every ESP32-S3 pin the telemetry node uses, in one place.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef TELEMETRY_PINS_H
#define TELEMETRY_PINS_H

// ---------------- CAN3 transceiver (vehicle bus, 500 kbps) ----------------
// Kept exactly as the BERT26 sketch passed them to ESP32Can.begin(). Its comment
// and serial text say TX=43 / RX=44, the code used TX=44 / RX=43; check the
// schematic before changing either.
static const int PIN_CAN_TX = 44;
static const int PIN_CAN_RX = 43;

#endif  // TELEMETRY_PINS_H

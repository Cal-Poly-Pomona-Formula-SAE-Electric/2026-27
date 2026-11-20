/**
 * @file    can_bytes.h
 * @brief   Byte-order helpers for packing and unpacking CAN frame data.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_CAN_BYTES_H
#define BMS_CAN_BYTES_H

#include <Arduino.h>

static inline uint16_t canReadU16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static inline uint16_t canReadU16Be(uint8_t hi, uint8_t lo) {
  return ((uint16_t)hi << 8) | (uint16_t)lo;
}

static inline void canWriteU16Be(uint8_t *dest, uint16_t value) {
  dest[0] = value >> 8;
  dest[1] = value & 0xFF;
}

#endif  // BMS_CAN_BYTES_H

/**
 * @file    bq79616.h
 * @brief   Bench sketch BQ79600/BQ79616 stack driver: UART frames, wake, auto-address,
 *          ADC setup, cell voltage and NTC temperature reads.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software (bq79616_parametricFinal_LimitedPolling.ino)
 * Worked on by: BERT27 restructure
 *
 * All BQ79616s have dual-TMUX NTC mux boards wired as:
 *   GPIO1 = A0, GPIO2 = A1, GPIO3 = A2  (mux address)
 *   GPIO4 = MUX0 EN,  GPIO5 = MUX0 OUT  (cells  1-8)
 *   GPIO6 = MUX1 EN,  GPIO7 = MUX1 OUT  (cells 9-16)
 */
#ifndef BENCH_BQ79616_H
#define BENCH_BQ79616_H

#include <Arduino.h>

// ---------------- Stack layout ----------------
// The only line to change: the number of BQ79616s in the stack.
static const uint8_t  BQ_STACK_DEVICE_COUNT = 6;
static const uint8_t  BQ_CELLS_PER_DEVICE   = 16;
static const uint16_t PACK_CELLS = (uint16_t)BQ_STACK_DEVICE_COUNT * BQ_CELLS_PER_DEVICE;

// Resets, wakes and addresses the stack, enables TSREF, starts the ADC and
// sets the 1-minute comms timeout. Restarts itself until every device answers.
void bqInit();
// Restarts continuous ADC conversions; needed again after a mux sweep.
void bqAdcInit();
// Fills voltagesV[0..15] = C1..C16. Returns false if the device did not answer.
bool bqReadVoltages(uint8_t deviceId, float voltagesV[BQ_CELLS_PER_DEVICE]);
// Fills tempsC[0..15]; a channel that cannot be read is NAN.
void bqReadTemps(uint8_t deviceId, float tempsC[BQ_CELLS_PER_DEVICE]);

#endif  // BENCH_BQ79616_H

/**
 * @file    pins.h
 * @brief   Every Teensy 4.1 pin the VCU uses, in one place.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_PINS_H
#define VCU_PINS_H

// ---------------- Display / touch pins ----------------
static const int PIN_TFT_MISO = 12;
static const int PIN_TFT_MOSI = 11;
static const int PIN_TFT_SCK  = 13;
static const int PIN_TFT_DC   = 8;
static const int PIN_TFT_CS   = 10;
static const int PIN_TFT_RST  = 9;
static const int PIN_TFT_LED  = 29;

static const int PIN_TOUCH_CS   = 7;
static const int PIN_TOUCH_IRQ  = 6;

// ---------------- Vehicle IO ----------------
static const int PIN_READY_BUTTON = 23;
static const int PIN_BUZZER       = 22;    // Change here if the buzzer is wired elsewhere.
static const int PIN_BRAKE_LIGHT  = 5;     // HIGH = brake light on
static const int PIN_FAN_1        = 1;
static const int PIN_FAN_2        = 2;

// Shutdown-circuit status inputs: HIGH = signal OK (green), LOW = tripped (red).
static const int PIN_BSPD_STATUS  = 24;
static const int PIN_IMD_STATUS   = 25;
static const int PIN_BMS_STATUS   = 28;

// HV battery current sensor analog input; sampled into the fast data log.
static const int PIN_HV_CURRENT_SENSE = 14;

// MPU6050 IMU on the default Wire pins (18=SDA, 19=SCL). The sensor's INT pin
// is wired to 33 and configured as data-ready, but samples are polled at the
// fast-log rate so the interrupt is informational only.
static const int PIN_IMU_INT = 33;

// EURO-XPD analog input pins from VCU_Pedal_Read_Test.
static const int PIN_APPS_1 = 38;         // Inverted channel
static const int PIN_APPS_2 = 41;         // Normal channel
static const int PIN_BSE_1  = 17;
static const int PIN_BSE_2  = 16;

// TSSI LED pins: low-side switches, HIGH = transistor on = LED on.
static const int PIN_TSSI_LED_RED   = 3;
static const int PIN_TSSI_LED_GREEN = 4;

#endif  // VCU_PINS_H

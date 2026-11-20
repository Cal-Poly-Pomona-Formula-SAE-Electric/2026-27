/**
 * @file    main.cpp
 * @brief   Bench sketch: listens for the FSAE Energy Meter CAN frames on the vehicle
 *          bus and prints them decoded over USB serial.
 *
 * Board:   BMS (Teensy 4.1), or any Teensy 4.1 with a transceiver on CAN3
 * Original Creator: BERT26 software (EM_CAN_UART_Bridge.ino)
 * Worked on by: BERT27 restructure
 *
 * The Energy Meter runs at 500 kbps (EM_User_Manual_2026, p.9 "CAN Baud Rate:
 * 500k"), which matches the BMS vehicle bus: CAN3, 500000 baud, 11-bit
 * standard frames. (The BMS's other bus, CAN1, is the 250 kbps charger bus.)
 *
 * The bridge shares the vehicle bus with the BMS, IMD and everything else, so
 * hardware acceptance filters make the FIFO deliver only the three IDs the
 * Energy Meter transmits. All Energy Meter signals are little-endian (Intel).
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---------------- CAN bus (same settings as the BMS vehicle bus) ----------------
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> emCan;
static constexpr uint32_t EM_CAN_BAUD_BPS = 500000;
// Pin numbers printed in the start-up banner only. FlexCAN_T4::begin() puts
// CAN3 on the Teensy 4.1 default pins (TX=31, RX=30) whatever these say.
static constexpr uint8_t PIN_EM_CAN_TX_LABEL = 30;
static constexpr uint8_t PIN_EM_CAN_RX_LABEL = 31;

static constexpr uint32_t USB_BAUD = 115200;

// ---------------- Energy Meter message IDs (11-bit), EM_User_Manual_2026 p.10-11 ----------------
// These are the only IDs the Energy Meter transmits, so they are the only
// frames accepted off the shared vehicle bus.
static constexpr uint32_t EM_MEASUREMENT_ID = 0x10D;  // 20 ms:  Current[A] f32, Voltage[V] f32
static constexpr uint32_t EM_STATUS_ID      = 0x40D;  // 100 ms: flags + Energy[Whr] f32
static constexpr uint32_t EM_TEMPERATURE_ID = 0x60D;  // 250 ms: muxed temperatures

static constexpr uint8_t EM_FRAME_LEN = 8;  // every EM frame is a full 8 bytes; shorter ones are ignored

// 0x40D status flag bits (byte 0).
static constexpr uint8_t EM_FLAG_VIOLATION         = 0x01;
static constexpr uint8_t EM_FLAG_LOGGING           = 0x02;
static constexpr uint8_t EM_FLAG_FAULT_ACTIVE      = 0x04;
static constexpr uint8_t EM_FLAG_FAULT_PREV_ACTIVE = 0x08;

// 0x60D layout: byte0 bits0-2 = mux index, bits3-7 = sensor count (mux 0),
// 5 temperatures per frame from byte 3, every temperature degC = raw * 0.5.
static constexpr uint8_t EM_TEMP_MUX_MASK         = 0x07;
static constexpr uint8_t EM_TEMP_SENSOR_COUNT_SHIFT = 3;
static constexpr uint8_t EM_TEMPS_PER_FRAME       = 5;
static constexpr uint8_t EM_TEMP_FIRST_BYTE       = 3;
static constexpr float   EM_TEMP_C_PER_BIT        = 0.5f;

// Up to 32 temperature sensors (mux 0..6, 5 temps per frame; mux 6 carries 2).
static constexpr uint8_t EM_MAX_TEMPS = 32;
static float   emTempsC[EM_MAX_TEMPS];
static bool    isEmTempValid[EM_MAX_TEMPS];
static uint8_t emTempSensorCount = 0;
static float   emTempMinC = NAN, emTempMaxC = NAN;

static constexpr uint8_t EM_TEMPS_PER_PRINT_ROW = 8;
static constexpr float   W_PER_KW               = 1000.0f;

// Latest measurement (0x10D, 20 ms), held until the next monitor print.
static float emCurrentA = NAN, emVoltageV = NAN;

// Output cadence: the temperature sweep (0x60D) is the slowest source of
// fresh data (~250 ms), so the monitor is printed once per sweep period.
// This keeps measurement and temperature matched at one rate and is robust
// regardless of how many sensors the meter reports (no reliance on a
// specific mux frame arriving).
static constexpr uint32_t MONITOR_PERIOD_MS = 250;
static uint32_t lastPrintMs = 0;

// ---------------- Decode helpers ----------------
static float emReadFloatLe(const uint8_t *bytes) {
  // Teensy 4.x is little-endian, so a straight copy matches Intel byte order.
  float value;
  memcpy(&value, bytes, sizeof(value));
  return value;
}

// Prints the latest measurement together with the most recent temperature
// set, so both are reported at the same cadence.
static void emPrintMonitor() {
  float powerKw = emCurrentA * emVoltageV / W_PER_KW;
  Serial.printf("[EM MEASUREMENT]  Current = %8.3f A   Voltage = %8.3f V   Power = %7.3f kW\n",
                emCurrentA, emVoltageV, powerKw);
  Serial.printf("[EM TEMPERATURE]  Sensors = %u   Min = %.1f C   Max = %.1f C\n",
                emTempSensorCount, emTempMinC, emTempMaxC);
  Serial.print("                  ");
  for (uint8_t i = 0; i < EM_MAX_TEMPS; i++) {
    if (!isEmTempValid[i]) continue;
    Serial.printf("T%-2u=%.1fC  ", i, emTempsC[i]);
    if ((i % EM_TEMPS_PER_PRINT_ROW) == EM_TEMPS_PER_PRINT_ROW - 1) Serial.print("\n                  ");
  }
  Serial.println();
}

// 0x10D Measurement: Current[A] float @0-3, Voltage[V] float @4-7.
// Folded in at the 20 ms frame rate; printed at the slower monitor rate.
static void emDecodeMeasurement(const CAN_message_t &frame) {
  if (frame.len < EM_FRAME_LEN) return;
  emCurrentA = emReadFloatLe(&frame.buf[0]);
  emVoltageV = emReadFloatLe(&frame.buf[4]);
}

// 0x40D Status: byte0 bits 0..3 = flags, Energy[Whr] float @4-7.
static void emDecodeStatus(const CAN_message_t &frame) {
  if (frame.len < EM_FRAME_LEN) return;
  uint8_t flags            = frame.buf[0];
  bool isViolation         = flags & EM_FLAG_VIOLATION;
  bool isLogging           = flags & EM_FLAG_LOGGING;
  bool isFaultActive       = flags & EM_FLAG_FAULT_ACTIVE;
  bool wasFaultActive      = flags & EM_FLAG_FAULT_PREV_ACTIVE;
  float energyWh           = emReadFloatLe(&frame.buf[4]);
  Serial.printf("[EM STATUS]       Violation = %-3s   Logging = %-3s   FaultActive = %-3s   FaultPrev = %-3s   Energy = %.2f Whr\n",
                isViolation ? "YES" : "no",
                isLogging ? "YES" : "no",
                isFaultActive ? "YES" : "no",
                wasFaultActive ? "YES" : "no",
                energyWh);
}

// 0x60D Temperature (multiplexed): byte0 bits0-2 = mux index.
//   mux 0: bits3-7 = num sensors; byte1 = min*0.5; byte2 = max*0.5;
//          bytes3-7 = Temp[0..4]
//   mux N: bytes3-7 = Temp[N*5 .. N*5+4]
//   All temps: degC = raw * 0.5 (unsigned byte)
// One full sweep arrives across several muxed frames; each frame is folded
// into emTempsC[] and printed on the monitor period.
static void emDecodeTemperature(const CAN_message_t &frame) {
  if (frame.len < EM_FRAME_LEN) return;
  uint8_t mux = frame.buf[0] & EM_TEMP_MUX_MASK;

  if (mux == 0) {
    emTempSensorCount = frame.buf[0] >> EM_TEMP_SENSOR_COUNT_SHIFT;
    emTempMinC = frame.buf[1] * EM_TEMP_C_PER_BIT;
    emTempMaxC = frame.buf[2] * EM_TEMP_C_PER_BIT;
  }

  uint8_t firstIndex = mux * EM_TEMPS_PER_FRAME;
  for (uint8_t i = 0; i < EM_TEMPS_PER_FRAME; i++) {
    uint8_t tempIndex = firstIndex + i;
    if (tempIndex >= EM_MAX_TEMPS) break;
    emTempsC[tempIndex]      = frame.buf[EM_TEMP_FIRST_BYTE + i] * EM_TEMP_C_PER_BIT;
    isEmTempValid[tempIndex] = true;
  }
}

// Only EM IDs reach here thanks to the FIFO filters.
void emCanReceive(const CAN_message_t &frame) {
  switch (frame.id) {
    case EM_MEASUREMENT_ID: emDecodeMeasurement(frame);  break;
    case EM_STATUS_ID:      emDecodeStatus(frame);       break;
    case EM_TEMPERATURE_ID: emDecodeTemperature(frame);  break;
    default:                                             break;  // filtered out, should not occur
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(USB_BAUD);
  Serial.println("[EM-BRIDGE] Energy Meter CAN -> USB decoder — Teensy 4.1");

  for (uint8_t i = 0; i < EM_MAX_TEMPS; i++) {
    emTempsC[i] = NAN;
    isEmTempValid[i] = false;
  }

  emCan.begin();
  emCan.setBaudRate(EM_CAN_BAUD_BPS);
  emCan.setMaxMB(16);
  emCan.enableFIFO();
  emCan.enableFIFOInterrupt();

  // Acceptance filters: reject everything on the shared vehicle bus, then
  // admit only the three Energy-Meter-transmitted 11-bit IDs.
  emCan.setFIFOFilter(REJECT_ALL);
  emCan.setFIFOFilter(0, EM_MEASUREMENT_ID, STD);
  emCan.setFIFOFilter(1, EM_STATUS_ID,      STD);
  emCan.setFIFOFilter(2, EM_TEMPERATURE_ID, STD);

  emCan.onReceive(emCanReceive);

  Serial.printf("[EM-BRIDGE] CAN3 @ %lu baud (TX%u/RX%u) — listening for EM IDs 0x%X, 0x%X, 0x%X only\n",
                (unsigned long)EM_CAN_BAUD_BPS, PIN_EM_CAN_TX_LABEL, PIN_EM_CAN_RX_LABEL,
                EM_MEASUREMENT_ID, EM_STATUS_ID, EM_TEMPERATURE_ID);
}

// ---------------- Main loop ----------------
void loop() {
  emCan.events();  // service the FIFO; emCanReceive() folds in the latest data

  if (millis() - lastPrintMs >= MONITOR_PERIOD_MS) {
    lastPrintMs = millis();
    emPrintMonitor();
  }
}

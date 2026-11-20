/**
 * @file    charger_can.cpp
 * @brief   Sets up CAN1 and talks to the HK-MF charger (Elcon protocol, 250 kbit/s, extended IDs).
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "charger_can.h"

#include "pack_config.h"
#include "can_bytes.h"
#include "state_machine.h"
#include "vehicle_can.h"

// CAN1 on Teensy 4.1 pins 22 (TX) and 23 (RX), set up by FlexCAN_T4::begin().
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> chargerCan;

static constexpr uint32_t CHARGER_CAN_BAUD_BPS = 250000;

volatile bool     hasSeenChargerCan = false;
volatile float    chargerOutputV    = 0.0f;
volatile float    chargerOutputA    = 0.0f;
volatile uint8_t  chargerStatus     = 0;
volatile uint32_t chargerLastRxMs   = 0;

// Elcon protocol scaling: 0.1 V and 0.1 A per bit, big-endian.
static constexpr float   ELCON_UNITS_PER_VOLT = 10.0f;
static constexpr float   ELCON_UNITS_PER_AMP  = 10.0f;
static constexpr float   ELCON_VOLTS_PER_UNIT = 0.1f;
static constexpr float   ELCON_AMPS_PER_UNIT  = 0.1f;
static constexpr uint8_t ELCON_CONTROL_CHARGE = 0x00;
static constexpr uint8_t ELCON_CONTROL_STOP   = 0x01;

void chargerCanSetup() {
  chargerCan.begin();
  chargerCan.setBaudRate(CHARGER_CAN_BAUD_BPS);
  chargerCan.setMaxMB(16);
  chargerCan.enableFIFO();
  chargerCan.enableFIFOInterrupt();
  chargerCan.onReceive(chargerCanReceive);
}

// ---------------- Transmit ----------------
// Voltage and current request. isStop zeroes the current and sets the stop flag.
void chargerCanSendCommand(bool isStop) {
  CAN_message_t command{};
  command.id = CHARGER_COMMAND_ID; command.flags.extended = 1; command.len = 8;
  uint16_t voltageRaw = (uint16_t)(CHARGER_REQUEST_V * ELCON_UNITS_PER_VOLT);  // 3958 = 395.8 V
  uint16_t currentRaw = isStop ? 0u : (uint16_t)(CHARGER_REQUEST_A * ELCON_UNITS_PER_AMP);  // 100 = 10.0 A
  canWriteU16Be(&command.buf[0], voltageRaw);
  canWriteU16Be(&command.buf[2], currentRaw);
  command.buf[4] = isStop ? ELCON_CONTROL_STOP : ELCON_CONTROL_CHARGE;
  chargerCan.write(command);
}

// Sent once from faultsEnter. Same as the stop command except the current
// request is left at CHARGER_REQUEST_A, as the original firmware did.
void chargerCanSendFaultStop() {
  CAN_message_t command{};
  command.id = CHARGER_COMMAND_ID; command.flags.extended = 1; command.len = 8;
  uint16_t voltageRaw = (uint16_t)(CHARGER_REQUEST_V * ELCON_UNITS_PER_VOLT);
  uint16_t currentRaw = (uint16_t)(CHARGER_REQUEST_A * ELCON_UNITS_PER_AMP);
  canWriteU16Be(&command.buf[0], voltageRaw);
  canWriteU16Be(&command.buf[2], currentRaw);
  command.buf[4] = ELCON_CONTROL_STOP;
  chargerCan.write(command);
}

// ---------------- Receive ----------------
void chargerCanReceive(const CAN_message_t &frame) {
  if (bmsState == STATE_INIT) hasSeenChargerCan = true;

  if (frame.flags.extended && frame.id == CHARGER_BROADCAST_ID && frame.len >= 5) {
    chargerOutputV  = canReadU16Be(frame.buf[0], frame.buf[1]) * ELCON_VOLTS_PER_UNIT;
    chargerOutputA  = canReadU16Be(frame.buf[2], frame.buf[3]) * ELCON_AMPS_PER_UNIT;
    chargerStatus   = frame.buf[4];
    chargerLastRxMs = millis();
  }

  // Cell dump, SOC set and current calibration are accepted on this bus too.
  vehicleCanHandleCommandFrame(frame, true);
}

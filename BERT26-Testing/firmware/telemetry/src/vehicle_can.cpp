/**
 * @file    vehicle_can.cpp
 * @brief   Starts CAN3, drains received frames to their module and sends SOC/calibration commands to the BMS.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * CAN IDs and byte layouts follow can/CAN_MESSAGES.md (BMS frames are big-endian).
 */
#include "vehicle_can.h"

#include "pins.h"
#include "live_signals.h"
#include "cell_dump.h"

// Functions used only in this file.
void canHandleFrame(long id, bool isExtended, uint8_t* frameData, int frameLength);

static constexpr TwaiSpeed CAN_SPEED = TWAI_SPEED_500KBPS;

// 0xFFFF tells ESP32-TWAI-CAN to keep its default TX queue depth (5 frames).
static constexpr uint16_t CAN_TX_QUEUE_LIBRARY_DEFAULT = 0xFFFF;
// Large enough to absorb a full 32-frame cell-voltage dump (sent 1 frame/5 ms
// by the BMS) even if loop() is briefly stalled by an HTTP response. The
// default depth (5) was dropping the tail of the longer voltage dumps, so
// areCellVoltagesValid (set on the last frame) never latched.
static constexpr uint16_t CAN_RX_QUEUE_FRAMES = 64;

// readFrame() timeout of 0 makes the drain loop non-blocking.
static constexpr uint32_t CAN_READ_NO_WAIT_MS = 0;

// ---------------- 0x314 SOC set ----------------
static constexpr uint8_t SET_SOC_LENGTH_BYTES = 2;
static constexpr float   SOC_UNITS_PER_PCT    = 10.0f;    // 0.1 %/bit
static constexpr float   SOC_MIN_UNITS        = 0.0f;     // 0.0 %
static constexpr float   SOC_MAX_UNITS        = 1000.0f;  // 100.0 %

// ---------------- 0x315 current calibration point ----------------
static constexpr uint8_t CURRENT_CAL_LENGTH_BYTES = 3;
static constexpr float   CAL_UNITS_PER_A          = 10.0f;     // 0.1 A/bit
static constexpr float   CAL_MIN_UNITS            = -32768.0f; // int16 range
static constexpr float   CAL_MAX_UNITS            = 32767.0f;

// ---------------- Setup ----------------
void canSetup() {
  if (!ESP32Can.begin(CAN_SPEED, PIN_CAN_TX, PIN_CAN_RX, CAN_TX_QUEUE_LIBRARY_DEFAULT, CAN_RX_QUEUE_FRAMES)) {
    Serial.println("CAN init failed — check wiring on TX=43 / RX=44");
  } else {
    Serial.println("CAN3 ready  500 kbps  TX=43 RX=44");
  }
}

// ---------------- Receive ----------------
// Drain all pending CAN frames so a cell dump is not left in the queue.
void canReceiveFrames() {
  CanFrame rxFrame;
  while (ESP32Can.readFrame(rxFrame, CAN_READ_NO_WAIT_MS)) {
    canHandleFrame(rxFrame.identifier, rxFrame.extd,
                   rxFrame.data, rxFrame.data_length_code);
  }
}

void canHandleFrame(long id, bool isExtended, uint8_t* frameData, int frameLength) {

  if (!isExtended) {
    switch ((uint16_t)id) {

      case BMS_STATUS_ID:
        liveSignalsHandleBmsStatusFrame(frameData, frameLength);
        break;

      case BMS_TEMP_ID:
        liveSignalsHandleBmsTempFrame(frameData, frameLength);
        break;

      case BMS_CELL_DATA_ID:
        cellDumpHandleFrame(frameData, frameLength);
        break;
    }
    return;
  }

  // Extended frame
  if ((unsigned long)id == CHARGER_STATUS_ID) {
    liveSignalsHandleChargerFrame(frameData, frameLength);
  }
}

// ---------------- BMS commands from the config page ----------------
void canSendSoc(float socPct) {
  uint16_t socRaw = (uint16_t)constrain(socPct * SOC_UNITS_PER_PCT, SOC_MIN_UNITS, SOC_MAX_UNITS);
  CanFrame txFrame = {};
  txFrame.identifier = BMS_SET_SOC_ID; txFrame.extd = CAN_STANDARD_FRAME; txFrame.data_length_code = SET_SOC_LENGTH_BYTES;
  txFrame.data[0] = (uint8_t)(socRaw >> 8);
  txFrame.data[1] = (uint8_t)(socRaw & 0xFF);
  ESP32Can.writeFrame(txFrame);
}

void canSendCurrentCalPoint(uint8_t calPoint, float currentA) {
  int16_t currentRaw = (int16_t)constrain(currentA * CAL_UNITS_PER_A, CAL_MIN_UNITS, CAL_MAX_UNITS);
  CanFrame txFrame = {};
  txFrame.identifier = BMS_CURRENT_CAL_ID; txFrame.extd = CAN_STANDARD_FRAME; txFrame.data_length_code = CURRENT_CAL_LENGTH_BYTES;
  txFrame.data[0] = calPoint;
  txFrame.data[1] = (uint8_t)((uint16_t)currentRaw >> 8);
  txFrame.data[2] = (uint8_t)((uint16_t)currentRaw & 0xFF);
  ESP32Can.writeFrame(txFrame);
}

/**
 * @file    main.cpp
 * @brief   Bench sketch: polls the Bender iso175 IMD over CAN for HV bus voltage
 *          and prints it with the isolation/status message.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software (IMD_Bus_Voltage.ino)
 * Worked on by: BERT27 restructure
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN3 uses the Teensy 4.1 default CAN3 pins, set by FlexCAN_T4::begin().
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> imdCan;

static constexpr uint32_t IMD_CAN_BAUD_BPS = 500000;

// ---------------- Bender iso175 CAN IDs (Bender-Standard defaults) ----------------
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_REQUEST_ID      = 0x22;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;

// ---------------- GET indices (spec D00415_03) ----------------
static constexpr uint8_t IMD_INDEX_HV_VOLTAGE   = 0x5E;  // HV system voltage, DataWord LE
static constexpr uint8_t IMD_INDEX_VOLTAGE_MODE = 0x64;  // 0xFD=AC, 0xFE=DC (default DC)
static constexpr uint8_t IMD_VOLTAGE_MODE_AC    = 0xFD;
static constexpr uint8_t IMD_VOLTAGE_MODE_DC    = 0xFE;
static constexpr uint8_t IMD_REQUEST_FILL_BYTE  = 0xFF;  // unused request bytes

// Voltage decoding per spec:
//   raw = DataWord (uint16, little-endian), 65535 = SNV (signal not valid)
//   voltage_V = (raw - 32128) * 0.05
static constexpr float    IMD_VOLTAGE_RAW_OFFSET = 32128.0f;
static constexpr float    IMD_VOLTS_PER_BIT      = 0.05f;
static constexpr uint16_t IMD_RAW_SNV            = 0xFFFF;

static constexpr uint32_t IMD_POLL_MS          = 500;   // poll at 2 Hz
static constexpr uint32_t SERIAL_WAIT_MS       = 2000;  // give a USB monitor time to attach, then run without one
static constexpr uint32_t IMD_STARTUP_DELAY_MS = 500;   // let the bus come up before the first request

static float imdDecodeVoltage(uint16_t raw) {
  return ((float)raw - IMD_VOLTAGE_RAW_OFFSET) * IMD_VOLTS_PER_BIT;
}

// Little-endian 16-bit from buf[n], buf[n+1].
static inline uint16_t canReadU16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

void imdSendGet(uint8_t index) {
  CAN_message_t request{};
  request.id     = IMD_REQUEST_ID;
  request.len    = 8;
  request.buf[0] = index;
  for (int i = 1; i < 8; i++) request.buf[i] = IMD_REQUEST_FILL_BYTE;
  imdCan.write(request);
}

// IMD_Info_General (0x37): isolation + status, always broadcast.
void imdHandleInfoGeneral(const CAN_message_t &frame) {
  uint16_t isolationKohm = canReadU16Le(frame.buf[0], frame.buf[1]);
  uint8_t  statusBits    = frame.buf[2];
  uint8_t  warningBits   = frame.buf[4];

  Serial.print("[0x37] R_iso=");
  if (isolationKohm == IMD_RAW_SNV) Serial.print("SNV");
  else { Serial.print(isolationKohm); Serial.print(" kOhm"); }

  Serial.print(" | status=0x"); Serial.print(statusBits, HEX);
  Serial.print(" | warn=0x");   Serial.println(warningBits, HEX);
}

// IMD_Response (0x23): answers to our GET requests.
void imdHandleResponse(const CAN_message_t &frame) {
  uint8_t index = frame.buf[0];

  if (index == IMD_INDEX_HV_VOLTAGE) {
    uint16_t raw = canReadU16Le(frame.buf[1], frame.buf[2]);
    Serial.print("[GET 0x5E] HV_System=");
    if (raw == IMD_RAW_SNV) Serial.println("SNV");
    else { Serial.print(imdDecodeVoltage(raw), 2); Serial.println(" V"); }

  } else if (index == IMD_INDEX_VOLTAGE_MODE) {
    Serial.print("[GET 0x64] Voltage_Mode=");
    if      (frame.buf[1] == IMD_VOLTAGE_MODE_AC) Serial.println("AC");
    else if (frame.buf[1] == IMD_VOLTAGE_MODE_DC) Serial.println("DC");
    else { Serial.print("0x"); Serial.println(frame.buf[1], HEX); }
  }
}

void imdCanReceive(const CAN_message_t &frame) {
  if      (frame.id == IMD_INFO_GENERAL_ID) imdHandleInfoGeneral(frame);
  else if (frame.id == IMD_RESPONSE_ID)     imdHandleResponse(frame);
}

elapsedMillis imdPollTimerMs;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < SERIAL_WAIT_MS) {}

  Serial.println("IMD Bus Voltage — Teensy 4.1 / CAN3 (pins 30/31)");

  imdCan.begin();
  imdCan.setBaudRate(IMD_CAN_BAUD_BPS);
  imdCan.setMaxMB(16);
  imdCan.enableFIFO();
  imdCan.enableFIFOInterrupt();
  imdCan.onReceive(imdCanReceive);

  // The voltage reading is only meaningful in DC mode, so check it once at start-up.
  delay(IMD_STARTUP_DELAY_MS);
  imdSendGet(IMD_INDEX_VOLTAGE_MODE);
  Serial.println("Requested Voltage_Mode (expect DC).");
}

// ---------------- Main loop ----------------
void loop() {
  imdCan.events();

  if (imdPollTimerMs >= IMD_POLL_MS) {
    imdPollTimerMs = 0;
    imdSendGet(IMD_INDEX_HV_VOLTAGE);
  }
}

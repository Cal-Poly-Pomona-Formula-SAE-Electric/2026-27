/**
 * @file    main.cpp
 * @brief   Bench sketch: prints the IMD HV voltage (CAN) next to both calibrated
 *          isolated-amp channel voltages and their raw ADC counts.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software (BMS_Voltage_Readout.ino)
 * Worked on by: BERT27 restructure
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---------------- Pins ----------------
static const int PIN_BMS_STATUS = 41;  // HIGH after 3 s so the SDC closes the negative contactor
static const int PIN_ISO_AMP_A  = 14;  // A0
static const int PIN_ISO_AMP_B  = 15;  // A1
// CAN3 (Bender iso175) uses the Teensy 4.1 default CAN3 pins, TX=31 RX=30,
// set by FlexCAN_T4::begin().

// ---------------- Calibration ----------------
// Chan B fit from ISO_Amp_Characterization, applied to both channels:
//   V = (ADC - ISO_AMP_OFFSET_COUNTS) / ISO_AMP_COUNTS_PER_V
static constexpr float ISO_AMP_COUNTS_PER_V  = 5.2957f;
static constexpr float ISO_AMP_OFFSET_COUNTS = 1.83f;
static constexpr int   ADC_RESOLUTION_BITS   = 12;
static constexpr int   ISO_AMP_SAMPLES       = 64;  // averaged per print for a stable reading

// ---------------- Bender iso175 CAN ----------------
static constexpr uint32_t IMD_CAN_BAUD_BPS        = 500000;
static constexpr uint32_t IMD_REQUEST_ID          = 0x22;
static constexpr uint32_t IMD_RESPONSE_ID         = 0x23;
static constexpr uint8_t  IMD_INDEX_HV_VOLTAGE    = 0x5E;
static constexpr uint8_t  IMD_REQUEST_FILL_BYTE   = 0xFF;  // unused request bytes
// iso175 "HV voltage" encoding: 0.05 V per bit around a 32128 offset, little-endian.
static constexpr float    IMD_VOLTAGE_RAW_OFFSET  = 32128.0f;
static constexpr float    IMD_VOLTS_PER_BIT       = 0.05f;
static constexpr uint16_t IMD_VOLTAGE_RAW_INVALID = 0xFFFF;
// Printed in the IMD column until the first valid reading arrives.
static constexpr float    IMD_VOLTAGE_PENDING_V   = -999.0f;

// ---------------- Timing ----------------
static constexpr uint32_t IMD_POLL_MS          = 300;   // IMD GET rate
static constexpr uint32_t PRINT_PERIOD_MS      = 500;   // Serial print rate
static constexpr uint32_t SERIAL_WAIT_MS       = 2000;  // give a USB monitor time to attach, then run without one
static constexpr uint32_t CONTACTOR_DELAY_MS   = 3000;  // wait before raising PIN_BMS_STATUS

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> imdCan;

volatile bool  isImdVoltageValid = false;
volatile float imdVoltageV       = 0.0f;

elapsedMillis imdPollTimerMs;
elapsedMillis printTimerMs;

static inline uint16_t canReadU16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float imdDecodeVoltage(uint16_t raw) {
  return ((float)raw - IMD_VOLTAGE_RAW_OFFSET) * IMD_VOLTS_PER_BIT;
}

static float isoAmpCountsToV(int adcCounts) {
  return ((float)adcCounts - ISO_AMP_OFFSET_COUNTS) / ISO_AMP_COUNTS_PER_V;
}

void imdSendGet(uint8_t index) {
  CAN_message_t request{};
  request.id     = IMD_REQUEST_ID;
  request.len    = 8;
  request.buf[0] = index;
  for (int i = 1; i < 8; i++) request.buf[i] = IMD_REQUEST_FILL_BYTE;
  imdCan.write(request);
}

void imdCanReceive(const CAN_message_t &frame) {
  if (frame.id == IMD_RESPONSE_ID && frame.buf[0] == IMD_INDEX_HV_VOLTAGE) {
    uint16_t raw = canReadU16Le(frame.buf[1], frame.buf[2]);
    if (raw == IMD_VOLTAGE_RAW_INVALID) {
      isImdVoltageValid = false;
    } else {
      imdVoltageV       = imdDecodeVoltage(raw);
      isImdVoltageValid = true;
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < SERIAL_WAIT_MS) {}
  Serial.println("BMS Voltage Readout -- Teensy 4.1");
  Serial.printf("Calibration: V = (ADC - %.2f) / %.4f\n", ISO_AMP_OFFSET_COUNTS, ISO_AMP_COUNTS_PER_V);
  Serial.println("  IMD_V (V) | Chan_A (V) | Chan_B (V) | ADC_A | ADC_B");

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(PIN_ISO_AMP_A, INPUT);
  pinMode(PIN_ISO_AMP_B, INPUT);

  imdCan.begin();
  imdCan.setBaudRate(IMD_CAN_BAUD_BPS);
  imdCan.setMaxMB(16);
  imdCan.enableFIFO();
  imdCan.enableFIFOInterrupt();
  imdCan.onReceive(imdCanReceive);

  // SAFETY: raising the status line lets the SDC close the negative contactor.
  pinMode(PIN_BMS_STATUS, OUTPUT);
  digitalWrite(PIN_BMS_STATUS, LOW);
  delay(CONTACTOR_DELAY_MS);
  digitalWrite(PIN_BMS_STATUS, HIGH);
  Serial.println("Negative contactor closed.");
}

// ---------------- Main loop ----------------
void loop() {
  imdCan.events();

  if (imdPollTimerMs >= IMD_POLL_MS) {
    imdPollTimerMs = 0;
    imdSendGet(IMD_INDEX_HV_VOLTAGE);
  }

  if (printTimerMs >= PRINT_PERIOD_MS) {
    printTimerMs = 0;

    long sumACounts = 0, sumBCounts = 0;
    for (int i = 0; i < ISO_AMP_SAMPLES; i++) {
      sumACounts += analogRead(PIN_ISO_AMP_A);
      sumBCounts += analogRead(PIN_ISO_AMP_B);
    }
    int isoAmpACounts = (int)(sumACounts / ISO_AMP_SAMPLES);
    int isoAmpBCounts = (int)(sumBCounts / ISO_AMP_SAMPLES);

    float isoAmpAV = isoAmpCountsToV(isoAmpACounts);
    float isoAmpBV = isoAmpCountsToV(isoAmpBCounts);
    float imdReadingV = isImdVoltageValid ? imdVoltageV : IMD_VOLTAGE_PENDING_V;

    Serial.printf("%9.2f | %10.2f | %10.2f | %5d | %5d\n",
                  imdReadingV, isoAmpAV, isoAmpBV, isoAmpACounts, isoAmpBCounts);
  }
}

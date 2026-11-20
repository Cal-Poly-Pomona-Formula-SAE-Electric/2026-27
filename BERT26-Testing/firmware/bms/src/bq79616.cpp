/**
 * @file    bq79616.cpp
 * @brief   TI BQ79600/BQ79616 stack driver: UART frames, CRC, wake, auto-address, ADC and GPIO setup, cell voltage and NTC temperature reads.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * Register sequences follow bq79616_parametricFinal_LimitedPolling.ino.
 */
#include "bq79616.h"

#include <math.h>

#include "pins.h"
#include "safety_limits.h"
#include "cell_monitor.h"

static uint16_t bqCrc16(const uint8_t *bytes, size_t length);
static void bqSendWrite(uint8_t writeType, uint8_t deviceId, uint16_t reg, const uint8_t *data, uint8_t length);
static void bqSendReadRequest(uint8_t readType, uint8_t deviceId, uint16_t reg, uint8_t byteCount);
static bool bqWaitForBytes(uint8_t *dest, size_t length, uint32_t timeoutMs);
static bool bqReadReg8(uint8_t deviceId, uint16_t reg, uint8_t &value);
static bool bqReadReg16(uint8_t deviceId, uint16_t reg, uint16_t &value);
static void bqWriteAll(uint16_t reg, uint8_t value);
static void bqWriteSingle(uint8_t deviceId, uint16_t reg, uint8_t value);
static void bqPulseTxLow(uint32_t pulseUs);
static void bqWake();
static void bqShutdownToActive();
static void bqAutoAddress();
static bool bqConfigureBusbarChannel();
static void bqEnableTsref(uint8_t deviceId);
static float bqNtcOhmToC(float measuredOhm);
static void bqSetMux(uint8_t deviceId, uint8_t bank, uint8_t channel);
static bool bqReadMuxTemp(uint8_t deviceId, uint8_t gpioNumber, float &tempC);
static inline uint16_t bqGpioReg(uint8_t gpioNumber);

// ---------------- Stack addressing ----------------
static const uint8_t BQ_DEVICE_COUNT = BQ_STACK_DEVICE_COUNT + 1;  // bridge plus monitors
static const uint8_t BQ_ID_BASE      = 0;                          // BQ79600 bridge
static const uint8_t BQ_ID_TOP       = BQ_STACK_DEVICE_COUNT;

static_assert(BYPASSED_BQ_CELL >= 9u, "This configuration expects BBVC_POSN1 (cells 9-16)");
static constexpr uint8_t BYPASSED_BBVC_MASK = 1u << (BYPASSED_BQ_CELL - 9u);  // bit 3 = 0x08

// ---------------- UART link ----------------
static const uint32_t BQ_UART_BAUD = 1000000;
static HardwareSerial &bqUart = Serial1;
static const uint32_t BQ_REG_READ_TIMEOUT_MS     = 30;
static const uint32_t BQ_VOLTAGE_READ_TIMEOUT_MS = 60;
// Pulse widths on the TX line, from the BQ79600 datasheet.
static const uint32_t BQ_HW_RESET_PULSE_US = 36000;
static const uint32_t BQ_WAKE_PULSE_US     = 2500;
static const uint32_t BQ_STA_PULSE_US      = 250;

// ---------------- Register map ----------------
static constexpr uint16_t BQ_REG_ACTIVE_CELL       = 0x0003;
static constexpr uint16_t BQ_REG_BBVC_POSN1        = 0x0005;
static constexpr uint16_t BQ_REG_ADC_CONF1         = 0x0007;
static constexpr uint16_t BQ_REG_GPIO_CONF1        = 0x000E;
static constexpr uint16_t BQ_REG_GPIO_CONF2        = 0x000F;
static constexpr uint16_t BQ_REG_GPIO_CONF3        = 0x0010;
static constexpr uint16_t BQ_REG_GPIO_CONF4        = 0x0011;
static constexpr uint16_t BQ_REG_COMM_TIMEOUT_CONF = 0x0019;
static constexpr uint16_t BQ_REG_DIR0_ADDR         = 0x0306;
static constexpr uint16_t BQ_REG_COMM_CTRL         = 0x0308;
static constexpr uint16_t BQ_REG_CONTROL1          = 0x0309;
static constexpr uint16_t BQ_REG_CONTROL2          = 0x030A;
static constexpr uint16_t BQ_REG_ADC_CTRL1         = 0x030D;
static constexpr uint16_t BQ_REG_OTP_ECC_DATAIN1   = 0x0343;
static constexpr uint16_t BQ_REG_OTP_ECC_DATAIN8   = 0x034A;
static constexpr uint16_t BQ_REG_PARTID            = 0x0500;
static constexpr uint16_t BQ_REG_VCELL16_HI        = 0x0568;
static constexpr uint16_t BQ_REG_TSREF_HI          = 0x058C;
static constexpr uint16_t BQ_REG_GPIO1_HI          = 0x058E;

// Register values written during setup.
static constexpr uint8_t BQ_ACTIVE_CELL_ALL_16     = 0x0A;  // all 16 positions active; BBVC marks the busbar
static constexpr uint8_t BQ_ADC_CONF1_VALUE        = 0x02;
static constexpr uint8_t BQ_ADC_CTRL1_RUN          = 0x0E;
static constexpr uint8_t BQ_CONTROL1_ADDR_WR       = 0x01;
static constexpr uint8_t BQ_CONTROL1_SEND_WAKE     = 0x20;
static constexpr uint8_t BQ_CONTROL2_TSREF_EN      = 0x01;
static constexpr uint8_t BQ_COMM_CTRL_STACK        = 0x02;
static constexpr uint8_t BQ_COMM_CTRL_TOP          = 0x03;
static constexpr uint8_t BQ_COMM_CTRL_BASE         = 0x00;
// Auto-shutdown after 1 min of no valid comms:
// CTL_ACT=1 (SHUTDOWN), CTL_TIME=100b (1 min), CTS_TIME=000 disabled.
static constexpr uint8_t BQ_COMM_TIMEOUT_1MIN_SHUTDOWN = 0x0C;

// GPIO_CONFx mode bits.
static constexpr uint8_t BQ_GPIO_MODE_OFF = 0b000;
static constexpr uint8_t BQ_GPIO_MODE_ADC = 0b010;
static constexpr uint8_t BQ_GPIO_MODE_HI  = 0b100;
static constexpr uint8_t BQ_GPIO_MODE_LO  = 0b101;

// Frame type bytes (FRMWRT) from the BQ79616 datasheet.
enum : uint8_t { FRMWRT_SGL_W = 0x90, FRMWRT_STK_W = 0xB0, FRMWRT_ALL_W = 0xD0 };
enum : uint8_t { FRMWRT_SGL_R = 0x00, FRMWRT_STK_R = 0x20, FRMWRT_ALL_R = 0x40 };

static constexpr float BQ_VOLTS_PER_LSB = 190.73e-6f;

// ---------------- NTC thermistor ----------------
static const float NTC_R25_OHM    = 10000.0f;
static const float NTC_BETA_K     = 3380.0f;
static const float NTC_T0_K       = 298.15f;
static const float NTC_PULLUP_OHM = 10000.0f;
// Quadratic correction of the measured resistance, fitted on the bench.
static const float NTC_CORR_A = -5.7e-6f;
static const float NTC_CORR_B =  0.846f;
static const float NTC_CORR_C =  180.0f;
static const float KELVIN_OFFSET_C = 273.15f;

// Mux GPIO outputs read for each half of a device's 16 thermistors.
static const uint8_t BQ_MUX_BANK0_GPIO = 5;
static const uint8_t BQ_MUX_BANK1_GPIO = 7;
static const uint32_t BQ_MUX_SETTLE_US = 35000;

static inline uint16_t bqGpioReg(uint8_t gpioNumber) {
  return (uint16_t)(BQ_REG_GPIO1_HI + 2u * (uint16_t)(gpioNumber - 1u));
}

// ---------------- UART frames ----------------
static uint16_t bqCrc16(const uint8_t *bytes, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; bit++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

static void bqSendWrite(uint8_t writeType, uint8_t deviceId, uint16_t reg, const uint8_t *data, uint8_t length) {
  uint8_t frame[16]; uint8_t frameLen = 0;
  frame[frameLen++] = 0x80 | (writeType + (length - 1));
  if (writeType == FRMWRT_SGL_W) frame[frameLen++] = deviceId;
  frame[frameLen++] = reg >> 8; frame[frameLen++] = reg & 0xFF;
  for (uint8_t i = 0; i < length; i++) frame[frameLen++] = data[i];
  uint16_t crc = bqCrc16(frame, frameLen);
  frame[frameLen++] = crc & 0xFF; frame[frameLen++] = crc >> 8;
  bqUart.write(frame, frameLen); bqUart.flush();
}

static void bqSendReadRequest(uint8_t readType, uint8_t deviceId, uint16_t reg, uint8_t byteCount) {
  uint8_t frame[8]; uint8_t frameLen = 0;
  frame[frameLen++] = 0x80 | readType;
  if (readType == FRMWRT_SGL_R) frame[frameLen++] = deviceId;
  frame[frameLen++] = reg >> 8; frame[frameLen++] = reg & 0xFF;
  frame[frameLen++] = byteCount - 1;
  uint16_t crc = bqCrc16(frame, frameLen);
  frame[frameLen++] = crc & 0xFF; frame[frameLen++] = crc >> 8;
  bqUart.write(frame, frameLen); bqUart.flush();
}

static bool bqWaitForBytes(uint8_t *dest, size_t length, uint32_t timeoutMs) {
  uint32_t startMs = millis(); size_t received = 0;
  while (received < length) {
    while (bqUart.available() && received < length) dest[received++] = bqUart.read();
    if (received == length) return true;
    if (millis() - startMs >= timeoutMs) return false;
  }
  return true;
}

// Response layout: [type][id][reg hi][reg lo][data...][crc lo][crc hi]
static bool bqReadReg8(uint8_t deviceId, uint16_t reg, uint8_t &value) {
  uint8_t response[7] = {};
  bqSendReadRequest(FRMWRT_SGL_R, deviceId, reg, 1);
  if (!bqWaitForBytes(response, 7, BQ_REG_READ_TIMEOUT_MS)) return false;
  value = response[4]; return true;
}

static bool bqReadReg16(uint8_t deviceId, uint16_t reg, uint16_t &value) {
  uint8_t response[8] = {};
  bqSendReadRequest(FRMWRT_SGL_R, deviceId, reg, 2);
  if (!bqWaitForBytes(response, 8, BQ_REG_READ_TIMEOUT_MS)) return false;
  value = ((uint16_t)response[4] << 8) | response[5]; return true;
}

static void bqWriteAll(uint16_t reg, uint8_t value) { bqSendWrite(FRMWRT_ALL_W, 0, reg, &value, 1); }
static void bqWriteSingle(uint8_t deviceId, uint16_t reg, uint8_t value) { bqSendWrite(FRMWRT_SGL_W, deviceId, reg, &value, 1); }

// ---------------- Wake and reset pulses ----------------
// The BQ79600 decodes a timed LOW on its RX line as WAKE, SHUTDOWN-to-ACTIVE
// or hardware reset, so the UART is released and the pin driven by hand.
static void bqPulseTxLow(uint32_t pulseUs) {
  bqUart.end();
  pinMode(PIN_BQ_UART_TX, OUTPUT); digitalWrite(PIN_BQ_UART_TX, LOW);
  delayMicroseconds(pulseUs);
  digitalWrite(PIN_BQ_UART_TX, HIGH);
  bqUart.begin(BQ_UART_BAUD, SERIAL_8N1);
  delayMicroseconds(200);
}

void bqHardwareReset()         { bqPulseTxLow(BQ_HW_RESET_PULSE_US); delay(100); }
static void bqWake()           { bqPulseTxLow(BQ_WAKE_PULSE_US); }
static void bqShutdownToActive() { bqPulseTxLow(BQ_STA_PULSE_US); }

// ---------------- Setup sequences ----------------
static void bqAutoAddress() {
  for (uint16_t reg = BQ_REG_OTP_ECC_DATAIN1; reg <= BQ_REG_OTP_ECC_DATAIN8; reg++) {
    uint8_t zero = 0; bqSendWrite(FRMWRT_STK_W, 0, reg, &zero, 1);
  }
  bqWriteAll(BQ_REG_CONTROL1, BQ_CONTROL1_ADDR_WR);
  for (uint8_t address = 0; address < BQ_DEVICE_COUNT; address++) bqWriteAll(BQ_REG_DIR0_ADDR, address);
  bqWriteAll(BQ_REG_COMM_CTRL, BQ_COMM_CTRL_STACK);
  bqWriteSingle(BQ_ID_TOP,  BQ_REG_COMM_CTRL, BQ_COMM_CTRL_TOP);
  bqWriteSingle(BQ_ID_BASE, BQ_REG_COMM_CTRL, BQ_COMM_CTRL_BASE);
}

static bool bqConfigureBusbarChannel() {
  // Cell 92 is physically bypassed, but it remains the twelfth measurement
  // channel on IC6. ACTIVE_CELL must stay at 16 because it only selects a
  // contiguous range starting at Cell1; BBVC_POSN1 identifies the interior gap.
  bqWriteSingle(BYPASSED_BQ_DEVICE, BQ_REG_BBVC_POSN1, BYPASSED_BBVC_MASK);
  delay(2);

  uint8_t readback = 0;
  if (!bqReadReg8(BYPASSED_BQ_DEVICE, BQ_REG_BBVC_POSN1, readback)) {
    Serial.println("[BQ] busbar configuration readback failed");
    return false;
  }
  if (readback != BYPASSED_BBVC_MASK) {
    Serial.printf("[BQ] busbar configuration mismatch: id%u reg=0x%02X expected=0x%02X\n",
                  BYPASSED_BQ_DEVICE, readback, BYPASSED_BBVC_MASK);
    return false;
  }

  Serial.printf("[BQ] busbar configured: pack index %u = id%u Cell%u (BBVC_POSN1=0x%02X)\n",
                BYPASSED_CELL_INDEX, BYPASSED_BQ_DEVICE,
                BYPASSED_BQ_CELL, BYPASSED_BBVC_MASK);
  return true;
}

void bqAdcInit() {
  bqWriteAll(BQ_REG_ACTIVE_CELL, BQ_ACTIVE_CELL_ALL_16);
  bqWriteAll(BQ_REG_ADC_CONF1,   BQ_ADC_CONF1_VALUE);
  bqWriteAll(BQ_REG_ADC_CTRL1,   BQ_ADC_CTRL1_RUN);
  delay(50);
}

static void bqEnableTsref(uint8_t deviceId) {
  uint8_t control2 = 0; bqReadReg8(deviceId, BQ_REG_CONTROL2, control2);
  bqWriteSingle(deviceId, BQ_REG_CONTROL2, control2 | BQ_CONTROL2_TSREF_EN); delay(3);
}

// ---------------- Cell voltages ----------------
bool bqReadVoltages(uint8_t deviceId, float voltagesV[BQ_CELLS_PER_DEVICE]) {
  uint8_t response[38] = {};
  bqSendReadRequest(FRMWRT_SGL_R, deviceId, BQ_REG_VCELL16_HI, 32);
  if (!bqWaitForBytes(response, 38, BQ_VOLTAGE_READ_TIMEOUT_MS)) return false;
  // Registers run VCELL16 down to VCELL1, so the array is filled in reverse.
  for (int i = 0; i < BQ_CELLS_PER_DEVICE; i++) {
    uint16_t raw = ((uint16_t)response[4 + i * 2] << 8) | response[4 + i * 2 + 1];
    voltagesV[15 - i] = (int16_t)raw * BQ_VOLTS_PER_LSB;
  }
  return true;
}

// ---------------- Cell temperatures ----------------
static float bqNtcOhmToC(float measuredOhm) {
  float correctedOhm = NTC_CORR_A * measuredOhm * measuredOhm + NTC_CORR_B * measuredOhm + NTC_CORR_C;
  if (correctedOhm <= 0.0f) return NAN;
  float inverseTempK = (1.0f / NTC_T0_K) + logf(correctedOhm / NTC_R25_OHM) / NTC_BETA_K;
  if (inverseTempK <= 0.0f) return NAN;
  return 1.0f / inverseTempK - KELVIN_OFFSET_C;
}

// Drives the external thermistor mux through the device GPIOs: three address
// lines plus one enable per bank, then restarts the ADC.
static void bqSetMux(uint8_t deviceId, uint8_t bank, uint8_t channel) {
  channel &= 0x7;
  uint8_t addr0 = (channel & 1) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_LO;
  uint8_t addr1 = (channel & 2) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_LO;
  uint8_t addr2 = (channel & 4) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_LO;
  uint8_t enable0 = (bank == 0) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_OFF;
  uint8_t enable1 = (bank == 1) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_OFF;
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF1, (uint8_t)((addr1 << 3) | addr0));
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF2, (uint8_t)((enable0 << 3) | addr2));
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF3, (uint8_t)((enable1 << 3) | BQ_GPIO_MODE_ADC));
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF4, (uint8_t)((BQ_GPIO_MODE_OFF << 3) | BQ_GPIO_MODE_ADC));
  bqWriteSingle(deviceId, BQ_REG_ADC_CTRL1, BQ_ADC_CTRL1_RUN);
}

static bool bqReadMuxTemp(uint8_t deviceId, uint8_t gpioNumber, float &tempC) {
  uint16_t tsrefRaw = 0, gpioRaw = 0;
  if (!bqReadReg16(deviceId, BQ_REG_TSREF_HI,       tsrefRaw)) return false;
  if (!bqReadReg16(deviceId, bqGpioReg(gpioNumber), gpioRaw))  return false;
  int16_t tsrefCode = (int16_t)tsrefRaw, gpioCode = (int16_t)gpioRaw;
  if (tsrefCode <= 0 || tsrefCode == (int16_t)0x8000) return false;
  if (gpioCode  <= 0 || gpioCode  == (int16_t)0x8000) return false;
  float ratio = (float)gpioCode / (float)tsrefCode;
  if (ratio <= 0.001f || ratio >= 0.999f) return false;
  float measuredOhm = NTC_PULLUP_OHM * ratio / (1.0f - ratio);
  tempC = bqNtcOhmToC(measuredOhm);
  return !isnan(tempC);
}

bool bqReadTemps(uint8_t deviceId, float tempsC[BQ_CELLS_PER_DEVICE]) {
  bool isAllOk = true;
  for (uint8_t cell = 0; cell < BQ_CELLS_PER_DEVICE; cell++) {
    uint8_t bank = (cell < 8) ? 0 : 1;
    uint8_t channel = cell & 0x7;
    uint8_t gpioOut = (bank == 0) ? BQ_MUX_BANK0_GPIO : BQ_MUX_BANK1_GPIO;
    bqSetMux(deviceId, bank, channel);
    if (cell == 0 || cell == 8) delayMicroseconds(BQ_MUX_SETTLE_US);  // extra settle on a bank change
    delayMicroseconds(BQ_MUX_SETTLE_US);
    float tempC = NAN;
    if (!bqReadMuxTemp(deviceId, gpioOut, tempC)) {
      Serial.printf("[BQ] temp read failed: id%d cell%d\n", deviceId, cell + 1);
      isAllOk = false;
    }
    tempsC[cell] = tempC;
  }
  return isAllOk;
}

// ---------------- Init ----------------
bool bqInit() {
  bqUart.begin(BQ_UART_BAUD, SERIAL_8N1);
  delay(500);
  bqHardwareReset(); delay(10);
  bqWake(); delay(15);
  bqWake(); delay(15);
  bqWriteSingle(BQ_ID_BASE, BQ_REG_CONTROL1, BQ_CONTROL1_SEND_WAKE); delay(100);  // BQ79600 SEND_WAKE to BQ79616 stack
  bqShutdownToActive(); delay(5);
  bqAutoAddress(); delay(5);

  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    uint8_t deviceId = stackIndex + 1; uint16_t partId = 0;
    if (!bqReadReg16(deviceId, BQ_REG_PARTID, partId)) {
      Serial.printf("[BQ] id%d no response\n", deviceId);
      return false;
    }
    Serial.printf("[BQ] id%d PARTID=0x%04X\n", deviceId, partId);
  }
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) bqEnableTsref(stackIndex + 1);
  if (!bqConfigureBusbarChannel()) return false;
  bqAdcInit();

  // SAFETY: the monitors shut themselves down if this firmware stops talking
  // to them for one minute.
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    uint8_t deviceId = stackIndex + 1;
    bqWriteSingle(deviceId, BQ_REG_COMM_TIMEOUT_CONF, BQ_COMM_TIMEOUT_1MIN_SHUTDOWN);
    delay(2);
    uint8_t readback = 0;
    if (bqReadReg8(deviceId, BQ_REG_COMM_TIMEOUT_CONF, readback)) {
      Serial.printf("[BQ] id%d COMM_TIMEOUT_CONF=0x%02X %s\n",
                    deviceId, readback, readback == BQ_COMM_TIMEOUT_1MIN_SHUTDOWN ? "OK" : "MISMATCH");
    } else {
      Serial.printf("[BQ] id%d COMM_TIMEOUT_CONF read FAIL\n", deviceId);
    }
  }

  for (uint16_t i = 0; i < PACK_CELLS; i++) cellTempsC[i] = NAN;
  Serial.println("[BQ] init done");
  return true;
}

// Up to BQ_INIT_MAX_ATTEMPTS tries before the caller faults.
bool bqInitWithRetries() {
  bool isBqOk = false;
  for (int attempt = 1; attempt <= BQ_INIT_MAX_ATTEMPTS && !isBqOk; attempt++) {
    if (attempt > 1) {
      Serial.printf("[BQ] Init failed — retry %d/%d, waiting %lums...\n",
                    attempt, BQ_INIT_MAX_ATTEMPTS, (unsigned long)BQ_INIT_RETRY_WAIT_MS);
      delay(BQ_INIT_RETRY_WAIT_MS);
    }
    isBqOk = bqInit();
  }
  return isBqOk;
}

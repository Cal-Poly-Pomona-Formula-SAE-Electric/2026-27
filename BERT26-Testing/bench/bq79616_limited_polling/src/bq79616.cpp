/**
 * @file    bq79616.cpp
 * @brief   Bench sketch BQ79600/BQ79616 stack driver: UART frames, wake, auto-address,
 *          ADC setup, cell voltage and NTC temperature reads.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software (bq79616_parametricFinal_LimitedPolling.ino)
 * Worked on by: BERT27 restructure
 */
#include "bq79616.h"

#include <math.h>

static uint16_t bqCrc16(const uint8_t *bytes, size_t length);
static void bqSendWrite(uint8_t writeType, uint8_t deviceId, uint16_t reg, const uint8_t *data, uint8_t length);
static void bqSendReadRequest(uint8_t readType, uint8_t deviceId, uint16_t reg, uint8_t byteCount);
static bool bqWaitForBytes(uint8_t *dest, size_t length, uint32_t timeoutMs);
static bool bqReadReg8(uint8_t deviceId, uint16_t reg, uint8_t &value);
static bool bqReadReg16(uint8_t deviceId, uint16_t reg, uint16_t &value);
static void bqWriteAll(uint16_t reg, uint8_t value);
static void bqWriteSingle(uint8_t deviceId, uint16_t reg, uint8_t value);
static void bqPulseTxLow(uint32_t pulseUs);
static void bqHardwareReset();
static void bqWake();
static void bqShutdownToActive();
static void bqAutoAddress();
static void bqEnableTsref(uint8_t deviceId);
static float bqNtcOhmToC(float measuredOhm);
static void bqSetMux(uint8_t deviceId, uint8_t bank, uint8_t channel);
static bool bqReadMuxTemp(uint8_t deviceId, uint8_t gpioNumber, float &tempC);
static inline uint16_t bqGpioReg(uint8_t gpioNumber);

// ---------------- Stack addressing ----------------
static const uint8_t BQ_DEVICE_COUNT = BQ_STACK_DEVICE_COUNT + 1;  // bridge plus monitors
static const uint8_t BQ_ID_BASE      = 0;                          // BQ79600 bridge
static const uint8_t BQ_ID_TOP       = BQ_STACK_DEVICE_COUNT;

// ---------------- UART link ----------------
// Serial1 TX. Held LOW for a timed pulse to wake or reset the BQ79600 bridge.
static const int PIN_BQ_UART_TX = 1;
static const uint32_t BQ_UART_BAUD = 1000000;
static HardwareSerial &bqUart = Serial1;
static const uint32_t BQ_REG_READ_TIMEOUT_MS     = 30;
static const uint32_t BQ_VOLTAGE_READ_TIMEOUT_MS = 60;
// Pulse widths on the TX line, from the BQ79600 datasheet.
static const uint32_t BQ_HW_RESET_PULSE_US = 36000;
static const uint32_t BQ_WAKE_PULSE_US     = 2500;
static const uint32_t BQ_STA_PULSE_US      = 250;
static const uint32_t BQ_UART_RESTART_US   = 200;   // UART settle after a pulse
static const uint32_t BQ_HW_RESET_WAIT_MS  = 100;

// Delays in the start-up sequence, from the original bench sketch.
static const uint32_t BQ_UART_START_WAIT_MS   = 500;
static const uint32_t BQ_AFTER_RESET_MS       = 10;
static const uint32_t BQ_AFTER_WAKE_MS        = 15;
static const uint32_t BQ_AFTER_STA_MS         = 5;
static const uint32_t BQ_AFTER_ADDRESS_MS     = 5;
static const uint32_t BQ_TSREF_SETTLE_MS      = 3;
static const uint32_t BQ_ADC_FIRST_SETTLE_MS  = 50;  // first conversion
static const uint32_t BQ_WRITE_VERIFY_WAIT_MS = 2;

// ---------------- Register map ----------------
static constexpr uint16_t BQ_REG_ACTIVE_CELL       = 0x0003;
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
static constexpr uint8_t BQ_ACTIVE_CELL_ALL_16 = 0x0A;  // enable all 16 cells
static constexpr uint8_t BQ_ADC_CONF1_LPF_ON   = 0x02;
static constexpr uint8_t BQ_ADC_CTRL1_RUN      = 0x0E;  // MAIN_GO | continuous
static constexpr uint8_t BQ_CONTROL1_ADDR_WR   = 0x01;
static constexpr uint8_t BQ_CONTROL2_TSREF_EN  = 0x01;
static constexpr uint8_t BQ_COMM_CTRL_STACK    = 0x02;
static constexpr uint8_t BQ_COMM_CTRL_TOP      = 0x03;
static constexpr uint8_t BQ_COMM_CTRL_BASE     = 0x00;
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

// Response layout: [type][id][reg hi][reg lo][data...][crc lo][crc hi]
static constexpr uint8_t BQ_RESPONSE_OVERHEAD_BYTES = 6;
static constexpr uint8_t BQ_RESPONSE_DATA_OFFSET    = 4;
static constexpr uint8_t BQ_VOLTAGE_DATA_BYTES      = 32;  // 16 cells x 2 bytes

static constexpr float BQ_VOLTS_PER_LSB = 190.73e-6f;
static constexpr int16_t BQ_ADC_INVALID_CODE = (int16_t)0x8000;

// ---------------- NTC thermistor ----------------
static const float NTC_R25_OHM    = 10000.0f;
static const float NTC_BETA_K     = 3380.0f;
static const float NTC_T0_K       = 298.15f;   // 25 + 273.15
static const float NTC_PULLUP_OHM = 10000.0f;
// Quadratic correction of the measured resistance, fitted on the bench.
static const float NTC_CORR_A = -5.7e-6f;
static const float NTC_CORR_B =  0.846f;
static const float NTC_CORR_C =  180.0f;
static const float KELVIN_OFFSET_C = 273.15f;
// A GPIO/TSREF ratio this close to 0 or 1 means an open or shorted thermistor.
static const float NTC_RATIO_MIN = 0.001f;
static const float NTC_RATIO_MAX = 0.999f;

// Mux GPIO outputs read for each half of a device's 16 thermistors.
static const uint8_t BQ_CELLS_PER_MUX_BANK = 8;
static const uint8_t BQ_MUX_BANK0_GPIO = 5;
static const uint8_t BQ_MUX_BANK1_GPIO = 7;
// Settle time per channel: 4 ms is enough for ~20 ADC round-robins at 200 us each.
static const uint32_t BQ_MUX_SETTLE_US = 4000;

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

static void bqSendWrite(uint8_t writeType, uint8_t deviceId, uint16_t reg,
                        const uint8_t *data, uint8_t length) {
  uint8_t frame[16];
  uint8_t frameLen = 0;
  frame[frameLen++] = 0x80 | (writeType + (length - 1));
  if (writeType == FRMWRT_SGL_W) frame[frameLen++] = deviceId;
  frame[frameLen++] = reg >> 8;
  frame[frameLen++] = reg & 0xFF;
  for (uint8_t i = 0; i < length; i++) frame[frameLen++] = data[i];
  uint16_t crc = bqCrc16(frame, frameLen);
  frame[frameLen++] = crc & 0xFF;
  frame[frameLen++] = crc >> 8;
  bqUart.write(frame, frameLen);
  bqUart.flush();
}

static void bqSendReadRequest(uint8_t readType, uint8_t deviceId, uint16_t reg, uint8_t byteCount) {
  uint8_t frame[8];
  uint8_t frameLen = 0;
  frame[frameLen++] = 0x80 | readType;
  if (readType == FRMWRT_SGL_R) frame[frameLen++] = deviceId;
  frame[frameLen++] = reg >> 8;
  frame[frameLen++] = reg & 0xFF;
  frame[frameLen++] = byteCount - 1;
  uint16_t crc = bqCrc16(frame, frameLen);
  frame[frameLen++] = crc & 0xFF;
  frame[frameLen++] = crc >> 8;
  bqUart.write(frame, frameLen);
  bqUart.flush();
}

// Waits for exactly length bytes, gives up after timeoutMs.
static bool bqWaitForBytes(uint8_t *dest, size_t length, uint32_t timeoutMs) {
  uint32_t startMs = millis();
  size_t received = 0;
  while (received < length) {
    while (bqUart.available() && received < length) dest[received++] = bqUart.read();
    if (received == length) return true;
    if (millis() - startMs >= timeoutMs) return false;
  }
  return true;
}

static bool bqReadReg8(uint8_t deviceId, uint16_t reg, uint8_t &value) {
  uint8_t response[BQ_RESPONSE_OVERHEAD_BYTES + 1] = {};
  bqSendReadRequest(FRMWRT_SGL_R, deviceId, reg, 1);
  if (!bqWaitForBytes(response, sizeof(response), BQ_REG_READ_TIMEOUT_MS)) return false;
  value = response[BQ_RESPONSE_DATA_OFFSET];
  return true;
}

static bool bqReadReg16(uint8_t deviceId, uint16_t reg, uint16_t &value) {
  uint8_t response[BQ_RESPONSE_OVERHEAD_BYTES + 2] = {};
  bqSendReadRequest(FRMWRT_SGL_R, deviceId, reg, 2);
  if (!bqWaitForBytes(response, sizeof(response), BQ_REG_READ_TIMEOUT_MS)) return false;
  value = ((uint16_t)response[BQ_RESPONSE_DATA_OFFSET] << 8) | response[BQ_RESPONSE_DATA_OFFSET + 1];
  return true;
}

static void bqWriteAll(uint16_t reg, uint8_t value) {
  bqSendWrite(FRMWRT_ALL_W, 0, reg, &value, 1);
}

static void bqWriteSingle(uint8_t deviceId, uint16_t reg, uint8_t value) {
  bqSendWrite(FRMWRT_SGL_W, deviceId, reg, &value, 1);
}

// ---------------- Wake and reset pulses ----------------
// The BQ79600 decodes a timed LOW on its RX line as WAKE, SHUTDOWN-to-ACTIVE
// or hardware reset, so the UART is released and the pin driven by hand.
static void bqPulseTxLow(uint32_t pulseUs) {
  bqUart.end();
  pinMode(PIN_BQ_UART_TX, OUTPUT);
  digitalWrite(PIN_BQ_UART_TX, LOW);
  delayMicroseconds(pulseUs);
  digitalWrite(PIN_BQ_UART_TX, HIGH);
  bqUart.begin(BQ_UART_BAUD, SERIAL_8N1);
  delayMicroseconds(BQ_UART_RESTART_US);
}

static void bqHardwareReset()    { bqPulseTxLow(BQ_HW_RESET_PULSE_US); delay(BQ_HW_RESET_WAIT_MS); }
static void bqWake()             { bqPulseTxLow(BQ_WAKE_PULSE_US); }
static void bqShutdownToActive() { bqPulseTxLow(BQ_STA_PULSE_US); }

// ---------------- Setup sequences ----------------
static void bqAutoAddress() {
  // DLL sync: dummy stack writes to the OTP ECC registers.
  for (uint16_t reg = BQ_REG_OTP_ECC_DATAIN1; reg <= BQ_REG_OTP_ECC_DATAIN8; reg++) {
    uint8_t zero = 0;
    bqSendWrite(FRMWRT_STK_W, 0, reg, &zero, 1);
  }
  bqWriteAll(BQ_REG_CONTROL1, BQ_CONTROL1_ADDR_WR);  // enable address write on all devices
  // Each broadcast write is taken by the next unaddressed device: 0..BQ_DEVICE_COUNT-1.
  for (uint8_t address = 0; address < BQ_DEVICE_COUNT; address++) {
    bqWriteAll(BQ_REG_DIR0_ADDR, address);
  }
  bqWriteAll(BQ_REG_COMM_CTRL, BQ_COMM_CTRL_STACK);
  bqWriteSingle(BQ_ID_TOP,  BQ_REG_COMM_CTRL, BQ_COMM_CTRL_TOP);
  bqWriteSingle(BQ_ID_BASE, BQ_REG_COMM_CTRL, BQ_COMM_CTRL_BASE);
}

// Leaves the ADC running continuously. Called at start-up and again after
// each temperature sweep.
void bqAdcInit() {
  bqWriteAll(BQ_REG_ACTIVE_CELL, BQ_ACTIVE_CELL_ALL_16);
  bqWriteAll(BQ_REG_ADC_CONF1,   BQ_ADC_CONF1_LPF_ON);
  bqWriteAll(BQ_REG_ADC_CTRL1,   BQ_ADC_CTRL1_RUN);
  delay(BQ_ADC_FIRST_SETTLE_MS);
}

static void bqEnableTsref(uint8_t deviceId) {
  uint8_t control2 = 0;
  bqReadReg8(deviceId, BQ_REG_CONTROL2, control2);
  bqWriteSingle(deviceId, BQ_REG_CONTROL2, control2 | BQ_CONTROL2_TSREF_EN);
  delay(BQ_TSREF_SETTLE_MS);
}

// ---------------- Cell voltages ----------------
bool bqReadVoltages(uint8_t deviceId, float voltagesV[BQ_CELLS_PER_DEVICE]) {
  uint8_t response[BQ_RESPONSE_OVERHEAD_BYTES + BQ_VOLTAGE_DATA_BYTES] = {};
  bqSendReadRequest(FRMWRT_SGL_R, deviceId, BQ_REG_VCELL16_HI, BQ_VOLTAGE_DATA_BYTES);
  if (!bqWaitForBytes(response, sizeof(response), BQ_VOLTAGE_READ_TIMEOUT_MS)) return false;
  // Registers run VCELL16 down to VCELL1, so the array is filled in reverse.
  for (int i = 0; i < BQ_CELLS_PER_DEVICE; i++) {
    uint16_t raw = ((uint16_t)response[BQ_RESPONSE_DATA_OFFSET + i * 2] << 8)
                 | response[BQ_RESPONSE_DATA_OFFSET + i * 2 + 1];
    voltagesV[BQ_CELLS_PER_DEVICE - 1 - i] = (int16_t)raw * BQ_VOLTS_PER_LSB;
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

// Sets the mux address and enable for one channel. Bank 0 = GPIO4/5
// (cells 1-8), bank 1 = GPIO6/7 (cells 9-16). Does not touch the ADC registers.
static void bqSetMux(uint8_t deviceId, uint8_t bank, uint8_t channel) {
  channel &= 0x7;
  uint8_t addr0   = (channel & 1) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_LO;
  uint8_t addr1   = (channel & 2) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_LO;
  uint8_t addr2   = (channel & 4) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_LO;
  uint8_t enable0 = (bank == 0) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_OFF;
  uint8_t enable1 = (bank == 1) ? BQ_GPIO_MODE_HI : BQ_GPIO_MODE_OFF;
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF1, (uint8_t)((addr1 << 3) | addr0));
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF2, (uint8_t)((enable0 << 3) | addr2));
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF3, (uint8_t)((enable1 << 3) | BQ_GPIO_MODE_ADC));  // GPIO5=ADC, GPIO6=enable1
  bqWriteSingle(deviceId, BQ_REG_GPIO_CONF4, (uint8_t)((BQ_GPIO_MODE_OFF << 3) | BQ_GPIO_MODE_ADC));  // GPIO8=off, GPIO7=ADC
}

// Reads TSREF and one GPIO ADC code. Returns false if either is invalid.
static bool bqReadMuxTemp(uint8_t deviceId, uint8_t gpioNumber, float &tempC) {
  uint16_t tsrefRaw = 0, gpioRaw = 0;
  if (!bqReadReg16(deviceId, BQ_REG_TSREF_HI,       tsrefRaw)) return false;
  if (!bqReadReg16(deviceId, bqGpioReg(gpioNumber), gpioRaw))  return false;

  int16_t tsrefCode = (int16_t)tsrefRaw;
  int16_t gpioCode  = (int16_t)gpioRaw;

  if (tsrefCode <= 0 || tsrefCode == BQ_ADC_INVALID_CODE) return false;
  if (gpioCode  <= 0 || gpioCode  == BQ_ADC_INVALID_CODE) return false;

  float ratio = (float)gpioCode / (float)tsrefCode;
  if (ratio <= NTC_RATIO_MIN || ratio >= NTC_RATIO_MAX) return false;

  float measuredOhm = NTC_PULLUP_OHM * ratio / (1.0f - ratio);
  tempC = bqNtcOhmToC(measuredOhm);
  return !isnan(tempC);
}

void bqReadTemps(uint8_t deviceId, float tempsC[BQ_CELLS_PER_DEVICE]) {
  for (uint8_t cell = 0; cell < BQ_CELLS_PER_DEVICE; cell++) {
    uint8_t bank    = (cell < BQ_CELLS_PER_MUX_BANK) ? 0 : 1;
    uint8_t channel = cell & 0x7;
    uint8_t gpioOut = (bank == 0) ? BQ_MUX_BANK0_GPIO : BQ_MUX_BANK1_GPIO;

    bqSetMux(deviceId, bank, channel);

    // Extra settle on the first channel and on the bank switchover.
    if (cell == 0 || cell == BQ_CELLS_PER_MUX_BANK) delayMicroseconds(BQ_MUX_SETTLE_US);
    delayMicroseconds(BQ_MUX_SETTLE_US);

    float tempC = NAN;
    bqReadMuxTemp(deviceId, gpioOut, tempC);
    tempsC[cell] = tempC;
  }
}

// ---------------- Init ----------------
void bqInit() {
  bqUart.begin(BQ_UART_BAUD, SERIAL_8N1);
  delay(BQ_UART_START_WAIT_MS);

  bqHardwareReset();
  delay(BQ_AFTER_RESET_MS);
  bqWake(); delay(BQ_AFTER_WAKE_MS);
  bqWake(); delay(BQ_AFTER_WAKE_MS);
  bqShutdownToActive(); delay(BQ_AFTER_STA_MS);

  bqAutoAddress();
  delay(BQ_AFTER_ADDRESS_MS);

  // Verify every stack device answers; start over if one does not.
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    uint8_t deviceId = stackIndex + 1;
    uint16_t partId = 0;
    if (bqReadReg16(deviceId, BQ_REG_PARTID, partId)) {
      Serial.printf("[init] id%d PARTID=0x%04X\n", deviceId, partId);
    } else {
      Serial.printf("[init] id%d no response — restarting\n", deviceId);
      bqInit();  // recursive retry: no limit, keeps trying until the stack answers
      return;
    }
  }

  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    bqEnableTsref(stackIndex + 1);
  }

  bqAdcInit();

  // SAFETY: the monitors shut themselves down if this sketch stops talking
  // to them for one minute. Written then read back on each stack device.
  for (uint8_t stackIndex = 0; stackIndex < BQ_STACK_DEVICE_COUNT; stackIndex++) {
    uint8_t deviceId = stackIndex + 1;
    bqWriteSingle(deviceId, BQ_REG_COMM_TIMEOUT_CONF, BQ_COMM_TIMEOUT_1MIN_SHUTDOWN);
    delay(BQ_WRITE_VERIFY_WAIT_MS);
    uint8_t readback = 0;
    if (bqReadReg8(deviceId, BQ_REG_COMM_TIMEOUT_CONF, readback)) {
      Serial.printf("[init] id%d COMM_TIMEOUT_CONF=0x%02X %s\n",
                    deviceId, readback, readback == BQ_COMM_TIMEOUT_1MIN_SHUTDOWN ? "OK" : "MISMATCH");
    } else {
      Serial.printf("[init] id%d COMM_TIMEOUT_CONF read FAIL\n", deviceId);
    }
  }
}

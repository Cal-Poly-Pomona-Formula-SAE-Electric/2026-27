/**
 * @file    main.cpp
 * @brief   Bench sketch: collects IMD voltage vs isolated-amp ADC samples and fits
 *          a line to calibrate both iso-amp channels.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software (ISO_Amp_Characterization.ino)
 * Worked on by: BERT27 restructure
 *
 * Set the HV supply to a voltage, send 's' to take a sample, repeat across the
 * range, then read the fit. The inverse fit is what goes into the BMS
 * calibration (V = (ADC - intercept) / slope).
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

struct IsoAmpSample { float imdVoltageV; int adcACounts; int adcBCounts; };
struct LinearFit    { double slope, intercept, rSquared; int sampleCount; };

// ---------------- Pins ----------------
static const int PIN_BMS_STATUS = 41;  // HIGH after 3 s so the SDC closes the negative contactor
static const int PIN_ISO_AMP_A  = 14;  // A0, isolated amp channel A
static const int PIN_ISO_AMP_B  = 15;  // A1, isolated amp channel B
// CAN3 (Bender iso175) uses the Teensy 4.1 default CAN3 pins, TX=31 RX=30,
// set by FlexCAN_T4::begin().

static constexpr int ISO_AMP_CHANNEL_A = 0;
static constexpr int ISO_AMP_CHANNEL_B = 1;

static constexpr int ADC_RESOLUTION_BITS = 12;
static constexpr int ISO_AMP_SAMPLES     = 64;  // ADC reads averaged per sample

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

static constexpr uint32_t IMD_POLL_MS        = 300;
static constexpr uint32_t SERIAL_WAIT_MS     = 2000;  // give a USB monitor time to attach, then run without one
static constexpr uint32_t CONTACTOR_DELAY_MS = 3000;  // wait before raising PIN_BMS_STATUS

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> imdCan;

// Latest IMD voltage, updated from the CAN callback.
volatile bool  isImdVoltageValid = false;
volatile float imdVoltageV       = 0.0f;

// ---------------- Sample storage ----------------
static constexpr int MAX_SAMPLES = 60;
static IsoAmpSample samples[MAX_SAMPLES];
static int          sampleCount = 0;

elapsedMillis imdPollTimerMs;

// ---------------- IMD ----------------
static inline uint16_t canReadU16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float imdDecodeVoltage(uint16_t raw) {
  return ((float)raw - IMD_VOLTAGE_RAW_OFFSET) * IMD_VOLTS_PER_BIT;
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

// ---------------- Linear least-squares: ADC = slope * V_imd + intercept ----------------
LinearFit fitCompute(int channel) {
  int n = sampleCount;
  LinearFit fit = {0, 0, 0, n};
  if (n < 2) return fit;

  double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0, sumYY = 0;
  for (int i = 0; i < n; i++) {
    double x = samples[i].imdVoltageV;
    double y = (channel == ISO_AMP_CHANNEL_A) ? samples[i].adcACounts : samples[i].adcBCounts;
    sumX += x; sumY += y; sumXY += x * y; sumXX += x * x; sumYY += y * y;
  }
  double denominator = n * sumXX - sumX * sumX;
  if (denominator == 0.0) return fit;  // every sample at the same voltage: slope undefined
  fit.slope     = (n * sumXY - sumX * sumY) / denominator;
  fit.intercept = (sumY - fit.slope * sumX) / n;

  double residualSumSq = 0, totalSumSq = 0, meanY = sumY / n;
  for (int i = 0; i < n; i++) {
    double y          = (channel == ISO_AMP_CHANNEL_A) ? samples[i].adcACounts : samples[i].adcBCounts;
    double predictedY = fit.slope * samples[i].imdVoltageV + fit.intercept;
    residualSumSq += (y - predictedY) * (y - predictedY);
    totalSumSq    += (y - meanY) * (y - meanY);
  }
  fit.rSquared = (totalSumSq > 0) ? (1.0 - residualSumSq / totalSumSq) : 1.0;
  return fit;
}

void fitPrint() {
  if (sampleCount < 2) { Serial.println("Need >= 2 samples for fit."); return; }
  LinearFit fitA = fitCompute(ISO_AMP_CHANNEL_A);
  LinearFit fitB = fitCompute(ISO_AMP_CHANNEL_B);
  Serial.println("\n=== Linear Fit  (ADC = slope * V_imd + intercept) ===");
  Serial.printf("  Chan A (pin 14): slope=%8.4f  intercept=%8.2f  R2=%.6f  n=%d\n",
                fitA.slope, fitA.intercept, fitA.rSquared, fitA.sampleCount);
  Serial.printf("  Chan B (pin 15): slope=%8.4f  intercept=%8.2f  R2=%.6f  n=%d\n",
                fitB.slope, fitB.intercept, fitB.rSquared, fitB.sampleCount);
  Serial.println("=== Inverse  (V = (ADC - intercept) / slope) ===");
  Serial.printf("  Chan A: V = (ADC - %.2f) / %.4f\n", fitA.intercept, fitA.slope);
  Serial.printf("  Chan B: V = (ADC - %.2f) / %.4f\n", fitB.intercept, fitB.slope);
}

void samplesPrint() {
  Serial.println("\n--- Samples ---");
  Serial.println("  # |   IMD_V  | ADC_A | ADC_B");
  for (int i = 0; i < sampleCount; i++) {
    Serial.printf("%3d | %8.2f | %5d | %5d\n",
                  i + 1, samples[i].imdVoltageV, samples[i].adcACounts, samples[i].adcBCounts);
  }
}

void samplesTake() {
  if (!isImdVoltageValid) { Serial.println("IMD voltage not ready yet."); return; }
  if (sampleCount >= MAX_SAMPLES) { Serial.println("Buffer full — clear with [c]."); return; }

  long sumACounts = 0, sumBCounts = 0;
  for (int i = 0; i < ISO_AMP_SAMPLES; i++) {
    sumACounts += analogRead(PIN_ISO_AMP_A);
    sumBCounts += analogRead(PIN_ISO_AMP_B);
  }
  int   adcACounts  = (int)(sumACounts / ISO_AMP_SAMPLES);
  int   adcBCounts  = (int)(sumBCounts / ISO_AMP_SAMPLES);
  float imdReadingV = imdVoltageV;

  samples[sampleCount++] = { imdReadingV, adcACounts, adcBCounts };
  Serial.printf("\nSample %d: IMD=%.2f V  |  ADC_A=%d  |  ADC_B=%d\n",
                sampleCount, imdReadingV, adcACounts, adcBCounts);
  fitPrint();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < SERIAL_WAIT_MS) {}
  Serial.println("ISO Amp Characterization -- Teensy 4.1");
  Serial.println("Set supply voltage, then:");
  Serial.println("  [s] take sample  [p] print samples  [c] clear  [f] print fit");

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

  if (Serial.available()) {
    char command = Serial.read();
    switch (command) {
      case 's': case 'S': samplesTake();                                  break;
      case 'p': case 'P': samplesPrint();                                 break;
      case 'c': case 'C': sampleCount = 0; Serial.println("Cleared.");   break;
      case 'f': case 'F': fitPrint();                                     break;
    }
  }
}

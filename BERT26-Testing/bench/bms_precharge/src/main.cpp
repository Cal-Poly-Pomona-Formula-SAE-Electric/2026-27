/**
 * @file    main.cpp
 * @brief   Bench sketch: precharge state machine test. Waits for the SDC to close
 *          the negative AIR, precharges, closes the positive AIR at 90% of pack
 *          voltage, then cross-checks the battery iso-amp against the IMD voltage.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software (BMS_Precharge.ino)
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2026 EV.5.6
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

enum BmsState {
  STATE_INIT,
  STATE_WAIT_SDC,
  STATE_PRECHARGE,
  STATE_CLOSE_POSITIVE,
  STATE_RUNNING,
  STATE_FAULT
};

// ---------------- Pins ----------------
static const int PIN_PACK_VOLTAGE_SENSE     = 14;  // A0, battery-side isolated amp (Chan A)
static const int PIN_INVERTER_VOLTAGE_SENSE = 15;  // A1, inverter-side isolated amp (Chan B)
static const int PIN_SDC_SENSE              = 20;  // Rising edge = negative contactor closed (from SDC)
static const int PIN_PRECHARGE_RELAY        = 24;  // Precharge relay low-side driver
static const int PIN_POS_AIR                = 25;  // Positive AIR low-side driver
static const int PIN_BMS_STATUS             = 41;  // BMS -> SDC: HIGH = nominal, LOW = fault
// CAN3 (Bender iso175) uses the Teensy 4.1 default CAN3 pins, TX=31 RX=30,
// set by FlexCAN_T4::begin().

// ---------------- Calibration ----------------
// Chan B linear fit from ISO_Amp_Characterization, applied to both channels:
//   V = (ADC - ISO_AMP_OFFSET_COUNTS) / ISO_AMP_COUNTS_PER_V
static constexpr float ISO_AMP_COUNTS_PER_V  = 5.2957f;
static constexpr float ISO_AMP_OFFSET_COUNTS = 1.83f;
static constexpr int   ADC_RESOLUTION_BITS   = 12;
static constexpr int   ISO_AMP_SAMPLES       = 16;  // averaged per reading

// ---------------- Precharge parameters (EV.5.6) ----------------
//   RC = 1200 ohm x 440 uF = 528 ms
//   t_90 = -RC*ln(0.1) ~= 1216 ms
//   EV.5.6.2 max = max(2 x 1216 ms, 1216 ms + 500 ms) = 2432 ms -> round up to 2500 ms
static constexpr float    PRECHARGE_DONE_FRACTION = 0.90f;  // 90% of battery voltage
static constexpr uint32_t PRECHARGE_TIMEOUT_MS    = 2500;   // EV.5.6.2
static constexpr uint32_t PRECHARGE_MIN_MS        = 200;    // faster than this means the resistor is bypassed
static constexpr uint32_t INIT_DELAY_MS           = 3000;   // boot stabilisation before PIN_BMS_STATUS rises
static constexpr uint32_t SDC_SETTLE_MS           = 1000;   // after the negative AIR closes, before precharge starts

// RUNNING-state IMD vs iso-amp cross-check: fault when the two differ by
// more than the larger of these.
static constexpr float DISAGREE_LIMIT_V        = 10.0f;
static constexpr float DISAGREE_LIMIT_FRACTION = 0.05f;  // 5% of the IMD voltage

// Below this during precharge the battery-side reading is a sensor fault, not a pack.
static constexpr float PACK_VOLTAGE_MIN_V = 10.0f;
// Below this the percent-of-pack column is printed as 0 instead of dividing.
static constexpr float PACK_VOLTAGE_PRINT_MIN_V = 0.1f;

// ---------------- Poll intervals ----------------
static constexpr uint32_t PRECHARGE_POLL_MS = 2;
static constexpr uint32_t RUNNING_POLL_MS   = 200;
static constexpr uint32_t IMD_POLL_MS       = 100;  // 10 Hz, the maximum per the iso175 spec
static constexpr uint32_t CLOSE_SETTLE_MS   = 100;  // AIR+ closed this long before the precharge relay opens
static constexpr uint32_t SERIAL_WAIT_MS    = 2000; // give a USB monitor time to attach, then run without one

// ---------------- CAN / Bender iso175 ----------------
// Redundant voltage only: isolation monitoring is the VCU's job.
static constexpr uint32_t IMD_CAN_BAUD_BPS        = 500000;
static constexpr uint32_t IMD_REQUEST_ID          = 0x22;
static constexpr uint32_t IMD_RESPONSE_ID         = 0x23;
static constexpr uint8_t  IMD_INDEX_HV_VOLTAGE    = 0x5E;
static constexpr uint8_t  IMD_REQUEST_FILL_BYTE   = 0xFF;  // unused request bytes
// iso175 "HV voltage" encoding: 0.05 V per bit around a 32128 offset, little-endian.
static constexpr float    IMD_VOLTAGE_RAW_OFFSET  = 32128.0f;
static constexpr float    IMD_VOLTS_PER_BIT       = 0.05f;
static constexpr uint16_t IMD_VOLTAGE_RAW_INVALID = 0xFFFF;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> imdCan;

// ---------------- State ----------------
BmsState bmsState = STATE_INIT;
volatile bool isNegContactorClosed = false;  // set by the SDC rising-edge interrupt

// IMD voltage, updated by the CAN receive callback.
volatile bool  isImdVoltageValid = false;
volatile float imdVoltageV       = 0.0f;

elapsedMillis initTimerMs;
elapsedMillis prechargeTimerMs;
elapsedMillis voltagePollTimerMs;
elapsedMillis imdPollTimerMs;
elapsedMillis closeSettleTimerMs;

void sdcHandleRisingEdge() {
  isNegContactorClosed = true;
}

// ---------------- Helpers ----------------
static inline uint16_t canReadU16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float imdDecodeVoltage(uint16_t raw) {
  return ((float)raw - IMD_VOLTAGE_RAW_OFFSET) * IMD_VOLTS_PER_BIT;
}

static float isoAmpCountsToV(int adcCounts) {
  return ((float)adcCounts - ISO_AMP_OFFSET_COUNTS) / ISO_AMP_COUNTS_PER_V;
}

// Averaged ADC read converted to volts.
static float isoAmpReadV(int pin, int samples = ISO_AMP_SAMPLES) {
  long sumCounts = 0;
  for (int i = 0; i < samples; i++) sumCounts += analogRead(pin);
  return isoAmpCountsToV((int)(sumCounts / samples));
}

void imdSendGet(uint8_t index) {
  CAN_message_t request{};
  request.id     = IMD_REQUEST_ID;
  request.len    = 8;
  request.buf[0] = index;
  for (int i = 1; i < 8; i++) request.buf[i] = IMD_REQUEST_FILL_BYTE;
  imdCan.write(request);
}

// SAFETY: latching fault. Opens both contactor drives and pulls the status
// line LOW so the SDC opens; only a power cycle clears it.
void faultsEnter(const char *reason) {
  digitalWrite(PIN_PRECHARGE_RELAY, LOW);
  digitalWrite(PIN_POS_AIR,         LOW);
  digitalWrite(PIN_BMS_STATUS,      LOW);  // open SDC -> AIRs de-energize
  bmsState = STATE_FAULT;
  Serial.print("[FAULT] ");
  Serial.println(reason);
  Serial.println("Power cycle required to reset.");
}

// Only the IMD voltage response is used; isolation monitoring is the VCU's job.
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

// ---------------- State handlers ----------------
void stateHandleInit() {
  if (initTimerMs >= INIT_DELAY_MS) {
    digitalWrite(PIN_BMS_STATUS, HIGH);  // BMS nominal: SDC may now close the negative AIR
    bmsState = STATE_WAIT_SDC;
    Serial.println("[BMS] INIT done -> WAIT_SDC | Pin 41 HIGH");
  }
}

void stateHandleWaitSdc() {
  if (isNegContactorClosed) {
    static bool isSettling = false;
    static elapsedMillis settleTimerMs;
    if (!isSettling) {
      isSettling = true;
      settleTimerMs = 0;
      Serial.println("[BMS] Neg contactor closed — settling 1000ms before precharge...");
    }
    if (settleTimerMs >= SDC_SETTLE_MS) {
      isSettling = false;
      // SAFETY: closes the precharge relay onto the HV bus.
      digitalWrite(PIN_PRECHARGE_RELAY, HIGH);
      prechargeTimerMs   = 0;
      voltagePollTimerMs = 0;
      imdPollTimerMs     = 0;
      bmsState = STATE_PRECHARGE;
      Serial.println("[BMS] -> PRECHARGE | Pin 24 HIGH");
    }
  }
}

void stateHandlePrecharge() {
  if (imdPollTimerMs >= IMD_POLL_MS) {
    imdPollTimerMs = 0;
    imdSendGet(IMD_INDEX_HV_VOLTAGE);
  }

  if (voltagePollTimerMs < PRECHARGE_POLL_MS) return;
  voltagePollTimerMs = 0;

  float packVoltageV = isoAmpReadV(PIN_PACK_VOLTAGE_SENSE, ISO_AMP_SAMPLES);
  uint32_t elapsedMs = prechargeTimerMs;

  // Battery voltage implausibly low: sensor or wiring problem.
  if (packVoltageV < PACK_VOLTAGE_MIN_V) {
    faultsEnter("Chan A (battery) < 10V during precharge — check sensor/wiring");
    return;
  }

  // EV.5.6.2: precharge took too long.
  if (elapsedMs > PRECHARGE_TIMEOUT_MS) {
    faultsEnter("Precharge timeout >1s (FSAE EV.5.6.2) — open/bad precharge resistor?");
    return;
  }

  // The inverter-side voltage comes from the IMD, so wait for a valid reading
  // before evaluating the threshold.
  if (!isImdVoltageValid) {
    Serial.printf("[PRECHARGE] %5lu ms | Batt=%6.2f V | IMD=pending\n", elapsedMs, packVoltageV);
    return;
  }

  float inverterVoltageV = imdVoltageV;
  Serial.printf("[PRECHARGE] %5lu ms | Batt=%6.2f V | IMD=%6.2f V | %.1f%%\n",
                elapsedMs, packVoltageV, inverterVoltageV,
                (packVoltageV > PACK_VOLTAGE_PRINT_MIN_V) ? (inverterVoltageV / packVoltageV * 100.0f) : 0.0f);

  // EV.5.6: the AIR+ may close once the inverter side reaches 90% of the pack.
  if (inverterVoltageV >= PRECHARGE_DONE_FRACTION * packVoltageV) {
    // Reached 90% suspiciously fast: bypassed resistor or shorted capacitor.
    if (elapsedMs < PRECHARGE_MIN_MS) {
      faultsEnter("Precharge too fast <50ms — bypassed resistor or shorted capacitor");
      return;
    }
    // SAFETY: closes the positive AIR, connecting the pack to the inverter.
    digitalWrite(PIN_POS_AIR, HIGH);
    closeSettleTimerMs = 0;
    bmsState = STATE_CLOSE_POSITIVE;
    Serial.printf("[BMS] Precharge complete in %lu ms (%.1f%%) -> CLOSE_POSITIVE | Pin 25 HIGH\n",
                  elapsedMs, inverterVoltageV / packVoltageV * 100.0f);
  }
}

void stateHandleClosePositive() {
  // After the settle time the AIR+ carries the load, so the precharge relay
  // can open without breaking current.
  if (closeSettleTimerMs >= CLOSE_SETTLE_MS) {
    digitalWrite(PIN_PRECHARGE_RELAY, LOW);
    voltagePollTimerMs = 0;
    imdPollTimerMs     = 0;
    bmsState = STATE_RUNNING;
    Serial.println("[BMS] Pin 24 LOW (precharge relay open) -> RUNNING");
  }
}

void stateHandleRunning() {
  // Battery-side iso-amp (Chan A) against the IMD (inverter/output side).
  if (voltagePollTimerMs >= RUNNING_POLL_MS) {
    voltagePollTimerMs = 0;

    float packVoltageV = isoAmpReadV(PIN_PACK_VOLTAGE_SENSE, ISO_AMP_SAMPLES);

    if (isImdVoltageValid) {
      float imdReadingV = imdVoltageV;
      float disagreeLimitV = fmaxf(DISAGREE_LIMIT_V, DISAGREE_LIMIT_FRACTION * imdReadingV);
      float deltaV = fabsf(packVoltageV - imdReadingV);

      Serial.printf("[RUNNING] Batt=%6.2f V | IMD=%6.2f V | delta=%.2f V\n",
                    packVoltageV, imdReadingV, deltaV);

      if (deltaV > disagreeLimitV) {
        Serial.printf("[BMS] Disagreement: Batt=%.1fV IMD=%.1fV delta=%.1fV threshold=%.1fV\n",
                      packVoltageV, imdReadingV, deltaV, disagreeLimitV);
        faultsEnter("Battery iso-amp and IMD voltage disagree beyond threshold");
        return;
      }
    } else {
      Serial.printf("[RUNNING] Batt=%6.2f V | IMD=pending\n", packVoltageV);
    }
  }

  if (imdPollTimerMs >= IMD_POLL_MS) {
    imdPollTimerMs = 0;
    imdSendGet(IMD_INDEX_HV_VOLTAGE);
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < SERIAL_WAIT_MS) {}
  Serial.println("BMS Precharge Firmware — Teensy 4.1");
  Serial.println("FSAE 2026 EV.5.6 | PM100DX 440uF | 390 Ohm | 90% threshold | 1s timeout");

  // SAFETY: contactor drives and the status line start LOW (open / fault).
  pinMode(PIN_BMS_STATUS,      OUTPUT); digitalWrite(PIN_BMS_STATUS,      LOW);
  pinMode(PIN_PRECHARGE_RELAY, OUTPUT); digitalWrite(PIN_PRECHARGE_RELAY, LOW);
  pinMode(PIN_POS_AIR,         OUTPUT); digitalWrite(PIN_POS_AIR,         LOW);

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(PIN_PACK_VOLTAGE_SENSE,     INPUT);
  pinMode(PIN_INVERTER_VOLTAGE_SENSE, INPUT);
  pinMode(PIN_SDC_SENSE,              INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SDC_SENSE), sdcHandleRisingEdge, RISING);

  imdCan.begin();
  imdCan.setBaudRate(IMD_CAN_BAUD_BPS);
  imdCan.setMaxMB(16);
  imdCan.enableFIFO();
  imdCan.enableFIFOInterrupt();
  imdCan.onReceive(imdCanReceive);

  initTimerMs = 0;
  bmsState    = STATE_INIT;
  Serial.println("[BMS] STATE_INIT — waiting 3s before signaling nominal to SDC...");
}

// ---------------- Main loop ----------------
void loop() {
  imdCan.events();

  switch (bmsState) {
    case STATE_INIT:           stateHandleInit();           break;
    case STATE_WAIT_SDC:       stateHandleWaitSdc();        break;
    case STATE_PRECHARGE:      stateHandlePrecharge();      break;
    case STATE_CLOSE_POSITIVE: stateHandleClosePositive();  break;
    case STATE_RUNNING:        stateHandleRunning();        break;
    case STATE_FAULT:          break;  // latched until a power cycle
  }
}

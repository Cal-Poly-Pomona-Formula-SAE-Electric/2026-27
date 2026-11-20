/**
 * @file    pedals.cpp
 * @brief   Reads the accelerator and brake pedals and runs the APPS plausibility and BSPC checks.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "pedals.h"

#include "pins.h"
#include "safety_limits.h"
#include "calibration.h"
#include "inverter_command.h"
#include "dashboard.h"

float pedalsScalePercentNormal(int rawValue, int rawMin, int rawMax);
float pedalsScalePercentInverted(int rawValue, int rawMin, int rawMax);
float pedalsConvertAdcCountsToV(int rawValue);
float pedalsScaleBrakePressurePercent(float voltageV);

static const int ADC_MAX_COUNTS      = (1 << ADC_RESOLUTION_BITS) - 1;
static const float ADC_REFERENCE_V = 3.3f;

static constexpr bool IS_APPS_SERIAL_LOG_ENABLED = true;

static constexpr uint32_t APPS_SERIAL_LOG_PERIOD_MS = 100;

float apps1Pct = 0.0f;
float apps2Pct = 0.0f;
float pedalPct = 0.0f;
float desiredTorqueNm = 0.0f;

bool isAppsPlausible = false;
bool isAppsDisagreeing = false;
uint32_t appsDisagreeStartMs = 0;
// Names the kind of APPS problem (channel disagreement vs. out-of-range
// sensor) for the dash fault texts; only meaningful while not plausible.
const char *appsFaultText = "APPS Disagree";
// True while brake + accelerator >25% travel are applied together; commands
// zero torque only for as long as the condition holds (no latch).
bool isBspcActive = false;
bool isAppsFilterInitialized = false;
int apps1RawLast = 0;
int apps2RawLast = 0;
int bse1RawLast = 0;
int bse2RawLast = 0;
float bse1VoltageV = 0.0f;
float bse2VoltageV = 0.0f;
float bse1Pct = 0.0f;
float bse2Pct = 0.0f;
float brakePressurePct = 0.0f;
bool isBraking = false;
bool isHardBraking = false;

// ---------------- Pedal ----------------
void pedalsUpdate() {
  int rawApps1 = analogRead(PIN_APPS_1);
  int rawApps2 = analogRead(PIN_APPS_2);
  apps1RawLast = rawApps1;
  apps2RawLast = rawApps2;

  float newApps1Pct = pedalsScalePercentInverted(rawApps1, APPS_1_RAW_MIN, APPS_1_RAW_MAX);
  float newApps2Pct = pedalsScalePercentNormal(rawApps2, APPS_2_RAW_MIN, APPS_2_RAW_MAX);

  if (!isAppsFilterInitialized) {
    apps1Pct = newApps1Pct;
    apps2Pct = newApps2Pct;
    isAppsFilterInitialized = true;
  } else {
    apps1Pct = APPS_EMA_ALPHA * newApps1Pct + (1.0f - APPS_EMA_ALPHA) * apps1Pct;
    apps2Pct = APPS_EMA_ALPHA * newApps2Pct + (1.0f - APPS_EMA_ALPHA) * apps2Pct;
  }

  bool isApps1InRange = rawApps1 >= APPS_1_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                        rawApps1 <= APPS_1_RAW_MAX + APPS_RAW_FAULT_MARGIN;
  bool isApps2InRange = rawApps2 >= APPS_2_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                        rawApps2 <= APPS_2_RAW_MAX + APPS_RAW_FAULT_MARGIN;

  float appsDifferencePct = fabsf(apps1Pct - apps2Pct);
  bool wasPlausible = isAppsPlausible;
  bool isAppsAbnormal = !isApps1InRange || !isApps2InRange ||
                        appsDifferencePct > APPS_IMPLAUSIBLE_PCT;

  if (isAppsAbnormal) {
    if (!isApps1InRange && !isApps2InRange) {
      appsFaultText = "APPS1+2 Range";
    } else if (!isApps1InRange) {
      appsFaultText = "APPS1 Range";
    } else if (!isApps2InRange) {
      appsFaultText = "APPS2 Range";
    } else {
      appsFaultText = "APPS Disagree";
    }
  }

  if (!isAppsAbnormal) {
    isAppsDisagreeing = false;
    isAppsPlausible = true;
  } else {
    uint32_t nowMs = millis();
    if (!isAppsDisagreeing) {
      isAppsDisagreeing = true;
      appsDisagreeStartMs = nowMs;
    }
    if (nowMs - appsDisagreeStartMs > APPS_IMPLAUSIBILITY_PERSIST_MS) {
      isAppsPlausible = false;
    }
  }

  float pedalTravelPct = (apps1Pct + apps2Pct) * 0.5f;

  pedalPct = pedalTravelPct;
  if (pedalPct <= PEDAL_ZERO_DEADBAND_PCT) {
    pedalPct = 0.0f;
  } else {
    // Rescale the remaining travel back up to 0-100% so output ramps
    // smoothly from zero instead of jumping past the deadband.
    pedalPct = (pedalPct - PEDAL_ZERO_DEADBAND_PCT) * 100.0f / (100.0f - PEDAL_ZERO_DEADBAND_PCT);
  }

  // BSPC: zero torque once the brake (light-braking threshold) and
  // accelerator >25% travel are applied together. Per EV.4.7/T.4.7 this latches:
  // once tripped, torque stays suppressed until the accelerator is released
  // below 5% travel, regardless of brake state. Releasing the brake alone does
  // not restore torque.
  bool wasBspcActive = isBspcActive;
  if (isBspcActive) {
    // Latched: only the accelerator returning below 5% clears the cut.
    if (pedalPct < BSPC_APPS_RESET_PCT) {
      isBspcActive = false;
    }
  } else if (isBraking && pedalPct > BSPC_APPS_THRESHOLD_PCT) {
    isBspcActive = true;
  }
  if (isBspcActive != wasBspcActive) {
    Serial.println(isBspcActive ? "[BSPC] Active; commanding zero torque"
                              : "[BSPC] Cleared");
  }

  if (isAppsPlausible) {
    desiredTorqueNm = isBspcActive ? 0.0f : (pedalPct / 100.0f) * MAX_TORQUE_NM;
  } else {
    desiredTorqueNm = 0.0f;
    if (wasPlausible) {
      Serial.print("[APPS] Implausible (");
      Serial.print(appsFaultText);
      Serial.print("): raw1=");
      Serial.print(rawApps1);
      Serial.print(" raw2=");
      Serial.print(rawApps2);
      Serial.print(" deviation=");
      Serial.print(appsDifferencePct, 1);
      Serial.print("% for >");
      Serial.print(APPS_IMPLAUSIBILITY_PERSIST_MS);
      Serial.println(" ms");
      dashShowFaultBanner(appsFaultText);
    }
    // T.4.2.10 only requires zero torque on implausibility; the car stays in
    // drive and torque returns once the channels agree again.
  }
}

void pedalsLogAppsTelemetry() {
  if (!IS_APPS_SERIAL_LOG_ENABLED) {
    return;
  }

  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs < APPS_SERIAL_LOG_PERIOD_MS) {
    return;
  }
  lastPrintMs = nowMs;

  Serial.print("[APPS] raw1=");
  Serial.print(apps1RawLast);
  Serial.print(" raw2=");
  Serial.print(apps2RawLast);
  Serial.print(" apps1=");
  Serial.print(apps1Pct, 1);
  Serial.print("% apps2=");
  Serial.print(apps2Pct, 1);
  Serial.print("% pedal=");
  Serial.print(pedalPct, 1);
  Serial.print("% torque=");
  Serial.print(desiredTorqueNm, 1);
  Serial.print("Nm cmd=");
  Serial.print(commandedTorqueNm, 1);
  Serial.print("Nm bspc=");
  Serial.print(isBspcActive ? "ACTIVE" : "clear");
  Serial.print(" plausible=");
  Serial.println(isAppsPlausible ? "YES" : "NO");
}

void pedalsUpdateBrakePressureSensors() {
  bse1RawLast = analogRead(PIN_BSE_1);
  bse2RawLast = analogRead(PIN_BSE_2);

  isBraking = (bse1RawLast > BSE_1_BRAKING_RAW_THRESHOLD) ||
            (bse2RawLast > BSE_2_BRAKING_RAW_THRESHOLD);
  isHardBraking = (bse1RawLast > BSE_1_HARD_BRAKING_RAW_THRESHOLD) ||
                (bse2RawLast > BSE_2_HARD_BRAKING_RAW_THRESHOLD);

  bse1VoltageV = pedalsConvertAdcCountsToV(bse1RawLast);
  bse2VoltageV = pedalsConvertAdcCountsToV(bse2RawLast);

  bse1Pct = pedalsScaleBrakePressurePercent(bse1VoltageV);
  bse2Pct = pedalsScaleBrakePressurePercent(bse2VoltageV);
  brakePressurePct = max(bse1Pct, bse2Pct);
}

float pedalsScalePercentNormal(int rawValue, int rawMin, int rawMax) {
  float scaledPct = (float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin);
  return constrain(scaledPct, 0.0f, 100.0f);
}

float pedalsScalePercentInverted(int rawValue, int rawMin, int rawMax) {
  float scaledPct = 100.0f - ((float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin));
  return constrain(scaledPct, 0.0f, 100.0f);
}

float pedalsConvertAdcCountsToV(int rawValue) {
  return ((float)rawValue * ADC_REFERENCE_V) / (float)ADC_MAX_COUNTS;
}

float pedalsScaleBrakePressurePercent(float voltageV) {
  float scaledPct = (voltageV - BSE_SENSOR_MIN_V) * 100.0f /
                 (BSE_SENSOR_MAX_V - BSE_SENSOR_MIN_V);
  scaledPct = constrain(scaledPct, 0.0f, 100.0f);
  return roundf(scaledPct * 100.0f) / 100.0f;
}

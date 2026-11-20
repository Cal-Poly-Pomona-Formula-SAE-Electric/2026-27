# Pedals
## Header File
```cpp
/**
 * @file    pedals.h
 * @brief   Reads the accelerator and brake pedals and runs the APPS plausibility and BSPC checks.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_PEDALS_H
#define VCU_PEDALS_H

#include <Arduino.h>

static const int ADC_RESOLUTION_BITS = 12;

extern float apps1Pct;
extern float apps2Pct;
extern float pedalPct;
extern float desiredTorqueNm;
extern bool isAppsPlausible;
extern const char *appsFaultText;
extern bool isBspcActive;
extern int apps1RawLast;
extern int apps2RawLast;
extern int bse1RawLast;
extern int bse2RawLast;
extern float brakePressurePct;
extern bool isBraking;
extern bool isHardBraking;

void pedalsUpdate();
void pedalsLogAppsTelemetry();
void pedalsUpdateBrakePressureSensors();

#endif  // VCU_PEDALS_H
```
## pedals.cpp
APPS = Accelerator Pedal Position
<!-- ### Global Variables
```cpp
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
```
The code first declares function prototypes for code compilation.</br>
```ADC_MAX_COUNTS``` sets the max value the analog-to-digital converter can return. ```pedals.h``` defines ```ADC_RESOLUTION_BITS``` as 12, which sets the max digital value as ```4095```.</br>
```ADC_REFERENCE_V``` is the ADC's reference voltage, which is used with ```ADC_MAX_COUNTS``` to convert raw ADC data into an estimated sensor voltage.</br>
```IS_APPS_SERIAL_LOG_ENABLED``` tracks the status of the APPS serial log, which is enabled on startup. ```APPS_SERIAL_LOG_PERIOD_MS``` sets the period of the APPS serial log.</br>
```apps1Pct```, ```apps2Pct```, and ```pedalPct``` are used to track the pedal position, initialized to ```0.0f```. ```desiredTorqueNm``` is also set to ```0.0f```.</br>
```isAppsPlausible``` tracks whether the sensor readings are plausible. ```isAppsDisagreeing``` tracks whether the sensor readings are outside the set range. ```appsDisagreeStartMs``` tracks the time in ms when the disagreement began.
```cpp
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
``` -->
### pedalsUpdate()
```cpp
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
```
The code reads sensor data from the APPS from two channels. If the APPS filter is not initialized, the percentages for APPS1 and APPS2 are updated with the sensor data, and the filtering status is updated to ```true```. If the filter is already initiailzed, the APPS readings are smoothed using an exponential moving average (```apps1Pct``` and ```apps2Pct```). For each raw APPS sensor reading, the code returns ```true``` if the data is between the calibrated minimum and maximum for the ADC, with an added tolerance ```APS_RAW_FAULT_MARGIN```. </br>
```appsDifferencePct``` then calculates the absolute difference bewteen the smoothed sensor readings. It then stores the current plausibility state in ```wasPlausible```, and if either of the APPS readings are out of range, or the difference between the smoothed readings is implausible, ```isAppsAbnormal``` is set ```true```.
```cpp
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
```
If the APPS readings are abnormal, it outputs the reason why.
```cpp
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
```
When the pedal readings are abnormal it gets the current time with ```millis()```. On the first abnormal reading ```isAppsDisagreeing``` is set and ```appsDisagreeStartMs``` records when it began. Each update checks how long the readings have been abnormal. After ```APPS_IMPLAUSIBILITY_PERSIST_MS``` it sets ```isAppsPlausible``` to ```false```. This prevents a brief flucuation from immediately triggering a implausibility fault, and if the readings return to normal the ```if``` statement clears the disagreement state.
```cpp
float pedalTravelPct = (apps1Pct + apps2Pct) * 0.5f;

pedalPct = pedalTravelPct;
if (pedalPct <= PEDAL_ZERO_DEADBAND_PCT) {
    pedalPct = 0.0f;
} else {
    // Rescale the remaining travel back up to 0-100% so output ramps
    // smoothly from zero instead of jumping past the deadband.
    pedalPct = (pedalPct - PEDAL_ZERO_DEADBAND_PCT) * 100.0f / (100.0f - PEDAL_ZERO_DEADBAND_PCT);
}
```
The readings from the two pedal position sensors are averaged and stored in ```pedalPct```, giving a single pedal position percentage. A zero deadband is applied--below this threshold the pedal is treated as fully released, which ignores sensor noise and slight offsets near zero. Valid values for ```pedalPct`` are rescaled so that the output starts smoothly from 0 instead of jumping.
```cpp
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
```
BSPC stands for brake system plausibility check. The code first saves the previous BSPC state. If BSPC is already active it stays latched until the accelerator pedal is released far enough (```pedalPct < BSPC_APPS_RESET_PCT```). If the brake is applied and the accelerator is above the threshold set by ```BSPC_APPS_THRESHOLD_PCT```, the safety trip is triggered. If there is a change in BSPC state it provides a status update in Serial.</br>
If APPS readings are plausible and BSPC is not active, the desired torque is calculated following the pedal position. If BSPC is active the torque is forced to ```0```. If the readings are implausible torque is forced to ```0```.
___

```cpp
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
```
This function logs telemetry for the accelerator pedal. If Serial is disabled it does nothing, disabling logging. It keeps track of the last time a message was printed, and only prints a message with a fixed interval determined by ```APPS_SERIAL_LOG_PERIOD_MS```. It then prints raw ADC data from APPS1 and APPS2, the smoothed pedal percentages from each sensor, the combined pedal percentage, calculated torque, commanded torque, BSPC status, and whether the APPS readings are plausible.
```cpp
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
```
It reads the raw ADC data from two brake pressure sensors. If either brake sensor indicates that the driver is braking by exceeding the threshold, ```isBraking``` is set ```true```. It sets ```hardBraking``` when braking is stronger than a second, higher threshold. It then converts the raw ADC data into voltage values. These are then used to calculate brake pressure percentages, and the highest of the two readings is chosen as the final brake pressure percentage.
```cpp
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
```
#### pedalsScalePercentNormal
Takes raw ADC data and linearly interpolates it from ```[rawMin, rawMax]``` to ```[0%, 100%]```. ```constrain``` ensures it is between 0 and 100,
#### pedalsScalePercentInverted
Same idea as ```pedalsScalePercentNormal``` but inverted. Linearly interpolates ```[rawMin, rawMax]``` to ```[100%, 0%]```, where a lower raw value becomes a larger percentage. 
#### pedalsConvertAdcCountsToV
Converts ADC data into voltage.
#### pedalsScaleBrakePressurePercent
Takes a brake pressure voltage and converts it into a brake pressure percentage using the sensor's min/max calibration range. It constrains the percentage to 0-100, and rounds to 2 decimal places.


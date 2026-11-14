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




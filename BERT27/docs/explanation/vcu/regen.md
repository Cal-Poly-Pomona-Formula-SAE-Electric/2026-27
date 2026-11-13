# Regen
```cpp
static constexpr uint32_t REGEN_VOLTAGE_WINDOW_MS = 10000;
// A 24-entry ring retains at least 10 seconds at the BMS's 500 ms cadence.
static constexpr uint8_t REGEN_VOLTAGE_WINDOW_CAPACITY = 24;
// The BMS monitors 96 channels, but one is a bypassed busbar channel; the pack
// has 95 active series cell banks.
static constexpr uint8_t REGEN_PACK_SERIES_BANKS = 95;
```
```REGEN_VOLTAGE_WINDOW_MS``` defines the duration of voltage regen as 10s.</br>
```REGEN_VOLTAGE_WINDOW_CAPACITY``` defines the capacity of a 24-entry ring buffer that can retain data for at least 10 seconds with a BMS cadence of 500ms.
```REGEN_PACK_SERIES_BANKS``` represents the number of active series cell banks in the BMS. It has 96 channels but only 95 monitor cell banks.
```cpp
struct RegenVoltageSampleWindow {
  float terminalV[REGEN_VOLTAGE_WINDOW_CAPACITY];
  float estimatedOpenCircuitV[REGEN_VOLTAGE_WINDOW_CAPACITY];
  uint32_t timestampMs[REGEN_VOLTAGE_WINDOW_CAPACITY];
  uint8_t count;
  uint8_t head;
};
```
This creates a struct that stores the terminal voltage, estimated open circuit voltage, the timestamp in ms, the count and the head, which will be the index of an entry in the struct.
```cpp
// Store one voltage/current pair for the regen charge-current model. This is
// called only when the two back-to-back BMS frames arrived close enough to be
// treated as one measurement.
bool regenPushVoltageSample(uint32_t nowMs) {
  // Reject missing/corrupt BMS voltage statistics. The normal operating range
  // is narrower, but these bounds avoid hiding a genuine high-cell condition.
  if (!hasCellVoltageStats || !hasBmsCurrent ||
      minCellVoltageMv < 1000 || maxCellVoltageMv > 5000 ||
      minCellVoltageMv > avgCellVoltageMv ||
      avgCellVoltageMv > maxCellVoltageMv ||
      fabsf(packCurrentA) > REGEN_PACK_CURRENT_SENSOR_MAX_A) {
    return false;
  }

  float terminalV = (float)maxCellVoltageMv / 1000.0f;
  // BMS convention: positive pack current is discharge, negative is charge.
  // Vterminal = Voc - Ipack*R, therefore Voc = Vterminal + Ipack*R.
  float estimatedOpenCircuitV = terminalV +
      packCurrentA * REGEN_CELL_BANK_RESISTANCE_OHM;
  if (estimatedOpenCircuitV < 0.0f || estimatedOpenCircuitV > 5.0f) {
    return false;
  }

  uint8_t slot = regenVoltageWindow.head;
  regenVoltageWindow.terminalV[slot] = terminalV;
  regenVoltageWindow.estimatedOpenCircuitV[slot] = estimatedOpenCircuitV;
  regenVoltageWindow.timestampMs[slot] = nowMs;
  regenVoltageWindow.head =
      (uint8_t)((slot + 1u) % REGEN_VOLTAGE_WINDOW_CAPACITY);
  if (regenVoltageWindow.count < REGEN_VOLTAGE_WINDOW_CAPACITY) {
    regenVoltageWindow.count++;
  }
  lastRegenVoltageSampleMs = nowMs;
  return true;
}
```
This function determines whether BMS data is valid to push. If cell voltage or BMS current readings are missing, the minimum cell voltage is below 1000, the maximum cell voltage is above 5000, the avg cell voltage is outside of the range [minCellVOltageMv, maxCellVoltageMv], or the current reading above the sensors max current reading, the reading is determined to be invalid and the function returns false.</br>
The next if statement checks if the open circuit voltage is outside of the range [0, 5]. If true, it is determined to be an invalid reading and the function returns false.</br>
If the reading passes both of these checks, it assigns values for terminal voltage, open circuit voltage, timestamp, and the index to ```regenVoltageWindow```. If the count is within the capacity the count increments, and the current time in ms is recoreded in ```lastRegenVoltageSampleMs```. Finally the function returns true for a valid voltage sample reading.
```cpp
// Return the worst terminal and current-corrected maximum-cell voltages still
// inside the 10-second window. Timestamp subtraction remains valid through the
// millis() rollover because all intervals are much shorter than 2^31 ms.
bool regenFindVoltageWindowMaximums(uint32_t nowMs, float &maxTerminalV,
                                    float &maxEstimatedOpenCircuitV) {
  bool isFound = false;
  for (uint8_t i = 0; i < regenVoltageWindow.count; i++) {
    if ((uint32_t)(nowMs - regenVoltageWindow.timestampMs[i]) >
        REGEN_VOLTAGE_WINDOW_MS) {
      continue;
    }

    float terminalV = regenVoltageWindow.terminalV[i];
    float estimatedOpenCircuitV =
        regenVoltageWindow.estimatedOpenCircuitV[i];
    if (!isFound || terminalV > maxTerminalV) maxTerminalV = terminalV;
    if (!isFound || estimatedOpenCircuitV > maxEstimatedOpenCircuitV) {
      maxEstimatedOpenCircuitV = estimatedOpenCircuitV;
    }
    isFound = true;
  }
  return isFound;
}
```
This function finds the maximum terminal voltage and open circuit voltage. The maximum age of any recorded sample is 10s, stored in ```REGEN_VOLTAGE_WINDOW_MS```. Starting from the most recent entry (index 0), the code loops over every sample's timestamp, and when a timestamp is detected to be older than 10s, it skips to the next sample. Otherwise, for every sample the terminal voltage and open cell voltage are checked against the current maximum for both. If a new maximum is detected the variable for it is updated, and the function returns true indicating that a max has been found.
```cpp
// Absolute allowed charge-current magnitude from the most conservative
// current-corrected cell voltage observed in the last 10 seconds. A recent
// terminal-voltage hit at the 4.180 V stop holds regen at zero for the same
// window, providing hysteresis below the BMS's 4.200 V hard cutoff.
float regenCalcCellChargeCurrentLimitA(uint32_t nowMs) {
  if (!isRegenBmsPairValid || !hasBmsCurrent || !hasCellVoltageStats ||
      (uint32_t)(nowMs - lastBmsCellVoltageRxMs) > REGEN_BMS_DATA_TIMEOUT_MS ||
      (uint32_t)(nowMs - lastBmsCurrentRxMs) > REGEN_BMS_DATA_TIMEOUT_MS ||
      regenVoltageWindow.count == 0 ||
      (uint32_t)(nowMs - lastRegenVoltageSampleMs) > REGEN_BMS_DATA_TIMEOUT_MS) {
    return 0.0f;
  }

  float maxTerminalV = 0.0f;
  float maxEstimatedOpenCircuitV = 0.0f;
  if (!regenFindVoltageWindowMaximums(nowMs, maxTerminalV,
                                      maxEstimatedOpenCircuitV)) {
    return 0.0f;
  }

  if (maxTerminalV >= REGEN_CELL_TERMINAL_STOP_V) return 0.0f;

  float voltageHeadroomV =
      REGEN_CELL_CURRENT_TARGET_V - maxEstimatedOpenCircuitV;
  if (voltageHeadroomV <= 0.0f) return 0.0f;
  return voltageHeadroomV / REGEN_CELL_BANK_RESISTANCE_OHM;
}
```
The function first checks for valid readings and the age of the current reading. If any readings are determined to be invalid, or the age of any reading exceeds the BMS timeout threshold, the function returns 0. Passing these checks, if there is no maximum voltage found, it also returns 0. If the max terminal voltage exceeeds the cell terminal voltage limit, it also returns 0. It then calculates the voltage headroom, and if the headroom is negative, the function returns 0. If the voltageheadroom is positive, the function returns as a float the current limit.
```cpp
// Returns a positive regen-torque magnitude. The caller applies it as negative
// PM100 torque. Mechanical braking remains fully independent of this request.
float regenCalculateTorqueNm() {
  if (!IS_REGEN_ENABLED || !isBraking || !hasBmsStatus || !isAppsPlausible ||
      isBspcActive || pedalPct > BSPC_APPS_RESET_PCT ||
      speedKmh <= REGEN_CUTOFF_SPEED_KMH) {
    return 0.0f;
  }

  // No fresh, synchronized BMS voltage/current pair means no safe basis for a
  // charging-current limit, so regen fails off.
  float cellChargeCurrentLimitA = regenCalcCellChargeCurrentLimitA(millis());
  if (cellChargeCurrentLimitA <= 0.0f) return 0.0f;

  // Triangular pressure curve: build regen through light braking, then taper
  // it away as the driver enters the heavy mechanical-braking region.
  float pressureFactor;
  if (brakePressurePct <= REGEN_LIGHT_BRAKE_END_PCT) {
    pressureFactor = brakePressurePct / REGEN_LIGHT_BRAKE_END_PCT;
  } else if (brakePressurePct < REGEN_HEAVY_BRAKE_END_PCT) {
    pressureFactor = (REGEN_HEAVY_BRAKE_END_PCT - brakePressurePct) /
                     (REGEN_HEAVY_BRAKE_END_PCT - REGEN_LIGHT_BRAKE_END_PCT);
  } else {
    pressureFactor = 0.0f;
  }

  // Fade smoothly from full regen at 15 km/h to zero at the 5 km/h cutoff.
  float speedFactor = constrain((speedKmh - REGEN_CUTOFF_SPEED_KMH) /
                                (REGEN_FULL_SPEED_KMH - REGEN_CUTOFF_SPEED_KMH),
                                0.0f, 1.0f);

  // Linearly reduce the allowed charging power from 20 kW at 90% SOC to zero
  // at 95% SOC. The resulting torque cap enforces P = torque*rpm/9549.
  float socFactor = constrain((REGEN_SOC_DISABLE_PCT - (float)stateOfChargePct) /
                              (REGEN_SOC_DISABLE_PCT - REGEN_SOC_DERATE_START_PCT),
                              0.0f, 1.0f);
  float torqueNm = REGEN_MAX_TORQUE_NM * pressureFactor * speedFactor;
  float absRpm = fabsf((float)motorRpm);
  if (absRpm > 1.0f) {
    // Use min-cell voltage times the 95 active series banks as a lower bound
    // on pack voltage. With unity conversion efficiency, this makes the
    // mechanical-power-to-current conversion conservative.
    float packVoltageLowerBoundV =
        ((float)minCellVoltageMv / 1000.0f) *
        (float)REGEN_PACK_SERIES_BANKS;
    if (packVoltageLowerBoundV <= 0.0f) return 0.0f;

    float regenPowerLimitKw = REGEN_MAX_POWER_KW * socFactor;
    float cellVoltagePowerLimitKw =
        packVoltageLowerBoundV * cellChargeCurrentLimitA / 1000.0f;
    if (cellVoltagePowerLimitKw < regenPowerLimitKw) {
      regenPowerLimitKw = cellVoltagePowerLimitKw;
    }

    float powerTorqueCapNm = TORQUE_PER_KW_PER_RPM *
                             regenPowerLimitKw / absRpm;
    if (torqueNm > powerTorqueCapNm) torqueNm = powerTorqueCapNm;
  }
  return constrain(torqueNm, 0.0f, REGEN_MAX_TORQUE_NM);
}
```
The function first checks if the regen is enabled. Then it checks the braking state, BMS status, plausibility, BSPC, the pedal position, and speed. If any of these values are bad it disables regen. It then checks the charge-current limit. If the charging current is not positiive it disables regen. It then creates a trianglular pressure curve for the regen using ```pressureFactor```, and smoothly fades regen from full at high speed to zero at the cutoff speed using ```speedFactor```. ```socFactor``` reduces regen as battery SOC increases towards the high end. It  establishes base torque ```torqueNm```, then checks if RPM is above 1. If this is true, it calculates ```powerTorqueCapNm``` using the pack voltage, cell charge current limit, max regen power, and SOC factor. It then returns the torque clamped between ```0.0f``` and ```REGEN_MAX_TORQUE```.

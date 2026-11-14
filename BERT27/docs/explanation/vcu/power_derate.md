# Power Derate
```cpp
// Below this rpm the power cap exceeds MAX_TORQUE_NM anyway; the guard also
// avoids dividing by zero / near-zero rpm.
static const int      POWER_DERATE_MIN_RPM       = 50;
static const uint32_t POWER_DERATE_SAMPLE_MS     = 250;    // rolling-average sample period
static const uint32_t POWER_DERATE_WINDOW_MS     = 15000;  // rolling-average window
static const uint8_t  POWER_DERATE_WINDOW_SAMPLES =
    (uint8_t)(POWER_DERATE_WINDOW_MS / POWER_DERATE_SAMPLE_MS);  // 60 samples

// Fixed-cadence rolling average over POWER_DERATE_WINDOW_SAMPLES samples. Defined
// here (ahead of any function) so the .ino auto-generated prototypes can see it.
struct RollingWindow {
  float    buf[POWER_DERATE_WINDOW_SAMPLES];
  float    sum;
  uint8_t  count;
  uint8_t  head;
};
```
```POWER_DERATE_MIN_RPM``` sets the minimum RPM for the derate. </br>
```POWER_DERATE_SAMPLE_MS``` sets the sampling rate. The VCU checks the battery and thermal signals every 250ms.</br>
```POWER_DERATE_WINDOWMS``` sets a 15-minute window for the rolling average, which serves to smooth out any brief spikes.</br>
```POWER_DERATE_WINDOW_SAMPLES``` calculates that number of samples held within a window.</br>
```struct``` ```RollingWindow``` is a ring buffer that stores:
* ```buf[POWER_DERATE_WINDOW_SAMPLES]``` - the recent sample values
* ```sum``` - the running total of values in the window 
* ```count``` - the number of valid samples in the window
* ```head``` - the index to overwrite next</br>

*```https://www.studyplan.dev/dsa/ring-buffer``` <-- useful reading on ring buffers*
```cpp
void deratePushRollingWindow(RollingWindow &w, float value);
bool derateAverageRollingWindow(const RollingWindow &w, float &out);
float derateCalcCellKw(float avgMv);

static RollingWindow minCellMvWindow = {};
static RollingWindow battMaxTempCWindow = {};
bool isPowerDerateActive = false;
// Active power limit (kW) while isPowerDerateActive; the lowest cap across all
// tripped conditions. Updated each sample by derateService().
float powerDerateKw = POWER_DERATE_KW;
```
The code declares function prototypes to allow code to compile cleanly.</br>
```minCellMvWindow``` stores a rolling history of the minimum battery cell voltage over the defined window.</br>
```battMaxTempCWindow``` stores a rolling history of the max battery temperature over the defined window.</br>
```isPowerDerateActive``` sets the status of the torque derate, which is by default ```false```.</br>
```powerDerateKw``` sets the power cap/limit in kilowatts, and it is the strictest amongst all limits.</br>
```cpp
// ---------------- Power derate ----------------
void deratePushRollingWindow(RollingWindow &w, float value) {
  if (w.count == POWER_DERATE_WINDOW_SAMPLES) {
    w.sum -= w.buf[w.head];           // evict oldest before overwriting
  } else {
    w.count++;
  }
  w.buf[w.head] = value;
  w.sum += value;
  w.head = (uint8_t)((w.head + 1) % POWER_DERATE_WINDOW_SAMPLES);
}
```
The function first checks the number of samples contained in the rolling window, and if it's equal to the number of samples set by ```POWER_DERATE_WINDOW_SAMPLES```, it removes the oldest sample. Otherwise it increments the count. It then adds the value of the rolling window to the running list of values, adds it to the sum, and moves the head to the next index. Essentially it pushes a new value into the rolling window.
```cpp
bool derateAverageRollingWindow(const RollingWindow &w, float &out) {
  if (w.count == 0) return false;
  out = w.sum / (float)w.count;
  return true;
}
```
This function returns a bool. If there is enough data to compute an average, it writes the average into ```out``` and returns ```true```. Otherwise, if the window is empty, it returns ```false```.
```cpp
// Linear low-voltage power limit (kW) for a rolling-average minimum cell of
// avgMv: POWER_DERATE_CELL_MAX_KW at CELL_FULL_MV, 0 kW at CELL_ZERO_MV, and
// linear in between. At or above CELL_FULL_MV there is no voltage derate, so
// this returns CELL_MAX_KW (callers treat >= full-voltage as "no cell trip").
float derateCalcCellKw(float avgMv) {
  if (avgMv >= (float)POWER_DERATE_CELL_FULL_MV) return POWER_DERATE_CELL_MAX_KW;
  if (avgMv <= (float)POWER_DERATE_CELL_ZERO_MV) return 0.0f;
  float frac = (avgMv - (float)POWER_DERATE_CELL_ZERO_MV) /
               (float)(POWER_DERATE_CELL_FULL_MV - POWER_DERATE_CELL_ZERO_MV);
  return POWER_DERATE_CELL_MAX_KW * frac;
}
```
This function determines the power cap depending on the minimum cell voltage over the rolling window ```avgMv```. </br>If average minimum cell voltage ```avgMv``` is above ```POWER_DERATE_CELL_FULL_MV```--which is the voltage threshold that is considered "healthy" and at which power derating is not needed--the power cap is set to the default ```POWER_RATE_CELL_MAX_KW```.</br> If ```avgMv``` is below ```POWER_DERATE_CELL_ZERO_MV```--the voltage threshold at which the pack is considered "dead/empty"--the power cap is set to ```0.0f```.</br>
Otherwise, it linearly interopolates a fraction between ```POWER_DERATE_CELL_FULL_MV``` and ```POWER_DERATE_CELL_ZERO_MV``` and returns a reduced power cap.
```cpp
// Torque that produces powerDerateKw at the given shaft speed, clamped to the
// normal max. Below POWER_DERATE_MIN_RPM the cap exceeds MAX_TORQUE_NM, so the
// limiter has no effect and we simply return the unrestricted max.
float derateCalcTorqueCapNm(int16_t rpm) {
  if (powerDerateKw <= 0.0f) return 0.0f;
  int absRpm = abs((int)rpm);
  if (absRpm < POWER_DERATE_MIN_RPM) return MAX_TORQUE_NM;
  float capNm = TORQUE_PER_KW_PER_RPM * powerDerateKw / (float)absRpm;
  if (capNm > MAX_TORQUE_NM) capNm = MAX_TORQUE_NM;
  return capNm;
}
```
This function calculates and returns the torque cap for the derate in Nm (newtons * meters). If the power cap is zero, it returns a torque cap of ```0.0f```. Otherwise, it calculates an absolute rpm value ```absRpm```. If the RPM is less than ```POWER_DERATE_MIN_RPM``` the max torque is set to the default ```MAX_TORQUE_NM```. It then calculates the max torque. If it exceeds ```MAX_TORQUE_NM``` it is clamped to the default max.
```cpp
// Samples the protection signals on a fixed cadence and updates the rolling
// averages, then sets isPowerDerateActive when any trip condition is met.
void derateService(uint32_t nowMs) {
  static uint32_t lastSampleMs = 0;
  if (nowMs - lastSampleMs < POWER_DERATE_SAMPLE_MS) return;
  lastSampleMs = nowMs;

  if (hasCellVoltageStats) deratePushRollingWindow(minCellMvWindow, (float)minCellVoltageMv);
  if (hasCellTempStats)    deratePushRollingWindow(battMaxTempCWindow, (float)maxCellTempC);

  bool isDerating = false;
  float capKw = POWER_DERATE_CELL_MAX_KW;  // lowest cap across active trips

  // Low cell voltage: linear derate on the rolling-average minimum cell.
  float avgCellMv;
  if (derateAverageRollingWindow(minCellMvWindow, avgCellMv) &&
      avgCellMv < (float)POWER_DERATE_CELL_FULL_MV) {
    isDerating = true;
    float cellLimitKw = derateCalcCellKw(avgCellMv);
    if (cellLimitKw < capKw) capKw = cellLimitKw;
  }

  if (hasPm100MotorTemp &&
      (float)pm100MotorTempTenthsC / 10.0f > POWER_DERATE_MOTOR_TEMP_C) {
    isDerating = true;
    if (POWER_DERATE_KW < capKw) capKw = POWER_DERATE_KW;
  }

  float avgBattTempC;
  if (derateAverageRollingWindow(battMaxTempCWindow, avgBattTempC) &&
      avgBattTempC > (float)POWER_DERATE_BATT_TEMP_C) {
    isDerating = true;
    if (POWER_DERATE_KW < capKw) capKw = POWER_DERATE_KW;
  }

  powerDerateKw = isDerating ? capKw : POWER_DERATE_CELL_MAX_KW;

  if (isDerating != isPowerDerateActive) {
    isPowerDerateActive = isDerating;
    if (isDerating) {
      Serial.print("[Power] Derate active; capping to ");
      Serial.print(powerDerateKw, 1);
      Serial.println(" kW");
    } else {
      Serial.println("[Power] Derate cleared; full power restored");
    }
  }
}
```
The function samples protection signals at a fixed frequency set by ```POWER_DERATE_SAMPLE_MS```. If the data is determined to be valid (```hasCellVoltageStats=True```, ```hasCellTempStats=True```), it pushes the data for voltage and temperature into the rolling window. If there is enough data to calculate an average (```derateAverageRollingWindow()=True```) and the average cell voltage is below the "healthy" treshold, derating is set active and the power cap is calculated. ```cellLimitKw``` is set as the new power cap if it's stricter than anything else.</br>
If the current motor temp is greater than the max temp ```POWER_DERATE_MOTOR_TEMP_C``` derating is turned on. It then sets the power cap equal to ```POWER_DERATE_KW``` if it is stricter than anything else.</br>
If data for the battery temperature is valid, and the average temperature exceeds the max ```POWER_DERATE_BATT_TEMP_C```, derating is turned on. It then sets the power cap to ```POWER_DERATE_KW``` if it's stricter than anything else.</br>
If derating is active, the power rate is set to the strictest cap from the previous conditions (either ```cellLimitKw``` or ```POWER_DERATE_KW```). If derating is off, it's set to the default cap of ```POWER_DERATE_CELL_MAX_KW```.</br>
The derating status is updated if a change is detected. If derating is turned on, a Serial message notifies the status change and the active power cap. If derating is turned off, it sends a message with the status change.

/**
 * @file    state_machine.cpp
 * @brief   BMS state machine: init, wait for SDC, precharge, close AIR+, running, charging, fault.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.5.6, EV.7.3
 */
#include "state_machine.h"

#include <math.h>

#include "pins.h"
#include "safety_limits.h"
#include "pack_config.h"
#include "bq79616.h"
#include "current_sensor.h"
#include "soc.h"
#include "pack_voltage.h"
#include "imd.h"
#include "shutdown_circuit.h"
#include "faults.h"
#include "charger_can.h"

static void stateHandleInit();
static void stateHandleWaitSdc();
static void stateHandlePrecharge();
static void stateHandleClosePositive();
static void stateHandleChargeSettle();
static void stateHandleRunning();
static void stateHandleChargeActive();
static void stateHandleChargeDone();
static void stateUpdateCurrentAndSoc(uint32_t &lastSocUpdateMs);

// ---------------- Timing ----------------
static constexpr uint32_t INIT_DELAY_MS       = 2000;   // CAN traffic window for mode detection
static constexpr uint32_t SDC_SETTLE_MS       = 1000;   // after the negative contactor closes
static constexpr uint32_t PRECHARGE_POLL_MS   = 2;
static constexpr uint32_t IMD_POLL_MS         = 100;
static constexpr uint32_t CLOSE_SETTLE_MS     = 100;    // AIR+ closed before the precharge relay opens
static constexpr uint32_t CHARGE_SETTLE_MS    = 2000;   // before the first charger request
static constexpr uint32_t CHARGER_MSG_MS      = 1000;   // charger stops if it hears nothing for 5 s
static constexpr uint32_t SOC_SAVE_MS         = 30000;
static constexpr uint32_t RUNNING_VOLT_MS     = 200;    // current, SOC and fault check period
static constexpr float    MS_PER_S            = 1000.0f;
static constexpr unsigned long MS_PER_MIN     = 60000UL;

// ---------------- Runtime state ----------------
BmsMode  bmsMode  = MODE_UNKNOWN;
BmsState bmsState = STATE_INIT;
bool     areContactorsClosed = false;

static elapsedMillis initTimerMs;
static elapsedMillis sdcSettleTimerMs;
static elapsedMillis prechargeTimerMs;
static elapsedMillis prechargeVoltTimerMs;
static elapsedMillis imdPollTimerMs;
static elapsedMillis closeSettleTimerMs;
static elapsedMillis chargeSettleTimerMs;
static elapsedMillis chargerMsgTimerMs;
static elapsedMillis chargerTimeoutTimerMs;
static elapsedMillis socSaveTimerMs;
static elapsedMillis runningVoltTimerMs;
static elapsedMillis postPrechargeTimerMs;
static elapsedMillis chargeCvTimerMs;     // CV-phase taper watchdog
static bool isInCvPhase = false;            // latched true once the charger reaches its CV plateau

// Started before bqInit so the INIT window includes BQ start-up time.
void stateMachineStartInitTimer() {
  initTimerMs = 0;
}

void stateMachineService() {
  switch (bmsState) {
    case STATE_INIT:           stateHandleInit();          break;
    case STATE_WAIT_SDC:       stateHandleWaitSdc();       break;
    case STATE_PRECHARGE:      stateHandlePrecharge();     break;
    case STATE_CLOSE_POSITIVE: stateHandleClosePositive(); break;
    case STATE_CHARGE_SETTLE:  stateHandleChargeSettle();  break;
    case STATE_RUNNING:        stateHandleRunning();       break;
    case STATE_CHARGE_ACTIVE:  stateHandleChargeActive();  break;
    case STATE_CHARGE_DONE:    stateHandleChargeDone();    break;
    case STATE_FAULT:          /* latched */               break;
  }
}

// Reads pack current and integrates SOC over the time since the last call.
static void stateUpdateCurrentAndSoc(uint32_t &lastSocUpdateMs) {
  uint32_t nowMs = millis();
  float elapsedS = (nowMs - lastSocUpdateMs) / MS_PER_S;
  lastSocUpdateMs = nowMs;
  runningVoltTimerMs = 0;

  packCurrentA = currentSensorReadA();
  socUpdate(elapsedS);
}

// ---------------- INIT ----------------
static void stateHandleInit() {
  if (initTimerMs >= INIT_DELAY_MS) {
    // Determine mode from CAN traffic observed during init
    if (hasSeenChargerCan) {
      bmsMode = MODE_CHARGE;
      Serial.println("[BMS] Mode: CHARGE");
    } else {
      bmsMode = MODE_DISCHARGE;
      Serial.println("[BMS] Mode: DISCHARGE");
    }
    spreadLimitV = CELL_DCIR_OHM * (PACK_POWER_MAX_W / PACK_NOMINAL_V / (float)CELLS_PARALLEL);
    Serial.printf("[BMS] Spread limit: %.3f V\n", spreadLimitV);
    socInit();
    digitalWrite(PIN_BMS_STATUS, HIGH);
    bmsState = STATE_WAIT_SDC;
    Serial.println("[BMS] INIT -> WAIT_SDC | Pin 41 HIGH");
  }
}

// ---------------- WAIT_SDC ----------------
static void stateHandleWaitSdc() {
  static bool isSettling = false;
  if (hasSdcChanged) {
    hasSdcChanged = false;
    if (isSdcHigh && !isSettling) {
      isSettling = true;
      sdcSettleTimerMs = 0;
      Serial.println("[BMS] Neg contactor closed — 200ms settle...");
    } else {
      isSettling = false;  // falling edge while waiting: abort any settle in progress
    }
  }
  if (isSettling && sdcSettleTimerMs >= SDC_SETTLE_MS) {
    isSettling = false;
    imdInvalidateReading();  // a fresh poll is required before the precharge check
    digitalWrite(PIN_PRECHARGE_RELAY, HIGH);
    prechargeTimerMs     = 0;
    prechargeVoltTimerMs = 0;
    imdPollTimerMs       = 0;
    bmsState = STATE_PRECHARGE;
    Serial.println("[BMS] -> PRECHARGE | Pin 24 HIGH");
  }
}

// ---------------- PRECHARGE ----------------
// EV.5.6.2: close AIR+ only once the inverter side has reached 90 % of the
// battery side, and not so fast that the precharge resistor must be bypassed.
static void stateHandlePrecharge() {
  bool isChargeMode = (bmsMode == MODE_CHARGE);

  if (imdPollTimerMs >= IMD_POLL_MS) {
    imdPollTimerMs = 0;
    imdRequestHvVoltage();
  }
  if (prechargeVoltTimerMs < PRECHARGE_POLL_MS) return;
  prechargeVoltTimerMs = 0;

  float batteryV = packVoltageReadV();
  uint32_t elapsedMs = prechargeTimerMs;

  if (batteryV < PRECHARGE_MIN_BATTERY_V) {
    faultsEnter(FAULT_PRECHARGE_TIMEOUT);  // treat as timeout: no battery seen
    return;
  }
  if (elapsedMs > PRECHARGE_TIMEOUT_MS) {
    faultsEnter(FAULT_PRECHARGE_TIMEOUT);
    return;
  }
  if (!isImdVoltageValid) {
    Serial.printf("[PRECHARGE] %5lu ms | Batt=%.2fV | IMD=pending\n", elapsedMs, batteryV);
    return;
  }

  float inverterV = imdVoltageV;
  float prechargePct = (batteryV > 0.1f) ? (inverterV / batteryV * 100.0f) : 0.0f;
  Serial.printf("[PRECHARGE] %5lu ms | Batt=%.2fV | IMD=%.2fV | %.1f%%\n",
                elapsedMs, batteryV, inverterV, prechargePct);

  if (inverterV >= PRECHARGE_DONE_RATIO * batteryV) {
    if (!isChargeMode && elapsedMs < PRECHARGE_MIN_MS) {
      faultsEnter(FAULT_PRECHARGE_FAST);
      return;
    }
    // SAFETY: AIR+ closes here; the precharge relay opens after CLOSE_SETTLE_MS.
    digitalWrite(PIN_POS_AIR, HIGH);
    closeSettleTimerMs = 0;
    bmsState = STATE_CLOSE_POSITIVE;
    Serial.printf("[BMS] Precharge done in %lu ms (%.1f%%) -> CLOSE_POSITIVE | Pin 25 HIGH\n",
                  elapsedMs, prechargePct);
  }

  sdcServiceFallingEdge();
}

// ---------------- CLOSE_POSITIVE ----------------
static void stateHandleClosePositive() {
  if (closeSettleTimerMs >= CLOSE_SETTLE_MS) {
    digitalWrite(PIN_PRECHARGE_RELAY, LOW);
    areContactorsClosed = true;
    if (bmsMode == MODE_CHARGE) {
      chargeSettleTimerMs = 0;
      bmsState = STATE_CHARGE_SETTLE;
      Serial.println("[BMS] -> CHARGE_SETTLE (2s before requesting charge)");
    } else {
      runningVoltTimerMs   = 0;
      imdPollTimerMs       = 0;
      socSaveTimerMs       = 0;
      postPrechargeTimerMs = 0;
      bmsState = STATE_RUNNING;
      Serial.println("[BMS] -> RUNNING");
    }
  }
  sdcServiceFallingEdge();
}

// ---------------- CHARGE_SETTLE ----------------
static void stateHandleChargeSettle() {
  if (chargeSettleTimerMs >= CHARGE_SETTLE_MS) {
    chargerMsgTimerMs     = 0;
    chargerTimeoutTimerMs = 0;
    socSaveTimerMs        = 0;
    isInCvPhase       = false;  // arm the CV watchdog fresh each session
    chargerLastRxMs = millis();
    bmsState = STATE_CHARGE_ACTIVE;
    Serial.println("[BMS] -> CHARGE_ACTIVE");
  }
  sdcServiceFallingEdge();
}

// ---------------- RUNNING ----------------
static void stateHandleRunning() {
  static uint32_t lastSocUpdateMs = 0;

  if (imdPollTimerMs >= IMD_POLL_MS) {
    imdPollTimerMs = 0;
    imdRequestHvVoltage();
  }

  // Current, SOC and fault check. BQ reads are done by cellMonitorService in the main loop.
  if (runningVoltTimerMs >= RUNNING_VOLT_MS) {
    stateUpdateCurrentAndSoc(lastSocUpdateMs);

    // Pack voltage sense cross-check: IMD_DISAGREE_MAX_COUNT consecutive hits
    // required before faulting, held off after precharge to avoid false
    // triggers while the bus settles.
    if (isImdVoltageValid && postPrechargeTimerMs >= PACK_VOLT_CHECK_HOLDOFF_MS) {
      static uint8_t imdDisagreeCount = 0;
      float isoAmpV    = packVoltageReadV();
      float thresholdV = fmaxf(IMD_DISAGREE_ABS_V, IMD_DISAGREE_FRACTION * isoAmpV);
      float diffV      = fabsf(imdVoltageV - isoAmpV);
      if (diffV > thresholdV) {
        imdDisagreeCount++;
        Serial.printf("[PACK_VOLT] DISAGREE %u/%u — IMD=%.1fV  IsoAmp=%.1fV  diff=%.1fV  thresh=%.1fV\n",
                      imdDisagreeCount, IMD_DISAGREE_MAX_COUNT, imdVoltageV, isoAmpV, diffV, thresholdV);
        if (imdDisagreeCount >= IMD_DISAGREE_MAX_COUNT) {
          delay(200);  // let serial output flush before fault
          faultsEnter(FAULT_PACK_VOLT_SENSE); return;
        }
      } else {
        imdDisagreeCount = 0;
      }
    }

    FaultCode faultCode = faultsCheckCells();
    if (faultCode != FAULT_NONE) { faultsEnter(faultCode); return; }
  }

  if (socSaveTimerMs >= SOC_SAVE_MS) { socSaveTimerMs = 0; socSave(); }

  sdcServiceFallingEdge();
}

// ---------------- CHARGE_ACTIVE ----------------
static void stateHandleChargeActive() {
  static uint32_t lastSocUpdateMs = 0;

  // The charger stops on its own if it hears nothing for 5 s.
  if (chargerMsgTimerMs >= CHARGER_MSG_MS) {
    chargerMsgTimerMs = 0;
    chargerCanSendCommand(false);
  }

  // Charger comms watchdog
  if (millis() - chargerLastRxMs > CHARGER_TIMEOUT_MS) {
    faultsEnter(FAULT_CHARGER_COMMS); return;
  }

  // Current, SOC and fault check. BQ reads are done by cellMonitorService in the main loop.
  if (runningVoltTimerMs >= RUNNING_VOLT_MS) {
    stateUpdateCurrentAndSoc(lastSocUpdateMs);  // negative = charging

    FaultCode faultCode = faultsCheckCells();
    if (faultCode != FAULT_NONE) { faultsEnter(faultCode); return; }

    // Arm the CV watchdog once the charger reaches the CV plateau (output
    // voltage at/near the configured 95s setpoint). From there the current must taper
    // below CHARGER_DONE_A within CV_TIMEOUT_MS or we fault out: guards
    // against a charger that never reports a falling current.
    if (!isInCvPhase && chargerOutputV >= (PACK_CHARGE_MAX_V - CHARGER_CV_PLATEAU_MARGIN_V)) {
      isInCvPhase = true;
      chargeCvTimerMs = 0;
      Serial.printf("[BMS] CV phase entered at %.1fV — taper watchdog armed (%lu min)\n",
                    chargerOutputV, (unsigned long)(CV_TIMEOUT_MS / MS_PER_MIN));
    }
    if (isInCvPhase && chargeCvTimerMs >= CV_TIMEOUT_MS) {
      faultsEnter(FAULT_CHARGE_TIMEOUT); return;
    }

    // CC/CV charge: the charger holds CV at the configured setpoint once the pack
    // reaches it, and output current tapers as the cells fill. Terminate only
    // when charger output current drops below the taper threshold (CV done).
    // Reaching CELL_MAX_V is the CC->CV transition, not the end of charge;
    // per-cell over-voltage stays a hard safety fault via faultsCheckCells().
    bool hasTapered = (chargerOutputA > 0.0f && chargerOutputA < CHARGER_DONE_A);
    if (hasTapered) {
      socPct = 100.0f; socSave();
      chargerCanSendCommand(true);
      digitalWrite(PIN_POS_AIR,         LOW);
      digitalWrite(PIN_PRECHARGE_RELAY, LOW);
      areContactorsClosed = false;
      bqHardwareReset();
      bmsState = STATE_CHARGE_DONE;
      Serial.printf("[BMS] Charge done (current tapered to %.2f A < %.2f A) -> CHARGE_DONE\n",
                    chargerOutputA, CHARGER_DONE_A);
      return;
    }
  }

  if (socSaveTimerMs >= SOC_SAVE_MS) { socSaveTimerMs = 0; socSave(); }
  sdcServiceFallingEdge();
}

// ---------------- CHARGE_DONE ----------------
// Re-init the BQ stack and return to WAIT_SDC for the next charge session.
static void stateHandleChargeDone() {
  static bool isBqReinitDone = false;
  if (!isBqReinitDone) { isBqReinitDone = true; bqInit(); }
  if (hasSdcChanged && !isSdcHigh) { hasSdcChanged = false; isBqReinitDone = false; sdcHandleShutdown(); }
  if (hasSdcChanged && isSdcHigh)  { hasSdcChanged = false; isBqReinitDone = false; bmsState = STATE_WAIT_SDC; }
}

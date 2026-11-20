/**
 * @file    fans.cpp
 * @brief   Sets the cooling fan PWM from motor/inverter temperature.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "fans.h"

#include "pins.h"
#include "calibration.h"
#include "inverter.h"

uint8_t fansConvertDutyPctToPwm(int requestedDutyPct);

static constexpr uint32_t FAN_PWM_FREQUENCY_HZ = 20000;
static constexpr uint8_t FAN_PWM_WRITE_MAX = 255;

// Duty cycles are set automatically from motor/inverter temps unless a
// manual override is entered over serial (digits = override, 'a' = auto).
int fan1DutyPct = FAN_DUTY_MIN_PCT;
int fan2DutyPct = FAN_DUTY_MIN_PCT;
int fan1LastDutyPct = -1;
int fan2LastDutyPct = -1;
bool isFanManualOverride = false;

// ---------------- Fans ----------------
void fansSetup() {
  pinMode(PIN_FAN_1, OUTPUT);
  pinMode(PIN_FAN_2, OUTPUT);
  analogWriteFrequency(PIN_FAN_1, FAN_PWM_FREQUENCY_HZ);
  analogWriteFrequency(PIN_FAN_2, FAN_PWM_FREQUENCY_HZ);
  fansService();
}

void fansUpdateDutyFromTemps() {
  if (isFanManualOverride) {
    return;
  }

  // Drive the fans from the hotter of the PM100 motor and inverter temps.
  bool hasTemp = false;
  float tempC = 0.0f;
  if (hasInverterTemp) {
    tempC = (float)inverterTempTenthsC / 10.0f;
    hasTemp = true;
  }
  if (hasPm100MotorTemp) {
    float motorTempC = (float)pm100MotorTempTenthsC / 10.0f;
    if (!hasTemp || motorTempC > tempC) {
      tempC = motorTempC;
    }
    hasTemp = true;
  }

  int dutyPct;
  if (!hasTemp || tempC <= FAN_TEMP_MIN_C) {
    dutyPct = FAN_DUTY_MIN_PCT;
  } else if (tempC >= FAN_TEMP_MAX_C) {
    dutyPct = FAN_DUTY_MAX_PCT;
  } else {
    float span = FAN_TEMP_MAX_C - FAN_TEMP_MIN_C;
    float fraction = (tempC - FAN_TEMP_MIN_C) / span;
    dutyPct = FAN_DUTY_MIN_PCT +
           (int)(fraction * (float)(FAN_DUTY_MAX_PCT - FAN_DUTY_MIN_PCT) + 0.5f);
  }

  fan1DutyPct = dutyPct;
  fan2DutyPct = dutyPct;
}

void fansService() {
  int duty1Pct = constrain(fan1DutyPct, 0, 100);
  int duty2Pct = constrain(fan2DutyPct, 0, 100);

  if (duty1Pct != fan1LastDutyPct) {
    fan1LastDutyPct = duty1Pct;
    analogWrite(PIN_FAN_1, fansConvertDutyPctToPwm(duty1Pct));
  }

  if (duty2Pct != fan2LastDutyPct) {
    fan2LastDutyPct = duty2Pct;
    analogWrite(PIN_FAN_2, fansConvertDutyPctToPwm(duty2Pct));
  }
}

uint8_t fansConvertDutyPctToPwm(int requestedDutyPct) {
  int dutyPct = constrain(requestedDutyPct, 0, 100);
  return (uint8_t)((dutyPct * FAN_PWM_WRITE_MAX + 50) / 100);
}

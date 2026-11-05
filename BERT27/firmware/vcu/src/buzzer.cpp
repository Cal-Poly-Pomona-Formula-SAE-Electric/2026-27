/**
 * @file    buzzer.cpp
 * @brief   Plays tone patterns on the buzzer, such as the ready-to-drive sound.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.10.7
 *
 * buzzerService() runs every loop() and steps through the active pattern
 * without blocking.
 */
#include "buzzer.h"

#include "pins.h"

void buzzerStop();

static const ToneStep TS_READY_CHIME_PATTERN[] = {
  {1568, 200, 0},
  {1047, 100, 0},
  {1319, 300, 1500}
};

const ToneStep *activeTonePattern = nullptr;
uint8_t activeToneStepCount = 0;
uint8_t activeToneIndex = 0;
bool isActiveToneInGap = false;
bool doesActiveToneRepeat = false;
uint32_t activeToneDeadlineMs = 0;

// ---------------- Buzzer ----------------
void buzzerPlayPattern(const ToneStep *pattern, uint8_t length, bool repeats) {
  if (pattern == nullptr || length == 0) {
    return;
  }

  activeTonePattern = pattern;
  activeToneStepCount = length;
  activeToneIndex = 0;
  isActiveToneInGap = false;
  doesActiveToneRepeat = repeats;
  activeToneDeadlineMs = millis() + activeTonePattern[0].onMs;
  tone(PIN_BUZZER, activeTonePattern[0].frequencyHz);
}

void buzzerService() {
  // 3-tone RTD-ready chime disabled for testing.
  // bool shouldChimeForRtd = rtdIsReadyForButton();
  //
  // if (activeTonePattern != RTD_PATTERN) {
  //   if (shouldChimeForRtd) {
  //     if (activeTonePattern != TS_READY_CHIME_PATTERN) {
  //       buzzerPlayPattern(TS_READY_CHIME_PATTERN,
  //                       sizeof(TS_READY_CHIME_PATTERN) / sizeof(TS_READY_CHIME_PATTERN[0]),
  //                       true);
  //     }
  //   } else if (activeTonePattern == TS_READY_CHIME_PATTERN) {
  //     buzzerStop();
  //     return;
  //   }
  // }

  if (activeTonePattern == nullptr) {
    return;
  }

  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - activeToneDeadlineMs) < 0) {
    return;
  }

  const ToneStep &step = activeTonePattern[activeToneIndex];

  if (!isActiveToneInGap && step.offMs > 0) {
    noTone(PIN_BUZZER);
    isActiveToneInGap = true;
    activeToneDeadlineMs = nowMs + step.offMs;
    return;
  }

  activeToneIndex++;
  isActiveToneInGap = false;

  if (activeToneIndex >= activeToneStepCount) {
    if (doesActiveToneRepeat) {
      activeToneIndex = 0;
      isActiveToneInGap = false;
      tone(PIN_BUZZER, activeTonePattern[activeToneIndex].frequencyHz);
      activeToneDeadlineMs = nowMs + activeTonePattern[activeToneIndex].onMs;
    } else {
      buzzerStop();
    }
    return;
  }

  tone(PIN_BUZZER, activeTonePattern[activeToneIndex].frequencyHz);
  activeToneDeadlineMs = nowMs + activeTonePattern[activeToneIndex].onMs;
}

void buzzerStop() {
  noTone(PIN_BUZZER);
  activeTonePattern = nullptr;
  activeToneStepCount = 0;
  activeToneIndex = 0;
  isActiveToneInGap = false;
  doesActiveToneRepeat = false;
  activeToneDeadlineMs = 0;
}

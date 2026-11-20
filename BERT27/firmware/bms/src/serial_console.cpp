/**
 * @file    serial_console.cpp
 * @brief   Serial commands for the bench: SOC=, CAL1=, CAL2=.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "serial_console.h"

#include <stdlib.h>
#include <string.h>

#include "current_sensor.h"
#include "soc.h"

static constexpr uint8_t CONSOLE_LINE_MAX = 32;

// Commands (send with newline):
//   SOC=XX.X      set state of charge (%)
//   CAL1=XX.X     record cal point 1: apply a known current, send its value in amps
//   CAL2=XX.X     record cal point 2: apply a different known current, send its value
void consoleHandleInput() {
  static char line[CONSOLE_LINE_MAX];
  static uint8_t lineLen = 0;
  while (Serial.available()) {
    char received = (char)Serial.read();
    if (received == '\n' || received == '\r') {
      if (lineLen == 0) return;
      line[lineLen] = '\0'; lineLen = 0;
      if (strncmp(line, "SOC=", 4) == 0) {
        socSet(atof(line + 4));
      } else if (strncmp(line, "CAL1=", 5) == 0) {
        currentSensorRecordCalPoint(1, atof(line + 5));
      } else if (strncmp(line, "CAL2=", 5) == 0) {
        currentSensorRecordCalPoint(2, atof(line + 5));
      } else {
        Serial.printf("[CMD] Unknown: %s\n", line);
      }
    } else if (lineLen < (uint8_t)(sizeof(line) - 1)) {
      line[lineLen++] = received;
    }
  }
}

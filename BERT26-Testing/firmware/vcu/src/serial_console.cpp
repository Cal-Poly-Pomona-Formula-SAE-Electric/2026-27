/**
 * @file    serial_console.cpp
 * @brief   Reads commands typed on the USB serial console: manual fan duty, log listing and log dumps.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "serial_console.h"

#include "fans.h"
#include "data_log.h"

void consoleHandleCommand(const char *cmd);

// Line-based serial console: 0-100 = manual fan duty, 'a' = auto fans,
// 'l' = list log files, 'd<n>' = hex-dump log session n for export.
void consoleHandleInput() {
  static char input[12];
  static uint8_t index = 0;

  while (Serial.available()) {
    char inputChar = (char)Serial.read();

    if (inputChar == '\n' || inputChar == '\r') {
      if (index > 0) {
        input[index] = '\0';
        index = 0;
        consoleHandleCommand(input);
      }
    } else if (index < sizeof(input) - 1) {
      input[index++] = inputChar;
    } else {
      index = 0;
    }
  }
}

void consoleHandleCommand(const char *cmd) {
  if (strcmp(cmd, "a") == 0 || strcmp(cmd, "A") == 0) {
    isFanManualOverride = false;
    Serial.println("[Fans] Automatic temperature control resumed");
    return;
  }

  if (strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
    logListFiles();
    return;
  }

  if ((cmd[0] == 'd' || cmd[0] == 'D') && cmd[1] >= '0' && cmd[1] <= '9') {
    logDumpFile((uint32_t)atoi(cmd + 1));
    return;
  }

  bool isAllDigits = cmd[0] != '\0';
  for (const char *cmdCursor = cmd; *cmdCursor; cmdCursor++) {
    if (*cmdCursor < '0' || *cmdCursor > '9') {
      isAllDigits = false;
      break;
    }
  }

  if (isAllDigits) {
    int value = atoi(cmd);
    // Manual override: enter 0-100 to force both fan PWM duties.
    if (value >= 0 && value <= 100) {
      isFanManualOverride = true;
      fan1DutyPct = value;
      fan2DutyPct = value;
      Serial.print("[Fans] Manual override duty set to ");
      Serial.print(value);
      Serial.println("% (send 'a' for auto)");
      return;
    }
  }

  Serial.println("[Console] Commands: 0-100 fan duty, 'a' auto fans, 'l' list logs, 'd<n>' dump log n");
}

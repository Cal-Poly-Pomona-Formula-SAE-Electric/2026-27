/**
 * @file    dashboard.cpp
 * @brief   Draws the ST7796 touchscreen dashboard and handles touch input.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "dashboard.h"

#include <SPI.h>
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>

#include "pins.h"
#include "calibration.h"
#include "vehicle_speed.h"
#include "fans.h"
#include "energy_meter.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "faults.h"
#include "power_derate.h"
#include "pedals.h"
#include "inverter_command.h"
#include "rtd.h"
#include "data_log.h"
#include "fault_text.h"

void dashReset();
char dashCheckIfTouchedRaw(int x, int y);
void dashFormatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool isValid);
void dashFormatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool isValid);
void dashFormatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool isValid);
void dashDrawLabel(int x, int y, const char *label);
void dashDrawTileFrame(int x, int y, const char *label);
void dashDrawDerateBanner(bool isForced);
uint16_t dashGetMotorTempColor(float tempC, bool isValid);
uint16_t dashGetBatteryTempColor(int tempC, bool isValid);
uint16_t dashGetCellSpreadColor(int spreadMv, bool isValid);
void dashDrawCachedText(int x, int y, int w, int h, uint8_t textSize,
                        const char *value, uint16_t valueColor,
                        char *lastValue, size_t lastValueSize,
                        uint16_t &lastColor, bool isForced);
void dashDrawDriveButtons(bool isForced);
void dashDrawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor);
void dashDrawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor);
void dashDrawShutdownStatusIndicators(bool isForced);
void dashDrawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                     int &lastState, bool isForced);
void dashDrawFaultBanner(bool isForced);

ST7796_t3 dashTft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
XPT2046_Touchscreen dashTouch(PIN_TOUCH_CS, PIN_TOUCH_IRQ);

static constexpr uint32_t FAULT_BANNER_DURATION_MS = 5000;

static constexpr float EM_TEMP_SCALE_C = 0.5f;        // degC = raw * 0.5

// Fault banner: a red strip over the drive-button row naming the most recent
// fault, shown for FAULT_BANNER_DURATION_MS after each fault event.
char faultBannerText[48] = "";
uint32_t faultBannerUntilMs = 0;
bool isFaultBannerVisible = false;
bool isFaultBannerDirty = false;

int backlightPwm = 255;
bool isScreenDirty = true;
bool isDashFullRedrawNeeded = true;

// ---------------- Display / touch ----------------
void dashSetup() {
  SPI.setMOSI(PIN_TFT_MOSI);
  SPI.setMISO(PIN_TFT_MISO);
  SPI.setSCK(PIN_TFT_SCK);

  pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_RST, HIGH);

  pinMode(PIN_TFT_LED, OUTPUT);
  analogWriteFrequency(PIN_TFT_LED, 20000);
  analogWrite(PIN_TFT_LED, backlightPwm);

  dashReset();

  dashTouch.begin();
  dashTouch.setRotation(1);

  dashTft.init(320, 480);
  dashTft.setRotation(135);
  dashTft.fillScreen(ST7735_BLACK);
  isDashFullRedrawNeeded = true;
  isScreenDirty = true;
}

void dashReset() {
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(20);
  digitalWrite(PIN_TFT_RST, LOW);
  delay(20);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(150);
}

void dashHandleTouch() {
  // RTD owns driveMode: D while ready to drive, N in every other RTD state.
  return;

  if (!dashTouch.touched()) {
    return;
  }

  TS_Point touchPoint = dashTouch.getPoint();
  char newMode = dashCheckIfTouchedRaw(touchPoint.x, touchPoint.y);
  if (newMode && newMode != driveMode) {
    rtdSetDriveMode(newMode);

    Serial.print("[Touch] Drive mode ");
    Serial.println(driveMode);
  }
}

char dashCheckIfTouchedRaw(int x, int y) {
  const int yMin = 2650;
  const int yMax = 3500;

  if (x > 950 && x < 1550 && y > yMin && y < yMax) return 'D';
  if (x > 1900 && x < 2500 && y > yMin && y < yMax) return 'N';
  if (x > 2900 && x < 3500 && y > yMin && y < yMax) return 'R';

  return 0;
}

void dashDraw() {
  // Primary-tile value caches (Zone B).
  static char lastSpeed[12] = "";   static uint16_t lastSpeedColor = ST7735_WHITE;
  static char lastDistance[20] = ""; static uint16_t lastDistanceColor = ST7735_WHITE;
  static char lastBattTemp[12] = "";    static uint16_t lastBattTempColor = ST7735_WHITE;
  static char lastMotorTemp[12] = "";    static uint16_t lastMotorTempColor = ST7735_WHITE;
  static char lastSpread[12] = "";  static uint16_t lastSpreadColor = ST7735_WHITE;
  static char lastPackV[12] = "";   static uint16_t lastPackVColor = ST7735_WHITE;
  static char lastMode[12] = "";    static uint16_t lastModeColor = ST7735_WHITE;

  // Diagnostics-strip caches (Zone C).
  static char lastPedal[24] = "";   static uint16_t lastPedalColor = ST7735_WHITE;
  static char lastTorque[16] = "";  static uint16_t lastTorqueColor = ST7735_WHITE;
  static char lastPower[16] = "";   static uint16_t lastPowerColor = ST7735_WHITE;
  static char lastSoc[24] = "";     static uint16_t lastSocColor = ST7735_WHITE;
  static char lastRpm[16] = "";     static uint16_t lastRpmColor = ST7735_WHITE;
  static char lastBms[24] = "";     static uint16_t lastBmsColor = ST7735_WHITE;
  static char lastInv[24] = "";     static uint16_t lastInvColor = ST7735_WHITE;
  static char lastCtr[16] = "";     static uint16_t lastCtrColor = ST7735_WHITE;
  static char lastMinCellV[16] = "";     static uint16_t lastMinCellVColor = ST7735_WHITE;
  static char lastMaxCellV[16] = "";     static uint16_t lastMaxCellVColor = ST7735_WHITE;
  static char lastMinCellTemp[16] = "";     static uint16_t lastMinCellTempColor = ST7735_WHITE;
  static char lastMaxCellTemp[16] = "";     static uint16_t lastMaxCellTempColor = ST7735_WHITE;
  static char lastGlv[16] = "";     static uint16_t lastGlvColor = ST7735_WHITE;
  static char lastPack[16] = "";    static uint16_t lastPackColor = ST7735_WHITE;
  static char lastInverterTemp[16] = "";    static uint16_t lastInverterTempColor = ST7735_WHITE;
  static char lastFan[16] = "";     static uint16_t lastFanColor = ST7735_WHITE;
  static char lastEmVoltage[24] = "";     static uint16_t lastEmVoltageColor = ST7735_WHITE;
  static char lastEmTemp[32] = "";     static uint16_t lastEmTempColor = ST7735_WHITE;
  static char lastEmStatus[24] = "";     static uint16_t lastEmStatusColor = ST7735_WHITE;
  static char lastFault[24] = "";   static uint16_t lastFaultColor = ST7735_WHITE;
  static char lastSd[24] = "";      static uint16_t lastSdColor = ST7735_WHITE;

  bool isForced = isDashFullRedrawNeeded;

  if (isForced) {
    dashTft.fillScreen(ST7735_BLACK);
    dashTft.setTextColor(ST7735_WHITE);
    dashTft.setTextSize(2);
    dashTft.setCursor(6, 6);
    dashTft.print("BERT26");

    // Six primary tiles: 3 columns x 2 rows.
    dashDrawTileFrame(2, 32, "SPEED");
    dashDrawTileFrame(162, 32, "BAT T");
    dashDrawTileFrame(322, 32, "MTR T");
    dashDrawTileFrame(2, 136, "SPREAD");
    dashDrawTileFrame(162, 136, "PACK V");
    dashDrawTileFrame(322, 136, "MODE");

    isDashFullRedrawNeeded = false;
  }

  char value[32];

  // ---- Header: SD/session status and power-derate banner ----
  uint16_t sdColor;
  if (!isSdCardStarted) {
    snprintf(value, sizeof(value), "SD:NO LOG:----");
    sdColor = ST7735_RED;
  } else {
    snprintf(value, sizeof(value), "SD:%s LOG:%04lu", isSdLogActive ? "OK" : "ERR",
             (unsigned long)logSessionNumber);
    sdColor = isSdLogActive ? ST7735_GREEN : ST7735_YELLOW;
  }
  dashDrawCachedText(100, 5, 196, 18, 1, value, sdColor,
                     lastSd, sizeof(lastSd), lastSdColor, isForced);
  dashDrawDerateBanner(isForced);

  // ---- Zone B: large primary tiles (value cell = tileX+6, tileY+34) ----
  snprintf(value, sizeof(value), "%.0f", speedMph);
  dashDrawCachedText(8, 66, 144, 48, 5, value, ST7735_WHITE,
                     lastSpeed, sizeof(lastSpeed), lastSpeedColor, isForced);

  snprintf(value, sizeof(value), "RUN:%.2fkm", cumulativeDistanceKm);
  dashDrawCachedText(8, 118, 144, 10, 1, value, ST7735_WHITE,
                     lastDistance, sizeof(lastDistance), lastDistanceColor, isForced);

  bool isBattTempValid = hasCellTempStats && maxCellTempC != CELL_TEMP_UNKNOWN_C;
  if (isBattTempValid) {
    snprintf(value, sizeof(value), "%dC", (int)maxCellTempC);
  } else {
    snprintf(value, sizeof(value), "--");
  }
  dashDrawCachedText(168, 66, 144, 48, 4, value,
                     dashGetBatteryTempColor((int)maxCellTempC, isBattTempValid),
                     lastBattTemp, sizeof(lastBattTemp), lastBattTempColor, isForced);

  {
    float shownMotorTempC = (float)pm100MotorTempTenthsC / 10.0f;
    if (hasPm100MotorTemp) {
      snprintf(value, sizeof(value), "%.0fC", shownMotorTempC);
    } else {
      snprintf(value, sizeof(value), "--");
    }
    dashDrawCachedText(328, 66, 144, 48, 4, value,
                       dashGetMotorTempColor(shownMotorTempC, hasPm100MotorTemp),
                       lastMotorTemp, sizeof(lastMotorTemp), lastMotorTempColor, isForced);
  }

  {
    int spreadMv = (int)maxCellVoltageMv - (int)minCellVoltageMv;
    if (hasCellVoltageStats) {
      snprintf(value, sizeof(value), "%dmV", spreadMv);
    } else {
      snprintf(value, sizeof(value), "--");
    }
    dashDrawCachedText(8, 170, 144, 48, 4, value,
                       dashGetCellSpreadColor(spreadMv, hasCellVoltageStats),
                       lastSpread, sizeof(lastSpread), lastSpreadColor, isForced);
  }

  if (hasInverterVoltageInfo) {
    snprintf(value, sizeof(value), "%.0fV", inverterDcBusVoltageV);
  } else {
    snprintf(value, sizeof(value), "--");
  }
  dashDrawCachedText(168, 170, 144, 48, 4, value,
                     hasInverterVoltageInfo ? ST7735_WHITE : ST7735_YELLOW,
                     lastPackV, sizeof(lastPackV), lastPackVColor, isForced);

  {
    bool isInDrive = (driveMode == 'D');
    const char *modeText = isInDrive ? "DRIVE" : "NEUTRAL";
    dashDrawCachedText(328, 172, 144, 28, 3, modeText,
                       isInDrive ? ST7735_GREEN : ST7735_WHITE,
                       lastMode, sizeof(lastMode), lastModeColor, isForced);
  }

  // ---- Zone C: diagnostics strip (size-1 grid) ----
  const int diagCol0XPx = 4, diagCol1XPx = 124, diagCol2XPx = 244, diagCol3XPx = 364;
  const int diagCellWidthPx = 116, diagCellHeightPx = 9;
  const int diagRow0YPx = 242, diagRow1YPx = 254, diagRow2YPx = 266, diagRow3YPx = 278, diagRow4YPx = 290, diagRow5YPx = 303;

  bool isFaultDetectionArmed = faultsIsBmsImdDetectionArmed();
  bool bms2026DisplayFault = hasBms2026FaultCode && bms2026FaultCode != 0;

  // diagRow0YPx: pedal/brake inputs, signed torque request, power, SOC/regen setting.
  // These pedal-derived diagnostics remain live while TS/RTD is off.
  snprintf(value, sizeof(value), "PED:%.0f%% BRK:%.0f%%", pedalPct, brakePressurePct);
  dashDrawCachedText(diagCol0XPx, diagRow0YPx, diagCellWidthPx, diagCellHeightPx, 1, value, isAppsPlausible ? ST7735_GREEN : ST7735_RED,
                     lastPedal, sizeof(lastPedal), lastPedalColor, isForced);

  snprintf(value, sizeof(value), "CMD:%+.1fNm", displayedCommandTorqueNm);
  dashDrawCachedText(diagCol1XPx, diagRow0YPx, diagCellWidthPx, diagCellHeightPx, 1, value, ST7735_WHITE,
                     lastTorque, sizeof(lastTorque), lastTorqueColor, isForced);

  bool hasInverterPower = hasInverterVoltageInfo && hasInverterCurrentInfo;
  if (hasInverterPower) {
    snprintf(value, sizeof(value), "PWR:%dkW", (int)roundf(powerKw));
  } else {
    snprintf(value, sizeof(value), "PWR:--");
  }
  dashDrawCachedText(diagCol2XPx, diagRow0YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasInverterPower ? ST7735_WHITE : ST7735_YELLOW,
                     lastPower, sizeof(lastPower), lastPowerColor, isForced);

  snprintf(value, sizeof(value), "SOC:%u%% REGEN:%s", (unsigned int)stateOfChargePct,
           IS_REGEN_ENABLED ? "ON" : "OFF");
  dashDrawCachedText(diagCol3XPx, diagRow0YPx, diagCellWidthPx, diagCellHeightPx, 1, value,
                     IS_REGEN_ENABLED ? ST7735_GREEN : ST7735_YELLOW,
                     lastSoc, sizeof(lastSoc), lastSocColor, isForced);

  // diagRow1YPx: RPM, BMS status, inverter status, contactor
  snprintf(value, sizeof(value), "RPM:%.0f", (float)motorRpm);
  dashDrawCachedText(diagCol0XPx, diagRow1YPx, diagCellWidthPx, diagCellHeightPx, 1, value, ST7735_WHITE,
                     lastRpm, sizeof(lastRpm), lastRpmColor, isForced);

  const char *bmsText = "WAIT";
  uint16_t bmsColor = ST7735_YELLOW;
  if (!isFaultDetectionArmed) {
    bmsText = "STARTUP"; bmsColor = ST7735_YELLOW;
  } else if (isBmsTimedOut) {
    bmsText = "TIMEOUT"; bmsColor = ST7735_RED;
  } else if (hasBmsFault || bms2026DisplayFault) {
    bmsText = "FAULT"; bmsColor = ST7735_RED;
  } else if (isBmsActive) {
    bmsText = "ACTIVE"; bmsColor = ST7735_GREEN;
  }
  snprintf(value, sizeof(value), "BMS:%s", bmsText);
  dashDrawCachedText(diagCol1XPx, diagRow1YPx, diagCellWidthPx, diagCellHeightPx, 1, value, bmsColor,
                     lastBms, sizeof(lastBms), lastBmsColor, isForced);

  {
    const char *invText;
    uint16_t invColor;
    if (!hasPm100FaultStatus) {
      invText = "--"; invColor = ST7735_YELLOW;
    } else if (hasPm100Fault) {
      invText = faultTextGetFirstPm100(pm100PostFaults, pm100RunFaults); invColor = ST7735_RED;
    } else {
      invText = "OK"; invColor = ST7735_GREEN;
    }
    snprintf(value, sizeof(value), "INV:%s", invText);
    dashDrawCachedText(diagCol2XPx, diagRow1YPx, diagCellWidthPx, diagCellHeightPx, 1, value, invColor,
                       lastInv, sizeof(lastInv), lastInvColor, isForced);
  }

  {
    bool areContactorsClosed = hasBmsStatus && (bmsFlags & 0x02) != 0;
    const char *ctrText = !hasBmsStatus ? "--" : (areContactorsClosed ? "CLOSED" : "OPEN");
    uint16_t ctrColor = !hasBmsStatus ? ST7735_YELLOW : (areContactorsClosed ? ST7735_GREEN : ST7735_RED);
    snprintf(value, sizeof(value), "CTR:%s", ctrText);
    dashDrawCachedText(diagCol3XPx, diagRow1YPx, diagCellWidthPx, diagCellHeightPx, 1, value, ctrColor,
                       lastCtr, sizeof(lastCtr), lastCtrColor, isForced);
  }

  // diagRow2YPx: cell voltage min/max, cell temp min/max
  if (hasCellVoltageStats) snprintf(value, sizeof(value), "Vmn:%.3f", minCellVoltageMv / 1000.0f);
  else snprintf(value, sizeof(value), "Vmn:--");
  dashDrawCachedText(diagCol0XPx, diagRow2YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                     lastMinCellV, sizeof(lastMinCellV), lastMinCellVColor, isForced);

  if (hasCellVoltageStats) snprintf(value, sizeof(value), "Vmx:%.3f", maxCellVoltageMv / 1000.0f);
  else snprintf(value, sizeof(value), "Vmx:--");
  dashDrawCachedText(diagCol1XPx, diagRow2YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                     lastMaxCellV, sizeof(lastMaxCellV), lastMaxCellVColor, isForced);

  if (hasCellTempStats && minCellTempC != CELL_TEMP_UNKNOWN_C) snprintf(value, sizeof(value), "Tmn:%dC", (int)minCellTempC);
  else snprintf(value, sizeof(value), "Tmn:--");
  dashDrawCachedText(diagCol2XPx, diagRow2YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                     lastMinCellTemp, sizeof(lastMinCellTemp), lastMinCellTempColor, isForced);

  if (hasCellTempStats && maxCellTempC != CELL_TEMP_UNKNOWN_C) snprintf(value, sizeof(value), "Tmx:%dC", (int)maxCellTempC);
  else snprintf(value, sizeof(value), "Tmx:--");
  dashDrawCachedText(diagCol3XPx, diagRow2YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                     lastMaxCellTemp, sizeof(lastMaxCellTemp), lastMaxCellTempColor, isForced);

  // diagRow3YPx: GLV, BMS pack voltage, inverter temp, fan
  {
    uint16_t glvColor;
    if (!hasGlvVoltage) {
      snprintf(value, sizeof(value), "GLV:--"); glvColor = ST7735_YELLOW;
    } else {
      snprintf(value, sizeof(value), "GLV:%.1fV", glvVoltageV);
      glvColor = (glvVoltageV < GLV_LOW_VOLTAGE_WARN_V) ? ST7735_RED : ST7735_WHITE;
    }
    dashDrawCachedText(diagCol0XPx, diagRow3YPx, diagCellWidthPx, diagCellHeightPx, 1, value, glvColor,
                       lastGlv, sizeof(lastGlv), lastGlvColor, isForced);
  }

  if (hasCellVoltageStats) snprintf(value, sizeof(value), "PACK:%.0fV", packVoltageV);
  else snprintf(value, sizeof(value), "PACK:--");
  dashDrawCachedText(diagCol1XPx, diagRow3YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                     lastPack, sizeof(lastPack), lastPackColor, isForced);

  if (hasInverterTemp) snprintf(value, sizeof(value), "INVT:%.1fC", inverterTempTenthsC / 10.0f);
  else snprintf(value, sizeof(value), "INVT:--");
  dashDrawCachedText(diagCol2XPx, diagRow3YPx, diagCellWidthPx, diagCellHeightPx, 1, value, hasInverterTemp ? ST7735_WHITE : ST7735_YELLOW,
                     lastInverterTemp, sizeof(lastInverterTemp), lastInverterTempColor, isForced);

  {
    int fan1ShownDutyPct = constrain(fan1DutyPct, 0, 100);
    int fan2ShownDutyPct = constrain(fan2DutyPct, 0, 100);
    snprintf(value, sizeof(value), "FAN:%d/%d", fan1ShownDutyPct, fan2ShownDutyPct);
    dashDrawCachedText(diagCol3XPx, diagRow3YPx, diagCellWidthPx, diagCellHeightPx, 1, value, ST7735_WHITE,
                       lastFan, sizeof(lastFan), lastFanColor, isForced);
  }

  // diagRow4YPx: energy meter V / T / status (shutdown chips occupy the far right)
  {
    bool hasEmComms = (hasEmMeasurement || hasEmStatus || hasEmTemps) && !isEmTimedOut;
    uint16_t liveColor = hasEmComms ? ST7735_WHITE : ST7735_YELLOW;

    if (hasEmMeasurement) snprintf(value, sizeof(value), "EMV:%.1fV", emVoltageV);
    else snprintf(value, sizeof(value), "EMV:--");
    dashDrawCachedText(diagCol0XPx, diagRow4YPx, diagCellWidthPx, diagCellHeightPx, 1, value, liveColor,
                       lastEmVoltage, sizeof(lastEmVoltage), lastEmVoltageColor, isForced);

    if (hasEmTempSummary) {
      snprintf(value, sizeof(value), "EMT:%u %d-%dC", emSensorCount,
               (int)((float)emMinTempRaw * EM_TEMP_SCALE_C),
               (int)((float)emMaxTempRaw * EM_TEMP_SCALE_C));
    } else {
      snprintf(value, sizeof(value), "EMT:--");
    }
    dashDrawCachedText(diagCol1XPx, diagRow4YPx, diagCellWidthPx, diagCellHeightPx, 1, value, liveColor,
                       lastEmTemp, sizeof(lastEmTemp), lastEmTempColor, isForced);

    uint16_t emColor;
    const char *emText = emGetStatusSummaryText(emColor);
    snprintf(value, sizeof(value), "EM:%s", emText);
    dashDrawCachedText(diagCol2XPx, diagRow4YPx, 112, diagCellHeightPx, 1, value, emColor,
                       lastEmStatus, sizeof(lastEmStatus), lastEmStatusColor, isForced);
  }

  // diagRow5YPx: wide fault text (left of the shutdown chips)
  {
    const char *faultText = faultTextGetForDashboard();
    bool isFaultActive = hasPm100Fault ||
                       (isFaultDetectionArmed &&
                        (faultsIsAnyActive() || isBmsTimedOut || isImdCanTimedOut || bms2026DisplayFault));
    uint16_t faultColor = isFaultActive ? ST7735_RED : (isFaultDetectionArmed ? ST7735_GREEN : ST7735_YELLOW);
    snprintf(value, sizeof(value), "FAULT:%s", faultText);
    dashDrawCachedText(diagCol0XPx, diagRow5YPx, 350, diagCellHeightPx, 1, value, faultColor,
                       lastFault, sizeof(lastFault), lastFaultColor, isForced);
  }

  dashDrawShutdownStatusIndicators(isForced);

  if (isFaultBannerVisible) {
    dashDrawFaultBanner(isForced);
  }
}

void dashFormatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool isValid) {
  if (!isValid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.3fV", (float)millivolts / 1000.0f);
}

void dashFormatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool isValid) {
  if (!isValid || tempC == CELL_TEMP_UNKNOWN_C) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%dC", (int)tempC);
}

void dashFormatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool isValid) {
  if (!isValid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.1fC", (float)tempTenthsC / 10.0f);
}

void dashDrawLabel(int x, int y, const char *label) {
  dashTft.setTextSize(2);
  dashTft.setTextColor(ST7735_WHITE);
  dashTft.setCursor(x, y);
  dashTft.print(label);
  dashTft.print(": ");
}

// One 156x100 primary tile: rounded border + size-2 title in the top-left.
// Drawn only on a full redraw; the value cell inside is repainted incrementally.
void dashDrawTileFrame(int x, int y, const char *label) {
  dashTft.drawRoundRect(x, y, 156, 100, 6, ST7735_WHITE);
  dashTft.setTextSize(2);
  dashTft.setTextColor(ST7735_WHITE);
  dashTft.setCursor(x + 6, y + 6);
  dashTft.print(label);
}

// Red "DERATE <n>kW" banner in the header while the power limiter is active;
// cleared to black when not. Custom drawn (filled box, not the text cache).
// Redraws on the active/inactive transition and whenever the shown (rounded)
// kW changes, so the linear voltage derate is reflected live.
void dashDrawDerateBanner(bool isForced) {
  static bool lastDerate = false;
  static int  lastShownKw = -1;
  static bool isInitialized = false;
  int shownKw = (int)roundf(powerDerateKw);
  if (!isForced && isInitialized && isPowerDerateActive == lastDerate &&
      (!isPowerDerateActive || shownKw == lastShownKw)) {
    return;
  }
  isInitialized = true;
  lastDerate = isPowerDerateActive;
  lastShownKw = shownKw;

  const int x = 300, y = 2, w = 176, h = 24;
  if (isPowerDerateActive) {
    char banner[20];
    snprintf(banner, sizeof(banner), "DERATE %dkW", shownKw);
    dashTft.fillRect(x, y, w, h, ST7735_RED);
    dashTft.setTextSize(2);
    dashTft.setTextColor(ST7735_BLACK);
    dashTft.setCursor(x + 8, y + 5);
    dashTft.print(banner);
    dashTft.setTextColor(ST7735_WHITE);
  } else {
    dashTft.fillRect(x, y, w, h, ST7735_BLACK);
  }
}

// Color bands (user-specified). Invalid/no-data is shown yellow.
uint16_t dashGetMotorTempColor(float tempC, bool isValid) {
  if (!isValid) return ST7735_YELLOW;
  if (tempC >= 95.0f) return ST7735_RED;
  if (tempC >= 65.0f) return ST7735_YELLOW;
  return ST7735_BLUE;
}

uint16_t dashGetBatteryTempColor(int tempC, bool isValid) {
  if (!isValid) return ST7735_YELLOW;
  if (tempC > 45) return ST7735_RED;
  if (tempC >= 30) return ST7735_YELLOW;
  return ST7735_BLUE;
}

uint16_t dashGetCellSpreadColor(int spreadMv, bool isValid) {
  if (!isValid) return ST7735_YELLOW;
  if (spreadMv > 200) return ST7735_RED;
  if (spreadMv >= 100) return ST7735_YELLOW;
  return ST7735_GREEN;
}

void dashDrawCachedText(int x, int y, int w, int h, uint8_t textSize,
                        const char *value, uint16_t valueColor,
                        char *lastValue, size_t lastValueSize,
                        uint16_t &lastColor, bool isForced) {
  if (!isForced && strcmp(value, lastValue) == 0 && valueColor == lastColor) {
    return;
  }

  dashTft.fillRect(x, y, w, h, ST7735_BLACK);
  dashTft.setTextSize(textSize);
  dashTft.setTextColor(valueColor);
  dashTft.setCursor(x, y);
  dashTft.print(value);
  dashTft.setTextColor(ST7735_WHITE);

  strncpy(lastValue, value, lastValueSize - 1);
  lastValue[lastValueSize - 1] = '\0';
  lastColor = valueColor;
}

void dashDrawDriveButtons(bool isForced) {
  static char lastDriveMode = 0;

  if (!isForced && driveMode == lastDriveMode) {
    return;
  }

  int y = (int)(dashTft.height() * 0.72f);
  int buttonW = 72;
  int buttonH = 54;
  int startX = 42;
  int gap = 24;

  // Clear only the button band; the cell-stats column at x>=335 shares this row.
  dashTft.fillRect(0, y - 8, 320, buttonH + 16, ST7735_BLACK);

  const char modes[2] = {'D', 'N'};
  for (int i = 0; i < 2; i++) {
    int x = startX + i * (buttonW + gap);
    bool selected = (driveMode == modes[i]);
    uint16_t color = selected ? ST7735_RED : ST7735_WHITE;

    dashTft.drawRoundRect(x, y, buttonW, buttonH, 6, color);
    dashTft.setTextSize(4);
    dashTft.setTextColor(color);
    dashTft.setCursor(x + 24, y + 12);
    dashTft.print(modes[i]);
  }

  dashTft.setTextColor(ST7735_WHITE);
  lastDriveMode = driveMode;
}

void dashDrawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor) {
  dashTft.setTextSize(2);
  dashTft.setTextColor(ST7735_WHITE);
  dashTft.setCursor(x, y);
  dashTft.print(label);
  dashTft.print(": ");
  dashTft.setTextColor(valueColor);
  dashTft.print(value);
  dashTft.setTextColor(ST7735_WHITE);
}

void dashDrawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor) {
  char text[16];
  snprintf(text, sizeof(text), "%d%s", value, units);
  dashDrawStatusText(x, y, label, text, valueColor);
}

// Shutdown-circuit status lights: GPIO HIGH = green, LOW = red.
void dashDrawShutdownStatusIndicators(bool isForced) {
  static int lastBspd = -1;
  static int lastImd = -1;
  static int lastBms = -1;

  dashDrawShutdownStatusIndicator(362, 291, "BSPD", digitalRead(PIN_BSPD_STATUS) == HIGH,
                                  lastBspd, isForced);
  dashDrawShutdownStatusIndicator(402, 291, "IMD", digitalRead(PIN_IMD_STATUS) == HIGH,
                                  lastImd, isForced);
  dashDrawShutdownStatusIndicator(442, 291, "BMS", digitalRead(PIN_BMS_STATUS) == HIGH,
                                  lastBms, isForced);
}

void dashDrawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                     int &lastState, bool isForced) {
  if (!isForced && (int)high == lastState) {
    return;
  }
  lastState = (int)high;

  const int w = 36;
  const int h = 16;
  uint16_t color = high ? ST7735_GREEN : ST7735_RED;
  dashTft.fillRoundRect(x, y, w, h, 4, color);
  dashTft.setTextSize(1);
  dashTft.setTextColor(ST7735_BLACK);
  int textW = (int)strlen(label) * 6;
  dashTft.setCursor(x + (w - textW) / 2, y + 4);
  dashTft.print(label);
  dashTft.setTextColor(ST7735_WHITE);
}

void dashShowFaultBanner(const char *text) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  if (!isFaultBannerVisible || strcmp(text, faultBannerText) != 0) {
    strncpy(faultBannerText, text, sizeof(faultBannerText) - 1);
    faultBannerText[sizeof(faultBannerText) - 1] = '\0';
    isFaultBannerDirty = true;
  }
  isFaultBannerVisible = true;
  faultBannerUntilMs = millis() + FAULT_BANNER_DURATION_MS;
  isScreenDirty = true;
}

void dashServiceFaultBanner() {
  if (!isFaultBannerVisible) {
    return;
  }

  if ((int32_t)(millis() - faultBannerUntilMs) >= 0) {
    isFaultBannerVisible = false;
    faultBannerText[0] = '\0';
    isDashFullRedrawNeeded = true;
    isScreenDirty = true;
  }
}

// Draws the fault banner over the drive-button row; the row is repainted via
// a full redraw when the banner expires.
void dashDrawFaultBanner(bool isForced) {
  if (!isForced && !isFaultBannerDirty) {
    return;
  }
  isFaultBannerDirty = false;

  int y = (int)(dashTft.height() * 0.72f) - 8;
  int h = 54 + 16;

  dashTft.fillRect(0, y, dashTft.width(), h, ST7735_RED);
  dashTft.setTextSize(2);
  dashTft.setTextColor(ST7735_WHITE);
  dashTft.setCursor(12, y + 8);
  dashTft.print("FAULT");
  dashTft.setTextSize(3);
  dashTft.setCursor(12, y + 32);
  dashTft.print(faultBannerText);
  dashTft.setTextColor(ST7735_WHITE);
}

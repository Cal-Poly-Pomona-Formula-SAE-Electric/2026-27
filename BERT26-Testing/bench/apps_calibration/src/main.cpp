/**
 * @file    main.cpp
 * @brief   Bench sketch: shows raw counts, min/max and travel % for both APPS channels
 *          (plus BSE raws) on the dash screen and serial, for pedal calibration.
 *
 * Board:   VCU (Teensy 4.1) with the ST7796 dash display
 * Original Creator: BERT26 software (APPS_Calibration.ino)
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 T.4.2.4
 */
#include <Arduino.h>
#include <SPI.h>
#include <ST7796_t3.h>

/*
  Shows, for both accelerator pedal channels (APPS 1/2):
    - the raw ADC counts, with the min/max seen since power-up
    - the pedal travel percentage each sensor reports, scaled with the same
      calibration constants and math as the VCU firmware (pedals.cpp)

  Also shows:
    - DEV: the difference between the two travel percentages (the VCU flags
      an implausibility above 10%)
    - SEP: the separation between the two raw outputs as % of ADC full scale
      (T.4.2.4 requires >=10% at any pedal position above 10% travel)

  Everything is streamed to the serial monitor at 10 Hz as well.

  Use: press the pedal slowly through full travel a few times, then read the
  min/max for each channel off the screen or serial log. Those are the values
  for APPS_1_RAW_MIN/MAX and APPS_2_RAW_MIN/MAX below and in the VCU's
  calibration.h. Update the constants here after calibrating so the
  percentages are right. (APPS 1 is the inverted channel: its raw count is
  highest at rest.)

  BSE 1/2 raws are on the bottom row for calibrating the brake thresholds.

  Send 'r' in the serial monitor to reset the min/max tracking.
*/

// ---------------- Display pins (same wiring as the VCU) ----------------
static const int PIN_TFT_MISO = 12;
static const int PIN_TFT_MOSI = 11;
static const int PIN_TFT_SCK  = 13;
static const int PIN_TFT_DC   = 8;
static const int PIN_TFT_CS   = 10;
static const int PIN_TFT_RST  = 9;
static const int PIN_TFT_LED  = 29;

// ---------------- Pedal pins (same wiring as the VCU) ----------------
static const int PIN_APPS_1 = 38;  // Inverted channel
static const int PIN_APPS_2 = 41;  // Normal channel
static const int PIN_BSE_1  = 17;
static const int PIN_BSE_2  = 16;

// Calibration constants, keep in sync with the VCU's calibration.h.
static const int APPS_1_RAW_MIN = 325;
static const int APPS_1_RAW_MAX = 2400;
static const int APPS_2_RAW_MIN = 2130;
static const int APPS_2_RAW_MAX = 4013;

static const int ADC_RESOLUTION_BITS = 12;
static const int ADC_MAX_COUNTS = (1 << ADC_RESOLUTION_BITS) - 1;

// Same threshold the VCU uses to flag the two channels as implausible.
static const float APPS_IMPLAUSIBLE_DEVIATION_PCT = 10.0f;

static const uint32_t UPDATE_PERIOD_MS = 100;  // 10 Hz screen + serial
static const uint32_t SERIAL_WAIT_MS   = 1500;  // give a USB monitor time to attach, then run without one

// ---------------- Display setup ----------------
static const int SCREEN_WIDTH_PX   = 320;
static const int SCREEN_HEIGHT_PX  = 480;
static const int SCREEN_ROTATION   = 135;  // same orientation as the VCU dash
static const int BACKLIGHT_FULL_PWM = 255;
// Reset pulse timing for the ST7796 controller.
static const uint32_t TFT_RESET_PULSE_MS  = 20;
static const uint32_t TFT_RESET_SETTLE_MS = 150;

// One cached previous value per text field on the screen, so a field is
// repainted only when its text changes.
static const int SCREEN_TEXT_SLOTS     = 10;
static const int SCREEN_TEXT_MAX_CHARS = 40;

void consoleHandleInput();
float appsScalePercentNormal(int rawValue, int rawMin, int rawMax);
float appsScalePercentInverted(int rawValue, int rawMin, int rawMax);
void screenDrawStaticLabels();
void screenDrawValues(int apps1RawCounts, int apps2RawCounts, float apps1Pct, float apps2Pct,
                      float deviationPct, float separationPct, int bse1RawCounts, int bse2RawCounts);
void screenDrawCachedText(int x, int y, int w, int h, uint8_t textSize,
                          const char *text, uint16_t color, int slot);

ST7796_t3 screenTft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

int apps1MinCounts = ADC_MAX_COUNTS;
int apps1MaxCounts = 0;
int apps2MinCounts = ADC_MAX_COUNTS;
int apps2MaxCounts = 0;

void setup() {
  Serial.begin(115200);

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(PIN_APPS_1, INPUT);
  pinMode(PIN_APPS_2, INPUT);
  pinMode(PIN_BSE_1, INPUT);
  pinMode(PIN_BSE_2, INPUT);

  SPI.setMOSI(PIN_TFT_MOSI);
  SPI.setMISO(PIN_TFT_MISO);
  SPI.setSCK(PIN_TFT_SCK);

  pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(TFT_RESET_PULSE_MS);
  digitalWrite(PIN_TFT_RST, LOW);
  delay(TFT_RESET_PULSE_MS);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(TFT_RESET_SETTLE_MS);

  pinMode(PIN_TFT_LED, OUTPUT);
  analogWrite(PIN_TFT_LED, BACKLIGHT_FULL_PWM);

  screenTft.init(SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX);
  screenTft.setRotation(SCREEN_ROTATION);
  screenTft.fillScreen(ST7735_BLACK);

  screenDrawStaticLabels();

  while (!Serial && millis() < SERIAL_WAIT_MS) {}
  Serial.println("BERT26 APPS calibration");
  Serial.println("Send 'r' to reset min/max");
}

void loop() {
  consoleHandleInput();

  static uint32_t lastUpdateMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }
  lastUpdateMs = nowMs;

  int apps1RawCounts = analogRead(PIN_APPS_1);
  int apps2RawCounts = analogRead(PIN_APPS_2);
  int bse1RawCounts = analogRead(PIN_BSE_1);
  int bse2RawCounts = analogRead(PIN_BSE_2);

  if (apps1RawCounts < apps1MinCounts) apps1MinCounts = apps1RawCounts;
  if (apps1RawCounts > apps1MaxCounts) apps1MaxCounts = apps1RawCounts;
  if (apps2RawCounts < apps2MinCounts) apps2MinCounts = apps2RawCounts;
  if (apps2RawCounts > apps2MaxCounts) apps2MaxCounts = apps2RawCounts;

  // Same travel scaling as the VCU (unfiltered: no EMA here).
  float apps1Pct = appsScalePercentInverted(apps1RawCounts, APPS_1_RAW_MIN, APPS_1_RAW_MAX);
  float apps2Pct = appsScalePercentNormal(apps2RawCounts, APPS_2_RAW_MIN, APPS_2_RAW_MAX);
  float deviationPct = fabsf(apps1Pct - apps2Pct);
  float separationPct = fabsf((float)(apps2RawCounts - apps1RawCounts)) * 100.0f / (float)ADC_MAX_COUNTS;

  Serial.print("[APPS] raw1=");
  Serial.print(apps1RawCounts);
  Serial.print(" (min ");
  Serial.print(apps1MinCounts);
  Serial.print(" max ");
  Serial.print(apps1MaxCounts);
  Serial.print(") pct1=");
  Serial.print(apps1Pct, 1);
  Serial.print("% raw2=");
  Serial.print(apps2RawCounts);
  Serial.print(" (min ");
  Serial.print(apps2MinCounts);
  Serial.print(" max ");
  Serial.print(apps2MaxCounts);
  Serial.print(") pct2=");
  Serial.print(apps2Pct, 1);
  Serial.print("% dev=");
  Serial.print(deviationPct, 1);
  Serial.print("% sep=");
  Serial.print(separationPct, 1);
  Serial.print("% | [BSE] raw1=");
  Serial.print(bse1RawCounts);
  Serial.print(" raw2=");
  Serial.println(bse2RawCounts);

  screenDrawValues(apps1RawCounts, apps2RawCounts, apps1Pct, apps2Pct,
                   deviationPct, separationPct, bse1RawCounts, bse2RawCounts);
}

void consoleHandleInput() {
  while (Serial.available()) {
    char command = (char)Serial.read();
    if (command == 'r' || command == 'R') {
      apps1MinCounts = ADC_MAX_COUNTS;
      apps1MaxCounts = 0;
      apps2MinCounts = ADC_MAX_COUNTS;
      apps2MaxCounts = 0;
      Serial.println("[APPS] min/max reset");
    }
  }
}

float appsScalePercentNormal(int rawValue, int rawMin, int rawMax) {
  float scaledPct = (float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin);
  return constrain(scaledPct, 0.0f, 100.0f);
}

float appsScalePercentInverted(int rawValue, int rawMin, int rawMax) {
  float scaledPct = 100.0f - ((float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin));
  return constrain(scaledPct, 0.0f, 100.0f);
}

void screenDrawStaticLabels() {
  screenTft.setTextColor(ST7735_WHITE);
  screenTft.setTextSize(2);
  screenTft.setCursor(14, 10);
  screenTft.print("APPS Calibration");

  screenTft.setTextColor(ST7735_YELLOW);
  screenTft.setCursor(14, 45);
  screenTft.print("APPS 1 (pin 38, inverted)");
  screenTft.setCursor(14, 130);
  screenTft.print("APPS 2 (pin 41)");

  screenTft.setTextColor(ST7735_WHITE);
  screenTft.setCursor(14, 250);
  screenTft.print("BSE raw:");

  screenTft.setTextSize(1);
  screenTft.setCursor(14, 300);
  screenTft.print("Serial 115200, 'r' resets min/max. DEV>10% = implausible, SEP<10% above 10% travel = T.4.2.4");
}

void screenDrawValues(int apps1RawCounts, int apps2RawCounts, float apps1Pct, float apps2Pct,
                      float deviationPct, float separationPct, int bse1RawCounts, int bse2RawCounts) {
  char text[SCREEN_TEXT_MAX_CHARS];
  uint16_t deviationColor = (deviationPct > APPS_IMPLAUSIBLE_DEVIATION_PCT) ? ST7735_RED : ST7735_GREEN;

  // APPS 1: raw, travel %, min/max
  snprintf(text, sizeof(text), "%4d", apps1RawCounts);
  screenDrawCachedText(14, 70, 110, 30, 4, text, ST7735_GREEN, 0);
  snprintf(text, sizeof(text), "%5.1f%%", apps1Pct);
  screenDrawCachedText(140, 70, 170, 30, 4, text, deviationColor, 1);
  snprintf(text, sizeof(text), "min %4d", apps1MinCounts);
  screenDrawCachedText(330, 63, 140, 16, 2, text, ST7735_WHITE, 2);
  snprintf(text, sizeof(text), "max %4d", apps1MaxCounts);
  screenDrawCachedText(330, 85, 140, 16, 2, text, ST7735_WHITE, 3);

  // APPS 2: raw, travel %, min/max
  snprintf(text, sizeof(text), "%4d", apps2RawCounts);
  screenDrawCachedText(14, 155, 110, 30, 4, text, ST7735_GREEN, 4);
  snprintf(text, sizeof(text), "%5.1f%%", apps2Pct);
  screenDrawCachedText(140, 155, 170, 30, 4, text, deviationColor, 5);
  snprintf(text, sizeof(text), "min %4d", apps2MinCounts);
  screenDrawCachedText(330, 148, 140, 16, 2, text, ST7735_WHITE, 6);
  snprintf(text, sizeof(text), "max %4d", apps2MaxCounts);
  screenDrawCachedText(330, 170, 140, 16, 2, text, ST7735_WHITE, 7);

  // T.4.2.4: channel deviation (implausibility) and raw separation margin.
  snprintf(text, sizeof(text), "DEV %5.1f%%  SEP %5.1f%%", deviationPct, separationPct);
  screenDrawCachedText(14, 215, 360, 18, 2, text, deviationColor, 8);

  snprintf(text, sizeof(text), "1: %4d   2: %4d", bse1RawCounts, bse2RawCounts);
  screenDrawCachedText(130, 250, 280, 18, 2, text, ST7735_CYAN, 9);
}

// Redraws a text field only when its value changes, to avoid flicker.
// slot selects one of the cached previous-value buffers.
void screenDrawCachedText(int x, int y, int w, int h, uint8_t textSize,
                          const char *text, uint16_t color, int slot) {
  static char lastText[SCREEN_TEXT_SLOTS][SCREEN_TEXT_MAX_CHARS];

  if (slot < 0 || slot >= SCREEN_TEXT_SLOTS) {
    return;
  }

  if (strcmp(text, lastText[slot]) == 0) {
    return;
  }

  screenTft.fillRect(x, y, w, h, ST7735_BLACK);
  screenTft.setTextSize(textSize);
  screenTft.setTextColor(color);
  screenTft.setCursor(x, y);
  screenTft.print(text);

  strncpy(lastText[slot], text, sizeof(lastText[slot]) - 1);
  lastText[slot][sizeof(lastText[slot]) - 1] = '\0';
}

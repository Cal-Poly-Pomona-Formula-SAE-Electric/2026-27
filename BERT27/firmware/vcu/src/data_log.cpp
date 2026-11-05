/**
 * @file    data_log.cpp
 * @brief   Writes the measured, derived and cell log files to the SD card and lists or dumps them over serial.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "data_log.h"

#include "pins.h"
#include "can_bytes.h"
#include "vehicle_speed.h"
#include "fans.h"
#include "imu.h"
#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "regen.h"
#include "pedals.h"
#include "inverter_command.h"
#include "rtd.h"
#include "can_trace.h"
#include "can_trace_imd.h"
#include "dashboard.h"

static constexpr uint16_t BMS_CELL_COUNT = 96;

// ---------------- Data logging ----------------
// Three binary files are created for each session: MEASnnnn.BIN contains values
// received directly from sensors/CAN, DERInnnn.BIN contains calculated/control
// values, and CELLnnnn.BIN contains every requested cell voltage/temperature.
// Every telemetry field is an IEEE-754 32-bit float; headers use integer framing.
static constexpr uint32_t LOG_FAST_PERIOD_MS   = 10;
static constexpr uint32_t LOG_MEDIUM_PERIOD_MS = 100;
static constexpr uint32_t LOG_SLOW_PERIOD_MS   = 1000;
static constexpr uint32_t LOG_SD_FLUSH_PERIOD_MS = 1000;
static constexpr uint8_t LOG_RECORD_FAST   = 0x01;
static constexpr uint8_t LOG_RECORD_MEDIUM = 0x02;
static constexpr uint8_t LOG_RECORD_SLOW   = 0x03;
static constexpr char MEASURED_LOG_MAGIC[8] = {'V','C','U','M','E','A','S','1'};
static constexpr char DERIVED_LOG_MAGIC[8]  = {'V','C','U','D','E','R','I','1'};
static constexpr char CELL_LOG_MAGIC[8]     = {'V','C','U','C','E','L','L','1'};

struct __attribute__((packed)) LogFileHeader {
  char magic[8];
  uint32_t session;
  uint8_t fastSize;
  uint8_t mediumSize;
  uint8_t slowSize;
  uint8_t imuSize;          // 0 in logs from firmware without an IMU
};

struct __attribute__((packed)) MeasuredFastRecord {
  float type, timeSeconds;
  float apps1AdcCounts, apps2AdcCounts, bse1AdcCounts, bse2AdcCounts;
  float hvCurrentAdcCounts, motorRpm, busVoltageV, busCurrentA;
};

struct __attribute__((packed)) DerivedFastRecord {
  float type, timeSeconds;
  float brakePressurePct, commandedTorqueNm, regenTorqueNm;
  float speedKmh, cumulativeDistanceKm, statusFlags;
};

struct __attribute__((packed)) MeasuredMediumRecord {
  float type, timeSeconds;
  float glvVoltageV, inverterTempC, motorTempC, socPct, batteryTempC;
  float bmsFlags, bmsFaultCode, imdWarnAlarms, pm100PostFaults, pm100RunFaults;
};

struct __attribute__((packed)) DerivedMediumRecord {
  float type, timeSeconds;
  float apps1Pct, apps2Pct, pedalPct, desiredTorqueNm, powerKw, speedMph;
  float fanDutyPct, rtdState;
};

struct __attribute__((packed)) MeasuredSlowRecord {
  float type, timeSeconds;
  float minCellVoltageV, maxCellVoltageV, avgCellVoltageV;
  float minCellTempC, maxCellTempC, avgCellTempC;
  float imdRIsoKohm, imdRIsoStatus, validFlags;
};

struct __attribute__((packed)) CellLogFileHeader {
  char magic[8];
  uint32_t session;
  uint16_t recordSize;
  uint16_t cellCount;
};

// type: 1 = voltage (value in V), 2 = temperature (value in degC).
struct __attribute__((packed)) CellLogRecord {
  float type, timeSeconds, cellIndex, value;
};

uint32_t logFindNextSessionNumber();
uint32_t logFindMaxSessionInDir(File dir);
uint16_t logGetStatusFlags();
void logWriteFastRecord(uint32_t nowMs);
void logWriteMediumRecord(uint32_t nowMs);
void logWriteSlowRecord(uint32_t nowMs);
void logFlushSdBuffer(File &file, bool measuredLog);
void logAppendCell(const void *data, size_t len);
void logFlushCellBuffer();
void logFlushAllSdBuffers();
void logListDir(File dir);

int hvCurrentRawLast = 0;

// Each log is staged independently in RAM and written in 512-byte chunks.
File sdMeasuredLogFile;
File sdDerivedLogFile;
File sdCellLogFile;
bool isSdCardStarted = false;
bool isSdLogActive = false;
uint32_t logSessionNumber = 0;
uint8_t measuredLogBuffer[512];
uint8_t derivedLogBuffer[512];
uint8_t cellLogBuffer[512];
size_t measuredLogBufferLen = 0;
size_t derivedLogBufferLen = 0;
size_t cellLogBufferLen = 0;

// ---------------- Data logging ----------------
void logSetup() {
  isSdCardStarted = SD.begin(BUILTIN_SDCARD);
  if (!isSdCardStarted) {
    Serial.println("[Log] No SD card found; data logging disabled");
    return;
  }

  logSessionNumber = logFindNextSessionNumber();

  char measuredName[16];
  char derivedName[16];
  char cellName[16];
  snprintf(measuredName, sizeof(measuredName), "MEAS%04lu.BIN", (unsigned long)logSessionNumber);
  snprintf(derivedName, sizeof(derivedName), "DERI%04lu.BIN", (unsigned long)logSessionNumber);
  snprintf(cellName, sizeof(cellName), "CELL%04lu.BIN", (unsigned long)logSessionNumber);

  LogFileHeader measuredHeader;
  memcpy(measuredHeader.magic, MEASURED_LOG_MAGIC, sizeof(measuredHeader.magic));
  measuredHeader.session = logSessionNumber;
  measuredHeader.fastSize = sizeof(MeasuredFastRecord);
  measuredHeader.mediumSize = sizeof(MeasuredMediumRecord);
  measuredHeader.slowSize = sizeof(MeasuredSlowRecord);
  measuredHeader.imuSize = sizeof(MeasuredImuRecord);

  LogFileHeader derivedHeader;
  memcpy(derivedHeader.magic, DERIVED_LOG_MAGIC, sizeof(derivedHeader.magic));
  derivedHeader.session = logSessionNumber;
  derivedHeader.fastSize = sizeof(DerivedFastRecord);
  derivedHeader.mediumSize = sizeof(DerivedMediumRecord);
  derivedHeader.slowSize = 0;
  derivedHeader.imuSize = 0;

  CellLogFileHeader cellHeader;
  memcpy(cellHeader.magic, CELL_LOG_MAGIC, sizeof(cellHeader.magic));
  cellHeader.session = logSessionNumber;
  cellHeader.recordSize = sizeof(CellLogRecord);
  cellHeader.cellCount = BMS_CELL_COUNT;

  sdMeasuredLogFile = SD.open(measuredName, FILE_WRITE);
  sdDerivedLogFile = SD.open(derivedName, FILE_WRITE);
  sdCellLogFile = SD.open(cellName, FILE_WRITE);
  if (sdMeasuredLogFile && sdDerivedLogFile && sdCellLogFile) {
    sdMeasuredLogFile.write(&measuredHeader, sizeof(measuredHeader));
    sdDerivedLogFile.write(&derivedHeader, sizeof(derivedHeader));
    sdCellLogFile.write(&cellHeader, sizeof(cellHeader));
    sdMeasuredLogFile.flush();
    sdDerivedLogFile.flush();
    sdCellLogFile.flush();
    isSdLogActive = true;
    Serial.print("[Log] Measured data: ");
    Serial.println(measuredName);
    Serial.print("[Log] Derived data:  ");
    Serial.println(derivedName);
    Serial.print("[Log] Per-cell data: ");
    Serial.println(cellName);
  } else {
    Serial.println("[Log] Failed to create all session logs; logging disabled");
    if (sdMeasuredLogFile) sdMeasuredLogFile.close();
    if (sdDerivedLogFile) sdDerivedLogFile.close();
    if (sdCellLogFile) sdCellLogFile.close();
  }
}

// One paired log per power cycle. Legacy LOG files are included so session
// numbers are never reused on a card containing logs from older firmware.
uint32_t logFindNextSessionNumber() {
  uint32_t maxSession = 0;

  File root = SD.open("/");
  if (root) {
    maxSession = logFindMaxSessionInDir(root);
    root.close();
  }

  return maxSession + 1;
}

uint32_t logFindMaxSessionInDir(File dir) {
  uint32_t maxSession = 0;

  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    unsigned int session = 0;
    bool isSessionLog = sscanf(entry.name(), "MEAS%u.BIN", &session) == 1 ||
                        sscanf(entry.name(), "DERI%u.BIN", &session) == 1 ||
                        sscanf(entry.name(), "CELL%u.BIN", &session) == 1 ||
                        sscanf(entry.name(), "LOG%u.BIN", &session) == 1;
    if (isSessionLog && (uint32_t)session > maxSession) {
      maxSession = session;
    }
    entry.close();
  }

  return maxSession;
}

void logService() {
  if (!isSdLogActive) {
    return;
  }

  static uint32_t lastFastMs = 0;
  static uint32_t lastMediumMs = 0;
  static uint32_t lastSlowMs = 0;
  static uint32_t lastSdFlushMs = 0;

  uint32_t nowMs = millis();

  if (nowMs - lastFastMs >= LOG_FAST_PERIOD_MS) {
    lastFastMs = nowMs;
    logWriteFastRecord(nowMs);
    imuWriteLogRecord(nowMs);
  }

  if (nowMs - lastMediumMs >= LOG_MEDIUM_PERIOD_MS) {
    lastMediumMs = nowMs;
    logWriteMediumRecord(nowMs);
  }

  if (nowMs - lastSlowMs >= LOG_SLOW_PERIOD_MS) {
    lastSlowMs = nowMs;
    logWriteSlowRecord(nowMs);
  }

  // Periodic flush bounds data loss at power-off to about one second.
  if (isSdLogActive && nowMs - lastSdFlushMs >= LOG_SD_FLUSH_PERIOD_MS) {
    lastSdFlushMs = nowMs;
    logFlushAllSdBuffers();
    if (isSdLogActive) {
      sdMeasuredLogFile.flush();
      sdDerivedLogFile.flush();
      sdCellLogFile.flush();
    }
  }
}

uint16_t logGetStatusFlags() {
  uint16_t flags = 0;
  if (isBraking)                              flags |= 1u << 0;
  if (isHardBraking)                          flags |= 1u << 1;
  if (isBspcActive)                           flags |= 1u << 2;
  if (isAppsPlausible)                        flags |= 1u << 3;
  if (isReadyToDriveLatched)                  flags |= 1u << 4;
  if (driveMode == 'D')                     flags |= 1u << 5;
  if (hasBmsFault)                             flags |= 1u << 6;
  if (hasImdFault)                             flags |= 1u << 7;
  if (hasPm100Fault)                           flags |= 1u << 8;
  if (isBmsTimedOut)                          flags |= 1u << 9;
  if (isImdCanTimedOut)                       flags |= 1u << 10;
  if (!bmsAreContactorsOpen())                    flags |= 1u << 11;
  if (digitalRead(PIN_BSPD_STATUS) == HIGH) flags |= 1u << 12;
  if (digitalRead(PIN_IMD_STATUS) == HIGH)  flags |= 1u << 13;
  if (digitalRead(PIN_BMS_STATUS) == HIGH)  flags |= 1u << 14;
  return flags;
}

void logWriteFastRecord(uint32_t nowMs) {
  hvCurrentRawLast = analogRead(PIN_HV_CURRENT_SENSE);

  MeasuredFastRecord measured;
  measured.type = (float)LOG_RECORD_FAST;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.apps1AdcCounts = (float)apps1RawLast;
  measured.apps2AdcCounts = (float)apps2RawLast;
  measured.bse1AdcCounts = (float)bse1RawLast;
  measured.bse2AdcCounts = (float)bse2RawLast;
  measured.hvCurrentAdcCounts = (float)hvCurrentRawLast;
  measured.motorRpm = (float)motorRpm;
  measured.busVoltageV = inverterDcBusVoltageV;
  measured.busCurrentA = inverterDcBusCurrentA;
  logAppendSd(sdMeasuredLogFile, true, &measured, sizeof(measured));

  DerivedFastRecord derived;
  derived.type = (float)LOG_RECORD_FAST;
  derived.timeSeconds = (float)nowMs / 1000.0f;
  derived.brakePressurePct = brakePressurePct;
  derived.commandedTorqueNm = commandedTorqueNm;
  derived.regenTorqueNm = regenTorqueNm;
  derived.speedKmh = speedKmh;
  derived.cumulativeDistanceKm = cumulativeDistanceKm;
  derived.statusFlags = (float)logGetStatusFlags();
  logAppendSd(sdDerivedLogFile, false, &derived, sizeof(derived));
}

void logWriteMediumRecord(uint32_t nowMs) {
  MeasuredMediumRecord measured;
  measured.type = (float)LOG_RECORD_MEDIUM;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.glvVoltageV = hasGlvVoltage ? glvVoltageV : NAN;
  measured.inverterTempC = hasInverterTemp ? (float)inverterTempTenthsC / 10.0f : NAN;
  measured.motorTempC = hasPm100MotorTemp ? (float)pm100MotorTempTenthsC / 10.0f : NAN;
  measured.socPct = hasBmsStatus ? (float)stateOfChargePct : NAN;
  measured.batteryTempC = hasCellTempStats ? (float)batteryTempC : NAN;
  measured.bmsFlags = (float)bmsFlags;
  measured.bmsFaultCode = hasBms2026FaultCode ? (float)bms2026FaultCode : NAN;
  measured.imdWarnAlarms = (float)imdWarnAlarms;
  measured.pm100PostFaults = (float)pm100PostFaults;
  measured.pm100RunFaults = (float)pm100RunFaults;
  logAppendSd(sdMeasuredLogFile, true, &measured, sizeof(measured));

  DerivedMediumRecord derived;
  derived.type = (float)LOG_RECORD_MEDIUM;
  derived.timeSeconds = (float)nowMs / 1000.0f;
  derived.apps1Pct = apps1Pct;
  derived.apps2Pct = apps2Pct;
  derived.pedalPct = pedalPct;
  derived.desiredTorqueNm = desiredTorqueNm;
  derived.powerKw = powerKw;
  derived.speedMph = speedMph;
  derived.fanDutyPct = (float)constrain(fan1DutyPct, 0, 100);
  derived.rtdState = (float)rtdState;
  logAppendSd(sdDerivedLogFile, false, &derived, sizeof(derived));
}

void logWriteSlowRecord(uint32_t nowMs) {
  MeasuredSlowRecord measured;
  measured.type = (float)LOG_RECORD_SLOW;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.minCellVoltageV = hasCellVoltageStats ? (float)minCellVoltageMv / 1000.0f : NAN;
  measured.maxCellVoltageV = hasCellVoltageStats ? (float)maxCellVoltageMv / 1000.0f : NAN;
  measured.avgCellVoltageV = hasCellVoltageStats ? (float)avgCellVoltageMv / 1000.0f : NAN;
  measured.minCellTempC = hasCellTempStats ? (float)minCellTempC : NAN;
  measured.maxCellTempC = hasCellTempStats ? (float)maxCellTempC : NAN;
  measured.avgCellTempC = hasCellTempStats ? (float)avgCellTempC : NAN;
  measured.imdRIsoKohm = (imdRIsoKohm != IMD_SNV_WORD) ? (float)imdRIsoKohm : NAN;
  measured.imdRIsoStatus = (float)imdRIsoStatus;
  measured.validFlags = (float)((hasCellVoltageStats ? 0x01 : 0) |
                                (hasCellTempStats ? 0x02 : 0) |
                                (hasGlvVoltage ? 0x04 : 0));
  logAppendSd(sdMeasuredLogFile, true, &measured, sizeof(measured));
}

void logAppendSd(File &file, bool measuredLog, const void *data, size_t len) {
  if (!isSdLogActive) {
    return;
  }

  uint8_t *buffer = measuredLog ? measuredLogBuffer : derivedLogBuffer;
  size_t &bufferLen = measuredLog ? measuredLogBufferLen : derivedLogBufferLen;
  if (bufferLen + len > 512) {
    logFlushSdBuffer(file, measuredLog);
    if (!isSdLogActive) {
      return;
    }
  }

  memcpy(buffer + bufferLen, data, len);
  bufferLen += len;
}

void logFlushSdBuffer(File &file, bool measuredLog) {
  uint8_t *buffer = measuredLog ? measuredLogBuffer : derivedLogBuffer;
  size_t &bufferLen = measuredLog ? measuredLogBufferLen : derivedLogBufferLen;
  if (!isSdLogActive || bufferLen == 0) {
    bufferLen = 0;
    return;
  }

  size_t written = file.write(buffer, bufferLen);
  if (written != bufferLen) {
    Serial.println("[Log] SD write failed; SD logging stopped");
    sdMeasuredLogFile.close();
    sdDerivedLogFile.close();
    sdCellLogFile.close();
    isSdLogActive = false;
    isScreenDirty = true;
  }
  bufferLen = 0;
}

void logAppendCell(const void *data, size_t len) {
  if (!isSdLogActive) return;
  if (cellLogBufferLen + len > sizeof(cellLogBuffer)) {
    logFlushCellBuffer();
    if (!isSdLogActive) return;
  }
  memcpy(cellLogBuffer + cellLogBufferLen, data, len);
  cellLogBufferLen += len;
}

void logFlushCellBuffer() {
  if (!isSdLogActive || cellLogBufferLen == 0) {
    cellLogBufferLen = 0;
    return;
  }
  if (sdCellLogFile.write(cellLogBuffer, cellLogBufferLen) != cellLogBufferLen) {
    Serial.println("[Log] SD write failed; SD logging stopped");
    sdMeasuredLogFile.close();
    sdDerivedLogFile.close();
    sdCellLogFile.close();
    isSdLogActive = false;
    isScreenDirty = true;
  }
  cellLogBufferLen = 0;
}

void logFlushAllSdBuffers() {
  logFlushSdBuffer(sdMeasuredLogFile, true);
  logFlushSdBuffer(sdDerivedLogFile, false);
  logFlushCellBuffer();
}

void logListFiles() {
  if (!isSdCardStarted) {
    Serial.println("[Log] SD card not available");
    return;
  }

  Serial.println("[Log] SD card:");
  File root = SD.open("/");
  if (root) {
    logListDir(root);
    root.close();
  }
}

void logListDir(File dir) {
  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    if (!entry.isDirectory()) {
      Serial.print("  ");
      Serial.print(entry.name());
      Serial.print("  ");
      Serial.print((unsigned long)entry.size());
      Serial.println(" bytes");
    }
    entry.close();
  }
}

// Hex-dumps all files in a session over serial so logs can be exported
// without pulling the SD card.
void logDumpFile(uint32_t session) {
  if (!isSdCardStarted) {
    Serial.println("[Log] SD card not available");
    return;
  }

  // Make sure everything buffered for the current session is on the card.
  logFlushAllSdBuffers();
  if (isSdLogActive) {
    sdMeasuredLogFile.flush();
    sdDerivedLogFile.flush();
    sdCellLogFile.flush();
  }

  const char *prefixes[3] = {"MEAS", "DERI", "CELL"};
  for (uint8_t fileIndex = 0; fileIndex < 3; fileIndex++) {
    char name[16];
    snprintf(name, sizeof(name), "%s%04lu.BIN", prefixes[fileIndex],
             (unsigned long)session);
    File dumpFile = SD.open(name, FILE_READ);
    if (!dumpFile) {
      Serial.print("[Log] ");
      Serial.print(name);
      Serial.println(" not found");
      continue;
    }

    Serial.print("[Log] DUMP BEGIN ");
    Serial.print(name);
    Serial.print(" ");
    Serial.println((unsigned long)dumpFile.size());

    uint8_t buf[32];
    int n;
    while ((n = dumpFile.read(buf, sizeof(buf))) > 0) {
      for (int i = 0; i < n; i++) tracePrintCanHexByte(buf[i]);
      Serial.println();
    }
    dumpFile.close();
    Serial.println("[Log] DUMP END");
  }
}

void logWriteCellDataResponse(const CAN_message_t &frame) {
  if (!isSdLogActive || frame.flags.extended || frame.len < 2) return;

  const uint16_t frameIndex = frame.buf[0];
  const float timeSeconds = (float)millis() / 1000.0f;

  if (lastCellDataRequestType == CELL_DATA_REQUEST_VOLTAGES) {
    for (uint8_t i = 0; i < 3 && (1 + i * 2 + 1) < frame.len; i++) {
      const uint16_t zeroBasedCell = frameIndex * 3u + i;
      if (zeroBasedCell >= BMS_CELL_COUNT) break;
      const uint16_t millivolts = canReadU16Be(frame.buf[1 + i * 2], frame.buf[2 + i * 2]);
      CellLogRecord record = {
        (float)CELL_DATA_REQUEST_VOLTAGES, timeSeconds, (float)(zeroBasedCell + 1u),
        millivolts == 0xFFFF ? NAN : (float)millivolts / 1000.0f
      };
      logAppendCell(&record, sizeof(record));
    }
  } else if (lastCellDataRequestType == CELL_DATA_REQUEST_TEMPERATURES) {
    for (uint8_t i = 0; i < 7 && (1 + i) < frame.len; i++) {
      const uint16_t zeroBasedCell = frameIndex * 7u + i;
      if (zeroBasedCell >= BMS_CELL_COUNT) break;
      const uint8_t rawTemperature = frame.buf[1 + i];
      CellLogRecord record = {
        (float)CELL_DATA_REQUEST_TEMPERATURES, timeSeconds, (float)(zeroBasedCell + 1u),
        rawTemperature == 0xFF ? NAN : (float)((int)rawTemperature - 40)
      };
      logAppendCell(&record, sizeof(record));
    }
  }
}

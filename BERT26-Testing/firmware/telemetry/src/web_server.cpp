/**
 * @file    web_server.cpp
 * @brief   HTTP server: serves the live and config pages, the /data JSON and the /set_soc and /set_cal commands.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "web_server.h"

#include <Arduino.h>
#include <WebServer.h>
#include <string.h>

#include "vehicle_can.h"
#include "live_signals.h"
#include "cell_dump.h"
#include "web_pages.h"

// Functions used only in this file.
void webServerSendLargeHtml(const char* pageHtml);
void webServerHandleRoot();
void webServerHandleConfig();
void webServerHandleSetSoc();
void webServerHandleSetCal();
void webServerHandleData();

static constexpr uint16_t HTTP_PORT        = 80;
static constexpr int      HTTP_OK          = 200;
static constexpr int      HTTP_BAD_REQUEST = 400;

// Pages are sent in pieces so the whole string never has to be copied into a
// String. Kept at the BERT26 value (likely meant as 16384; any size works).
static constexpr size_t HTML_CHUNK_BYTES = 16184;
// /data is built in small Strings and flushed once one passes this length.
static constexpr unsigned int JSON_CHUNK_FLUSH_CHARS = 300;
static constexpr unsigned int JSON_DECIMAL_PLACES    = 3;

// Accepted ranges for the config page commands.
static constexpr float   SOC_MIN_PCT    = 0.0f;
static constexpr float   SOC_MAX_PCT    = 100.0f;
static constexpr uint8_t CAL_POINT_LOW  = 1;  // two-point calibration: points 1 and 2
static constexpr uint8_t CAL_POINT_HIGH = 2;

static WebServer webServer(HTTP_PORT);

// ---------------- Setup ----------------
void webServerSetup() {
  webServer.on("/", webServerHandleRoot);
  webServer.on("/data", webServerHandleData);
  webServer.on("/config", webServerHandleConfig);
  webServer.on("/set_soc", webServerHandleSetSoc);
  webServer.on("/set_cal", webServerHandleSetCal);
  webServer.begin();
  Serial.println("HTTP server started");
}

void webServerService() {
  webServer.handleClient();
}

// ---------------- Pages ----------------
void webServerSendLargeHtml(const char* pageHtml) {
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(HTTP_OK, "text/html", "");
  size_t lengthBytes = strlen(pageHtml);
  for (size_t offset = 0; offset < lengthBytes; offset += HTML_CHUNK_BYTES) {
    size_t chunkBytes = (lengthBytes - offset < HTML_CHUNK_BYTES) ? lengthBytes - offset : HTML_CHUNK_BYTES;
    webServer.sendContent(pageHtml + offset, chunkBytes);
  }
  webServer.sendContent("");
}

void webServerHandleRoot()   { webServerSendLargeHtml(livePageHtml); }
void webServerHandleConfig() { webServerSendLargeHtml(configPageHtml); }

// ---------------- Commands ----------------
void webServerHandleSetSoc() {
  String socArg = webServer.arg("soc");
  float socPct = socArg.toFloat();
  if (socArg.length() == 0 || socPct < SOC_MIN_PCT || socPct > SOC_MAX_PCT) {
    webServer.send(HTTP_BAD_REQUEST, "application/json", "{\"ok\":false,\"error\":\"range\"}");
    return;
  }
  canSendSoc(socPct);
  webServer.send(HTTP_OK, "application/json", "{\"ok\":true}");
}

void webServerHandleSetCal() {
  String pointArg = webServer.arg("point");
  String currentArg = webServer.arg("current");
  uint8_t calPoint = (uint8_t)pointArg.toInt();
  if (calPoint < CAL_POINT_LOW || calPoint > CAL_POINT_HIGH || currentArg.length() == 0) {
    webServer.send(HTTP_BAD_REQUEST, "application/json", "{\"ok\":false,\"error\":\"args\"}");
    return;
  }
  canSendCurrentCalPoint(calPoint, currentArg.toFloat());
  webServer.send(HTTP_OK, "application/json", "{\"ok\":true}");
}

// ---------------- /data JSON ----------------
void webServerHandleData() {
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(HTTP_OK, "application/json", "");

  // Scalar signals
  String json = "{\"timeSent\":" + String(millis()) + ",";
  for (int i = 0; i < SIGNAL_COUNT; i++) {
    json += "\"" + String(liveSignalNames[i]) + "\":" + String(liveSignalValues[i], JSON_DECIMAL_PLACES) + ",";
  }

  // Cell voltages (mV); empty array until first complete voltage dump
  json += "\"cellVoltages\":[";
  webServer.sendContent(json);
  if (areCellVoltagesValid) {
    String chunk = "";
    for (int cell = 0; cell < PACK_CELL_COUNT; cell++) {
      if (cell > 0) chunk += ",";
      chunk += String(cellVoltagesMv[cell]);
      if (chunk.length() > JSON_CHUNK_FLUSH_CHARS) { webServer.sendContent(chunk); chunk = ""; }
    }
    webServer.sendContent(chunk);
  }

  // Cell temperatures (°C); null for sensors that returned 0xFF
  webServer.sendContent("],\"cellTemps\":[");
  if (areCellTempsValid) {
    String chunk = "";
    for (int cell = 0; cell < PACK_CELL_COUNT; cell++) {
      if (cell > 0) chunk += ",";
      chunk += (cellTempsC[cell] != CELL_TEMP_NOT_RECEIVED_C) ? String(cellTempsC[cell]) : "null";
      if (chunk.length() > JSON_CHUNK_FLUSH_CHARS) { webServer.sendContent(chunk); chunk = ""; }
    }
    webServer.sendContent(chunk);
  }

  webServer.sendContent("]}");
  webServer.sendContent("");
}

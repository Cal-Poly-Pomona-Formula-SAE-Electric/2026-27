/**
 * @file    main.cpp
 * @brief   Starts every telemetry module and runs the main loop.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include <Arduino.h>
#include "vehicle_can.h"
#include "live_signals.h"
#include "cell_dump.h"
#include "wifi_ap.h"
#include "web_server.h"

/*
  BERT26 telemetry (BMS Wi-Fi broadcast) sketch

  Functions:
    - Listens on CAN3 for BMS status (0x310), temperatures (0x311) and charger status (0x18FF50E5).
    - Asks the BMS for a cell voltage or temperature dump (0x312/0x313) every 5 s, in turn.
    - Runs the "BERT.26" Wi-Fi access point with a captive DNS.
    - Serves the live BMS page, its /data JSON, and a config page that sends
      SOC override (0x314) and current sensor calibration (0x315) to the BMS.
*/

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  liveSignalsReset();
  cellDumpReset();

  canSetup();
  wifiApSetup();
  webServerSetup();
}

// ---------------- Main loop ----------------
void loop() {
  wifiApService();
  cellDumpService();
  canReceiveFrames();
  webServerService();
}

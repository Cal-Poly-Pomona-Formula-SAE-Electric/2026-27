/**
 * @file    wifi_ap.h
 * @brief   Runs the "BERT.26" Wi-Fi access point and a captive DNS that answers every name with the node's IP.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef TELEMETRY_WIFI_AP_H
#define TELEMETRY_WIFI_AP_H

void wifiApSetup();
void wifiApService();

#endif  // TELEMETRY_WIFI_AP_H

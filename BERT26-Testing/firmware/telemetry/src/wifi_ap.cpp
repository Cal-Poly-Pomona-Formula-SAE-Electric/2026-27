/**
 * @file    wifi_ap.cpp
 * @brief   Runs the "BERT.26" Wi-Fi access point and a captive DNS that answers every name with the node's IP.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "wifi_ap.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>

static constexpr const char* WIFI_AP_SSID     = "BERT.26";
static constexpr const char* WIFI_AP_PASSWORD = "BERT2026";

static constexpr uint16_t DNS_PORT = 53;  // standard DNS port

// softAPConfig() right after softAP() can be lost while the AP is still
// starting, so wait briefly between the two.
static constexpr uint32_t AP_START_SETTLE_MS = 50;

static DNSServer dnsServer;
static IPAddress apLocalIp(192, 168, 1, 1);
static IPAddress apGatewayIp(192, 168, 1, 1);
static IPAddress apSubnetMask(255, 255, 255, 0);

// ---------------- Setup ----------------
void wifiApSetup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  delay(AP_START_SETTLE_MS);
  WiFi.softAPConfig(apLocalIp, apGatewayIp, apSubnetMask);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // "*" answers every DNS name, so any address typed on a phone opens the page.
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
}

// ---------------- Service ----------------
void wifiApService() {
  dnsServer.processNextRequest();
}

/**
 * @file    web_pages.h
 * @brief   HTML/JavaScript for the live BMS page (/) and the config page (/config).
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef TELEMETRY_WEB_PAGES_H
#define TELEMETRY_WEB_PAGES_H

extern const char* livePageHtml;    // polls /data every 1 s; cell grids, SOC, current, flags
extern const char* configPageHtml;  // SOC override (0x314) and current sensor calibration (0x315)

#endif  // TELEMETRY_WEB_PAGES_H

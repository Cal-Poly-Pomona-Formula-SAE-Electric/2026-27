/**
 * @file    web_server.h
 * @brief   HTTP server: serves the live and config pages, the /data JSON and the /set_soc and /set_cal commands.
 *
 * Board:   Telemetry (ESP32-S3)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef TELEMETRY_WEB_SERVER_H
#define TELEMETRY_WEB_SERVER_H

void webServerSetup();
void webServerService();

#endif  // TELEMETRY_WEB_SERVER_H

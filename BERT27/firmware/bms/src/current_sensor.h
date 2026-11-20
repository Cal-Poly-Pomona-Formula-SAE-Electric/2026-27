/**
 * @file    current_sensor.h
 * @brief   Reads pack current from the LEM DHAB S/124 and stores its two-point calibration in EEPROM.
 *
 * Board:   BMS (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef BMS_CURRENT_SENSOR_H
#define BMS_CURRENT_SENSOR_H

#include <Arduino.h>

// Positive = discharge, negative = charge or regen.
extern float packCurrentA;

float currentSensorReadA();
void currentSensorLoadCalibration();
void currentSensorRecordCalPoint(uint8_t pointNumber, float actualA);

#endif  // BMS_CURRENT_SENSOR_H

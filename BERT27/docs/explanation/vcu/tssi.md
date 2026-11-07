# TSSI
## tssi.h
```cpp
/**
 * @file    tssi.h
 * @brief   Drives the TSSI lights: green when safe, red flashing on an IMD or BMS fault.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_TSSI_H
#define VCU_TSSI_H

#include <Arduino.h>

bool tssiHasFaultInWarnBits(uint16_t warnBits);
void tssiSetup();
void tssiService();
void tssiPrintWarnBits(uint16_t bits);

#endif  // VCU_TSSI_H
```
The header sets up guards, and defines four functions that will be used in ```tssi.cpp```.
## tssi.cpp
The code first declares and defines variables for the 

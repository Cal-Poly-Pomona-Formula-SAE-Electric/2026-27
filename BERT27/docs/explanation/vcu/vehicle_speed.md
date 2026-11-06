## Vehicle Speed
# vehicle_speed.h
```cpp
#ifndef VCU_VEHICLE_SPEED_H
#define VCU_VEHICLE_SPEED_H

#include <Arduino.h>

// Road speed from motor rpm (set in loop()), and the distance driven this power-up.
extern float speedMph;
extern float speedKmh;
extern float cumulativeDistanceKm;

void speedUpdateCumulativeDistance(uint32_t nowMs);

#endif  // VCU_VEHICLE_SPEED_H
```
This header file defines variables and functions that will be used in ```vehicle_speed.cpp```

# vehicle_speed.cpp
```cpp
#include "vehicle_speed.h"

float speedMph = 0.0f;
float speedKmh = 0.0f;
float cumulativeDistanceKm = 0.0f;
uint32_t lastDistanceUpdateMs = 0;

// Integrate the RPM-derived road speed. This is intentionally approximate:
// MPH_PER_RPM already embodies the vehicle's gearing and tire circumference.
void speedUpdateCumulativeDistance(uint32_t nowMs) {
  if (lastDistanceUpdateMs != 0) {
    float elapsedHours = (float)(nowMs - lastDistanceUpdateMs) / 3600000.0f;
    cumulativeDistanceKm += speedKmh * elapsedHours;
  }
  lastDistanceUpdateMs = nowMs;
}

```
This function is used to calculate and approximate for the vehicle's road speed.
It consists of one function, ```speedUpdateCulmulativeDistance()```, and accepts as input the current time in milliseconds ```nowMs```.</br></br>
The file defines four functions: three floats consisting of ```speedMph```, ```speedKmh```, and ```cumulativeDistanceKm```, and one 32-bit integer ```lastDistanceUpdateMs```. If time has elapsed since the last update, the program calculates the elapsed time in hours ```elapsedHours```, and converts from ms to seconds. Using the elapsed time, the total distance covered is updated, and the program saves the time at which this function executed. 
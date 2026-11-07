# Vehicle Can
## vehicle_can.h
```cpp
/**
 * @file    vehicle_can.h
 * @brief   Sets up CAN3, sends frames and dispatches received frames to their module.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_VEHICLE_CAN_H
#define VCU_VEHICLE_CAN_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint32_t PM100_TEMPERATURE_1_FRAME_ID = 0x0A0;
static constexpr uint32_t PM100_TEMPERATURE_3_FRAME_ID = 0x0A2;
static constexpr uint32_t PM100_RPM_FRAME_ID = 0x0A5;
static constexpr uint32_t PM100_CURRENT_INFO_FRAME_ID = 0x0A6;
static constexpr uint32_t PM100_VOLTAGE_INFO_FRAME_ID = 0x0A7;
static constexpr uint32_t PM100_INTERNAL_VOLTAGES_FRAME_ID = 0x0A9;
static constexpr uint32_t PM100_FAULT_CODES_FRAME_ID = 0x0AB;
static constexpr uint32_t PM100_TORQUE_CAPABILITY_FRAME_ID = 0x0B1;
// BERT26 vehicle CAN definitions for human-readable log translations.
static constexpr uint32_t BMS_STATUS_2026_ID        = 0x310;
static constexpr uint32_t BMS_TEMPERATURE_2026_ID   = 0x311;
static constexpr uint32_t BMS_CELL_RESPONSE_2026_ID = 0x313;
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;

extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> vehicleCan;

void canSetup();
void canSendFrame(CAN_message_t &frame);
void canReceiveFrames();
void canLogFrame(const char *direction, const CAN_message_t &frame);

#endif  // VCU_VEHICLE_CAN_H
```
The header file first sets up guards. If the header is not defined, it goes on to define all the variables within the file. If for whatever reason the header file is defined again ```ifndef VCU_VEHICLE_CAN_H``` will return ```false``` and will skip defining all the variables+functions again.</br> Variables that will be used in ```vehicle_can.cpp```are declared and initialized, as well as some functions that will be used in the .cpp file. ```extern``` is used to declare a global ```vehicleCan``` object belonging to class ```FlexCAN_T4```.

## vehicle_can.cpp
This code manages the CAN communication between the VCU and the other electronic systems in the vehicle, including the inverter, BMS, IMD, and energy meter. This is all done via the CAN outputs on the Teensy 4.1, and is connected to an external CAN transceiver.</br></br>
There are five main functions:
### canSetup()
```cpp
void canSetup() {
  vehicleCan.begin();
  vehicleCan.setBaudRate(CAN_BAUD_BPS);
  vehicleCan.setMaxMB(16);
  vehicleCan.enableFIFO();

  Serial.println("[CAN] CAN3 started at 500 kbit/s on pins 30=RX, 31=TX");
}
```
This function initializes the CAN3 controller, configuring the communication bitrate, maximum number of hardware mailboxes, and enables FIFO (first-in, first-out) to manage incoming CAN messages.
### canSendFrame()
```cpp
void canSendFrame(CAN_message_t &frame) {
  noInterrupts();
  vehicleCan.write(frame);
  interrupts();
  canLogFrame("TX", frame);
}
```
This function transmits CAN messages from the VCU to the Teensy. It takes a CAN frame as input and transmits it to the CAN controller. Interrupts are temporarily disabled during this and renabled once the CAN frame is written. It also logs the attempt for debugging purposes.
### canReceiveFrame()
```cpp
void canReceiveFrames() {
  CAN_message_t rxFrame;
  while (vehicleCan.read(rxFrame)) {
    canLogFrame("RX", rxFrame);
    canDispatchReceivedFrame(rxFrame);
  }
}
```
This function checks the CAN receive buffer for any incoming messages. When a message is available it is stored inside rxFrame, logged, and passed onto ```canDispatchReceivedFrame()```. The function continues running until no frames are available.
### canDispatchReceivedFrame()
```cpp
void canDispatchReceivedFrame(const CAN_message_t &frame) {
  if (!hasPm100Traffic && frame.id >= PM100_TEMPERATURE_1_FRAME_ID &&
      frame.id <= PM100_TORQUE_CAPABILITY_FRAME_ID) {
    hasPm100Traffic = true;
    firstPm100FrameMs = millis();
  }

  switch (frame.id) {
    case IMD_INFO_GENERAL_ID:
    case IMD_RESPONSE_ID:
      imdHandleFrame(frame);
      break;

    case PM100_TEMPERATURE_1_FRAME_ID:
      inverterHandleTemperatures1Frame(frame);
      break;

    case PM100_TEMPERATURE_3_FRAME_ID:
      inverterHandleTemperatures3Frame(frame);
      break;

    case PM100_RPM_FRAME_ID:
      inverterHandleRpmFrame(frame);
      break;

    case PM100_CURRENT_INFO_FRAME_ID:
      inverterHandleCurrentInfoFrame(frame);
      break;

    case PM100_VOLTAGE_INFO_FRAME_ID:
      inverterHandleVoltageInfoFrame(frame);
      break;

    case PM100_INTERNAL_VOLTAGES_FRAME_ID:
      inverterHandleInternalVoltagesFrame(frame);
      break;

    case PM100_FAULT_CODES_FRAME_ID:
      inverterHandleFaultCodesFrame(frame);
      break;

    case PM100_PARAM_RESPONSE_ID:
      inverterHandleParamResponseFrame(frame);
      break;

    case BMS_STATUS_2026_ID:
      bmsHandleStatusFrame(frame);
      break;

    case BMS_TEMPERATURE_2026_ID:
      bmsHandleTempFrame(frame);
      break;

    case BMS_CELL_RESPONSE_2026_ID:
      logWriteCellDataResponse(frame);
      break;

    case EM_MEASUREMENT_ID:
      emHandleMeasurementFrame(frame);
      break;

    case EM_STATUS_ID:
      emHandleStatusFrame(frame);
      break;

    case EM_TEMPERATURE_ID:
      emHandleTemperatureFrame(frame);
      break;

    default:
      break;
  }
}
```
This function indentifies incoming CAN messages using predefined indentifiers and directs them to other software modules for furter processing. For example, a BMS status message would be passed onto the BMS module. The code also checks for messasges from the inverter, and once a message is detected it records the time of detection.
### canLogFrame() & canLogFrameToSerial()
```cpp
void canLogFrame(const char *direction, const CAN_message_t &frame) {
  traceUpdateState(direction, frame);

  // Future SD-card logging should mirror this raw record and the decoded record here.
  if (IS_CAN_LOG_SERIAL_ENABLED) {
    canLogFrameToSerial(direction, frame);
    traceLogToSerial(direction, frame);
  }
}

void canLogFrameToSerial(const char *direction, const CAN_message_t &frame) {
  Serial.print("[CAN ");
  Serial.print(direction);
  Serial.print("] t=");
  Serial.print(millis());
  Serial.print(" id=0x");
  Serial.print(frame.id, HEX);
  Serial.print(frame.flags.extended ? " ext" : " std");
  Serial.print(" len=");
  Serial.print(frame.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < frame.len; i++) {
    tracePrintCanHexByte(frame.buf[i]);
    if (i + 1 < frame.len) {
      Serial.print(' ');
    }
  }

  Serial.println();
}
```
These functions are used to monitors CAN communication for debugging. ```canLogFrame()``` updates the CAN frame, and calls ```canLogFrametoSerial()``` to display the message. The serial ooutput includes the transmission direction (TX/RX), timestamp, ID, format, msg length, and msg data in hex format. 
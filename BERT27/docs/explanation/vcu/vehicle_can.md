# Vehicle Can
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

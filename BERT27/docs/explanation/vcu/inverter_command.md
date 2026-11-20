# Inverter Command
### inverterCmdSetupTimer
Caclulates the latest value of commanded torque via ```inverterCmdUpdateState()```, sends a CAN command via ```inverterCmdTimerISR```, and begins the 100 Hz PM100 command timer.

### inverterCmdTimerIsr()
This function is called every 10 ms and sends the PM100 a torque command

### inverterCmdUpdateState()
It first initializes default values for the torque, direction, and inverter enable. It calculates the commanded torque from ```inverterCmdCalculateDisplayedTorqueNM()``` and assigns the value to ```displayedCommandTorqueNm```. Then it checks if torque is allowed:
* if ready to drive is latched
* if the car is in Drive
* there are no active faults
* BMS contacts are closed
<br><br>

If torque is allowed it calculates the torque from Nm to raw for the PM100, sets the direction, and enables the inverter. Finally it stores the command values. They are wrapped in ```noInterrupts()``` and ```interrupts()``` to prevent an interrupt from causing a partial update.

### inverterCmdCalculateDisplayedTorqueNm()
It first checks if the regen torque is positive. If so the torque is set negative. <br>If power derate is active and the torque is positive, it calculates and clamps torque to a cap. Otherwise the commanded torque is untouched. Finally it converts torque from Nm to PM100 raw units, rounding it to the nearest raw unit.

### inverterCmdQueueTxLogFromISR()
It calculates the next queue position in the circular buffer. If the queue is full, the frame is lost, recorded, and the function exits. Otherwise it copies the frame into the queue position and advances the next queue position.

### inverterCmdDequeueTxLog()
This function removes a frame from the circular buffer. If the queue is not empty, the oldest frame ```pm100TxLogTail``` is copied into ```frame```. It then advances the read position (the tail), and if the tail reaches the end it wraps back to 0. It then sets ```hasMessage = true```, indicating that a frame was retreived.

### inverterCmdServiceTxLogging()
It begins by removing a CAN frame from the circular buffer, and it does so until there are no more frames to remove. If the count of frames dropped changes, it prints a running total of frames dropped.

### inverterCmdGetDirectionByte
Returns the direction of the inverter. ***Looks incomplete?***


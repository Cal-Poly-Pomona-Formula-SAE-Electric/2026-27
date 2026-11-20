# Inverter
## inverterHandleTemperatures1Frame()
It reads the raw temperatures for inverter modules A-C, as well as the temp of the gate driver from the CAN bus. ```hottestTempRaw``` stores the highest temperature read, which is then passed onto ```inverterTempTenthsC```. The function indicates that the inverter temperature has been recorded and that the dashboard display needs to be redrawn: ```isScreenDirty = true;```.

## inverterHandleTemperatures3Frame()
It reads the temperature of the pm100 motor from the CAN bus. It then indicates that the motor temperature has been read and signals that the dashboard needs to be redrawn.

## inverterHandleRpmFrame()
It reads the RPM of the motor from the CAN bus and signals an update for the dashboard.

## inverterHandleCurrentInfoFrame()
Reads the inverter DC voltage and signals an update for the dashboard display.

## inverterHandleCurrentInfoFrame()
Reads the inverter DC current and signals an update for the dashboard display.

## inverterHandleInternalVoltagesFrame()
Reads the grounded low voltage (GLV) system voltage and signals update for the dashboard display.

## inverterHandleParamResponseFrame()
Checks the address of the CAN frame to see which command the inverter is responding to. If ```PM1000_PARAM_CAN_ACTIVE_MSGS``` it reports wheether enabling CAN messages suceeded. If ```PM100_PARAM_FAULT_CLEAR``` it reports whether the fault clear command was accepted or rejected.

## inverterServiceBroadcastEnable()
If any of these conditions are met:
* inverter is sending GLV voltage data
* PM100 traffic is detected
* Ready-to-drive is active
* PM100 has attempted more than the max number of broadcasts
* has been less than 5 seconds after the first PM100 frame<br><br>

The function does nothing. Otherwise, it records the number of broadcast attempts by the PM100, tracking the time of the last attempt. It then enables inverter broadcasts.

## inverterSendBroadcastEnable()
~ Don't really understand CAN well enough to explain this

## inverterServiceFaultClear
If any of these conditions are met:
* there is no fault status from the PM100
* PM100 has a fault
* there is no BMS status
* the BMS contacts are open
* there is no inverter voltage info
* the inverter DC voltage is less than the minimum PM100 voltage
* attempts to clear a fault on the PM100 exceed the max set by ```PM1000_FAULT_CLEAR_MAX_ATTEMPTS```
* the window for another fault clear attempt hasn't passed
<br><br>

The function does nothing. Otherwise, it records the number of fault clear attempts, and records the time of the last attempt. It then clears the fault and sends a message notifying that contacts have been closed, along with the inverter DC voltage and the number of fault clear attempts.

## inverterSendFaultClear()
~ same as for ```inverterSendBroadcastEnable```

## inverterHandleFaultCodesFrame
It tracks whether there has been a previous fault. It then reads PM100 post faults and run faults. If a fault is detected and it's the first, it displays the fault on the dasboard. Otherwise if there is no fault and there was a previous fault that got cleared, it sends a ```"[PM100] Faults cleared"```. It then resets the number of fault attempts and signals an update for the dashboard display.
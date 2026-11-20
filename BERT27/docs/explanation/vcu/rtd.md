# Ready-to-drive
*not sure if necessary to document functioin declarations at top of file*
```cpp
static constexpr uint32_t RTD_BLOCKED_LOG_PERIOD_MS = 1000;

// ---------------- Runtime state ----------------
char driveMode = 'N';  // RTD state machine sets D only while ready to drive.

RtdState rtdState = RTD_STATE_FAULT_BLOCKED;
// Compatibility flag used by the display and PM100 gate; kept in sync with rtdState.
bool isReadyToDriveLatched = false;

bool wasReadyButtonPressed = false;

// EV.10.2: ready-to-drive sound, 1-3 seconds. Solid continuous tone for the
// first 1.25 s (satisfies the >=1 s continuous requirement), then a short
// playful flourish. Step on+off times sum to 2000 ms total.
static const ToneStep RTD_PATTERN[] = {
  {2093, 1250,  0},  // C7 - continuous main sound (1.25 s)
  {2637,  120, 40},  // E7
  {3136,  120, 40},  // G7
  {2637,  120, 40},  // E7
  {3136,  270,  0}   // G7 - final flourish
};
```
The code first defines ```RTD_BLOCKED_LOG_PERIOD_MS```, setting the logging period for blocked RTD events, set to ```1000```ms.
It also defines the starting state of the car, which is ```N``` for neutral. The RTD state machine sets the state to ```D``` only once the car is determined as ready to drive.
```bool```s ```isReadyToDriveLatched``` and ```wasReadyButtonPressed``` are flags that help determine the RTD state.
The ```RTD_PATTERN``` is a static constant array of ToneStep structures that defines a ready-to-drive sound lasting between 1 to 3 seconds. It consists of a solid continuous tone for the first 1.25 seconds, followed by a short  flourish, with the total step on and off times summing to 2000 milliseconds.
```cpp
// ---------------- Ready to drive state machine ----------------
void rtdService() {
  rtdRefreshStateFromInputs();

  bool isPressed = rtdReadButtonPressed();

  if (isPressed && !wasReadyButtonPressed) {
    rtdHandleButtonPressed();
    isScreenDirty = true;
  }

  wasReadyButtonPressed = isPressed;
}
```
```rtdService()``` manages the state of the RTD state machine. It refreshes the current state by calling ```rtdRefreshStateFromInputs()```, detects buttons presses via ```bool isPressed```, and handles the button press event to update the sceen status. It also tracks whether the button was previously pressed to prevent repeated actions.
```cpp
bool rtdReadButtonPressed() {
  return digitalRead(PIN_READY_BUTTON) == HIGH;
}

bool rtdAreInputsReady() {
  if (!faultsIsBmsImdDetectionArmed()) {
    return false;
  }

  // APPS implausibility intentionally does not gate RTD; it zeroes the torque
  // command instead (see pedalsUpdate/inverterCmdUpdateState).
  return !faultsIsAnyActive() && !bmsAreContactorsOpen();
}

bool rtdIsReadyForButton() {
  return rtdState == RTD_STATE_READY_FOR_BUTTON;
}
```
```rtdReadButtonPressed()``` returns true if ```PIN_READY_BUTTON``` when the pin reads ```HIGH```. Otherwise it returns false. It only reads the button, it doesn't change the state.</br>
```rtdAreInputsReady``` checks if the BMS IMD fault detection is armed, if there are any active faults, and if any BMS connectors are open. If any of these checks fail, it returns false.</br>
```rtdIsReadyForButton``` sets the state to ```RTD_STATE_READY_FOR_BUTTON```.
```cpp
void rtdDrop(const char *reason) {
  bool wasLatched = (rtdState == RTD_STATE_READY_TO_DRIVE) || isReadyToDriveLatched;

  rtdSetState(rtdAreInputsReady() ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);

  if (wasLatched && reason != nullptr) {
    Serial.print("[RTD] Dropped; ");
    Serial.println(reason);
    if (strcmp(reason, "RTD button pressed") != 0) {
      dashShowFaultBanner(reason);
    }
  }
}
```
```rtdDrop``` controls exiting ready-to-drive mode. It accepts as input a pointer to ```char reason```. It first records whether the system was latched. Then it sets the state to ```READY_FOR_BUTTON``` if the inputs are ready. If not, the state is set to ```FAULT_BLOCKED```. If the system is latched and given a non-null reason, it logs the drop and shows a fault banner, except when the reason is ```"RTD button pressed"```. Pressing the RTD button while driving will unlatch RTD without displaying a fault while any other reason will report the cause.
```cpp
// Periodically prints why RTD_STATE_FAULT_BLOCKED hasn't cleared, e.g. so a
// stuck contactor/fault/IMD flag after an HV power cycle shows up in the
// serial log instead of just leaving the dashboard stuck on "N".
void rtdLogBlockedReason() {
  static uint32_t lastLogMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastLogMs < RTD_BLOCKED_LOG_PERIOD_MS) {
    return;
  }
  lastLogMs = nowMs;

  if (!faultsIsBmsImdDetectionArmed()) {
    Serial.println("[RTD] Blocked: startup fault-detection delay active");
    return;
  }

  Serial.print("[RTD] Blocked:");
  if (hasImdFault)            Serial.print(" imdFault");
  if (hasBmsFault)            Serial.print(" bmsFault");
  if (bmsAreContactorsOpen())    Serial.print(" contactorsOpen");
  if (isImdCanTimedOut)      Serial.print(" imdCanTimedOut");
  if (isBmsTimedOut)         Serial.print(" bmsTimedOut");
  Serial.print(" bmsFlags=0x");
  tracePrintCanHexByte(bmsFlags);
  Serial.print(" imdWarnAlarms=0x");
  Serial.print(imdWarnAlarms, HEX);
  Serial.println();
}
```
```rtdLogBlockedReason``` periodically prints to serial monitor why ```FAULT_BLOCKED``` hasn't cleared, providing more information when otherwise the dashboard would be stuck on "N". It establishes a period for sending a message in ```RTD_BLOCKED_LOG_PERIOD_MS```, and only runs the rest of the function once that time has elapsed. If it is time for a new message, it first checks if the IMD fault detection is armed. If it isn't armed it prints the reason for RTD blocked: ```"...startup fault-detection delay active"```. It also detects for faults with the IMD, BMS, BMS contacts, IMD CAN timeout, and the BMS timeout. If any of those faults are active it outputs the corresponding message.
```cpp
void rtdRefreshStateFromInputs() {
  if (rtdState == RTD_STATE_READY_TO_DRIVE) {
    if (!rtdAreInputsReady()) {
      rtdDrop(rtdGetDropCauseText());
    }
    return;
  }

  bool areInputsReady = rtdAreInputsReady();
  if (!areInputsReady) {
    rtdLogBlockedReason();
  }

  rtdSetState(areInputsReady ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);
}
```
```rtdRefreshStateFromInputs()``` first checks if the current state is ```RTD_STATE_READY_TO_DRIVE```. If the inputs are net ready, it unlatches RTD and sends a fault message. It also logs the reason why RTD is blocked. Depending on if the inputs are ready, it sets the RTD state to ```READY_FOR_BUTTON``` if inputs are ready and ```FAULT_BLOCKED``` otherwise.
```cpp
void rtdHandleButtonPressed() {
  Serial.print("[RTD] Button pressed, state=");
  Serial.println(rtdGetStateName(rtdState));

  switch (rtdState) {
    case RTD_STATE_READY_FOR_BUTTON:
      if (IS_BRAKE_REQUIRED_FOR_RTD && !isBraking) {
        Serial.println("[RTD] Not latched; brake pedal must be pressed");
        break;
      }
      rtdSetState(RTD_STATE_READY_TO_DRIVE);
      buzzerPlayPattern(RTD_PATTERN, sizeof(RTD_PATTERN) / sizeof(RTD_PATTERN[0]), false);
      Serial.println("[RTD] Ready to drive latched");
      break;

    case RTD_STATE_READY_TO_DRIVE:
      rtdDrop("RTD button pressed");
      break;

    case RTD_STATE_FAULT_BLOCKED:
    default:
      rtdSetState(RTD_STATE_FAULT_BLOCKED);
      Serial.println(faultsIsAnyActive() ? "[RTD] Not latched; anyFault is active"
                                       : "[RTD] Not latched; contactors open");
      break;
  }
}
```
```rtdHandleButtonPressed()``` handles assigning a RTD state once the RTD button is pressed. It first logs the button press and the RTD state at the time of the button press. It then checks the RTD state. If when pressed the state was ```READY_FOR_BUTTON```, it checks if the brake is pressed. If not the code aborts. If it detects that the brake is pressed, it sets the state to ```READY_TO_DRIVE```, plays the corresponding sound, and prints the status update to serial monitor. </br>
If the state is already ```READY_TO_DRIVE``` it drops it with the reason ```"RTD button pressed"```. Finally, if the state is ```FAULT_BLOCKED```, or isn't ```READY_TO_DRIVE``` or ```READY_FOR_BUTTON```, it will set the state to ```FAULT_BLOCKED``` and output to serial monitor the reason why.</br>
If there are any active faults it'll output ```"[RTD] Not latched; anyFault is active"``` and if not ```"[RTD] Not latched; contactors open"```.
```cpp
void rtdSetState(RtdState newState) {
  char newDriveMode = (newState == RTD_STATE_READY_TO_DRIVE) ? 'D' : 'N';

  if (rtdState == newState) {
    isReadyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
    if (driveMode != newDriveMode) {
      rtdSetDriveMode(newDriveMode);
      isScreenDirty = true;
    }
    return;
  }

  rtdState = newState;
  isReadyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
  rtdSetDriveMode(newDriveMode);
  isScreenDirty = true;
}
```
This function handles setting the RTD state. If the input state is ```READY_TO_DRIVE```, the drive mode is ```'D'```. Otherwise the drive mode is ```'N'```, and it is assigned to ```newDriveMode```. If the inputted state is the same as the current state, and that state is ```READY_TO_DRIVE```, it sets ```isReadyToDriveLatched``` to true. </br>
If ```newDriveMode``` is different from the current drive mode, ```newDriveMode``` is set as the new current drive mode, and sets ```isScreenDirty``` true. </br>*does this mean the dashboard screen needs to updated?*
```cpp
// Names the specific condition currently blocking RTD, for serial logs and
// the dashboard fault banner.
const char *rtdGetDropCauseText() {
  if (isBmsTimedOut && hasBmsStatus) return "BMS Timeout";
  if (isImdCanTimedOut) return "IMD Timeout";
  if (hasBmsFault) {
    return (hasBms2026FaultCode && bms2026FaultCode != 0)
               ? faultTextGetBmsTemperatureCode(bms2026FaultCode)
               : "BMS Fault";
  }
  if (hasImdFault) return faultTextGetFirstTssi(imdWarnAlarms);
  if (bmsAreContactorsOpen()) return "Contactors Open";
  return "Fault";
}
```
This function generates a text message explaining the reason for a drop from ready-to-drive. It handles messages for BMS/IMD timeout, BMS fault, and open BMS contacts. If none of these are the reason for the drop it simply returns ```"Fault"```.
```cpp
const char *rtdGetStateName(RtdState state) {
  switch (state) {
    case RTD_STATE_FAULT_BLOCKED:
      return "FAULT_BLOCKED";
    case RTD_STATE_READY_FOR_BUTTON:
      return "READY_FOR_BUTTON";
    case RTD_STATE_READY_TO_DRIVE:
      return "READY_TO_DRIVE";
    default:
      return "UNKNOWN";
  }
}
```
This function generates and returns the various state names.
```cpp
void rtdSetDriveMode(char newDriveMode) {
  if (newDriveMode != 'D' && newDriveMode != 'N') {
    return;
  }

  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  isScreenDirty = true;
}
```
This function sets the drive mode. If an invalid input is detected (not ```'N'``` or ```'D'```), it aborts. It likewise aborts when there is no change in drivemode. If the the drive mode is changed it updates the current drive mode and sets ````isScreenDirty``` true.
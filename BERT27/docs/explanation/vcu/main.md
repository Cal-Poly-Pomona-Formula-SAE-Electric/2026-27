# Main
## setup()
```cpp
// ---------------- Setup ----------------
void setup() {
  startupFaultDelayStartMs = millis();

  Serial.begin(115200);

  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  fansSetup();

  pinMode(PIN_READY_BUTTON, INPUT);
  wasReadyButtonPressed = rtdReadButtonPressed();
  rtdRefreshStateFromInputs();

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(PIN_APPS_1, INPUT);
  pinMode(PIN_APPS_2, INPUT);
  pinMode(PIN_BSE_1, INPUT);
  pinMode(PIN_BSE_2, INPUT);
  pinMode(PIN_HV_CURRENT_SENSE, INPUT);

  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  digitalWrite(PIN_BRAKE_LIGHT, LOW);

  // Pulldowns so a disconnected status line reads LOW and shows red.
  pinMode(PIN_BSPD_STATUS, INPUT_PULLDOWN);
  pinMode(PIN_IMD_STATUS, INPUT_PULLDOWN);
  pinMode(PIN_BMS_STATUS, INPUT_PULLDOWN);

  tssiSetup();
  canSetup();
  inverterCmdSetupTimer();

  while (!Serial && millis() < 1500) {}

  Serial.println("BERT26 VCU starting");
  Serial.print("[Startup] BMS/IMD fault detection delayed for ");
  Serial.print(STARTUP_BMS_IMD_FAULT_DELAY_MS);
  Serial.println(" ms");

  imuSetup();

  logSetup();

  dashSetup();

  dashDraw();
}
```
The setup function initializes all hardware, sensor, communication buses, safeguards, display, and logging before the control loop begins.
## loop()
```consoleHandleInput()``` processses user input or debug input from Serial.</br>
```fansUpdateDutyFromTemps()``` ```...```</br>
```canReceiveFrames()``` receives messages from the CAN network, which includes status updates from the BMS, inverter, and other systems.</br>
```pedalsUpdateBrakePressureSensors()``` reads the brake pressure sensors and determines whether braking is active. ```pedalsUpdate()``` reads the accelerator pedal sensors, checks plausibility of data, applies deadband scaling, evaluates BSPC logic, and computes desired torque.<br>
```digitalWrite(PIN_BRAKE_LIGHT, isBraking ? HIGH : LOW);``` toggles the brake light based on whether the systems determines braking is active.<br>
```pedalsLogAppsTelemetry()``` ```...```
<br><br>

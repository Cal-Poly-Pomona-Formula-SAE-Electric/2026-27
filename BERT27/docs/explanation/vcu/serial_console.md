# Serial Console
## serial_console.cpp
### consoleHandleInput()
```cpp
void consoleHandleInput() {
  static char input[12];
  static uint8_t index = 0;

  while (Serial.available()) {
    char inputChar = (char)Serial.read();

    if (inputChar == '\n' || inputChar == '\r') {
      if (index > 0) {
        input[index] = '\0';
        index = 0;
        consoleHandleCommand(input);
      }
    } else if (index < sizeof(input) - 1) {
      input[index++] = inputChar;
    } else {
      index = 0;
    }
  }
}
```
The code first defines the length of the input as 12, initializing an index variable to ```0```. When the serial is available, the functions enters a loop. First it converts the serial input into ```char```, then it indexes through the char array. If it detects a newline or carriage return character and the index is greater than ```0```, it replaces that character with the null character ```\0```, marking the end of the char array. The index is then reset and the input is passed off to ```consoleHandleCommand()```. If no ```\n``` or ```\r``` character is detected, and the index is within the size of the ```char input[]```, ```inputChar``` is assigned as the value of the next index in ```input[]``` and the index is incremented by 1. If both these conditions fail, the index returns to 0.
### consoleHandleCommand()
```cpp
void consoleHandleCommand(const char *cmd) {
  if (strcmp(cmd, "a") == 0 || strcmp(cmd, "A") == 0) {
    isFanManualOverride = false;
    Serial.println("[Fans] Automatic temperature control resumed");
    return;
  }

  if (strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
    logListFiles();
    return;
  }

  if ((cmd[0] == 'd' || cmd[0] == 'D') && cmd[1] >= '0' && cmd[1] <= '9') {
    logDumpFile((uint32_t)atoi(cmd + 1));
    return;
  }

  bool isAllDigits = cmd[0] != '\0';
  for (const char *cmdCursor = cmd; *cmdCursor; cmdCursor++) {
    if (*cmdCursor < '0' || *cmdCursor > '9') {
      isAllDigits = false;
      break;
    }
  }

  if (isAllDigits) {
    int value = atoi(cmd);
    // Manual override: enter 0-100 to force both fan PWM duties.
    if (value >= 0 && value <= 100) {
      isFanManualOverride = true;
      fan1DutyPct = value;
      fan2DutyPct = value;
      Serial.print("[Fans] Manual override duty set to ");
      Serial.print(value);
      Serial.println("% (send 'a' for auto)");
      return;
    }
  }

  Serial.println("[Console] Commands: 0-100 fan duty, 'a' auto fans, 'l' list logs, 'd<n>' dump log n");
}
```
The function takes as input a pointer to c-style string cmd. It then compares whether the inputted command matches ```a``` or ```A``` (strcmp compares two c strings and returns ```0``` when they match). If either is true, the fan manual override is turned off and automatic temperature control resumes, and a message is sent through the serial to inform of the status change. It then exits the function</br>
It then checks whether the inputted command was an ```L/l```. If true, it executes ```logListFiles()``` then exits the function.</br>
The next check looks for input in the form of ```Dxx``` or ```dxx``` where ```x``` represents a number. It checks that the second digit is a valid number (between 0 and 9). Then it calls ```logDumpFile((uint32_t)atoi(cmd + 1))```. If sample input ```cmd = d42```, ```cmd + 1``` points to the second character ```4```, and the string becomes ```42``` (ignoring ```d```). This way the code extracts only the numeric portion of ```cmd```.</br>
The code then declares ```bool isAllDigits```. If ```cmd[]``` contains at least one character it assigns to ```true```, but if it's empty it assigns ```false```. The code initializes a loop with counter variable ```*cmdCursor``` that points to the first character of ```cmd```. It checks if each character is a valid number (0-9), and if it detects any non-digit character, ```isAllDigits``` is set ```false``` and the function returns.</br>
If the command is a number, its converted into an integer and assigned to ```value```. If the value is in between 0-100, ```isFanManualOverride``` is set ```true```, and the duty percent for Fan1 and Fan2 are assigned ```value```. The serial monitor then prints out a message displaying the assigned duties, as well as a reminder to input ```a``` for auto. The last line outputs a message to serial containing all valid commands.


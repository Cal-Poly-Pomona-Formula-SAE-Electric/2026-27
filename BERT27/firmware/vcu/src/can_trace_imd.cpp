/**
 * @file    can_trace_imd.cpp
 * @brief   Prints Bender iso175 IMD CAN frames as readable text on the serial console.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "can_trace_imd.h"

#include "can_bytes.h"
#include "imd.h"
#include "tssi.h"
#include "can_trace.h"

void tracePrintImdRequestIndexName(uint8_t requestIndex);
void tracePrintImdResponseErrorName(uint8_t errorCode);
void tracePrintImdInfoMessageSelectorName(uint8_t selector);
void tracePrintImdProfileName(uint8_t profile);
void tracePrintImdRIsoStatusName(uint8_t status);
void tracePrintImdDeviceActivityName(uint8_t activity);
void tracePrintImdLockStateName(uint8_t state);
void tracePrintImdEarthliftStateName(uint8_t state);
void tracePrintImdVoltageModeName(uint8_t mode);
void tracePrintImdBaudRateName(uint8_t baudRate);
void tracePrintImdAsciiPayloadFrom(const CAN_message_t &frame, uint8_t startIndex);
void tracePrintImdWordValue(uint16_t value);
void tracePrintImdResistanceValue(uint16_t resistanceKohm);
void tracePrintImdVoltageValue(uint16_t raw);
void tracePrintImdCapacityValue(uint16_t raw);
void tracePrintImdPercentValue(uint8_t value);
void tracePrintImdCounterValue(uint8_t value);

static constexpr int32_t IMD_VOLTAGE_OFFSET_RAW   = 32128;
static constexpr float IMD_VOLTAGE_V_PER_BIT      = 0.05f;

void tracePrintImdRequest(const CAN_message_t &frame) {
  if (frame.len < 1) {
    Serial.print("short frame, expected request index");
    return;
  }

  Serial.print("index=0x");
  tracePrintCanHexByte(frame.buf[0]);
  Serial.print(" (");
  tracePrintImdRequestIndexName(frame.buf[0]);
  Serial.print(") data=");
  tracePrintRawPayloadFrom(frame, 1);
}

void tracePrintImdResponse(const CAN_message_t &frame) {
  if (frame.len < 1) {
    Serial.print("short frame, expected response index");
    return;
  }

  uint8_t index = frame.buf[0];

  if (index == 0xFF) {
    Serial.print("error");
    if (frame.len >= 2) {
      Serial.print(" code=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      tracePrintImdResponseErrorName(frame.buf[1]);
      Serial.print(')');
    }
    if (frame.len >= 3) {
      Serial.print(" requestedIndex=0x");
      tracePrintCanHexByte(frame.buf[2]);
      Serial.print('(');
      tracePrintImdRequestIndexName(frame.buf[2]);
      Serial.print(')');
    }
    return;
  }

  Serial.print("index=0x");
  tracePrintCanHexByte(index);
  Serial.print('(');
  tracePrintImdRequestIndexName(index);
  Serial.print(") ");

  if (frame.len < 2) {
    Serial.print("missing response data");
    return;
  }

  uint16_t word = (frame.len >= 3) ? canReadU16Le(frame.buf[1], frame.buf[2]) : frame.buf[1];

  switch (index) {
    case 0x0A:
    case 0x0C:
    case 0x0E:
    case 0x1E:
    case 0x20:
    case 0x22:
      Serial.print("value=");
      tracePrintImdWordValue(word);
      break;

    case 0x10:
    case 0x12:
    case 0x14:
    case 0x16:
    case 0x18:
    case 0x1A:
    case 0x1C:
      Serial.print("ascii=");
      tracePrintImdAsciiPayloadFrom(frame, 1);
      break;

    case 0x2A:
    case 0x2E:
      Serial.print("percent=");
      tracePrintImdPercentValue(frame.buf[1]);
      break;

    case 0x2C:
    case 0x36:
    case 0x54:
    case 0x5C:
      Serial.print("counter=");
      tracePrintImdCounterValue(frame.buf[1]);
      break;

    case 0x30:
      Serial.print("selfHoldingAlarm=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      switch (frame.buf[1]) {
        case 0xFC: Serial.print("automatic alarm reset"); break;
        case 0xFD: Serial.print("self-holding alarm"); break;
        case 0xFE: Serial.print("reserved"); break;
        case IMD_SNV_BYTE: Serial.print("SNV"); break;
        default: Serial.print("unknown"); break;
      }
      Serial.print(')');
      break;

    case 0x38:
    case 0x3A:
      Serial.print("profile=");
      Serial.print(frame.buf[1]);
      Serial.print('(');
      tracePrintImdProfileName(frame.buf[1]);
      Serial.print(')');
      break;

    case 0x3E:
      Serial.print("quality=");
      tracePrintImdPercentValue(frame.buf[1]);
      break;

    case 0x40:
    case 0x42:
    case 0x46:
    case 0x4A:
    case 0x4C:
    case 0x4E:
    case 0x7E:
      Serial.print("resistance=");
      tracePrintImdResistanceValue(word);
      break;

    case 0x44:
      Serial.print("rIsoStatus=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      tracePrintImdRIsoStatusName(frame.buf[1]);
      Serial.print(')');
      break;

    case 0x48:
    case 0x50:
      Serial.print("seconds=");
      tracePrintImdWordValue(word);
      break;

    case 0x52:
      Serial.print("capacity=");
      tracePrintImdCapacityValue(word);
      break;

    case 0x58:
      Serial.print("selfTestPeriod=");
      if (word == 0) {
        Serial.print("disabled");
      } else if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((uint32_t)word * 10u);
        Serial.print("s");
      }
      break;

    case 0x5E:
    case 0x60:
    case 0x62:
      Serial.print("voltage=");
      tracePrintImdVoltageValue(word);
      break;

    case 0x64:
      Serial.print("mode=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      tracePrintImdVoltageModeName(frame.buf[1]);
      Serial.print(')');
      break;

    case 0x66:
    case 0x72:
      Serial.print("voltage=");
      tracePrintImdWordValue(word);
      Serial.print("V");
      break;

    case 0x68:
      Serial.print("deviceActivity=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      tracePrintImdDeviceActivityName(frame.buf[1]);
      Serial.print(')');
      break;

    case 0x6A:
    case 0x6B:
      Serial.print("lock=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      tracePrintImdLockStateName(frame.buf[1]);
      Serial.print(')');
      break;

    case 0x6C:
      Serial.print("warningsAndAlarms=0x");
      Serial.print(word, HEX);
      tssiPrintWarnBits(word);
      break;

    case 0x70:
    case 0x71:
      Serial.print("earthlift=0x");
      tracePrintCanHexByte(frame.buf[1]);
      Serial.print('(');
      tracePrintImdEarthliftStateName(frame.buf[1]);
      Serial.print(')');
      break;

    case 0x74:
      Serial.print("maxVoltageDifference=");
      if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((float)word * 0.01f, 2);
        Serial.print("V");
      }
      break;

    case 0x76:
    case 0x77:
      Serial.print("message=");
      if (frame.len >= 2) {
        tracePrintImdInfoMessageSelectorName(frame.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" canId=");
      if (frame.len >= 3) {
        Serial.print("0x");
        tracePrintCanHexByte(frame.buf[2]);
      } else {
        Serial.print("missing");
      }
      break;

    case 0x78:
    case 0x79:
      Serial.print("message=");
      if (frame.len >= 2) {
        tracePrintImdInfoMessageSelectorName(frame.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" period=");
      if (frame.len >= 3) {
        if (frame.buf[2] == 0) {
          Serial.print("deactivated");
        } else if (frame.buf[2] == IMD_SNV_BYTE) {
          Serial.print("SNV");
        } else {
          Serial.print((uint16_t)frame.buf[2] * 100u);
          Serial.print("ms");
        }
      } else {
        Serial.print("missing");
      }
      break;

    case 0x7B:
      Serial.print("baudRate=");
      tracePrintImdBaudRateName(frame.buf[1]);
      break;

    default:
      Serial.print("payload=");
      tracePrintRawPayloadFrom(frame, 1);
      break;
  }
}

void tracePrintImdInfoGeneral(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint16_t rIsoCorrectedKohm = canReadU16Le(frame.buf[0], frame.buf[1]);
  uint8_t rIsoStatus = frame.buf[2];
  uint8_t isolationCounter = frame.buf[3];
  uint8_t warningsLowByte = frame.buf[4];
  uint8_t reservedByte = frame.buf[5];
  uint8_t deviceActivity = frame.buf[6];

  Serial.print("rIsoCorrected=");
  tracePrintImdResistanceValue(rIsoCorrectedKohm);
  Serial.print(" rIsoStatus=0x");
  tracePrintCanHexByte(rIsoStatus);
  Serial.print('(');
  tracePrintImdRIsoStatusName(rIsoStatus);
  Serial.print(") isolationCounter=");
  tracePrintImdCounterValue(isolationCounter);
  Serial.print(" warningsLow=0x");
  tracePrintCanHexByte(warningsLowByte);
  tssiPrintWarnBits(warningsLowByte);
  Serial.print(" reserved5=0x");
  tracePrintCanHexByte(reservedByte);
  Serial.print(" deviceActivity=0x");
  tracePrintCanHexByte(deviceActivity);
  Serial.print('(');
  tracePrintImdDeviceActivityName(deviceActivity);
  Serial.print(") byte7=0x");
  tracePrintCanHexByte(frame.buf[7]);
}

void tracePrintImdInfoIsolationDetail(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("rIsoNeg=");
  tracePrintImdResistanceValue(canReadU16Le(frame.buf[0], frame.buf[1]));
  Serial.print(" rIsoPos=");
  tracePrintImdResistanceValue(canReadU16Le(frame.buf[2], frame.buf[3]));
  Serial.print(" rIsoOriginal=");
  tracePrintImdResistanceValue(canReadU16Le(frame.buf[4], frame.buf[5]));
  Serial.print(" isolationCounter=");
  tracePrintImdCounterValue(frame.buf[6]);
  Serial.print(" quality=");
  tracePrintImdPercentValue(frame.buf[7]);
}

void tracePrintImdInfoVoltage(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("hvSystem=");
  tracePrintImdVoltageValue(canReadU16Le(frame.buf[0], frame.buf[1]));
  Serial.print(" hvNegToEarth=");
  tracePrintImdVoltageValue(canReadU16Le(frame.buf[2], frame.buf[3]));
  Serial.print(" hvPosToEarth=");
  tracePrintImdVoltageValue(canReadU16Le(frame.buf[4], frame.buf[5]));
  Serial.print(" voltageCounter=");
  tracePrintImdCounterValue(frame.buf[6]);
  Serial.print(" byte7=0x");
  tracePrintCanHexByte(frame.buf[7]);
}

void tracePrintImdInfoItSystem(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("capacity=");
  tracePrintImdCapacityValue(canReadU16Le(frame.buf[0], frame.buf[1]));
  Serial.print(" capacityCounter=");
  tracePrintImdCounterValue(frame.buf[2]);
  Serial.print(" unbalance=");
  tracePrintImdPercentValue(frame.buf[3]);
  Serial.print(" unbalanceCounter=");
  tracePrintImdCounterValue(frame.buf[4]);
  Serial.print(" byte5=0x");
  tracePrintCanHexByte(frame.buf[5]);
  Serial.print(" byte6=0x");
  tracePrintCanHexByte(frame.buf[6]);
  Serial.print(" byte7=0x");
  tracePrintCanHexByte(frame.buf[7]);
}

void tracePrintImdRequestIndexName(uint8_t requestIndex) {
  switch (requestIndex) {
    case 0x0A: Serial.print("bootloader build number"); break;
    case 0x0C: Serial.print("bootloader D-number"); break;
    case 0x0E: Serial.print("bootloader version"); break;
    case 0x10: Serial.print("hardware AH history"); break;
    case 0x12: Serial.print("hardware AH number"); break;
    case 0x14: Serial.print("hardware AH number part B"); break;
    case 0x16: Serial.print("hardware item number"); break;
    case 0x18: Serial.print("hardware article number part B"); break;
    case 0x1A: Serial.print("serial number"); break;
    case 0x1C: Serial.print("serial number part B"); break;
    case 0x1E: Serial.print("software build number"); break;
    case 0x20: Serial.print("software D-number"); break;
    case 0x22: Serial.print("software version"); break;
    case 0x2A: Serial.print("unbalance measured value"); break;
    case 0x2C: Serial.print("unbalance measurement counter"); break;
    case 0x2E:
    case 0x2F: Serial.print("unbalance threshold"); break;
    case 0x30:
    case 0x31: Serial.print("self-holding alarm activation"); break;
    case 0x33: Serial.print("reset alarm"); break;
    case 0x36: Serial.print("isolation measurement counter"); break;
    case 0x38:
    case 0x39: Serial.print("isolation active profile"); break;
    case 0x3A:
    case 0x3B: Serial.print("isolation power-on profile"); break;
    case 0x3E: Serial.print("isolation quality"); break;
    case 0x40: Serial.print("R_iso_neg"); break;
    case 0x42: Serial.print("R_iso_pos"); break;
    case 0x44: Serial.print("R_iso_status"); break;
    case 0x46:
    case 0x47: Serial.print("isolation threshold error"); break;
    case 0x48:
    case 0x49: Serial.print("isolation measurement timeout threshold"); break;
    case 0x4A:
    case 0x4B: Serial.print("isolation threshold warning"); break;
    case 0x4C: Serial.print("R_iso_corrected"); break;
    case 0x4E: Serial.print("R_iso_original"); break;
    case 0x50: Serial.print("time since last isolation measurement"); break;
    case 0x52: Serial.print("capacity measured value"); break;
    case 0x54: Serial.print("capacity measurement counter"); break;
    case 0x57: Serial.print("trigger self test"); break;
    case 0x58:
    case 0x59: Serial.print("self test period"); break;
    case 0x5C: Serial.print("voltage measurement counter"); break;
    case 0x5E: Serial.print("HV system voltage"); break;
    case 0x60: Serial.print("HV negative to earth voltage"); break;
    case 0x62: Serial.print("HV positive to earth voltage"); break;
    case 0x64:
    case 0x65: Serial.print("voltage mode"); break;
    case 0x66:
    case 0x67: Serial.print("undervoltage threshold"); break;
    case 0x68: Serial.print("device activity"); break;
    case 0x6A:
    case 0x6B: Serial.print("parameter lock"); break;
    case IDX_WARN_ALARMS: Serial.print("warnings and alarms"); break;
    case 0x6F: Serial.print("factory reset"); break;
    case 0x70:
    case 0x71: Serial.print("earthlift status"); break;
    case 0x72:
    case 0x73: Serial.print("threshold first reference estimation"); break;
    case 0x74:
    case 0x75: Serial.print("pre-estimation max difference"); break;
    case 0x76:
    case 0x77: Serial.print("interface CAN-ID"); break;
    case 0x78:
    case 0x79: Serial.print("interface periodic cycle time"); break;
    case 0x7B: Serial.print("interface baudrate"); break;
    case 0x7E:
    case 0x7F: Serial.print("isolation initial value"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdResponseErrorName(uint8_t errorCode) {
  switch (errorCode) {
    case 0x23: Serial.print("unknown or invalid request"); break;
    case 0x24: Serial.print("set command failed, parameter locked"); break;
    case 0x25: Serial.print("data1 range overflow"); break;
    case 0x26: Serial.print("data2 range overflow"); break;
    case 0x27: Serial.print("CAN-ID already in use"); break;
    case 0x28: Serial.print("write failed"); break;
    case 0x29: Serial.print("read failed"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdInfoMessageSelectorName(uint8_t selector) {
  switch (selector) {
    case 0x00: Serial.print("Request"); break;
    case 0x01: Serial.print("Response"); break;
    case 0x02: Serial.print("IMD_Info_General"); break;
    case 0x03: Serial.print("IMD_Info_IsolationDetail"); break;
    case 0x04: Serial.print("IMD_Info_Voltage"); break;
    case 0x05: Serial.print("IMD_Info_IT-System"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdProfileName(uint8_t profile) {
  switch (profile) {
    case 1: Serial.print("standard fast startup"); break;
    case 2: Serial.print("standard"); break;
    case 3: Serial.print("high capacity fast startup"); break;
    case 4: Serial.print("high capacity"); break;
    case 5: Serial.print("disturbed"); break;
    case 6: Serial.print("service"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdRIsoStatusName(uint8_t status) {
  switch (status) {
    case 0xFC: Serial.print("startup estimated"); break;
    case 0xFD: Serial.print("startup first measurement"); break;
    case 0xFE: Serial.print("normal operation"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdDeviceActivityName(uint8_t activity) {
  switch (activity) {
    case 0: Serial.print("initialization"); break;
    case 1: Serial.print("normal operation"); break;
    case 2: Serial.print("self test"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdLockStateName(uint8_t state) {
  switch (state) {
    case 0xFC: Serial.print("parameter write enable"); break;
    case 0xFD: Serial.print("parameter write disable"); break;
    case 0xFE: Serial.print("reserved"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdEarthliftStateName(uint8_t state) {
  switch (state) {
    case 0:
    case 0xFC: Serial.print("earth connection closed"); break;
    case 1:
    case 0xFD: Serial.print("earth connection open"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdVoltageModeName(uint8_t mode) {
  switch (mode) {
    case 0xFD: Serial.print("AC"); break;
    case 0xFE: Serial.print("DC"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdBaudRateName(uint8_t baudRate) {
  switch (baudRate) {
    case 0x01: Serial.print("1 Mbit/s"); break;
    case 0x02: Serial.print("800 kbit/s"); break;
    case 0x03: Serial.print("666 kbit/s"); break;
    case 0x04: Serial.print("500 kbit/s"); break;
    case 0x05: Serial.print("250 kbit/s"); break;
    case 0x06: Serial.print("125 kbit/s"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintImdAsciiPayloadFrom(const CAN_message_t &frame, uint8_t startIndex) {
  Serial.print('"');
  for (uint8_t i = startIndex; i < frame.len; i++) {
    uint8_t value = frame.buf[i];
    if (value == IMD_SNV_BYTE) {
      break;
    }
    if (value >= 32 && value <= 126) {
      Serial.write(value);
    } else {
      Serial.print('.');
    }
  }
  Serial.print('"');
}

void tracePrintImdWordValue(uint16_t value) {
  if (value == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
  }
}

void tracePrintImdResistanceValue(uint16_t resistanceKohm) {
  if (resistanceKohm == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(resistanceKohm);
    Serial.print("kOhm");
  }
}

void tracePrintImdVoltageValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    float voltageV = ((int32_t)raw - IMD_VOLTAGE_OFFSET_RAW) * IMD_VOLTAGE_V_PER_BIT;
    Serial.print(voltageV, 2);
    Serial.print("V");
  }
}

void tracePrintImdCapacityValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print((float)raw * 0.1f, 1);
    Serial.print("uF");
  }
}

void tracePrintImdPercentValue(uint8_t value) {
  if (value == IMD_SNV_BYTE) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
    Serial.print('%');
  }
}

void tracePrintImdCounterValue(uint8_t value) {
  Serial.print(value);
}

/**
 * @file    can_trace.cpp
 * @brief   Prints CAN frames as readable text on the serial console.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "can_trace.h"

#include "can_bytes.h"
#include "imd.h"
#include "bms_link.h"
#include "vehicle_can.h"
#include "inverter_command.h"
#include "can_trace_imd.h"
#include "fault_text.h"

static constexpr bool IS_CAN_TRANSLATION_SERIAL_ENABLED = true;

static constexpr uint8_t CELL_DATA_REQUEST_UNKNOWN      = 0x00;

// Bender iso175 IMD/TSSI frames from VCU_TSSI_CODE_5_31_26.ino.
static constexpr uint32_t IMD_INFO_ISOLATION_DETAIL_ID = 0x38;
static constexpr uint32_t IMD_INFO_VOLTAGE_ID = 0x39;
static constexpr uint32_t IMD_INFO_IT_SYSTEM_ID = 0x3A;
static constexpr uint32_t IMD_FALLBACK_REQUEST_ID = 0x0FF;

uint8_t lastCellDataRequestType = CELL_DATA_REQUEST_UNKNOWN;

typedef void (*CanTranslationPrinter)(const CAN_message_t &frame);

struct CanFrameTranslation {
  uint32_t id;
  const char *name;
  CanTranslationPrinter print;
};

void tracePrintBmsStatus2026(const CAN_message_t &frame);
void tracePrintBmsTemperature2026(const CAN_message_t &frame);
void tracePrintCellDataRequest2026(const CAN_message_t &frame);
void tracePrintCellDataResponse2026(const CAN_message_t &frame);
void tracePrintPm100Command(const CAN_message_t &frame);
void tracePrintPm100Temperatures1(const CAN_message_t &frame);
void tracePrintPm100Temperatures3(const CAN_message_t &frame);
void tracePrintPm100MotorPosition(const CAN_message_t &frame);
void tracePrintPm100CurrentInfo(const CAN_message_t &frame);
void tracePrintPm100VoltageInfo(const CAN_message_t &frame);
void tracePrintPm100InternalVoltages(const CAN_message_t &frame);
void tracePrintPm100FaultCodes(const CAN_message_t &frame);
void tracePrintPm100TorqueCapability(const CAN_message_t &frame);
void tracePrintBmsTemperatureFaultCode(uint8_t faultCode);
void tracePrintCellDataRequestName(uint8_t requestType);
void tracePrintCellVoltageValue(uint16_t millivolts);

static const CanFrameTranslation CAN_FRAME_TRANSLATIONS[] = {
  {BMS_STATUS_2026_ID,        "BMS Status",        tracePrintBmsStatus2026},
  {BMS_TEMPERATURE_2026_ID,   "BMS Temperature",   tracePrintBmsTemperature2026},
  {BMS_CELL_REQUEST_2026_ID,  "Cell Data Request", tracePrintCellDataRequest2026},
  {BMS_CELL_RESPONSE_2026_ID, "Cell Data Response", tracePrintCellDataResponse2026},
  {IMD_REQUEST_ID, "IMD Request", tracePrintImdRequest},
  {IMD_FALLBACK_REQUEST_ID, "IMD Fallback Request", tracePrintImdRequest},
  {IMD_RESPONSE_ID, "IMD Response", tracePrintImdResponse},
  {IMD_INFO_GENERAL_ID, "IMD Info General", tracePrintImdInfoGeneral},
  {IMD_INFO_ISOLATION_DETAIL_ID, "IMD Info Isolation Detail", tracePrintImdInfoIsolationDetail},
  {IMD_INFO_VOLTAGE_ID, "IMD Info Voltage", tracePrintImdInfoVoltage},
  {IMD_INFO_IT_SYSTEM_ID, "IMD Info IT-System", tracePrintImdInfoItSystem},
  {PM100_CMD_ID, "PM100 Command", tracePrintPm100Command},
  {PM100_TEMPERATURE_1_FRAME_ID, "PM100 Temperatures #1", tracePrintPm100Temperatures1},
  {PM100_TEMPERATURE_3_FRAME_ID, "PM100 Temperatures #3", tracePrintPm100Temperatures3},
  {PM100_RPM_FRAME_ID, "PM100 Motor Position", tracePrintPm100MotorPosition},
  {PM100_CURRENT_INFO_FRAME_ID, "PM100 Current Information", tracePrintPm100CurrentInfo},
  {PM100_VOLTAGE_INFO_FRAME_ID, "PM100 Voltage Information", tracePrintPm100VoltageInfo},
  {PM100_INTERNAL_VOLTAGES_FRAME_ID, "PM100 Internal Voltages", tracePrintPm100InternalVoltages},
  {PM100_FAULT_CODES_FRAME_ID, "PM100 Fault Codes", tracePrintPm100FaultCodes},
  {PM100_TORQUE_CAPABILITY_FRAME_ID, "PM100 Torque Capability", tracePrintPm100TorqueCapability}
};

static constexpr uint8_t CAN_FRAME_TRANSLATION_COUNT =
    sizeof(CAN_FRAME_TRANSLATIONS) / sizeof(CAN_FRAME_TRANSLATIONS[0]);

// ---------------- CAN decoded logging ----------------
void traceUpdateState(const char *direction, const CAN_message_t &frame) {
  (void)direction;

  if (frame.flags.extended || frame.id != BMS_CELL_REQUEST_2026_ID || frame.len < 1) {
    return;
  }

  lastCellDataRequestType = frame.buf[0];
}

void traceLogToSerial(const char *direction, const CAN_message_t &frame) {
  if (!IS_CAN_TRANSLATION_SERIAL_ENABLED) {
    return;
  }

  for (uint8_t i = 0; i < CAN_FRAME_TRANSLATION_COUNT; i++) {
    const CanFrameTranslation &translation = CAN_FRAME_TRANSLATIONS[i];
    if (frame.id != translation.id) {
      continue;
    }

    Serial.print("[CAN ");
    Serial.print(direction);
    Serial.print(" DECODED] ");
    Serial.print(translation.name);
    Serial.print(": ");
    translation.print(frame);
    Serial.println();
    return;
  }
}

void tracePrintBmsStatus2026(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t flags = frame.buf[0];
  uint16_t minCellMv = canReadU16Be(frame.buf[1], frame.buf[2]);
  uint16_t maxCellMv = canReadU16Be(frame.buf[3], frame.buf[4]);
  uint16_t avgCellMv = canReadU16Be(frame.buf[5], frame.buf[6]);

  Serial.print("flags=0x");
  tracePrintCanHexByte(flags);
  Serial.print(" fault=");
  Serial.print((flags & (1u << 0)) ? "YES" : "NO");
  Serial.print(" posContactorClosed=");
  Serial.print((flags & (1u << 1)) ? "YES" : "NO");
  Serial.print(" chargeMode=");
  Serial.print((flags & (1u << 2)) ? "YES" : "NO");
  Serial.print(" minCell=");
  Serial.print(minCellMv);
  Serial.print("mV maxCell=");
  Serial.print(maxCellMv);
  Serial.print("mV avgCell=");
  Serial.print(avgCellMv);
  Serial.print("mV soc=");
  if (frame.buf[7] == 0xFF) {
    Serial.print("unknown");
  } else {
    Serial.print(frame.buf[7]);
    Serial.print('%');
  }
}

void tracePrintBmsTemperature2026(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t faultCode = frame.buf[3];
  uint16_t packCurrentRaw = canReadU16Be(frame.buf[4], frame.buf[5]);
  float packCurrentA = ((int32_t)packCurrentRaw - 3000) * 0.1f;

  Serial.print("minCellTemp=");
  Serial.print((int)frame.buf[0] - 40);
  Serial.print("C maxCellTemp=");
  Serial.print((int)frame.buf[1] - 40);
  Serial.print("C avgCellTemp=");
  Serial.print((int)frame.buf[2] - 40);
  Serial.print("C faultCode=");
  Serial.print(faultCode);
  Serial.print('(');
  tracePrintBmsTemperatureFaultCode(faultCode);
  Serial.print(") packCurrent=");
  Serial.print(packCurrentA, 1);
  Serial.print("A reserved=");
  tracePrintCanHexByte(frame.buf[6]);
  Serial.print(' ');
  tracePrintCanHexByte(frame.buf[7]);
}

void tracePrintCellDataRequest2026(const CAN_message_t &frame) {
  if (frame.len < 1) {
    Serial.print("short frame, expected request byte");
    return;
  }

  Serial.print("request=0x");
  tracePrintCanHexByte(frame.buf[0]);
  Serial.print(" (");
  tracePrintCellDataRequestName(frame.buf[0]);
  Serial.print(')');
}

void tracePrintCellDataResponse2026(const CAN_message_t &frame) {
  if (frame.len < 1) {
    Serial.print("short frame, expected frame index");
    return;
  }

  uint8_t frameIndex = frame.buf[0];

  Serial.print("frameIndex=");
  Serial.print(frameIndex);

  if (lastCellDataRequestType == CELL_DATA_REQUEST_VOLTAGES) {
    uint8_t availablePairs = (frame.len > 1) ? ((frame.len - 1) / 2) : 0;
    if (availablePairs > 3) {
      availablePairs = 3;
    }

    Serial.print(" mode=voltages cells=");
    for (uint8_t i = 0; i < availablePairs; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 3u + i + 1u);
      Serial.print('=');
      tracePrintCellVoltageValue(canReadU16Be(frame.buf[1 + (i * 2)], frame.buf[2 + (i * 2)]));
    }
    if (availablePairs == 0) {
      Serial.print("none");
    }
    if (frame.len > 7) {
      Serial.print(" extra=");
      tracePrintCanHexByte(frame.buf[7]);
    }
  } else if (lastCellDataRequestType == CELL_DATA_REQUEST_TEMPERATURES) {
    uint8_t availableTemps = (frame.len > 1) ? (frame.len - 1) : 0;
    if (availableTemps > 7) {
      availableTemps = 7;
    }

    Serial.print(" mode=temperatures cells=");
    for (uint8_t i = 0; i < availableTemps; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 7u + i + 1u);
      Serial.print('=');
      Serial.print((int)frame.buf[1 + i] - 40);
      Serial.print('C');
    }
    if (availableTemps == 0) {
      Serial.print("none");
    }
  } else {
    Serial.print(" mode=unknown lastRequest=0x");
    tracePrintCanHexByte(lastCellDataRequestType);
    Serial.print(" payload=");
    tracePrintRawPayloadFrom(frame, 1);
  }
}

void tracePrintPm100Command(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t torqueRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t speedCommandRpm = canReadS16Le(frame.buf[2], frame.buf[3]);
  uint8_t directionCommand = frame.buf[4];
  uint8_t inverterEnable = frame.buf[5];
  uint8_t inverterDischarge = frame.buf[6];
  uint8_t speedModeEnable = frame.buf[7];

  Serial.print("torqueCommand=");
  Serial.print((float)torqueRaw / 10.0f, 1);
  Serial.print("Nm speedCommand=");
  Serial.print(speedCommandRpm);
  Serial.print("rpm direction=0x");
  tracePrintCanHexByte(directionCommand);
  Serial.print('(');
  if (directionCommand == 0x00) {
    Serial.print("reverse/clockwise");
  } else if (directionCommand == 0x01) {
    Serial.print("forward/counterclockwise");
  } else {
    Serial.print("unknown");
  }
  Serial.print(") inverterEnable=");
  Serial.print(inverterEnable ? "ON" : "OFF");
  Serial.print(" inverterEnableRaw=0x");
  tracePrintCanHexByte(inverterEnable);
  Serial.print(" inverterDischarge=0x");
  tracePrintCanHexByte(inverterDischarge);
  Serial.print(" speedModeEnable=0x");
  tracePrintCanHexByte(speedModeEnable);
}

void tracePrintPm100Temperatures1(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t moduleATempRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t moduleBTempRaw = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t moduleCTempRaw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t gateDriverTempRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  Serial.print("moduleA=");
  Serial.print((float)moduleATempRaw / 10.0f, 1);
  Serial.print("C moduleB=");
  Serial.print((float)moduleBTempRaw / 10.0f, 1);
  Serial.print("C moduleC=");
  Serial.print((float)moduleCTempRaw / 10.0f, 1);
  Serial.print("C gateDriver=");
  Serial.print((float)gateDriverTempRaw / 10.0f, 1);
  Serial.print("C");
}

void tracePrintPm100Temperatures3(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t coolantTempRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t hotSpotTempRaw = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t motorTempRaw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t torqueShudderRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  Serial.print("coolant=");
  Serial.print((float)coolantTempRaw / 10.0f, 1);
  Serial.print("C hotSpot=");
  Serial.print((float)hotSpotTempRaw / 10.0f, 1);
  Serial.print("C motor=");
  Serial.print((float)motorTempRaw / 10.0f, 1);
  Serial.print("C torqueShudder=");
  Serial.print((float)torqueShudderRaw / 10.0f, 1);
  Serial.print("Nm");
}

void tracePrintPm100MotorPosition(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t motorAngleRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t motorSpeedRpm = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t electricalFrequencyRaw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t deltaResolverRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  Serial.print("motorAngle=");
  Serial.print((float)motorAngleRaw / 10.0f, 1);
  Serial.print("deg motorSpeed=");
  Serial.print(motorSpeedRpm);
  Serial.print("rpm electricalFrequency=");
  Serial.print((float)electricalFrequencyRaw / 10.0f, 1);
  Serial.print("Hz deltaResolver=");
  Serial.print((float)deltaResolverRaw / 10.0f, 1);
  Serial.print("deg");
}

void tracePrintPm100CurrentInfo(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t phaseACurrentRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t phaseBCurrentRaw = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t phaseCCurrentRaw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t dcBusCurrentRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  Serial.print("phaseA=");
  Serial.print((float)phaseACurrentRaw / 10.0f, 1);
  Serial.print("A phaseB=");
  Serial.print((float)phaseBCurrentRaw / 10.0f, 1);
  Serial.print("A phaseC=");
  Serial.print((float)phaseCCurrentRaw / 10.0f, 1);
  Serial.print("A dcBus=");
  Serial.print((float)dcBusCurrentRaw / 10.0f, 1);
  Serial.print("A");
}

void tracePrintPm100VoltageInfo(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t dcBusVoltageRaw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t outputVoltageRaw = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t vabVdVoltageRaw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t vbcVqVoltageRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  Serial.print("dcBus=");
  Serial.print((float)dcBusVoltageRaw / 10.0f, 1);
  Serial.print("V output=");
  Serial.print((float)outputVoltageRaw / 10.0f, 1);
  Serial.print("V vabOrVd=");
  Serial.print((float)vabVdVoltageRaw / 10.0f, 1);
  Serial.print("V vbcOrVq=");
  Serial.print((float)vbcVqVoltageRaw / 10.0f, 1);
  Serial.print("V");
}

void tracePrintPm100InternalVoltages(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t ref1V5Raw = canReadS16Le(frame.buf[0], frame.buf[1]);
  int16_t ref2V5Raw = canReadS16Le(frame.buf[2], frame.buf[3]);
  int16_t ref5V0Raw = canReadS16Le(frame.buf[4], frame.buf[5]);
  int16_t glvRaw = canReadS16Le(frame.buf[6], frame.buf[7]);

  Serial.print("ref1V5=");
  Serial.print((float)ref1V5Raw / 100.0f, 2);
  Serial.print("V ref2V5=");
  Serial.print((float)ref2V5Raw / 100.0f, 2);
  Serial.print("V ref5V0=");
  Serial.print((float)ref5V0Raw / 100.0f, 2);
  Serial.print("V glv12V=");
  Serial.print((float)glvRaw / 100.0f, 2);
  Serial.print("V");
}

void tracePrintPm100FaultCodes(const CAN_message_t &frame) {
  if (frame.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint32_t postFaults = canReadU32Le(frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3]);
  uint32_t runFaults = canReadU32Le(frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);

  Serial.print("post=0x");
  Serial.print((unsigned long)postFaults, HEX);
  Serial.print(" run=0x");
  Serial.print((unsigned long)runFaults, HEX);
  Serial.print(" active=");

  bool hasPrintedFault = false;
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      if (hasPrintedFault) Serial.print(',');
      Serial.print(faultTextGetPm100Post(bit));
      hasPrintedFault = true;
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      if (hasPrintedFault) Serial.print(',');
      Serial.print(faultTextGetPm100Run(bit));
      hasPrintedFault = true;
    }
  }

  if (!hasPrintedFault) {
    Serial.print("none");
  }
}

void tracePrintPm100TorqueCapability(const CAN_message_t &frame) {
  if (frame.len < 2) {
    Serial.print("short frame, expected at least 2 bytes");
    return;
  }

  int16_t positiveTorqueRaw = canReadS16Le(frame.buf[0], frame.buf[1]);

  Serial.print("positiveTorqueCapability=");
  Serial.print((float)positiveTorqueRaw / 10.0f, 1);
  Serial.print("Nm");

  if (frame.len >= 4) {
    int16_t negativeTorqueRaw = canReadS16Le(frame.buf[2], frame.buf[3]);
    Serial.print(" possibleNegativeTorqueCapability=");
    Serial.print((float)negativeTorqueRaw / 10.0f, 1);
    Serial.print("Nm");
  }

  if (frame.len > 4) {
    Serial.print(" reserved=");
    tracePrintRawPayloadFrom(frame, 4);
  }
}

void tracePrintRawPayloadFrom(const CAN_message_t &frame, uint8_t startIndex) {
  if (startIndex >= frame.len) {
    Serial.print("none");
    return;
  }

  for (uint8_t i = startIndex; i < frame.len; i++) {
    if (i > startIndex) {
      Serial.print(' ');
    }
    tracePrintCanHexByte(frame.buf[i]);
  }
}

void tracePrintBmsTemperatureFaultCode(uint8_t faultCode) {
  Serial.print(faultTextGetBmsTemperatureCode(faultCode));
}

void tracePrintCellDataRequestName(uint8_t requestType) {
  switch (requestType) {
    case CELL_DATA_REQUEST_VOLTAGES: Serial.print("cell voltages"); break;
    case CELL_DATA_REQUEST_TEMPERATURES: Serial.print("cell temperatures"); break;
    default: Serial.print("unknown"); break;
  }
}

void tracePrintCellVoltageValue(uint16_t millivolts) {
  if (millivolts == 0xFFFF) {
    Serial.print("pad");
  } else {
    Serial.print(millivolts);
    Serial.print("mV");
  }
}

void tracePrintCanHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

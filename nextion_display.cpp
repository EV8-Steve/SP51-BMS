/*
  Nextion live-data sender for SP51_basic_BMS.

  This preserves the component IDs and command order used by the SimpBMS
  dashupdate() routine. Numeric fields keep their original scales; the two
  YES/NO indicators use the Nextion text attribute (.txt) because they carry
  text values. The Renesas SoftwareSerial library transmits one byte
  synchronously, so the completed frame is paced one byte per main-loop pass
  instead of blocking the BMS loop for an entire display refresh.
*/

#include "nextion_display.h"

#include "batterycomms.h"
#include <SoftwareSerial.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

namespace {
constexpr uint8_t NEXTION_RX_PIN = 4;
constexpr uint8_t NEXTION_TX_PIN = 5;
constexpr uint32_t NEXTION_BAUD = 9600UL;
constexpr size_t NEXTION_FRAME_CAPACITY = 512U;
constexpr uint16_t NEXTION_FIRMWARE_VERSION = 1U;
constexpr size_t NEXTION_STATUS_CAPACITY = 32U;

SoftwareSerial nextionSerial(NEXTION_RX_PIN, NEXTION_TX_PIN);
char nextionFrame[NEXTION_FRAME_CAPACITY];
size_t nextionFrameLength = 0;
size_t nextionFrameOffset = 0;
bool nextionReady = false;
bool nextionWasBlockedByPinConflict = false;
bool nextionStatusDirty = false;
char nextionStackStatus[NEXTION_STATUS_CAPACITY] = {};
char nextionLastStateText[16] = " Boot ";

bool inputUsesNextionPin(uint8_t pin) {
  return pin == NEXTION_RX_PIN || pin == NEXTION_TX_PIN;
}

bool hasNextionPinConflict(const BmsSettings &settings) {
  return inputUsesNextionPin(settings.runInputPin) ||
         inputUsesNextionPin(settings.chargeInputPin) ||
         inputUsesNextionPin(settings.isolationMonitorPin);
}

const char *stateText(BmsOperatingState state) {
  // Match the legacy dash labels. Disarmed maps to Boot; Fault maps to Error.
  switch (state) {
    case BmsOperatingState::Disarmed: return " Boot ";
    case BmsOperatingState::Ready: return " Ready ";
    case BmsOperatingState::Precharge: return " Precharge ";
    case BmsOperatingState::Drive: return " Drive ";
    case BmsOperatingState::Charge: return " Charge ";
    case BmsOperatingState::Fault: return " Error ";
    default: return " Error ";
  }
}

bool appendCommand(const char *format, ...) {
  const size_t remaining = sizeof(nextionFrame) - nextionFrameLength;
  if (remaining <= 3U) return false;

  // Leave three bytes for Nextion's required 0xFF command terminator.
  const size_t commandCapacity = remaining - 3U;
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(nextionFrame + nextionFrameLength,
                                commandCapacity, format, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= commandCapacity) {
    return false;
  }

  nextionFrameLength += static_cast<size_t>(written);
  nextionFrame[nextionFrameLength++] = static_cast<char>(0xFFU);
  nextionFrame[nextionFrameLength++] = static_cast<char>(0xFFU);
  nextionFrame[nextionFrameLength++] = static_cast<char>(0xFFU);
  return true;
}

bool appendNumber(const char *component, int32_t value) {
  return appendCommand("%s.val=%ld", component, static_cast<long>(value));
}

bool appendText(const char *component, const char *value) {
  return appendCommand("%s.txt=\"%s\"", component, value);
}

bool completeCellPack(const BatterySnapshot &snapshot,
                      const BmsSettings &settings,
                      const BatteryLimitStatus &limits,
                      float &packMillivolts) {
  if (!limits.readingsValid || snapshot.moduleCount != settings.stackModules ||
      snapshot.validCellCount !=
          static_cast<uint16_t>(settings.stackModules) * settings.cellsPerModule) {
    return false;
  }

  packMillivolts = 0.0f;
  for (uint8_t moduleIndex = 0; moduleIndex < snapshot.moduleCount;
       ++moduleIndex) {
    const ModuleReading &module = snapshot.modules[moduleIndex];
    if (!module.cellReadOk || module.validCellCount != settings.cellsPerModule) {
      return false;
    }
    for (uint8_t cell = 0; cell < settings.cellsPerModule; ++cell) {
      packMillivolts += module.cellMv[cell];
    }
  }
  return true;
}

void collectTemperatures(const BatterySnapshot &snapshot,
                         int32_t &averageC, int32_t &lowC, int32_t &highC) {
  float sumC = 0.0f;
  float minimumC = 0.0f;
  float maximumC = 0.0f;
  uint16_t count = 0;

  const uint8_t moduleCount = snapshot.moduleCount < BATTERY_MAX_STACK_MODULES
      ? snapshot.moduleCount : BATTERY_MAX_STACK_MODULES;
  for (uint8_t moduleIndex = 0; moduleIndex < moduleCount; ++moduleIndex) {
    const ModuleReading &module = snapshot.modules[moduleIndex];
    for (uint8_t channel = 0; channel < BATTERY_GPIO_COUNT; ++channel) {
      if (!(module.ntcEstimateMask & static_cast<uint8_t>(1U << channel))) {
        continue;
      }
      const float temperature = module.temperatureC[channel];
      if (count == 0U || temperature < minimumC) minimumC = temperature;
      if (count == 0U || temperature > maximumC) maximumC = temperature;
      sumC += temperature;
      ++count;
    }
  }

  if (count == 0U) {
    averageC = 0;
    lowC = 0;
    highC = 0;
    return;
  }

  averageC = static_cast<int32_t>(lroundf(sumC / count));
  lowC = static_cast<int32_t>(lroundf(minimumC));
  highC = static_cast<int32_t>(lroundf(maximumC));
}
}  // namespace

bool nextionDisplayBegin(const BmsSettings &settings) {
  if (hasNextionPinConflict(settings)) {
    nextionWasBlockedByPinConflict = true;
    nextionReady = false;
    return false;
  }

  nextionWasBlockedByPinConflict = false;
  nextionReady = nextionSerial.begin(NEXTION_BAUD, SERIAL_8N1) == 1;
  return nextionReady;
}

void nextionDisplaySetStackStatus(const char *statusText) {
  const char *safeText = statusText ? statusText : "";
  if (strncmp(nextionStackStatus, safeText,
              sizeof(nextionStackStatus)) == 0) {
    return;
  }
  snprintf(nextionStackStatus, sizeof(nextionStackStatus), "%s", safeText);
  nextionStatusDirty = true;
}

bool nextionDisplayQueue(const BatterySnapshot &snapshot,
                         const BmsSettings &settings,
                         const BatteryLimitStatus &limits,
                         const CurrentSensorStatus &current,
                         BmsOperatingState state,
                         bool acPresent,
                         const SocStatus &soc,
                         const OutlanderChargerStatus &charger) {
  if (!nextionReady || nextionFrameOffset < nextionFrameLength) return false;

  nextionFrameLength = 0;
  nextionFrameOffset = 0;

  float packMillivolts = 0.0f;
  const bool packValid =
      completeCellPack(snapshot, settings, limits, packMillivolts);
  int32_t averageTemperatureC = 0;
  int32_t lowTemperatureC = 0;
  int32_t highTemperatureC = 0;
  collectTemperatures(snapshot, averageTemperatureC, lowTemperatureC,
                      highTemperatureC);

  const int32_t socPercent = static_cast<int32_t>(lroundf(soc.percent));
  const int32_t currentDeciAmp = current.valid
      ? static_cast<int32_t>(lroundf(current.currentMilliamp / 100.0f)) : 0;
  const int32_t lowCellMv = limits.readingsValid
      ? static_cast<int32_t>(lroundf(limits.lowestCellV * 1000.0f)) : 0;
  const int32_t highCellMv = limits.readingsValid
      ? static_cast<int32_t>(lroundf(limits.highestCellV * 1000.0f)) : 0;
  const int32_t cellDeltaMv = limits.readingsValid
      ? highCellMv - lowCellMv : 0;
  const int32_t packDecivolts = packValid
      ? static_cast<int32_t>(lroundf(packMillivolts / 100.0f)) : 0;
  const int32_t chargerDeciAmps =
      static_cast<int32_t>(lroundf(charger.dcChargeCurrentA * 10.0f));

  // Keep the command names and order used by dashupdate(). The original
  // cellbal/ac YES/NO fields are text indicators, so they use .txt attributes.
  snprintf(nextionLastStateText, sizeof(nextionLastStateText), "%s",
           stateText(state));
  const char *displayStatus = nextionStackStatus[0]
      ? nextionStackStatus : nextionLastStateText;
  const bool ok = appendCommand("stat.txt=\"%s\"", displayStatus) &&
      appendNumber("soc", socPercent) &&
      appendNumber("soc1", socPercent) &&
      appendNumber("current", currentDeciAmp) &&
      appendNumber("temp", averageTemperatureC) &&
      appendNumber("templow", lowTemperatureC) &&
      appendNumber("temphigh", highTemperatureC) &&
      appendNumber("volt", packDecivolts) &&
      appendNumber("lowcell", lowCellMv) &&
      appendNumber("highcell", highCellMv) &&
      appendNumber("firm", NEXTION_FIRMWARE_VERSION) &&
      appendNumber("celldelta", cellDeltaMv) &&
      appendNumber("chargercurrent", chargerDeciAmps) &&
      appendNumber("chargervolts", charger.hvBatteryVoltageV) &&
      appendNumber("chargertemp", charger.chargerTemp2C) &&
      appendText("cellbal", batteryCommsBalanceIsActive() ? "YES" : "NO") &&
      appendText("ac", acPresent ? "YES" : "NO");

  if (!ok) {
    nextionFrameLength = 0;
    nextionFrameOffset = 0;
    return false;
  }
  nextionStatusDirty = false;
  return true;
}

void nextionDisplayService() {
  if (!nextionReady && nextionWasBlockedByPinConflict &&
      !hasNextionPinConflict(settingsGet())) {
    // Let a user correct a legacy D4/D5 input mapping in the Outputs menu
    // without requiring a reboot before the display link comes up.
    nextionDisplayBegin(settingsGet());
  }

  if (!nextionReady) return;

  if (nextionFrameOffset >= nextionFrameLength && nextionStatusDirty) {
    nextionFrameLength = 0;
    nextionFrameOffset = 0;
    const char *displayStatus = nextionStackStatus[0]
        ? nextionStackStatus : nextionLastStateText;
    if (!appendCommand("stat.txt=\"%s\"", displayStatus)) return;
    nextionStatusDirty = false;
  }
  if (nextionFrameOffset >= nextionFrameLength) return;

  // SoftwareSerial.write waits for the current byte to finish. One byte per
  // loop keeps CAN polling and the BMS tasks responsive between characters.
  nextionSerial.write(static_cast<uint8_t>(nextionFrame[nextionFrameOffset++]));
  if (nextionFrameOffset >= nextionFrameLength) {
    nextionFrameLength = 0;
    nextionFrameOffset = 0;
  }
}

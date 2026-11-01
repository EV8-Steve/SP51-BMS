/*
  Read-only LEM CAB500-C/SP7 CAN input.

  This module starts the Nano R4 CAN peripheral when the LEM sensor is selected,
  filters the configured standard CAN ID, decodes current frames, and marks data
  stale after its configured timeout. It never transmits CAN frames.
*/
#include "battery_can.h"

#include <Arduino_CAN.h>

namespace {
constexpr uint16_t CAN_RETRY_MS = 1000;
constexpr uint32_t CURRENT_ZERO_CODE = 0x80000000UL;

CurrentSensorStatus sensorStatus = {};
bool canStarted = false;
uint32_t activeBitrate = 0;
uint32_t lastBeginAttemptMs = 0;

bool sensorNeedsCan(const BmsSettings &settings) {
  return settings.currentSensor == CurrentSensorType::LemCab500Can;
}

void beginCanIfNeeded(const BmsSettings &settings) {
  if (!sensorNeedsCan(settings)) {
    if (canStarted) {
      CAN.end();
      canStarted = false;
      activeBitrate = 0;
    }
    sensorStatus.canReady = false;
    return;
  }

  if (canStarted && activeBitrate == settings.currentCanBitrate) {
    sensorStatus.canReady = true;
    return;
  }

  const uint32_t now = millis();
  if (!canStarted && now - lastBeginAttemptMs < CAN_RETRY_MS) return;
  lastBeginAttemptMs = now;

  if (canStarted) {
    CAN.end();
    canStarted = false;
  }
  canStarted = CAN.begin(settings.currentCanBitrate);
  activeBitrate = canStarted ? settings.currentCanBitrate : 0;
  sensorStatus.canReady = canStarted;
}

void acceptCurrentFrame(const CanMsg &message) {
  if (message.data_length != 8) {
    ++sensorStatus.invalidFrameCount;
    return;
  }

  // LEM current is a big-endian unsigned word centred at 0x80000000:
  // subtracting that zero point produces signed milliamps.
  const uint32_t rawCurrent =
      (static_cast<uint32_t>(message.data[0]) << 24) |
      (static_cast<uint32_t>(message.data[1]) << 16) |
      (static_cast<uint32_t>(message.data[2]) << 8) |
      static_cast<uint32_t>(message.data[3]);

  // LEM encodes 0 mA as 0x80000000, +1 mA as 0x80000001 and -1 mA as
  // 0x7FFFFFFF. The status byte uses its most significant bit for error.
  const int64_t decodedMilliamp =
      static_cast<int64_t>(rawCurrent) - static_cast<int64_t>(CURRENT_ZERO_CODE);
  const uint8_t status = message.data[4];
  sensorStatus.sensorFault = (status & 0x80U) != 0;
  sensorStatus.errorCode = static_cast<uint8_t>(status & 0x7FU);
  sensorStatus.currentMilliamp = static_cast<int32_t>(decodedMilliamp);
  sensorStatus.lastFrameAtMs = millis();
  sensorStatus.frameSeen = true;
  ++sensorStatus.frameCount;

  if (sensorStatus.sensorFault || rawCurrent == 0xFFFFFFFFUL ||
      decodedMilliamp < -600000LL || decodedMilliamp > 600000LL) {
    ++sensorStatus.invalidFrameCount;
    sensorStatus.valid = false;
  } else {
    sensorStatus.valid = true;
  }
}
}

void batteryCanBegin(const BmsSettings &settings) {
  sensorStatus = {};
  lastBeginAttemptMs = millis() - CAN_RETRY_MS;
  beginCanIfNeeded(settings);
}

void batteryCanPoll(const BmsSettings &settings) {
  beginCanIfNeeded(settings);
  if (!canStarted || !sensorNeedsCan(settings)) return;

  while (CAN.available()) {
    const CanMsg message = CAN.read();
    if (!message.isStandardId() ||
        message.getStandardId() != settings.currentCanId) {
      continue;
    }
    acceptCurrentFrame(message);
  }
}

CurrentSensorStatus batteryCanCurrent(const BmsSettings &settings) {
  // Return a snapshot, applying timeout and optional sign inversion without
  // changing the last decoded raw sensor state.
  CurrentSensorStatus result = sensorStatus;
  if (!sensorNeedsCan(settings)) {
    result.valid = false;
    result.canReady = false;
  } else if (!result.frameSeen ||
             millis() - result.lastFrameAtMs > settings.currentSensorTimeoutMs) {
    result.valid = false;
  }
  if (result.valid && settings.invertCurrent) {
    result.currentMilliamp = -result.currentMilliamp;
  }
  return result;
}

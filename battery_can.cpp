/*
  Shared Nano R4 CAN bus for the LEM CAB500-C/SP7 sensor and Outlander OBC.

  This module owns CAN begin/read/write so both devices share one controller.
  It decodes the configured LEM frame, forwards Outlander status frames, and
  queues standard transmissions only when the transmit mailbox is ready.
*/
#include "battery_can.h"

#include <Arduino_CAN.h>

#include "outlander_charger.h"

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

bool chargerNeedsCan(const BmsSettings &settings) {
  return settings.chargerType == ChargerType::Outlander;
}

bool canBusNeeded(const BmsSettings &settings) {
  return sensorNeedsCan(settings) || chargerNeedsCan(settings);
}

uint32_t requestedCanBitrate(const BmsSettings &settings) {
  // The LEM and Outlander OBC are attached to one physical CAN segment.
  return chargerNeedsCan(settings) ? settings.chargerCanBitrate
                                   : settings.currentCanBitrate;
}

void beginCanIfNeeded(const BmsSettings &settings) {
  if (!canBusNeeded(settings)) {
    if (canStarted) {
      CAN.end();
      canStarted = false;
      activeBitrate = 0;
    }
    sensorStatus.canReady = false;
    return;
  }

  const uint32_t bitrate = requestedCanBitrate(settings);
  if (chargerNeedsCan(settings) &&
      settings.currentCanBitrate != settings.chargerCanBitrate) {
    if (canStarted) {
      CAN.end();
      canStarted = false;
      activeBitrate = 0;
    }
    sensorStatus.canReady = false;
    return;
  }

  if (canStarted && activeBitrate == bitrate) {
    sensorStatus.canReady = sensorNeedsCan(settings);
    return;
  }

  const uint32_t now = millis();
  if (!canStarted && now - lastBeginAttemptMs < CAN_RETRY_MS) return;
  lastBeginAttemptMs = now;

  if (canStarted) {
    CAN.end();
    canStarted = false;
  }
  canStarted = CAN.begin(bitrate);
  activeBitrate = canStarted ? bitrate : 0;
  sensorStatus.canReady = canStarted && sensorNeedsCan(settings);
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
  // Outlander telemetry must still be drained when the CAN controller is
  // enabled for the charger but the optional LEM sensor is turned off.
  if (!canStarted) return;

  while (CAN.available()) {
    const CanMsg message = CAN.read();
    if (!message.isStandardId()) continue;

    const uint16_t id = static_cast<uint16_t>(message.getStandardId());
    if (sensorNeedsCan(settings) && id == settings.currentCanId) {
      acceptCurrentFrame(message);
    }
    if (chargerNeedsCan(settings) && (id == 0x389U || id == 0x38AU)) {
      outlanderChargerAcceptCanFrame(id, message.data_length, message.data);
    }
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

bool batteryCanBusReady() {
  return canStarted;
}

bool batteryCanSendStandardFrame(uint16_t id, const uint8_t *data,
                                 uint8_t length) {
  if (!canStarted || id > 0x7FFU || length > CanMsg::MAX_DATA_LENGTH ||
      (length > 0U && !data) || !CAN.txReady()) return false;
  const CanMsg message(arduino::CanStandardId(id), length, data);
  return CAN.write(message) > 0;
}

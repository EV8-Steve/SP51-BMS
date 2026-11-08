/*
  Mitsubishi Outlander onboard-charger CAN support.

  This module decodes charger reports and creates the same 0x285 wake and
  0x286 charge-request frames used by the supplied SimpBMS sketch. The shared
  CAN driver remains owned by battery_can.cpp so LEM and OBC frames coexist.
*/
#include "outlander_charger.h"

#include <math.h>

#include "battery_can.h"

namespace {
constexpr uint16_t OUTLANDER_WAKE_ID = 0x285U;
constexpr uint16_t OUTLANDER_COMMAND_ID = 0x286U;
constexpr uint16_t OUTLANDER_CHARGER_STATUS_ID = 0x389U;
constexpr uint16_t OUTLANDER_SECONDARY_STATUS_ID = 0x38AU;
constexpr uint32_t OUTLANDER_STATUS_TIMEOUT_MS = 500UL;
constexpr uint8_t OUTLANDER_MAX_CURRENT_DECIAMP = 120U;

bool charger389Seen = false;
bool charger38ASeen = false;
uint32_t charger389AtMs = 0;
uint32_t charger38AAtMs = 0;
uint32_t lastTxAtMs = 0;
bool txIntervalStarted = false;
OutlanderChargerStatus chargerStatus = {};

bool queueStandardFrame(uint16_t id, const uint8_t data[8],
                        bool canBusReady, uint8_t txMode) {
  if (!canBusReady) return false;
  if (!batteryCanSendStandardFrame(id, data, 8U)) {
    ++chargerStatus.txBusyCount;
    return false;
  }
  lastTxAtMs = millis();
  txIntervalStarted = true;
  chargerStatus.lastTxMode = txMode;
  ++chargerStatus.txFrameCount;
  return true;
}

bool sendWake(bool canBusReady) {
  // SimpBMS Outlander wake ping: byte 2 is 0xB6, all remaining bytes zero.
  const uint8_t data[8] = {0x00U, 0x00U, 0xB6U, 0x00U,
                           0x00U, 0x00U, 0x00U, 0x00U};
  return queueStandardFrame(OUTLANDER_WAKE_ID, data, canBusReady, 1U);
}
}

void outlanderChargerBegin() {
  charger389Seen = false;
  charger38ASeen = false;
  charger389AtMs = 0;
  charger38AAtMs = 0;
  lastTxAtMs = 0;
  txIntervalStarted = false;
  chargerStatus = {};
}

void outlanderChargerAcceptCanFrame(uint16_t id, uint8_t length,
                                    const uint8_t *data) {
  if (!data || (id != OUTLANDER_CHARGER_STATUS_ID &&
                id != OUTLANDER_SECONDARY_STATUS_ID)) return;
  if (length != 8U) {
    ++chargerStatus.invalidFrameCount;
    return;
  }

  const uint32_t now = millis();
  ++chargerStatus.frameCount;
  if (id == OUTLANDER_CHARGER_STATUS_ID) {
    // 0x389: HV volts are 2 V/bit; AC volts are 1 V/bit; current is 0.1 A/bit.
    charger389Seen = true;
    charger389AtMs = now;
    chargerStatus.hvBatteryVoltageV = static_cast<uint16_t>(data[0]) * 2U;
    chargerStatus.acMainsVoltageV = data[1];
    chargerStatus.dcChargeCurrentA = static_cast<float>(data[2]) * 0.1f;
    chargerStatus.chargerTemp1C = static_cast<int16_t>(data[3]) - 40;
    chargerStatus.chargerTemp2C = static_cast<int16_t>(data[4]) - 40;
    chargerStatus.status389 = data[5];
    chargerStatus.acMainsCurrentA = static_cast<float>(data[6]) * 0.1f;
    chargerStatus.dcChargeCurrent2A = static_cast<float>(data[7]) * 0.1f;
  } else {
    // 0x38A carries two additional temperatures, DC bus volts, EVSE pilot and status.
    charger38ASeen = true;
    charger38AAtMs = now;
    // This sensor tracks the other temperatures with an additional +5 C raw offset.
    chargerStatus.chargerTemp3C = static_cast<int16_t>(data[0]) - 45;
    chargerStatus.chargerTemp4C = static_cast<int16_t>(data[1]) - 40;
    chargerStatus.dcBusVoltageV = static_cast<uint16_t>(data[2]) * 2U;
    chargerStatus.evsePilotPercent = data[3];
    chargerStatus.status38A = data[4];
  }
}

OutlanderChargerStatus outlanderChargerGetStatus(bool canBusReady) {
  OutlanderChargerStatus result = chargerStatus;
  const uint32_t now = millis();
  result.canBusReady = canBusReady;
  result.charger389Fresh = charger389Seen &&
      now - charger389AtMs <= OUTLANDER_STATUS_TIMEOUT_MS;
  result.charger38AFresh = charger38ASeen &&
      now - charger38AAtMs <= OUTLANDER_STATUS_TIMEOUT_MS;
  result.telemetryFresh = result.charger389Fresh || result.charger38AFresh;
  result.mainsPresent = result.charger389Fresh &&
      (result.status389 & 0x02U) != 0U;
  result.charging = result.charger389Fresh &&
      (result.status389 & 0x08U) != 0U;
  result.chargerError = result.charger389Fresh &&
      (result.status389 & 0x10U) != 0U;
  result.dcdcRequestActive = result.charger389Fresh &&
      (result.status389 & 0x40U) != 0U;
  result.pilotPresent = result.charger389Fresh &&
      (result.status389 & 0x80U) != 0U;
  result.waitForMains = result.charger38AFresh &&
      (result.status38A & 0x04U) != 0U;
  result.readyForCharge = result.charger38AFresh &&
      (result.status38A & 0x08U) != 0U;

  result.lastFrameAgeMs = 0xFFFFFFFFUL;
  if (charger389Seen) result.lastFrameAgeMs = now - charger389AtMs;
  if (charger38ASeen) {
    const uint32_t secondaryAge = now - charger38AAtMs;
    if (secondaryAge < result.lastFrameAgeMs) {
      result.lastFrameAgeMs = secondaryAge;
    }
  }
  return result;
}

bool outlanderChargerAvailableForCharge(const BmsSettings &settings,
                                        bool canBusReady) {
  if (settings.chargerType != ChargerType::Outlander ||
      !settings.chargerTxEnabled || !canBusReady) return false;
  const OutlanderChargerStatus status = outlanderChargerGetStatus(canBusReady);
  return status.charger389Fresh && status.mainsPresent &&
         !status.chargerError;
}

bool outlanderChargerSendSetpoint(const BmsSettings &settings,
                                  float requestedCurrentA,
                                  bool canBusReady) {
  if (settings.chargerType != ChargerType::Outlander ||
      !settings.chargerTxEnabled || !canBusReady ||
      !(requestedCurrentA >= 0.0f)) return false;

  // The reference SimpBMS command uses 0.1 V/bit and 0.1 A/bit, capped at
  // 12 A. The project charge-current request already includes cell-voltage taper.
  const float requestedDecivolts =
      settings.chargeVoltageV * settings.seriesCells * 10.0f;
  if (!(requestedDecivolts > 0.0f) || requestedDecivolts > 65535.0f) {
    return false;
  }
  uint32_t targetDecivolts = static_cast<uint32_t>(requestedDecivolts + 0.5f);
  uint32_t currentDeciamps =
      static_cast<uint32_t>(requestedCurrentA * 10.0f + 0.5f);
  if (currentDeciamps > OUTLANDER_MAX_CURRENT_DECIAMP) {
    currentDeciamps = OUTLANDER_MAX_CURRENT_DECIAMP;
  }

  const uint8_t data[8] = {
      static_cast<uint8_t>((targetDecivolts >> 8) & 0xFFU),
      static_cast<uint8_t>(targetDecivolts & 0xFFU),
      static_cast<uint8_t>(currentDeciamps),
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  return queueStandardFrame(OUTLANDER_COMMAND_ID, data, canBusReady,
                            currentDeciamps == 0U ? 3U : 2U);
}

void outlanderChargerTask(const BmsSettings &settings, bool readyState,
                          bool chargePrechargeActive, bool chargeState,
                          bool chargeRequested, float requestedCurrentA,
                          bool canBusReady) {
  if (settings.chargerType != ChargerType::Outlander ||
      !settings.chargerTxEnabled || !canBusReady) return;

  const uint32_t now = millis();
  const uint32_t period = settings.chargerMessagePeriodMs;
  if (txIntervalStarted && now - lastTxAtMs < period) return;

  if (chargeState) {
    // Continue sending zero current while the state machine is trying to stop.
    outlanderChargerSendSetpoint(settings,
                                 chargeRequested ? requestedCurrentA : 0.0f,
                                 canBusReady);
  } else if (readyState || chargePrechargeActive) {
    // Wake/report polling keeps OBC presence visible before contactors close.
    sendWake(canBusReady);
  }
}

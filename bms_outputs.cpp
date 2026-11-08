/*
  Active-HIGH low-side FET gate outputs for the configured BMS roles.

  A pin-map change first drives the old pin LOW, then configures the new pin
  LOW. The settings layer restricts outputs to D6/D13/D11/D10 and rejects any
  pin already assigned to another input or output role. The main-negative and
  precharge SME selections are placeholders: any former Nano pin is driven LOW
  and no SME hardware command is sent.
*/
#include "bms_outputs.h"

namespace {
constexpr uint8_t NO_PIN = 0xFFU;
uint8_t configuredPins[4] = {NO_PIN, NO_PIN, NO_PIN, NO_PIN};
bool outputActive[4] = {false, false, false, false};

bool allowedPin(uint8_t pin) {
  return pin == 6U || pin == 13U || pin == 11U || pin == 10U;
}

bool usesNanoFet(BmsOutputRole role, const BmsSettings &settings) {
  if (role == BmsOutputRole::MainNegative) {
    return settings.mainNegativeControl == ContactorControlType::NanoLowSideFet;
  }
  if (role == BmsOutputRole::Precharge) {
    return settings.prechargeControl == ContactorControlType::NanoLowSideFet;
  }
  return true;
}

void setOff(uint8_t pin) {
  if (!allowedPin(pin)) return;
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}
}

void bmsOutputsBegin(const BmsSettings &settings) {
  for (uint8_t index = 0; index < 4U; ++index) {
    configuredPins[index] = NO_PIN;
    outputActive[index] = false;
  }
  bmsOutputsApplySettings(settings);
}

void bmsOutputsApplySettings(const BmsSettings &settings) {
  for (uint8_t index = 0; index < 4U; ++index) {
    const BmsOutputRole role = static_cast<BmsOutputRole>(index);
    const uint8_t assignedPin = settingsGetOutputPin(role);
    const bool useNanoFet = usesNanoFet(role, settings);
    // Switching a role to the unimplemented SME placeholder first drives its
    // former FET pin LOW, then removes it from Nano GPIO control.
    const uint8_t nextPin = useNanoFet ? assignedPin : NO_PIN;
    if (nextPin == configuredPins[index]) {
      // Also force the retained, inactive Nano pin LOW on startup in SME mode.
      if (!useNanoFet) setOff(assignedPin);
      continue;
    }
    setOff(configuredPins[index]);
    if (!useNanoFet) setOff(assignedPin);
    configuredPins[index] = NO_PIN;
    outputActive[index] = false;
    if (allowedPin(nextPin)) {
      setOff(nextPin);
      configuredPins[index] = nextPin;
    }
  }
}

bool bmsOutputRequest(BmsOutputRole role, bool active,
                     const BmsSettings &settings) {
  bmsOutputsApplySettings(settings);
  const uint8_t index = static_cast<uint8_t>(role);
  if (index >= 4U || configuredPins[index] == NO_PIN ||
      !usesNanoFet(role, settings)) return false;
  if (role == BmsOutputRole::ChargerRelay &&
      settings.chargerType != ChargerType::ExternalRelay) {
    outputActive[index] = false;
    digitalWrite(configuredPins[index], LOW);
    return false;
  }
  outputActive[index] = active;
  // All external drivers are active HIGH into low-side logic-level FET gates.
  digitalWrite(configuredPins[index], active ? HIGH : LOW);
  return true;
}

bool bmsOutputIsActive(BmsOutputRole role) {
  const uint8_t index = static_cast<uint8_t>(role);
  return index < 4U && outputActive[index];
}

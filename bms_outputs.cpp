/*
  Active-HIGH low-side FET gate outputs for the four configured BMS roles.

  A pin-map change first drives the old pin LOW, then configures the new pin
  LOW. The settings layer restricts outputs to D6/D13/D11/D10 and rejects any
  pin already assigned to another input or output role.
*/
#include "bms_outputs.h"

namespace {
constexpr uint8_t NO_PIN = 0xFFU;
uint8_t configuredPins[4] = {NO_PIN, NO_PIN, NO_PIN, NO_PIN};
bool outputActive[4] = {false, false, false, false};

bool allowedPin(uint8_t pin) {
  return pin == 6U || pin == 13U || pin == 11U || pin == 10U;
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
    const uint8_t nextPin = settingsGetOutputPin(role);
    if (nextPin == configuredPins[index]) continue;
    setOff(configuredPins[index]);
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
  if (index >= 4U || configuredPins[index] == NO_PIN) return false;
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

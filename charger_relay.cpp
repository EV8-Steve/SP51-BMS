/*
  Adapter for the optional external charger relay output.

  The relay is only accepted when ChargerType::ExternalRelay is selected. This
  does not implement the Outlander OBC CAN protocol; charger CAN transmit is
  deliberately disabled in the current build.
*/
#include "charger_relay.h"

#include "bms_outputs.h"

namespace {
bool relayOn = false;
}

void chargerRelayBegin(const BmsSettings &settings) {
  bmsOutputsBegin(settings);
  relayOn = false;
}

void chargerRelayApplySettings(const BmsSettings &settings) {
  bmsOutputsApplySettings(settings);
  if (settings.chargerType != ChargerType::ExternalRelay) {
    bmsOutputRequest(BmsOutputRole::ChargerRelay, false, settings);
    relayOn = false;
  }
}

bool chargerRelayRequest(bool enable, const BmsSettings &settings) {
  chargerRelayApplySettings(settings);
  const bool accepted =
      bmsOutputRequest(BmsOutputRole::ChargerRelay, enable, settings);
  if (!accepted) {
    relayOn = false;
    return false;
  }
  relayOn = enable;
  return true;
}

bool chargerRelayIsOn() {
  return relayOn && bmsOutputIsActive(BmsOutputRole::ChargerRelay);
}

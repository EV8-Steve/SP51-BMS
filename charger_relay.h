#pragma once

#include <Arduino.h>

#include "settings.h"

// Initialize relay bookkeeping and the shared mapped FET outputs.
void chargerRelayBegin(const BmsSettings &settings);
// Apply pin/profile changes, keeping the relay OFF for non-relay profiles.
void chargerRelayApplySettings(const BmsSettings &settings);
// Request relay state; enabling is rejected unless ExternalRelay is selected.
bool chargerRelayRequest(bool enable, const BmsSettings &settings);
// True only when both the module and shared output driver report the relay ON.
bool chargerRelayIsOn();

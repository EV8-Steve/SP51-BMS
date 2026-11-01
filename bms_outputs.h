#pragma once

#include <Arduino.h>

#include "settings.h"

// Initialize all mapped gates LOW and apply the persisted pin assignments.
void bmsOutputsBegin(const BmsSettings &settings);
// Reconcile live pin maps, forcing old/new pins LOW during a mapping change.
void bmsOutputsApplySettings(const BmsSettings &settings);
// Drive one logical role; returns false for an unassigned or incompatible role.
bool bmsOutputRequest(BmsOutputRole role, bool active,
                     const BmsSettings &settings);
// Report the last requested logical output state.
bool bmsOutputIsActive(BmsOutputRole role);

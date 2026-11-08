#pragma once

#include <Arduino.h>

#include "battery_can.h"
#include "batterycomms.h"
#include "bms_limits.h"
#include "bms_state_machine.h"
#include "outlander_charger.h"
#include "settings.h"
#include "soc_estimator.h"

// Start the Nextion SoftwareSerial link on the fixed D4/D5 pins. Returns
// false if a saved configurable input currently occupies either pin or if
// the board core cannot initialize SoftwareSerial on the selected pins.
bool nextionDisplayBegin(const BmsSettings &settings);

// Temporarily uses the existing stat.txt component to show stack recovery.
// Pass an empty string to return the component to the BMS operating state.
void nextionDisplaySetStackStatus(const char *statusText);

// Queue one complete display update. A new frame is rejected while the prior
// frame is still being sent, so a slow display never accumulates stale data.
bool nextionDisplayQueue(const BatterySnapshot &snapshot,
                         const BmsSettings &settings,
                         const BatteryLimitStatus &limits,
                         const CurrentSensorStatus &current,
                         BmsOperatingState state,
                         bool acPresent,
                         const SocStatus &soc,
                         const OutlanderChargerStatus &charger);

// Send at most one byte per call. The Renesas SoftwareSerial byte write is
// synchronous, so this bounds the transmit pause to about one byte time.
void nextionDisplayService();

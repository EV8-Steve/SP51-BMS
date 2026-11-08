#pragma once

#include "batterycomms.h"
#include "battery_can.h"
#include "bms_limits.h"
#include "bms_state_machine.h"
#include "outlander_charger.h"
#include "soc_estimator.h"

// Start the USB serial console and print version/configuration information.
void serialConsoleBegin();
void serialConsolePrintStartup();
void serialConsolePrintInitResult(bool ok, const char *error);
// Store the latest stack status; emit its one-line notice only outside menus.
void serialConsoleSetStackStatus(const char *statusText);
// Consume complete newline-terminated console commands without blocking scans.
void serialConsolePollInput();
// Drain a completed live frame in small chunks without blocking CAN service.
void serialConsoleServiceOutput();
bool serialConsoleMenuIsOpen();
bool serialConsoleTakeRetryRequest();
// Print one fixed-width live frame after a complete stack scan.
void serialConsolePrintSnapshot(const BatterySnapshot &snapshot,
                                const BmsSettings &settings,
                                const CurrentSensorStatus &current,
                                const BatteryLimitStatus &limits,
                                const BmsStateStatus &state,
                                const OutlanderChargerStatus &charger,
                                const SocStatus &soc);

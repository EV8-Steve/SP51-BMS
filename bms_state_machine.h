#pragma once

#include <Arduino.h>

#include "battery_can.h"
#include "bms_limits.h"
#include "settings.h"

enum class BmsOperatingState : uint8_t {
  // Extra states provide a safe startup and a latched response to faults.
  Disarmed = 0,
  Ready,
  Precharge,
  Drive,
  Charge,
  Fault
};

struct BmsStateStatus {
  BmsOperatingState state;
  bool armed;
  bool runRequested;
  bool chargeRequested;
  bool currentValid;
  bool cellDataFresh;
  uint32_t prechargeElapsedMs;
  int32_t currentMilliamp;
  const char *reason;
};

// Initialize input pins and force all mapped outputs OFF; auto-arm is never
// restored from EEPROM or across a Nano reset.
void bmsStateMachineBegin(const BmsSettings &settings);
// Run input debounce, start/advance the state machine and enforce interlocks.
void bmsStateMachineUpdate(const BmsSettings &settings,
                           const BatteryLimitStatus &limits,
                           const CurrentSensorStatus &current,
                           bool stackReady, bool cellDataFresh,
                           bool configurationMenuOpen);
// Command all outputs OFF and inhibit held requests until inputs release.
void bmsStateMachineDisarm(const BmsSettings &settings);
// Clear a latched fault only when both debounced input requests are inactive.
bool bmsStateMachineClearFault();
// Settings may change only when outputs are not in an active operating state.
bool bmsStateMachineCanEditSettings();
// Return current state, input, current and precharge status for the console.
BmsStateStatus bmsStateMachineStatus();
const char *bmsOperatingStateName(BmsOperatingState state);

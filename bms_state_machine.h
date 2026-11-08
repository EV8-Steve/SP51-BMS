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

enum class BmsFaultCode : uint8_t;

struct BmsStateStatus {
  BmsOperatingState state;
  bool armed;
  bool runRequested;
  bool chargeInputActive;
  bool chargerMainsDetected;
  bool chargeRequested;
  bool chargePrechargeActive;
  bool currentValid;
  bool cellDataFresh;
  bool stackCommunicationFault;  // Active BQ recovery/failure annunciator.
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
// Stack recovery is allowed to run while Drive outputs remain held. If all
// retries fail, the Drive state waits for the RUN input to go inactive before
// latching Fault and opening outputs.
void bmsStateMachineSetStackRecovery(bool active, bool retriesExhausted);
// A latched fault can be cleared manually after a complete fresh stack sample
// and inactive-input checks. Communication-only EEPROM lockouts are also
// cleared automatically after three consecutive complete scans with requests
// inactive; other fault types remain manually latched.
void bmsStateMachineSetStackHealth(bool ready, bool dataFresh,
                                   bool readingsValid);
void bmsStateMachineReportStackFault(const char *reason);
// Restore an EEPROM-latched fault after the Nano reboots; no log duplicate is
// written, and normal CLEARFAULT checks still apply.
void bmsStateMachineRestorePersistentFault();
// Command all outputs OFF and inhibit held requests until inputs release.
void bmsStateMachineDisarm(const BmsSettings &settings);
// Clear a latched fault only when both debounced input requests are inactive.
bool bmsStateMachineClearFault();
// Future Bender/SME input code can call this to latch the existing Fault state.
// Returns false while isolation monitoring is disabled.
bool bmsStateMachineReportIsolationFault();
// Settings may change only when outputs are not in an active operating state.
bool bmsStateMachineCanEditSettings();
// Return current state, input, current and precharge status for the console.
BmsStateStatus bmsStateMachineStatus();
const char *bmsOperatingStateName(BmsOperatingState state);

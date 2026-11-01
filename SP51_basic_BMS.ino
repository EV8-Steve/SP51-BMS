/*
  SP51_basic_BMS — Nano R4 state-machine BMS foundation

  Continuously reads all 12 cell-voltage channels and configured GPIO/TSREF
  ADC channels from each BQ79616 stack module. The proven manual base COMH
  TX/RX override remains enabled. The USB live table is refreshed on a 500 ms
  target cadence and hidden while configuration menus are open; scans and
  state-machine safety monitoring continue in the background.

  The working state machine has Ready, Precharge, Drive and Charge states,
  plus Disarmed and Fault. The isolated 12 V Run input requests Drive and the
  isolated 240 V-present input requests charging. With valid stack/current
  interlocks, the controller enters READY automatically; the physical inputs
  request operation, so no terminal ARM command is required. The active-HIGH
  Nano outputs drive the configurable low-side FET gates for main positive,
  main negative, precharge and external charger relay. This build does not
  initialize S-box features from the original SP51 hardware. Contactor and
  current-sensing hardware is external.
  The isolated Run and 240 V inputs default active LOW to match the inverted
  interface outputs; their pins and active levels are menu-configurable.

  Precharge follows the SimpBMS sequence: close main negative and precharge,
  wait for the configured minimum time and current threshold, close main
  positive, then open precharge. A separate timeout latches a fault. Outlander
  charger CAN transmit remains disabled; charging via the relay assumes the
  external charger is independently configured and safe for this pack.

  The signed coulomb-count SOC calculation and voltage-only seed approach are
  adapted from Jamie's SimpBMS. SOC and Ah-in/Ah-out totals are retained in a
  rotating CRC-protected EEPROM journal. Old Victron/Tesla/BMW-Mini protocols
  and CSC-specific CRC/control are omitted.
  Current build notes:
  - BQ UART: Serial1, D1/TX and D0/RX, 1 Mbps 8N1.
  - Base uses COMH; COML is not connected on the base board.
  - Settings and pin mappings are stored in Nano R4 EEPROM with versioned
    migration from earlier settings records.
  - NTC values remain provisional at 22k/10k/B3380.
  - All contactor and charger outputs start OFF; the controller requires valid
    complete stack readings and fresh LEM CAN current before it can arm.
*/

#include <Arduino.h>

#include "batterycomms.h"
#include "battery_can.h"
#include "bms_limits.h"
#include "bms_state_machine.h"
#include "charger_relay.h"
#include "serial_console.h"
#include "soc_estimator.h"
#include "settings.h"

namespace {
BatterySnapshot liveSnapshot;
BatteryLimitStatus liveLimits = {};
CurrentSensorStatus liveCurrent = {};
SocStatus liveSoc = {};
bool scanActive = false;
bool haveCompleteCellScan = false;
uint32_t lastCompleteScanAtMs = 0;
uint32_t lastCompleteScanDurationMs = 0;
uint32_t nextScanDueMs = 0;

uint32_t cellDataTimeoutMs(const BmsSettings &settings) {
  // Use the larger of a cadence allowance and the measured scan duration so a
  // slow but complete multi-device pass is not declared stale mid-scan.
  const uint32_t cadenceTimeout =
      static_cast<uint32_t>(settings.samplePeriodMs) * 3UL;
  const uint32_t scanTimeout = lastCompleteScanDurationMs * 2UL +
                               settings.samplePeriodMs;
  return cadenceTimeout > scanTimeout ? cadenceTimeout : scanTimeout;
}

void startStackCommunication() {
  // A retry invalidates the old cell snapshot; state control waits for another
  // full valid scan before it can return to READY.
  char error[96] = {};
  scanActive = false;
  haveCompleteCellScan = false;
  liveLimits = {};
  batteryCommsCancelScan();
  const bool ok = batteryCommsBegin(settingsGet(), error, sizeof(error));
  serialConsolePrintInitResult(ok, error);
  if (ok) nextScanDueMs = millis();
}
}

void setup() {
  // Load configuration before starting CAN, output maps and the BQ stack.
  serialConsoleBegin();
  settingsBegin();
  socEstimatorBegin();
  batteryCanBegin(settingsGet());
  chargerRelayBegin(settingsGet());
  bmsStateMachineBegin(settingsGet());
  serialConsolePrintStartup();
  startStackCommunication();
}

void loop() {
  const BmsSettings &settings = settingsGet();
  serialConsolePollInput();
  batteryCanPoll(settings);
  chargerRelayApplySettings(settings);

  if (serialConsoleTakeRetryRequest()) {
    bmsStateMachineDisarm(settings);
    startStackCommunication();
  }

  const uint32_t now = millis();
  const bool menuOpen = serialConsoleMenuIsOpen();
  const bool stackReady = batteryCommsIsReady();
  liveCurrent = batteryCanCurrent(settings);

  if (stackReady && !scanActive &&
      static_cast<int32_t>(now - nextScanDueMs) >= 0) {
    batteryCommsStartScan(liveSnapshot, settings);
    scanActive = true;
  }

  bool completedThisLoop = false;
  if (scanActive) {
    const BatteryScanStepResult result =
        batteryCommsScanStep(liveSnapshot, settings);
    if (result == BatteryScanStepResult::Complete) {
      scanActive = false;
      liveLimits = batteryLimitsUpdate(liveSnapshot, settings);
      haveCompleteCellScan = liveLimits.readingsValid;
      if (haveCompleteCellScan) {
        lastCompleteScanAtMs = liveSnapshot.finishedAtMs;
        lastCompleteScanDurationMs = liveSnapshot.durationMs;
      }
      completedThisLoop = true;

      nextScanDueMs = liveSnapshot.startedAtMs + settings.samplePeriodMs;
      if (static_cast<int32_t>(nextScanDueMs - millis()) <= 0) {
        // A full-stack pass is never overlapped; if it exceeds the target,
        // start its successor as soon as the current pass finishes.
        nextScanDueMs = millis();
      }
    } else if (result == BatteryScanStepResult::NotReady) {
      scanActive = false;
    }
  }

  // Keep old complete data usable during the next pass, but invalidate it once
  // it exceeds the measured freshness window.
  const bool cellDataFresh = haveCompleteCellScan && liveLimits.readingsValid &&
      millis() - lastCompleteScanAtMs <= cellDataTimeoutMs(settings);
  liveSoc = socEstimatorUpdate(settings, liveLimits, liveCurrent);
  bmsStateMachineUpdate(settings, liveLimits, liveCurrent, stackReady,
                        cellDataFresh, menuOpen);

  if (completedThisLoop && !menuOpen) {
    serialConsolePrintSnapshot(liveSnapshot, settings, liveCurrent, liveLimits,
                               bmsStateMachineStatus(), liveSoc);
  }
}

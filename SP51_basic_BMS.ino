/*
  SP51_basic_BMS — Nano R4 state-machine BMS foundation

  Continuously reads all 12 cell-voltage channels and configured GPIO/TSREF
  ADC channels from each BQ79616 stack module. The proven manual base COMH
  TX/RX override remains enabled. The USB live table is refreshed on a 500 ms
  target cadence and hidden while configuration menus are open; scans and
  state-machine safety monitoring continue in the background. BQ scan I/O is
  non-blocking; the cooperative loop services CAN each pass and schedules
  state and charger work on 10 ms periods.

  The working state machine has Ready, Precharge, Drive and Charge states,
  plus Disarmed and Fault. The isolated 12 V Run input requests Drive. The
  isolated 240 V-present input or fresh Outlander CAN mains status requests
  charging. With valid stack/current
  interlocks, the controller enters READY automatically; the physical inputs
  request operation, so no terminal ARM command is required. The active-HIGH
  Nano outputs drive the configurable low-side FET gates for main positive,
  main negative, precharge and external charger relay. This build does not
  initialize S-box features from the original SP51 hardware. Contactor and
  current-sensing hardware is external.
  The isolated Run and 240 V inputs default active LOW to match the inverted
  interface outputs; their pins and active levels are menu-configurable.
  A separate Nextion dashboard is updated from completed stack snapshots on
  SoftwareSerial D4 (RX) / D5 (TX); the BQ UART remains on D0/D1.

  Precharge follows the SimpBMS sequence: close main negative and precharge,
  wait for the configured minimum time and current threshold, close main
  positive, then open precharge. A separate timeout latches a fault. The
  Outlander OBC shares the 500 kbit/s CAN bus with the LEM sensor: Disarmed and
  Ready send 0x285 wake frames and read 0x389/0x38A status; fresh 0x389 AC-mains
  status can request Charge, which sends 0x286 voltage/current requests.

  The signed coulomb-count SOC calculation and voltage-only seed approach are
  adapted from Tom_EVnut's SimpBMS. SOC and Ah-in/Ah-out totals are retained in a
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
#include "fault_log.h"
#include "charger_relay.h"
#include "nextion_display.h"
#include "outlander_charger.h"
#include "serial_console.h"
#include "soc_estimator.h"
#include "settings.h"

#include <stdio.h>
#include <string.h>

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
constexpr uint16_t CONTROL_TASK_PERIOD_MS = 10;
constexpr uint16_t CHARGER_TASK_PERIOD_MS = 10;
constexpr uint16_t SOC_PERSISTENCE_SERVICE_PERIOD_MS = 1000;
uint32_t nextControlTaskAtMs = 0;
uint32_t nextChargerTaskAtMs = 0;
uint32_t nextSocPersistenceServiceAtMs = 0;
uint32_t lastSocFrameCount = 0;
bool socFrameBaselineSet = false;
constexpr uint8_t STACK_SCAN_FAILURES_BEFORE_RESTART = 3;
constexpr uint8_t MAX_STACK_RESTART_ATTEMPTS = 3;
constexpr uint32_t STACK_RESTART_RETRY_DELAY_MS = 5000UL;
enum class StackLinkState : uint8_t {
  Starting,
  Online,
  Restarting,
  Verifying,
  RetryWait,
  Failed
};
StackLinkState stackLinkState = StackLinkState::Starting;
uint8_t consecutiveIncompleteScans = 0;
uint8_t stackRestartAttempts = 0;
uint32_t nextStackRestartAtMs = 0;
char stackStatusText[48] = "STACK STARTUP";
bool stackFaultEventLogged = false;

void serviceCanDuringBqTransaction() {
  batteryCanPoll(settingsGet());
}

bool periodicTaskDue(uint32_t now, uint32_t &nextDueMs, uint16_t periodMs) {
  if (nextDueMs == 0U) {
    nextDueMs = now + periodMs;
    return true;
  }
  if (static_cast<int32_t>(now - nextDueMs) < 0) return false;

  // Skip missed slots instead of running a burst of catch-up work.
  const uint32_t missed = (now - nextDueMs) / periodMs + 1U;
  nextDueMs += missed * periodMs;
  return true;
}

uint32_t cellDataTimeoutMs(const BmsSettings &settings) {
  // Use the larger of a cadence allowance and the measured scan duration so a
  // slow but complete multi-device pass is not declared stale mid-scan.
  const uint32_t cadenceTimeout =
      static_cast<uint32_t>(settings.samplePeriodMs) * 3UL;
  const uint32_t scanTimeout = lastCompleteScanDurationMs * 2UL +
                               settings.samplePeriodMs;
  return cadenceTimeout > scanTimeout ? cadenceTimeout : scanTimeout;
}

void publishStackStatus(const char *consoleText,
                        const char *nextionText = nullptr) {
  snprintf(stackStatusText, sizeof(stackStatusText), "%s",
           consoleText ? consoleText : "UNKNOWN");
  serialConsoleSetStackStatus(stackStatusText);
  nextionDisplaySetStackStatus(nextionText ? nextionText : stackStatusText);
}

bool startStackCommunication(bool recoveryAttempt) {
  // A retry invalidates the old cell snapshot. Drive has a dedicated recovery
  // hold policy; other states cannot arm until a new full scan is valid.
  char error[96] = {};
  if (recoveryAttempt && stackRestartAttempts < MAX_STACK_RESTART_ATTEMPTS) {
    ++stackRestartAttempts;
  }
  scanActive = false;
  haveCompleteCellScan = false;
  lastCompleteScanAtMs = 0;
  lastCompleteScanDurationMs = 0;
  liveLimits = {};
  batteryCommsCancelScan();
  if (recoveryAttempt) {
    bmsStateMachineSetStackRecovery(true, false);
    char status[48] = {};
    snprintf(status, sizeof(status), "STACK FAULT RETRY %u/%u",
             static_cast<unsigned>(stackRestartAttempts),
             static_cast<unsigned>(MAX_STACK_RESTART_ATTEMPTS));
    stackLinkState = StackLinkState::Restarting;
    publishStackStatus(status, "STACK FAULT");
  } else {
    stackLinkState = StackLinkState::Starting;
    publishStackStatus("STACK STARTUP", "STACK START");
  }

  const bool ok = batteryCommsBegin(settingsGet(), error, sizeof(error));
  serialConsolePrintInitResult(ok, error);
  if (ok) {
    stackLinkState = StackLinkState::Verifying;
    consecutiveIncompleteScans = 0;
    nextStackRestartAtMs = 0;
    nextScanDueMs = millis();
    if (!recoveryAttempt) publishStackStatus("STACK VERIFY", "VERIFYING");
    // A recovery attempt keeps its visible restart flag until a full valid
    // stack scan proves every configured module responded.
    return true;
  }

  nextStackRestartAtMs = millis() + STACK_RESTART_RETRY_DELAY_MS;
  if (recoveryAttempt &&
      stackRestartAttempts >= MAX_STACK_RESTART_ATTEMPTS) {
    stackLinkState = StackLinkState::Failed;
    bmsStateMachineSetStackRecovery(true, true);
    publishStackStatus("STACK FAULT: RETRIES EXHAUSTED", "STACK FAULT");
    if (bmsStateMachineStatus().state != BmsOperatingState::Drive) {
      bmsStateMachineReportStackFault(
          "Persistent BQ stack communication fault; retries exhausted. The communication lockout clears after three consecutive complete scans with Run/charge requests inactive.");
    }
  } else {
    stackLinkState = StackLinkState::RetryWait;
    if (recoveryAttempt) {
      char status[48] = {};
      snprintf(status, sizeof(status), "STACK FAULT RETRY %u/%u",
               static_cast<unsigned>(stackRestartAttempts),
               static_cast<unsigned>(MAX_STACK_RESTART_ATTEMPTS));
      publishStackStatus(status, "STACK FAULT");
    } else {
      publishStackStatus("STACK INIT FAILED", "STACK FAILED");
    }
  }
  return false;
}

void recordIncompleteStackScan() {
  if (stackLinkState == StackLinkState::RetryWait ||
      stackLinkState == StackLinkState::Failed) {
    return;
  }

  if (consecutiveIncompleteScans < STACK_SCAN_FAILURES_BEFORE_RESTART) {
    ++consecutiveIncompleteScans;
  }

  if (!stackFaultEventLogged) {
    faultLogRecord(BmsFaultCode::StackCommunication,
                   bmsStateMachineStatus().state);
    stackFaultEventLogged = true;
  }
  bmsStateMachineSetStackRecovery(true, false);

  if (stackLinkState == StackLinkState::Verifying && stackRestartAttempts) {
    char status[48] = {};
    snprintf(status, sizeof(status), "VERIFY %u/%u SCAN %u/%u",
             static_cast<unsigned>(stackRestartAttempts),
             static_cast<unsigned>(MAX_STACK_RESTART_ATTEMPTS),
             static_cast<unsigned>(consecutiveIncompleteScans),
             static_cast<unsigned>(STACK_SCAN_FAILURES_BEFORE_RESTART));
    publishStackStatus(status, "STACK FAULT");
  } else {
    char status[48] = {};
    snprintf(status, sizeof(status), "SCAN FAIL %u/%u",
             static_cast<unsigned>(consecutiveIncompleteScans),
             static_cast<unsigned>(STACK_SCAN_FAILURES_BEFORE_RESTART));
    publishStackStatus(status, "STACK FAULT");
  }

  if (consecutiveIncompleteScans < STACK_SCAN_FAILURES_BEFORE_RESTART) return;

  if (stackRestartAttempts >= MAX_STACK_RESTART_ATTEMPTS) {
    stackLinkState = StackLinkState::Failed;
    bmsStateMachineSetStackRecovery(true, true);
    publishStackStatus("STACK FAULT: RETRIES EXHAUSTED", "STACK FAULT");
    if (bmsStateMachineStatus().state != BmsOperatingState::Drive) {
      bmsStateMachineReportStackFault(
          "Persistent BQ stack communication fault; retries exhausted. The communication lockout clears after three consecutive complete scans with Run/charge requests inactive.");
    }
    return;
  }

  startStackCommunication(true);
}

void recordCompleteStackScan() {
  consecutiveIncompleteScans = 0;
  stackRestartAttempts = 0;
  nextStackRestartAtMs = 0;
  stackFaultEventLogged = false;
  bmsStateMachineSetStackRecovery(false, false);
  stackLinkState = StackLinkState::Online;
  publishStackStatus("STACK ONLINE", "");
}
}

void setup() {
  // Load configuration before starting CAN, output maps and the BQ stack.
  serialConsoleBegin();
  settingsBegin();
  faultLogBegin();
  socEstimatorBegin();
  batteryCanBegin(settingsGet());
  batteryCommsSetBackgroundService(serviceCanDuringBqTransaction);
  outlanderChargerBegin();
  chargerRelayBegin(settingsGet());
  bmsStateMachineBegin(settingsGet());
  bmsStateMachineRestorePersistentFault();
  if (nextionDisplayBegin(settingsGet())) {
    Serial.println(F("Nextion SoftwareSerial ready: D5/TX at 9600 baud (D4/RX reserved)."));
  } else {
    Serial.println(F("Nextion display paused: D4/D5 conflict with a saved input mapping or SoftwareSerial could not start."));
  }
  serialConsolePrintStartup();
  startStackCommunication(false);
}

void loop() {
  const BmsSettings &settings = settingsGet();
  // CAN receive is serviced first on every pass. Live USB output is drained
  // in bounded chunks and terminal commands wait until a frame is complete.
  batteryCanPoll(settings);
  serialConsoleServiceOutput();
  serialConsolePollInput();

  if (serialConsoleTakeRetryRequest()) {
    const BmsOperatingState state = bmsStateMachineStatus().state;
    const bool active = state == BmsOperatingState::Precharge ||
        state == BmsOperatingState::Drive ||
        state == BmsOperatingState::Charge;
    if (!active) {
      bmsStateMachineDisarm(settings);
      consecutiveIncompleteScans = 0;
      stackRestartAttempts = 0;
      stackFaultEventLogged = false;
      startStackCommunication(true);
    }
  }

  uint32_t now = millis();
  if (stackLinkState == StackLinkState::RetryWait &&
      static_cast<int32_t>(now - nextStackRestartAtMs) >= 0) {
    startStackCommunication(true);
    now = millis();
  }
  const bool menuOpen = serialConsoleMenuIsOpen();
  bool stackReady = batteryCommsIsReady();
  liveCurrent = batteryCanCurrent(settings);

  if (stackReady && !scanActive && !batteryCommsBalanceBusy() &&
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
        recordCompleteStackScan();
      } else {
        recordIncompleteStackScan();
      }
      completedThisLoop = true;

      nextScanDueMs = liveSnapshot.startedAtMs + settings.samplePeriodMs;
      if (static_cast<int32_t>(nextScanDueMs - millis()) <= 0) {
        // A full-stack pass is never overlapped; if it exceeds the target,
        // start its successor as soon as the current pass finishes.
        nextScanDueMs = millis();
      }
      stackReady = batteryCommsIsReady();
    } else if (result == BatteryScanStepResult::NotReady) {
      scanActive = false;
      recordIncompleteStackScan();
      stackReady = batteryCommsIsReady();
    }
  }

  // Keep old complete data usable during the next pass, but invalidate it once
  // it exceeds the measured freshness window.
  const bool cellDataFresh = haveCompleteCellScan && liveLimits.readingsValid &&
      millis() - lastCompleteScanAtMs <= cellDataTimeoutMs(settings);
  bmsStateMachineSetStackHealth(stackReady, cellDataFresh,
                                liveLimits.readingsValid);

  // SOC integrates every newly observed CAN frame and refreshes its voltage
  // seed on completed scans; it no longer performs EEPROM writes in this path.
  if (completedThisLoop || !socFrameBaselineSet ||
      liveCurrent.frameCount != lastSocFrameCount) {
    liveSoc = socEstimatorUpdate(settings, liveLimits, liveCurrent);
    lastSocFrameCount = liveCurrent.frameCount;
    socFrameBaselineSet = true;
  }

  const uint32_t taskNow = millis();
  const bool controlTaskDue = periodicTaskDue(taskNow, nextControlTaskAtMs,
                                             CONTROL_TASK_PERIOD_MS);
  if (completedThisLoop || controlTaskDue) {
    bmsStateMachineUpdate(settings, liveLimits, liveCurrent, stackReady,
                          cellDataFresh, menuOpen);
    chargerRelayApplySettings(settings);
  }

  if (periodicTaskDue(taskNow, nextChargerTaskAtMs,
                      CHARGER_TASK_PERIOD_MS)) {
    const BmsStateStatus stateStatus = bmsStateMachineStatus();
    outlanderChargerTask(settings,
                         stateStatus.state == BmsOperatingState::Ready ||
                             stateStatus.state == BmsOperatingState::Disarmed,
                         stateStatus.chargePrechargeActive,
                         stateStatus.state == BmsOperatingState::Charge,
                         stateStatus.chargeRequested,
                         liveLimits.chargeCurrentRequestA,
                         batteryCanBusReady());
  }

  // EEPROM journal writes are low priority and run only between full scans,
  // never while a BQ request/reply transaction is being serviced.
  if (!scanActive && periodicTaskDue(millis(),
          nextSocPersistenceServiceAtMs,
          SOC_PERSISTENCE_SERVICE_PERIOD_MS)) {
    socEstimatorServicePersistence();
    liveSoc = socEstimatorStatus(settings);
  }

  const BmsStateStatus stateStatus = bmsStateMachineStatus();
  const bool canBusReady = batteryCanBusReady();

  if (completedThisLoop) {
    batteryCommsBalanceObserveScan(liveSnapshot, settings);
  }
  const bool balancingStateAllowed =
      stateStatus.state == BmsOperatingState::Disarmed ||
      stateStatus.state == BmsOperatingState::Ready;
  const uint32_t absoluteCurrentMa = liveCurrent.currentMilliamp < 0
      ? static_cast<uint32_t>(-static_cast<int64_t>(liveCurrent.currentMilliamp))
      : static_cast<uint32_t>(liveCurrent.currentMilliamp);
  const bool balancingInterlocksOkay = stackReady && cellDataFresh &&
      haveCompleteCellScan && balancingStateAllowed &&
      !stateStatus.runRequested && !stateStatus.chargeRequested &&
      !stateStatus.chargeInputActive && !stateStatus.chargerMainsDetected &&
      liveCurrent.valid &&
      absoluteCurrentMa <= static_cast<uint32_t>(
          settings.prechargeFinishCurrentA * 1000.0f);
  batteryCommsBalanceService(settings, balancingInterlocksOkay);

  if (completedThisLoop) {
    const OutlanderChargerStatus chargerStatus =
        outlanderChargerGetStatus(canBusReady);
    // The Nextion dashboard is independent of the USB menu and refreshes on
    // each completed stack pass. Its UART service is paced below, one byte per
    // loop, so it cannot hold the whole application for a complete frame.
    nextionDisplayQueue(liveSnapshot, settings, liveLimits, liveCurrent,
                       stateStatus.state,
                       stateStatus.chargeInputActive ||
                           stateStatus.chargerMainsDetected,
                       liveSoc, chargerStatus);
    if (!menuOpen) {
      serialConsolePrintSnapshot(liveSnapshot, settings, liveCurrent, liveLimits,
                                 stateStatus, chargerStatus, liveSoc);
    }
  }

  nextionDisplayService();
}

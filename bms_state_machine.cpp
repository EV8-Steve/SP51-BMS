/*
  Input-driven BMS operating state machine.

  Run input requests Drive. The isolated 240 V input requests Charge, and a
  fresh Outlander CAN mains report also requests Charge when that profile is
  selected.
  The controller starts DISARMED after reset, auto-arms only after complete
  stack data, fresh LEM current and required contactor mappings are available,
  and the outputs required for the requested path, then waits in READY. Direct
  pack charging needs the external charger relay but can leave contactors
  unmapped. The machine keeps checking interlocks during active states. Critical
  faults latch FAULT and command every mapped output LOW. A BQ communication
  outage during Drive is the explicit exception: bounded stack recovery runs
  while Drive output commands are held until RUN is removed; persistent failure
  then latches FAULT. Communication-only lockouts clear after three consecutive
  valid stack scans when Run and charge requests are inactive.
*/
#include "bms_state_machine.h"

#include "bms_outputs.h"
#include "charger_relay.h"
#include "fault_log.h"
#include "outlander_charger.h"

namespace {
constexpr uint16_t INPUT_DEBOUNCE_MS = 30;
constexpr uint8_t STACK_SCANS_TO_AUTO_CLEAR_COMM_FAULT = 3U;

struct DebouncedInput {
  bool candidate;
  bool stable;
  bool initialized;
  uint32_t changedAtMs;
};

DebouncedInput runInput = {};
DebouncedInput chargeInput = {};
BmsStateStatus stateStatus = {};
bool armed = false;
uint32_t stateEnteredAtMs = 0;
uint32_t prechargeStartedAtMs = 0;
BmsOperatingState prechargeDestination = BmsOperatingState::Drive;
const char *faultReason = nullptr;
uint8_t configuredRunInputPin = 0xFFU;
uint8_t configuredChargeInputPin = 0xFFU;
bool armInterlocksValid = false;
bool manualDisarmInhibit = false;
bool stackRecoveryActive = false;
bool stackRecoveryRetriesExhausted = false;
bool stackHealthyForFaultClear = false;
uint8_t consecutiveHealthyStackScans = 0U;

bool isCommunicationFault(BmsFaultCode code) {
  return code == BmsFaultCode::StackCommunication ||
         code == BmsFaultCode::StackRecoveryFailed;
}

void clearRecoveredCommunicationFaultIfReady() {
  // A complete scan is counted by bmsStateMachineSetStackRecovery(false).
  // Clear only communication lockouts after several consecutive full scans,
  // and only once both physical operation requests have been released.
  if (consecutiveHealthyStackScans < STACK_SCANS_TO_AUTO_CLEAR_COMM_FAULT ||
      stateStatus.state != BmsOperatingState::Fault ||
      stateStatus.runRequested || stateStatus.chargeRequested ||
      !stackHealthyForFaultClear || !faultLogHasPersistentFault() ||
      !isCommunicationFault(faultLogPersistentFaultCode())) {
    return;
  }

  bmsStateMachineClearFault();
}

void configureInputs(const BmsSettings &settings) {
  // Input pin assignments are persistent settings. Reset the debounce history
  // whenever a user changes a pin while the controller is safely idle.
  // The isolated interface actively drives a logic level, so use INPUT rather
  // than enabling the Nano pull-up; active-high/active-low meaning is handled
  // explicitly in updateInput().
  if (configuredRunInputPin != settings.runInputPin) {
    if (configuredRunInputPin != 0xFFU) pinMode(configuredRunInputPin, INPUT);
    configuredRunInputPin = settings.runInputPin;
    pinMode(configuredRunInputPin, INPUT);
    runInput = {};
  }
  if (configuredChargeInputPin != settings.chargeInputPin) {
    if (configuredChargeInputPin != 0xFFU) pinMode(configuredChargeInputPin, INPUT);
    configuredChargeInputPin = settings.chargeInputPin;
    pinMode(configuredChargeInputPin, INPUT);
    chargeInput = {};
  }
}

bool updateInput(DebouncedInput &input, uint8_t pin, bool activeHigh,
                 uint32_t now) {
  // Require a stable level across the configured debounce interval so contact
  // bounce or a brief transient cannot request a state transition. For the
  // installed inverted isolators, activeHigh=false makes a LOW pin level mean
  // an asserted Run or 240 V request.
  const bool rawLevel = digitalRead(pin) == HIGH;
  const bool active = activeHigh ? rawLevel : !rawLevel;
  if (!input.initialized) {
    input.candidate = active;
    input.stable = false;
    input.changedAtMs = now;
    input.initialized = true;
  } else if (active != input.candidate) {
    input.candidate = active;
    input.changedAtMs = now;
  } else if (input.stable != input.candidate &&
             now - input.changedAtMs >= INPUT_DEBOUNCE_MS) {
    input.stable = input.candidate;
  }
  return input.stable;
}

void setState(BmsOperatingState state, const char *reason, uint32_t now) {
  const bool willBeInDriveCycle = state == BmsOperatingState::Drive ||
      (state == BmsOperatingState::Precharge &&
       prechargeDestination == BmsOperatingState::Drive);
  stateStatus.state = state;
  stateStatus.reason = reason;
  // Synchronize the logger on every state transition, including a failed
  // Drive precharge that never became an active Precharge state.
  faultLogSetDriveCycleActive(willBeInDriveCycle);
  if (state != BmsOperatingState::Precharge) {
    stateStatus.prechargeElapsedMs = 0;
  }
  stateEnteredAtMs = now;
}

bool requestChargerRelay(bool active, const BmsSettings &settings) {
  if (active) return chargerRelayRequest(true, settings);
  // The relay output must be driven low even when another charger profile is
  // selected or the relay pin is unassigned; those cases are safe OFF states.
  chargerRelayRequest(false, settings);
  return true;
}

void allOutputsOff(const BmsSettings &settings) {
  chargerRelayRequest(false, settings);
  bmsOutputRequest(BmsOutputRole::Precharge, false, settings);
  bmsOutputRequest(BmsOutputRole::MainPositive, false, settings);
  bmsOutputRequest(BmsOutputRole::MainNegative, false, settings);
}

bool contactorPinsAssigned(const BmsSettings &settings) {
  return settings.mainPositivePin != 0xFFU &&
         settings.mainNegativeControl == ContactorControlType::NanoLowSideFet &&
         settings.mainNegativePin != 0xFFU &&
         settings.prechargeControl == ContactorControlType::NanoLowSideFet &&
         settings.prechargePin != 0xFFU;
}

bool chargerRelayAvailable(const BmsSettings &settings) {
  return settings.chargerType == ChargerType::ExternalRelay &&
         settings.chargerRelayPin != 0xFFU;
}

bool chargerCanRequestCharge(const BmsSettings &settings) {
  if (settings.chargerType == ChargerType::ExternalRelay) {
    return chargerRelayAvailable(settings);
  }
  if (settings.chargerType == ChargerType::Outlander) {
    return outlanderChargerAvailableForCharge(settings,
                                               batteryCanBusReady());
  }
  return false;
}

bool openMappedContactors(const BmsSettings &settings) {
  bool ok = true;
  if (settings.mainNegativePin != 0xFFU) {
    ok = bmsOutputRequest(BmsOutputRole::MainNegative, false, settings) && ok;
  }
  if (settings.mainPositivePin != 0xFFU) {
    ok = bmsOutputRequest(BmsOutputRole::MainPositive, false, settings) && ok;
  }
  if (settings.prechargePin != 0xFFU) {
    ok = bmsOutputRequest(BmsOutputRole::Precharge, false, settings) && ok;
  }
  return ok;
}

bool safeForDrive(const BatteryLimitStatus &limits) {
  // Over-voltage blocks charging but must not block discharge used to bring a
  // high cell back down. Under-voltage/discharge cutoff still blocks Drive.
  return limits.readingsValid && !limits.underVoltage &&
         !limits.dischargeBlocked;
}

bool safeForCharge(const BatteryLimitStatus &limits) {
  return limits.readingsValid && !limits.overVoltage &&
         !limits.chargePausedByHysteresis;
}

void latchFault(const BmsSettings &settings, BmsFaultCode code,
                const char *reason, uint32_t now,
                bool recordEvent = true, bool persistFault = true);

bool startPrecharge(BmsOperatingState destination,
                    const BmsSettings &settings, uint32_t now) {
  if (destination == BmsOperatingState::Drive) {
    faultLogStartDriveCycle();
  }
  // SimpBMS sequence: first close main negative, then energize precharge.
  // Main positive remains open until the minimum-time/current criteria pass.
  allOutputsOff(settings);
  if (!bmsOutputRequest(BmsOutputRole::MainNegative, true, settings) ||
      !bmsOutputRequest(BmsOutputRole::Precharge, true, settings)) {
    latchFault(settings, BmsFaultCode::PrechargeOutput,
               "Precharge could not energize required mapped outputs.", now);
    return false;
  }
  prechargeStartedAtMs = now;
  stateStatus.prechargeElapsedMs = 0;
  prechargeDestination = destination;
  setState(BmsOperatingState::Precharge,
           "Negative contactor and precharge path active; waiting for time/current criteria.",
           now);
  return true;
}

void latchFault(const BmsSettings &settings, BmsFaultCode code,
                const char *reason, uint32_t now, bool recordEvent,
                bool persistFault) {
  const BmsOperatingState faultedState = stateStatus.state;
  // Ask the Outlander OBC for zero current before opening the pack path when
  // possible; the contactors still open immediately if CAN cannot queue it.
  if (stateStatus.state == BmsOperatingState::Charge &&
      settings.chargerType == ChargerType::Outlander) {
    outlanderChargerSendSetpoint(settings, 0.0f, batteryCanBusReady());
  }
  allOutputsOff(settings);
  armed = false;
  if (faultedState != BmsOperatingState::Fault) {
    if (persistFault) faultLogSetPersistentFault(code);
    if (recordEvent) faultLogRecord(code, faultedState);
  }
  faultReason = reason;
  setState(BmsOperatingState::Fault, faultReason, now);
}

bool finishPrecharge(const BmsSettings &settings, uint32_t now) {
  // Match the SimpBMS sequence: close the positive main contactor, then open
  // the precharge path. The negative contactor remains closed.
  if (!bmsOutputRequest(BmsOutputRole::MainPositive, true, settings) ||
      !bmsOutputRequest(BmsOutputRole::Precharge, false, settings)) {
    latchFault(settings, BmsFaultCode::PrechargeOutput,
               "Could not transfer from precharge to main contactor.", now);
    return false;
  }
  if (prechargeDestination == BmsOperatingState::Charge) {
    if (settings.chargerType == ChargerType::Outlander) {
      if (!outlanderChargerAvailableForCharge(settings,
                                              batteryCanBusReady())) {
        allOutputsOff(settings);
        setState(BmsOperatingState::Ready,
                 "Outlander mains/CAN status was lost during precharge; outputs OFF.", now);
        return false;
      }
      setState(BmsOperatingState::Charge,
               "Precharge complete; Outlander OBC CAN current control enabled.", now);
    } else {
      if (settings.chargerType != ChargerType::ExternalRelay ||
          settings.chargerRelayPin == 0xFFU ||
          !chargerRelayRequest(true, settings)) {
        latchFault(settings, BmsFaultCode::Charger,
                   "Charge relay is not configured for external-relay charging.", now);
        return false;
      }
      setState(BmsOperatingState::Charge,
               "Precharge complete; external charger relay enabled.", now);
    }
  } else {
    setState(BmsOperatingState::Drive,
             "Precharge complete; main contactors closed for Drive.", now);
  }
  return true;
}

bool currentBelowPrechargeThreshold(const CurrentSensorStatus &current,
                                    const BmsSettings &settings) {
  int32_t magnitude = current.currentMilliamp;
  if (magnitude < 0) magnitude = -magnitude;
  return static_cast<float>(magnitude) <=
         settings.prechargeFinishCurrentA * 1000.0f;
}
}

void bmsStateMachineBegin(const BmsSettings &settings) {
  configuredRunInputPin = 0xFFU;
  configuredChargeInputPin = 0xFFU;
  configureInputs(settings);
  runInput = {};
  chargeInput = {};
  // Never restore an armed state after an MCU reboot; all gates start OFF.
  armed = false;
  manualDisarmInhibit = false;
  armInterlocksValid = false;
  faultReason = nullptr;
  stackRecoveryActive = false;
  stackRecoveryRetriesExhausted = false;
  stackHealthyForFaultClear = false;
  consecutiveHealthyStackScans = 0U;
  stateStatus = {};
  stateStatus.state = BmsOperatingState::Disarmed;
  stateStatus.reason = "Waiting for valid stack/current interlocks; Run requests Drive and charge input/OBC mains requests Charge.";
  allOutputsOff(settings);
}

void bmsStateMachineUpdate(const BmsSettings &settings,
                           const BatteryLimitStatus &limits,
                           const CurrentSensorStatus &current,
                           bool stackReady, bool cellDataFresh,
                           bool configurationMenuOpen) {
  const uint32_t now = millis();
  configureInputs(settings);
  stateStatus.runRequested =
      updateInput(runInput, settings.runInputPin, settings.runInputActiveHigh, now);
  stateStatus.chargeInputActive =
      updateInput(chargeInput, settings.chargeInputPin,
                  settings.chargeInputActiveHigh, now);
  stateStatus.chargerMainsDetected = false;
  if (settings.chargerType == ChargerType::Outlander) {
    const OutlanderChargerStatus charger =
        outlanderChargerGetStatus(batteryCanBusReady());
    stateStatus.chargerMainsDetected = charger.charger389Fresh &&
                                       charger.mainsPresent;
  }
  // The configured isolated 240 V input remains supported. With the Outlander
  // profile, fresh charger CAN mains status can also request charging directly.
  stateStatus.chargeRequested = stateStatus.chargeInputActive ||
                                stateStatus.chargerMainsDetected;
  stateStatus.currentValid = current.valid;
  stateStatus.currentMilliamp = current.currentMilliamp;
  stateStatus.cellDataFresh = cellDataFresh;
  const bool directChargerCanOperate =
      chargerRelayAvailable(settings) ||
      (settings.chargerType == ChargerType::Outlander &&
       settings.chargerTxEnabled && batteryCanBusReady());
  const bool directChargeOutputsReady = !settings.chargerBehindContactors &&
      stateStatus.chargeRequested && directChargerCanOperate;
  const bool requiredOutputsMapped = contactorPinsAssigned(settings) ||
                                     directChargeOutputsReady;
  armInterlocksValid = stackReady && cellDataFresh &&
                       limits.readingsValid && current.valid &&
                       requiredOutputsMapped &&
                       settings.currentSensor == CurrentSensorType::LemCab500Can &&
                       !settings.isolationMonitorEnabled &&
                       faultLogAvailable();

  clearRecoveredCommunicationFaultIfReady();

  if (settings.isolationMonitorEnabled &&
      (stateStatus.state == BmsOperatingState::Precharge ||
       stateStatus.state == BmsOperatingState::Drive ||
       stateStatus.state == BmsOperatingState::Charge)) {
    latchFault(settings, BmsFaultCode::IsolationMonitor,
               "Isolation monitoring was enabled during operation, but its Bender/SME backend is not implemented.",
               now);
    return;
  }

  if (settings.isolationMonitorEnabled &&
      (stateStatus.state == BmsOperatingState::Disarmed ||
       stateStatus.state == BmsOperatingState::Ready)) {
    armed = false;
    allOutputsOff(settings);
    setState(BmsOperatingState::Disarmed,
             "Isolation monitor is enabled but Bender/SME input handling is not implemented; control remains disarmed.",
             now);
    return;
  }

  if (settings.currentSensor == CurrentSensorType::SmePlaceholder &&
      (stateStatus.state == BmsOperatingState::Disarmed ||
       stateStatus.state == BmsOperatingState::Ready)) {
    armed = false;
    allOutputsOff(settings);
    setState(BmsOperatingState::Disarmed,
             "SME current sensor is a placeholder; its decoder is not implemented, so BMS control remains disarmed.",
             now);
    return;
  }

  if (!armed) {
    allOutputsOff(settings);
    if (stateStatus.state == BmsOperatingState::Fault) return;
    if (manualDisarmInhibit) {
      if (!stateStatus.runRequested && !stateStatus.chargeRequested) {
        manualDisarmInhibit = false;
        setState(BmsOperatingState::Disarmed,
                 "Run and charge requests returned inactive; automatic control may re-arm when interlocks are valid.", now);
      } else {
      stateStatus.reason = "Manually disarmed; waiting for Run and charge requests to return inactive.";
      }
      return;
    }
    if (!armInterlocksValid) {
      if (!faultLogAvailable()) {
        stateStatus.reason = "Persistent fault-log EEPROM storage is unavailable; BMS arming is inhibited.";
        return;
      }
      if (settings.isolationMonitorEnabled) {
        stateStatus.reason = "Isolation monitoring is enabled, but Bender/SME input handling is not implemented; disable it until the monitor is integrated.";
        return;
      }
      if ((settings.mainNegativeControl == ContactorControlType::SmePlaceholder ||
           settings.prechargeControl == ContactorControlType::SmePlaceholder) &&
          !contactorPinsAssigned(settings)) {
        stateStatus.reason = "SME main-negative/precharge control is a placeholder; select Nano low-side FET control for Drive or behind-contactor Charge.";
        return;
      }
      if (settings.currentSensor == CurrentSensorType::SmePlaceholder) {
        stateStatus.reason = "SME current sensor is selected as a placeholder; its decoder is not implemented, so control remains disarmed.";
        return;
      }
      setState(BmsOperatingState::Disarmed,
               "Waiting for fresh stack/current data and outputs required by the selected operating path.", now);
      return;
    }
    if (stateStatus.runRequested && stateStatus.chargeRequested) {
      setState(BmsOperatingState::Disarmed,
               "Run and charge requests are both active; automatic start is inhibited.", now);
      return;
    }
    // No terminal ARM command is needed: once checks pass, a Run, isolated
    // charge input or Outlander mains report can request operation.
    armed = true;
    setState(BmsOperatingState::Ready,
             "Interlocks valid; automatically armed and waiting for Run, 240 V input or Outlander mains.", now);
  }

  if (stateStatus.state == BmsOperatingState::Fault) {
    allOutputsOff(settings);
    return;
  }

  if (stateStatus.state == BmsOperatingState::Ready && configurationMenuOpen) {
    // Do not begin a new operating cycle while the user is editing settings;
    // active Drive/Charge cycles continue to be supervised below.
    allOutputsOff(settings);
    stateStatus.reason = "Armed and Ready; menu open, waiting for menu exit before starting.";
    return;
  }

  const bool active = stateStatus.state == BmsOperatingState::Precharge ||
                      stateStatus.state == BmsOperatingState::Drive ||
                      stateStatus.state == BmsOperatingState::Charge;
  const bool chargingPath = stateStatus.state == BmsOperatingState::Charge ||
      (stateStatus.state == BmsOperatingState::Precharge &&
       prechargeDestination == BmsOperatingState::Charge);

  // A transient BQ stack outage during Drive is reported and retried without
  // taking the ordinary DISARM path or dropping the contactor commands. Keep
  // other live interlocks active. If recovery is exhausted, retain this state
  // only until the physical RUN request is removed, then latch Fault.
  if (stateStatus.state == BmsOperatingState::Drive &&
      stackRecoveryActive) {
    if (!current.valid) {
      latchFault(settings, BmsFaultCode::CurrentSensor,
                 "LEM current data became stale or faulted during stack recovery.",
                 now);
      return;
    }
    if (!stateStatus.runRequested) {
      latchFault(settings,
                 stackRecoveryRetriesExhausted
                     ? BmsFaultCode::StackRecoveryFailed
                     : BmsFaultCode::StackCommunication,
                 "RUN request removed while the BQ stack communication fault was active; communication lockout clears after three consecutive healthy scans with requests inactive.",
                 now);
      return;
    }
    if (!bmsOutputRequest(BmsOutputRole::MainNegative, true, settings) ||
        !bmsOutputRequest(BmsOutputRole::MainPositive, true, settings) ||
        !bmsOutputRequest(BmsOutputRole::Precharge, false, settings) ||
        !requestChargerRelay(false, settings)) {
      latchFault(settings, BmsFaultCode::ContactorOutput,
                 "Drive outputs could not be maintained during BQ stack recovery.",
                 now);
      return;
    }
    stateStatus.reason = stackRecoveryRetriesExhausted
        ? "STACK FAULT: retries exhausted; Drive outputs held until RUN is removed. Communication lockout clears after three healthy scans with requests inactive."
        : "STACK FAULT: automatic BQ recovery running; Drive outputs held while RUN remains active.";
    return;
  }

  if (active && !stackReady) {
    latchFault(settings, BmsFaultCode::StackCommunication,
               "BQ stack communication is not ready.", now);
    return;
  }
  if (active && !cellDataFresh) {
    latchFault(settings, BmsFaultCode::CellDataStale,
               "Cell data became stale during an active state.", now);
    return;
  }
  if (active && !limits.readingsValid) {
    latchFault(settings, BmsFaultCode::CellDataInvalid,
               "Cell data became invalid during an active state.", now);
    return;
  }
  // A high cell blocks charging. Do not fault or inhibit Drive for OV alone:
  // controlled discharge is how the pack can recover from a high cell.
  if (active && chargingPath && limits.overVoltage) {
    latchFault(settings, BmsFaultCode::CellOverVoltage,
               "Cell over-voltage limit reached during Charge or charge precharge.", now);
    return;
  }
  if (active && !chargingPath && !safeForDrive(limits)) {
    latchFault(settings, BmsFaultCode::CellUnderVoltage,
               "A configured cell under-voltage or discharge limit was reached.", now);
    return;
  }
  if (active && !current.valid) {
    latchFault(settings, BmsFaultCode::CurrentSensor,
               "LEM current data became stale or faulted.", now);
    return;
  }

  if (stateStatus.state == BmsOperatingState::Ready) {
    allOutputsOff(settings);
    if (!stackReady || !cellDataFresh || !limits.readingsValid || !current.valid) {
      stateStatus.reason = "Waiting for fresh, valid full-stack cell and current readings.";
      return;
    }
    if (stateStatus.runRequested && stateStatus.chargeRequested) {
      stateStatus.reason = "RUN and charge requests are both active; request inhibited.";
      return;
    }
    if (stateStatus.runRequested) {
      if (!contactorPinsAssigned(settings)) {
        stateStatus.reason =
            (settings.mainNegativeControl == ContactorControlType::SmePlaceholder ||
             settings.prechargeControl == ContactorControlType::SmePlaceholder)
                ? "Run inhibited: SME main-negative/precharge control is a placeholder and is not implemented."
                : "Run request inhibited: map main positive, main negative and precharge outputs.";
        return;
      }
      if (!safeForDrive(limits)) {
        if (limits.underVoltage || limits.dischargeBlocked) {
          latchFault(settings, BmsFaultCode::CellUnderVoltage,
                     "Cell under-voltage or discharge cut-off triggered Fault on a Drive request.",
                     now);
          return;
        }
        stateStatus.reason = "Run request inhibited by cell under-voltage/discharge limit.";
        return;
      }
      startPrecharge(BmsOperatingState::Drive, settings, now);
      return;
    }
    if (stateStatus.chargeRequested) {
      if (limits.overVoltage) {
        stateStatus.reason = "Charge inhibited by cell over-voltage; discharge remains available.";
        return;
      }
      if (settings.chargerType == ChargerType::Outlander) {
        const OutlanderChargerStatus charger =
            outlanderChargerGetStatus(batteryCanBusReady());
        if (!settings.chargerTxEnabled) {
          stateStatus.reason = "Charge inhibited: enable Outlander CAN in the Charging menu.";
          return;
        }
        if (!charger.canBusReady) {
          stateStatus.reason = "Charge inhibited: shared CAN bus is not ready.";
          return;
        }
        if (!charger.charger389Fresh) {
          stateStatus.reason = "Charge inhibited: waiting for fresh Outlander 0x389 telemetry; Ready sends 0x285 wake frames.";
          return;
        }
        if (charger.chargerError) {
          stateStatus.reason = "Charge inhibited: Outlander 0x389 reports an OBC/CAN error.";
          return;
        }
        if (!charger.mainsPresent) {
          stateStatus.reason = "Charge inhibited: Outlander reports no AC mains; waiting for charger plug-in.";
          return;
        }
      } else if (settings.chargerType != ChargerType::ExternalRelay ||
                 settings.chargerRelayPin == 0xFFU) {
        stateStatus.reason = "Charge inhibited: select External Relay with a mapped output, or enable Outlander CAN.";
        return;
      }
      if (!safeForCharge(limits) || limits.highestCellV >=
          settings.chargeVoltageV - settings.chargeHysteresisV) {
        stateStatus.reason = "Charge inhibited by cell voltage / charge hysteresis.";
        return;
      }
      if (settings.chargerBehindContactors) {
        if (!contactorPinsAssigned(settings)) {
          stateStatus.reason =
              (settings.mainNegativeControl == ContactorControlType::SmePlaceholder ||
               settings.prechargeControl == ContactorControlType::SmePlaceholder)
                  ? "Charge inhibited: SME main-negative/precharge control is a placeholder and is not implemented."
                  : "Charge inhibited: behind-contactors mode needs mapped main and precharge outputs.";
          return;
        }
        startPrecharge(BmsOperatingState::Charge, settings, now);
      } else if (settings.chargerType == ChargerType::Outlander) {
        setState(BmsOperatingState::Charge,
                 "Outlander charger detected; direct-pack OBC CAN charge enabled.", now);
      } else if (requestChargerRelay(true, settings)) {
        setState(BmsOperatingState::Charge,
                 "Direct-pack charge enabled; main contactors remain open.", now);
      } else {
        stateStatus.reason = "Direct-pack charge inhibited: external charger relay could not be enabled.";
      }
      return;
    }
    stateStatus.reason = "Armed and Ready; waiting for RUN or charge input/OBC mains detection.";
    return;
  }

  if (stateStatus.state == BmsOperatingState::Precharge) {
    const bool requestRemains = prechargeDestination == BmsOperatingState::Drive
        ? stateStatus.runRequested : stateStatus.chargeRequested;
    if (!requestRemains ||
        (stateStatus.runRequested && stateStatus.chargeRequested)) {
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               "Precharge aborted because its request changed; outputs OFF.", now);
      return;
    }
    if (prechargeDestination == BmsOperatingState::Charge &&
        settings.chargerType == ChargerType::Outlander &&
        !outlanderChargerAvailableForCharge(settings,
                                            batteryCanBusReady())) {
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               "Outlander mains/CAN status was lost during precharge; outputs OFF.", now);
      return;
    }
    if (prechargeDestination == BmsOperatingState::Charge &&
        (!safeForCharge(limits) ||
         limits.highestCellV >= settings.chargeVoltageV -
                                    settings.chargeHysteresisV)) {
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               "Charge precharge stopped because the cell-voltage target was reached.", now);
      return;
    }
    if (now - prechargeStartedAtMs >= settings.prechargeTimeoutMs) {
      latchFault(settings, BmsFaultCode::PrechargeTimeout,
                 "Precharge timed out before current fell below its threshold.", now);
      return;
    }
    const uint32_t elapsed = now - prechargeStartedAtMs;
    if (elapsed >= settings.prechargeMinimumMs &&
        currentBelowPrechargeThreshold(current, settings)) {
      finishPrecharge(settings, now);
      return;
    }
    stateStatus.prechargeElapsedMs = elapsed;
    stateStatus.reason = "Precharge active; waiting for minimum time and finish-current threshold.";
    return;
  }

  if (stateStatus.state == BmsOperatingState::Drive) {
    if (!bmsOutputRequest(BmsOutputRole::MainNegative, true, settings) ||
        !bmsOutputRequest(BmsOutputRole::MainPositive, true, settings) ||
        !bmsOutputRequest(BmsOutputRole::Precharge, false, settings) ||
        !requestChargerRelay(false, settings)) {
      latchFault(settings, BmsFaultCode::ContactorOutput,
                 "A mapped Drive output could not be applied.", now);
      return;
    }
    if (stateStatus.runRequested && stateStatus.chargeRequested) {
      stateStatus.reason = "Drive active; simultaneous charge request ignored.";
    } else if (!stateStatus.runRequested && stateStatus.chargeRequested) {
      if (chargerCanRequestCharge(settings) && safeForCharge(limits) &&
          limits.highestCellV < settings.chargeVoltageV -
                                    settings.chargeHysteresisV) {
        if (settings.chargerBehindContactors) {
          const bool chargerEnabled =
              settings.chargerType == ChargerType::Outlander ||
              requestChargerRelay(true, settings);
          if (chargerEnabled) {
            setState(BmsOperatingState::Charge,
                     "RUN request removed; charge accepted with contactors already closed.", now);
          } else {
            allOutputsOff(settings);
            setState(BmsOperatingState::Ready,
                     "Drive stopped; external charger relay could not be enabled.", now);
          }
        } else {
          allOutputsOff(settings);
          const bool chargerEnabled =
              settings.chargerType == ChargerType::Outlander ||
              requestChargerRelay(true, settings);
          if (chargerEnabled) {
            setState(BmsOperatingState::Charge,
                     settings.chargerType == ChargerType::Outlander
                         ? "RUN request removed; direct-pack Outlander CAN charge enabled."
                         : "RUN request removed; direct-pack relay charge enabled.", now);
          } else {
            setState(BmsOperatingState::Ready,
                     "Drive stopped; direct charger relay could not be enabled.", now);
          }
        }
      } else {
        allOutputsOff(settings);
        setState(BmsOperatingState::Ready,
                 "Drive request ended; charge request is not configured or permitted.", now);
      }
    } else if (!stateStatus.runRequested) {
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               "RUN request removed; contactors opened.", now);
    } else {
      stateStatus.reason = "RUN active; main contactors closed.";
    }
    return;
  }

  if (stateStatus.state == BmsOperatingState::Charge) {
    if (settings.chargerType == ChargerType::Outlander) {
      const OutlanderChargerStatus charger =
          outlanderChargerGetStatus(batteryCanBusReady());
      if (!charger.canBusReady || !charger.charger389Fresh ||
          charger.chargerError || !charger.mainsPresent) {
        latchFault(settings, BmsFaultCode::Charger,
                   "Outlander CAN telemetry, mains or charger status became unsafe during Charge.",
                   now);
        return;
      }
    }
    const bool contactorOutputsOk = settings.chargerBehindContactors
        ? (contactorPinsAssigned(settings) &&
           bmsOutputRequest(BmsOutputRole::MainNegative, true, settings) &&
           bmsOutputRequest(BmsOutputRole::MainPositive, true, settings) &&
           bmsOutputRequest(BmsOutputRole::Precharge, false, settings))
        : openMappedContactors(settings);
    if (!contactorOutputsOk) {
      latchFault(settings, BmsFaultCode::ContactorOutput,
                 "Charge contactor outputs do not match the selected charger connection path.", now);
      return;
    }
    if (!stateStatus.chargeRequested || stateStatus.runRequested ||
        limits.chargePausedByHysteresis) {
      if (settings.chargerType == ChargerType::Outlander &&
          batteryCanBusReady() &&
          !outlanderChargerSendSetpoint(settings, 0.0f,
                                        batteryCanBusReady())) {
        stateStatus.reason = "Stopping Outlander charge; waiting for a CAN transmit slot before opening the contactors.";
        return;
      }
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               limits.chargePausedByHysteresis
                   ? "Charge stopped at the configured cell target/hysteresis."
                   : "Charge request ended or RUN became active; outputs OFF.", now);
      return;
    }
    if (settings.chargerType == ChargerType::Outlander) {
      stateStatus.reason = settings.chargerBehindContactors
          ? "Outlander charging; CAN setpoint active with main contactors closed."
          : "Outlander charging; CAN setpoint active with main contactors open.";
    } else {
      if (!requestChargerRelay(true, settings)) {
        latchFault(settings, BmsFaultCode::Charger,
                   "External charger relay output could not be enabled.", now);
        return;
      }
      stateStatus.reason = settings.chargerBehindContactors
          ? "Charge active through external charger relay with main contactors closed."
          : "Charge active through external charger relay; main contactors remain open.";
    }
  }
}

void bmsStateMachineSetStackRecovery(bool active, bool retriesExhausted) {
  stackRecoveryActive = active;
  stackRecoveryRetriesExhausted = active && retriesExhausted;
  stateStatus.stackCommunicationFault = active;
  if (active) {
    consecutiveHealthyStackScans = 0U;
  } else if (consecutiveHealthyStackScans <
             STACK_SCANS_TO_AUTO_CLEAR_COMM_FAULT) {
    // The caller clears recovery only after a complete, valid full-stack scan.
    ++consecutiveHealthyStackScans;
  }
}

void bmsStateMachineSetStackHealth(bool ready, bool dataFresh,
                                   bool readingsValid) {
  stackHealthyForFaultClear = ready && dataFresh && readingsValid;
}

void bmsStateMachineReportStackFault(const char *reason) {
  if (stateStatus.state == BmsOperatingState::Fault) {
    faultLogRecord(BmsFaultCode::StackRecoveryFailed, stateStatus.state);
    return;
  }
  if (stateStatus.state == BmsOperatingState::Drive &&
      stateStatus.runRequested) {
    // Preserve the user-selected Drive hold policy; the regular update will
    // latch the persistent fault when the physical RUN request goes inactive.
    return;
  }
  latchFault(settingsGet(), BmsFaultCode::StackRecoveryFailed,
             reason ? reason : "BQ stack recovery failed.", millis());
}

void bmsStateMachineRestorePersistentFault() {
  if (!faultLogHasPersistentFault()) return;
  const BmsFaultCode code = faultLogPersistentFaultCode();
  const char *reason = isCommunicationFault(code)
      ? "Communication fault restored from EEPROM; waiting for three consecutive complete stack scans and inactive Run/charge requests to clear automatically."
      : "Persistent BMS fault restored from EEPROM. Resolve the cause, restore a healthy stack scan, and use CLEARFAULT before re-arming.";
  latchFault(settingsGet(), code, reason, millis(), false, false);
}

void bmsStateMachineDisarm(const BmsSettings &settings) {
  // This command is an immediate software stop. If a request is still held,
  // remain inhibited until both physical requests return inactive.
  if (stateStatus.state == BmsOperatingState::Drive) {
    stateStatus.reason = "DISARM command ignored in Drive; remove the physical RUN request for a controlled state transition.";
    return;
  }
  if (stateStatus.state == BmsOperatingState::Charge &&
      settings.chargerType == ChargerType::Outlander) {
    outlanderChargerSendSetpoint(settings, 0.0f, batteryCanBusReady());
  }
  allOutputsOff(settings);
  armed = false;
  if (stateStatus.state == BmsOperatingState::Fault) {
    manualDisarmInhibit = true;
    stateStatus.reason = "Fault remains latched; use CLEARFAULT after Run and all charge requests are inactive.";
    return;
  }
  manualDisarmInhibit = true;
  stateStatus.prechargeElapsedMs = 0;
  setState(BmsOperatingState::Disarmed,
           "Disarmed by command; all mapped outputs OFF.", millis());
}

bool bmsStateMachineClearFault() {
  // A latched fault can only be cleared after both debounced requests release;
  // the machine then remains disarmed until fresh readings/request handling.
  if (stateStatus.state != BmsOperatingState::Fault ||
      stateStatus.runRequested || stateStatus.chargeRequested ||
      !stackHealthyForFaultClear || !faultLogClearPersistentFault()) return false;
  armed = false;
  manualDisarmInhibit = false;
  faultReason = nullptr;
  stateStatus.stackCommunicationFault = false;
  stateStatus.prechargeElapsedMs = 0;
  consecutiveHealthyStackScans = 0U;
  setState(BmsOperatingState::Disarmed,
           "Fault cleared; waiting for fresh interlocks and a Run/charge request.", millis());
  return true;
}

bool bmsStateMachineReportIsolationFault() {
  const BmsSettings &settings = settingsGet();
  if (!settings.isolationMonitorEnabled) return false;
  if (stateStatus.state != BmsOperatingState::Fault) {
    latchFault(settings, BmsFaultCode::IsolationMonitor,
               "Isolation monitor reported an unsafe isolation condition.",
               millis());
  }
  return true;
}

bool bmsStateMachineCanEditSettings() {
  return stateStatus.state == BmsOperatingState::Disarmed ||
         stateStatus.state == BmsOperatingState::Ready ||
         stateStatus.state == BmsOperatingState::Fault;
}

BmsStateStatus bmsStateMachineStatus() {
  if (stateStatus.state == BmsOperatingState::Precharge) {
    stateStatus.prechargeElapsedMs = millis() - prechargeStartedAtMs;
  }
  stateStatus.chargePrechargeActive =
      stateStatus.state == BmsOperatingState::Precharge &&
      prechargeDestination == BmsOperatingState::Charge;
  stateStatus.armed = armed;
  return stateStatus;
}

const char *bmsOperatingStateName(BmsOperatingState state) {
  switch (state) {
    case BmsOperatingState::Disarmed: return "DISARMED";
    case BmsOperatingState::Ready: return "READY";
    case BmsOperatingState::Precharge: return "PRECHARGE";
    case BmsOperatingState::Drive: return "DRIVE";
    case BmsOperatingState::Charge: return "CHARGE";
    case BmsOperatingState::Fault: return "FAULT";
    default: return "UNKNOWN";
  }
}

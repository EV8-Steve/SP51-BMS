/*
  Input-driven BMS operating state machine.

  Run and isolated 240 V-present inputs request Drive and Charge respectively.
  The controller starts DISARMED after reset, auto-arms only after complete
  stack data, fresh LEM current and required contactor mappings are available,
  then waits in READY. It keeps checking those interlocks during active states;
  lost/unsafe readings latch FAULT and command every mapped output LOW.
*/
#include "bms_state_machine.h"

#include "bms_outputs.h"
#include "charger_relay.h"

namespace {
constexpr uint16_t INPUT_DEBOUNCE_MS = 30;

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
  stateStatus.state = state;
  stateStatus.reason = reason;
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
         settings.mainNegativePin != 0xFFU &&
         settings.prechargePin != 0xFFU;
}

bool safeForDrive(const BatteryLimitStatus &limits) {
  return limits.readingsValid && !limits.overVoltage &&
         !limits.underVoltage && !limits.dischargeBlocked;
}

bool safeForCharge(const BatteryLimitStatus &limits) {
  return limits.readingsValid && !limits.overVoltage &&
         !limits.chargePausedByHysteresis;
}

bool startPrecharge(BmsOperatingState destination,
                    const BmsSettings &settings, uint32_t now) {
  // SimpBMS sequence: first close main negative, then energize precharge.
  // Main positive remains open until the minimum-time/current criteria pass.
  allOutputsOff(settings);
  if (!bmsOutputRequest(BmsOutputRole::MainNegative, true, settings) ||
      !bmsOutputRequest(BmsOutputRole::Precharge, true, settings)) {
    allOutputsOff(settings);
    armed = false;
    setState(BmsOperatingState::Fault,
             "Precharge could not energize required mapped outputs.", now);
    faultReason = stateStatus.reason;
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

void latchFault(const BmsSettings &settings, const char *reason, uint32_t now) {
  allOutputsOff(settings);
  armed = false;
  faultReason = reason;
  setState(BmsOperatingState::Fault, faultReason, now);
}

bool finishPrecharge(const BmsSettings &settings, uint32_t now) {
  // Match the SimpBMS sequence: close the positive main contactor, then open
  // the precharge path. The negative contactor remains closed.
  if (!bmsOutputRequest(BmsOutputRole::MainPositive, true, settings) ||
      !bmsOutputRequest(BmsOutputRole::Precharge, false, settings)) {
    latchFault(settings, "Could not transfer from precharge to main contactor.", now);
    return false;
  }
  if (prechargeDestination == BmsOperatingState::Charge) {
    if (settings.chargerType != ChargerType::ExternalRelay ||
        settings.chargerRelayPin == 0xFFU ||
        !chargerRelayRequest(true, settings)) {
      latchFault(settings, "Charge relay is not configured for external-relay charging.", now);
      return false;
    }
    setState(BmsOperatingState::Charge,
             "Precharge complete; external charger relay enabled.", now);
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
  stateStatus = {};
  stateStatus.state = BmsOperatingState::Disarmed;
  stateStatus.reason = "Waiting for valid stack/current interlocks; Run or 240 V inputs request operation.";
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
  stateStatus.chargeRequested =
      updateInput(chargeInput, settings.chargeInputPin,
                  settings.chargeInputActiveHigh, now);
  stateStatus.currentValid = current.valid;
  stateStatus.currentMilliamp = current.currentMilliamp;
  stateStatus.cellDataFresh = cellDataFresh;
  armInterlocksValid = stackReady && cellDataFresh &&
                       limits.readingsValid && !limits.overVoltage && current.valid &&
                       contactorPinsAssigned(settings) &&
                       settings.currentSensor == CurrentSensorType::LemCab500Can;

  if (!armed) {
    allOutputsOff(settings);
    if (stateStatus.state == BmsOperatingState::Fault) return;
    if (manualDisarmInhibit) {
      if (!stateStatus.runRequested && !stateStatus.chargeRequested) {
        manualDisarmInhibit = false;
        setState(BmsOperatingState::Disarmed,
                 "Run/charge inputs returned inactive; automatic control may re-arm when interlocks are valid.", now);
      } else {
        stateStatus.reason = "Manually disarmed; waiting for Run and charge inputs to return inactive.";
      }
      return;
    }
    if (!armInterlocksValid) {
      setState(BmsOperatingState::Disarmed,
               "Waiting for complete fresh stack data, valid LEM current and mapped contactor outputs.", now);
      return;
    }
    if (stateStatus.runRequested && stateStatus.chargeRequested) {
      setState(BmsOperatingState::Disarmed,
               "Run and 240 V inputs are both active; automatic start is inhibited.", now);
      return;
    }
    // No terminal ARM command is needed: once checks pass, the physical Run or
    // 240 V input is the operating request. Both active together are rejected.
    armed = true;
    setState(BmsOperatingState::Ready,
             "Interlocks valid; automatically armed and waiting for Run or 240 V input.", now);
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
  if (active && !stackReady) {
    latchFault(settings, "BQ stack communication is not ready.", now);
    return;
  }
  if (active && !cellDataFresh) {
    latchFault(settings, "Cell data became stale during an active state.", now);
    return;
  }
  if (active && (!limits.readingsValid || limits.overVoltage)) {
    latchFault(settings, "Cell data is invalid or a cell over-voltage limit was reached.", now);
    return;
  }
  const bool chargingPath = stateStatus.state == BmsOperatingState::Charge ||
      (stateStatus.state == BmsOperatingState::Precharge &&
       prechargeDestination == BmsOperatingState::Charge);
  if (active && !chargingPath && !safeForDrive(limits)) {
    latchFault(settings, "A configured cell under-voltage or discharge limit was reached.", now);
    return;
  }
  if (active && !current.valid) {
    latchFault(settings, "LEM current data became stale or faulted.", now);
    return;
  }

  if (stateStatus.state == BmsOperatingState::Ready) {
    allOutputsOff(settings);
    if (!stackReady || !cellDataFresh || !limits.readingsValid ||
        limits.overVoltage || !current.valid) {
      stateStatus.reason = "Waiting for fresh, valid full-stack cell and current readings.";
      return;
    }
    if (stateStatus.runRequested && stateStatus.chargeRequested) {
      stateStatus.reason = "RUN and charge-present inputs are both active; request inhibited.";
      return;
    }
    if (stateStatus.runRequested) {
      if (!safeForDrive(limits)) {
        stateStatus.reason = "Run request inhibited by cell under-voltage/discharge limit.";
        return;
      }
      startPrecharge(BmsOperatingState::Drive, settings, now);
      return;
    }
    if (stateStatus.chargeRequested) {
      if (settings.chargerType != ChargerType::ExternalRelay ||
          settings.chargerRelayPin == 0xFFU) {
        stateStatus.reason = "Charge inhibited: select External Relay and map its output; Outlander CAN is disabled.";
        return;
      }
      if (!safeForCharge(limits) || limits.highestCellV >=
          settings.chargeVoltageV - settings.chargeHysteresisV) {
        stateStatus.reason = "Charge inhibited by cell voltage / charge hysteresis.";
        return;
      }
      startPrecharge(BmsOperatingState::Charge, settings, now);
      return;
    }
    stateStatus.reason = "Armed and Ready; waiting for RUN or charge-present input.";
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
        (!safeForCharge(limits) ||
         limits.highestCellV >= settings.chargeVoltageV -
                                    settings.chargeHysteresisV)) {
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               "Charge precharge stopped because the cell-voltage target was reached.", now);
      return;
    }
    if (now - prechargeStartedAtMs >= settings.prechargeTimeoutMs) {
      latchFault(settings, "Precharge timed out before current fell below its threshold.", now);
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
      latchFault(settings, "A mapped Drive output could not be applied.", now);
      return;
    }
    if (stateStatus.runRequested && stateStatus.chargeRequested) {
      stateStatus.reason = "Drive active; simultaneous charge request ignored.";
    } else if (!stateStatus.runRequested && stateStatus.chargeRequested) {
      if (settings.chargerType == ChargerType::ExternalRelay &&
          settings.chargerRelayPin != 0xFFU &&
        safeForCharge(limits) &&
        limits.highestCellV < settings.chargeVoltageV -
                                    settings.chargeHysteresisV &&
          requestChargerRelay(true, settings)) {
        setState(BmsOperatingState::Charge,
                 "RUN request removed; charge request accepted with main contactors closed.", now);
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
    if (!bmsOutputRequest(BmsOutputRole::MainNegative, true, settings) ||
        !bmsOutputRequest(BmsOutputRole::MainPositive, true, settings) ||
        !bmsOutputRequest(BmsOutputRole::Precharge, false, settings)) {
      latchFault(settings, "A mapped Charge contactor output could not be applied.", now);
      return;
    }
    if (!stateStatus.chargeRequested || stateStatus.runRequested ||
        limits.chargePausedByHysteresis) {
      allOutputsOff(settings);
      setState(BmsOperatingState::Ready,
               limits.chargePausedByHysteresis
                   ? "Charge stopped at the configured cell target/hysteresis."
                   : "Charge request ended or RUN became active; outputs OFF.", now);
      return;
    }
    if (!requestChargerRelay(true, settings)) {
      latchFault(settings, "External charger relay output could not be enabled.", now);
      return;
    }
    stateStatus.reason = "Charge active through external charger relay.";
  }
}

void bmsStateMachineDisarm(const BmsSettings &settings) {
  // This command is an immediate software stop. If a request is still held,
  // remain inhibited until both physical requests return inactive.
  allOutputsOff(settings);
  armed = false;
  if (stateStatus.state == BmsOperatingState::Fault) {
    manualDisarmInhibit = true;
    stateStatus.reason = "Fault remains latched; use CLEARFAULT with both isolated requests inactive.";
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
      stateStatus.runRequested || stateStatus.chargeRequested) return false;
  armed = false;
  manualDisarmInhibit = false;
  faultReason = nullptr;
  stateStatus.prechargeElapsedMs = 0;
  setState(BmsOperatingState::Disarmed,
           "Fault cleared; waiting for fresh interlocks and a physical Run/240 V request.", millis());
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

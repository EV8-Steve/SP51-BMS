/*
  Pack state-of-charge estimator.

  Coulomb counting follows the SimpBMS signed-current approach. Positive
  current adds charge and negative current removes charge. Capacity is the
  configured total pack capacity in Ah; series cell count does not multiply it.
  When no valid retained SOC exists, the configured lowest-cell voltage curve
  seeds the first estimate. SOC and cumulative Ah-in/Ah-out are periodically
  saved through the settings module's CRC-protected rotating EEPROM records.
*/
#include "soc_estimator.h"

#include <math.h>

namespace {
constexpr uint32_t SOC_SAVE_PERIOD_MS = 60000UL;

bool initialized = false;
bool seededFromVoltage = false;
bool stateStored = false;
bool dirty = false;
bool saveRequested = false;
float socPercent = 0.0f;
float chargedAh = 0.0f;
float dischargedAh = 0.0f;
uint32_t lastSavedAtMs = 0;
uint32_t lastSaveAttemptAtMs = 0;

uint32_t lastSeenFrameCount = 0;
bool previousCurrentValid = false;
int32_t previousCurrentMilliamp = 0;
uint32_t previousCurrentAtMs = 0;

float clampPercent(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 100.0f) return 100.0f;
  return value;
}

float estimatePercentFromVoltage(const BmsSettings &settings,
                                 float lowestCellV) {
  const float lowMv = static_cast<float>(settings.socLowCellMv);
  const float highMv = static_cast<float>(settings.socHighCellMv);
  const float measuredMv = lowestCellV * 1000.0f;
  const float fraction = (measuredMv - lowMv) / (highMv - lowMv);
  const float percent = settings.socLowPercent +
      fraction * (settings.socHighPercent - settings.socLowPercent);
  return clampPercent(percent);
}

uint32_t toMilliAh(float ampHours) {
  if (!(ampHours > 0.0f)) return 0;
  const double milliAh = static_cast<double>(ampHours) * 1000.0;
  if (milliAh >= 4294967295.0) return 0xFFFFFFFFUL;
  return static_cast<uint32_t>(milliAh + 0.5);
}

PersistentSocState makePersistentState() {
  PersistentSocState saved = {};
  const float boundedPercent = clampPercent(socPercent);
  saved.percentBasisPoints = static_cast<uint16_t>(boundedPercent * 100.0f + 0.5f);
  saved.chargedMilliAh = toMilliAh(chargedAh);
  saved.dischargedMilliAh = toMilliAh(dischargedAh);
  return saved;
}

bool saveNow() {
  if (!initialized) return false;
  lastSaveAttemptAtMs = millis();
  const bool saved = settingsSaveSocState(makePersistentState());
  if (saved) {
    stateStored = true;
    dirty = false;
    saveRequested = false;
    lastSavedAtMs = lastSaveAttemptAtMs;
  }
  return saved;
}

void initializeFromVoltage(const BmsSettings &settings,
                           const BatteryLimitStatus &limits) {
  if (!limits.readingsValid) return;
  socPercent = estimatePercentFromVoltage(settings, limits.lowestCellV);
  initialized = true;
  seededFromVoltage = true;
  dirty = true;
  // Ask the scheduler to save this first seed outside the CAN/BQ hot path.
  saveRequested = true;
}

void integrateCurrent(const BmsSettings &settings,
                      const CurrentSensorStatus &current,
                      bool useForSoc) {
  const uint32_t now = millis();
  if (!current.valid) {
    if (previousCurrentValid &&
        now - previousCurrentAtMs > settings.currentSensorTimeoutMs) {
      previousCurrentValid = false;
    }
    return;
  }
  if (current.frameCount == lastSeenFrameCount) return;
  lastSeenFrameCount = current.frameCount;

  if (!previousCurrentValid) {
    previousCurrentMilliamp = current.currentMilliamp;
    previousCurrentAtMs = current.lastFrameAtMs;
    previousCurrentValid = true;
    return;
  }

  const uint32_t elapsedMs = current.lastFrameAtMs - previousCurrentAtMs;
  if (elapsedMs == 0U || elapsedMs > settings.currentSensorTimeoutMs) {
    // A long CAN gap has unknown current between samples; rebase without
    // inventing charge/discharge during the missing-data interval.
    previousCurrentMilliamp = current.currentMilliamp;
    previousCurrentAtMs = current.lastFrameAtMs;
    return;
  }

  // Trapezoidal integration smooths the step between adjacent current frames.
  const double averageMilliamp =
      (static_cast<double>(previousCurrentMilliamp) + current.currentMilliamp) * 0.5;
  const double deltaAh = averageMilliamp * elapsedMs / 3600000000.0;
  if (deltaAh > 0.0) {
    chargedAh += static_cast<float>(deltaAh);
  } else if (deltaAh < 0.0) {
    dischargedAh += static_cast<float>(-deltaAh);
  }
  if (useForSoc && settings.batteryCapacityAh > 0.0f) {
    socPercent = clampPercent(socPercent + static_cast<float>(
        deltaAh * 100.0 / settings.batteryCapacityAh));
  }
  dirty = true;
  previousCurrentMilliamp = current.currentMilliamp;
  previousCurrentAtMs = current.lastFrameAtMs;
}

void persistIfDue() {
  const uint32_t now = millis();
  if (!initialized || !dirty ||
      now - lastSaveAttemptAtMs < SOC_SAVE_PERIOD_MS) return;
  if (!saveRequested && now - lastSavedAtMs < SOC_SAVE_PERIOD_MS) return;
  saveNow();
}
}

void socEstimatorBegin() {
  initialized = false;
  seededFromVoltage = false;
  stateStored = false;
  dirty = false;
  saveRequested = false;
  socPercent = 0.0f;
  chargedAh = 0.0f;
  dischargedAh = 0.0f;
  lastSavedAtMs = millis();
  lastSaveAttemptAtMs = millis() - SOC_SAVE_PERIOD_MS;
  lastSeenFrameCount = 0;
  previousCurrentValid = false;
  previousCurrentMilliamp = 0;
  previousCurrentAtMs = 0;

  PersistentSocState retained = {};
  if (settingsLoadSocState(retained) && retained.percentBasisPoints <= 10000U) {
    socPercent = retained.percentBasisPoints / 100.0f;
    chargedAh = retained.chargedMilliAh / 1000.0f;
    dischargedAh = retained.dischargedMilliAh / 1000.0f;
    initialized = true;
    stateStored = true;
    dirty = false;
  }
}

SocStatus socEstimatorUpdate(const BmsSettings &settings,
                             const BatteryLimitStatus &limits,
                             const CurrentSensorStatus &current) {
  if (settings.socMode == SocMode::VoltageOnly) {
    if (limits.readingsValid) {
      const float estimate = estimatePercentFromVoltage(settings, limits.lowestCellV);
      if (!initialized || fabsf(estimate - socPercent) >= 0.005f) dirty = true;
      socPercent = estimate;
      initialized = true;
      seededFromVoltage = true;
    }
    // Voltage-only mode still accumulates Ah-in/Ah-out for the throughput
    // display; current affects SOC only while coulomb counting is selected.
    if (initialized) integrateCurrent(settings, current, false);
  } else {
    if (!initialized) initializeFromVoltage(settings, limits);
    if (initialized) integrateCurrent(settings, current, true);
  }

  return socEstimatorStatus(settings);
}

void socEstimatorServicePersistence() {
  persistIfDue();
}

bool socEstimatorSetPercent(float percent, const BmsSettings &settings) {
  if (settings.socMode != SocMode::CoulombCount ||
      !(percent >= 0.0f && percent <= 100.0f)) return false;
  const float previousPercent = socPercent;
  const bool previousInitialized = initialized;
  const bool previousVoltageSeeded = seededFromVoltage;
  const bool previousStateStored = stateStored;
  const bool previousDirty = dirty;
  const bool previousSaveRequested = saveRequested;
  const bool previousCurrentBaselineValid = previousCurrentValid;
  const int32_t previousBaselineMilliamp = previousCurrentMilliamp;
  const uint32_t previousBaselineAtMs = previousCurrentAtMs;
  const uint32_t previousSavedAtMs = lastSavedAtMs;
  const uint32_t previousSaveAttemptAtMs = lastSaveAttemptAtMs;
  socPercent = percent;
  initialized = true;
  seededFromVoltage = false;
  previousCurrentValid = false;
  dirty = true;
  saveRequested = true;
  if (saveNow()) return true;

  // A calibration command reports success only when its CRC record verifies.
  // Roll back the live estimate as well if persistence fails.
  socPercent = previousPercent;
  initialized = previousInitialized;
  seededFromVoltage = previousVoltageSeeded;
  stateStored = previousStateStored;
  dirty = previousDirty;
  saveRequested = previousSaveRequested;
  previousCurrentValid = previousCurrentBaselineValid;
  previousCurrentMilliamp = previousBaselineMilliamp;
  previousCurrentAtMs = previousBaselineAtMs;
  lastSavedAtMs = previousSavedAtMs;
  lastSaveAttemptAtMs = previousSaveAttemptAtMs;
  return false;
}

bool socEstimatorFlush() {
  if (!initialized) return true;
  dirty = true;
  return saveNow();
}

SocStatus socEstimatorStatus(const BmsSettings &settings) {
  SocStatus status = {};
  status.initialized = initialized;
  status.voltageSeeded = seededFromVoltage;
  status.stateStored = stateStored;
  status.pendingSave = dirty;
  status.percent = initialized ? clampPercent(socPercent) : 0.0f;
  status.remainingAh = initialized
      ? settings.batteryCapacityAh * status.percent / 100.0f : 0.0f;
  status.chargedAh = chargedAh;
  status.dischargedAh = dischargedAh;
  return status;
}

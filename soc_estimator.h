#pragma once

#include <Arduino.h>

#include "battery_can.h"
#include "bms_limits.h"
#include "settings.h"

// Pack SOC status derived from signed current integration or the voltage curve.
struct SocStatus {
  bool initialized;
  bool voltageSeeded;
  bool stateStored;
  bool pendingSave;
  float percent;
  float remainingAh;
  float chargedAh;
  float dischargedAh;
};

// Load retained SOC at boot; if none exists, seed after the first valid cell scan.
void socEstimatorBegin();
// Integrate each fresh LEM frame once and periodically persist SOC/throughput.
SocStatus socEstimatorUpdate(const BmsSettings &settings,
                             const BatteryLimitStatus &limits,
                             const CurrentSensorStatus &current);
// Manually set and immediately persist SOC as a calibration/commissioning action.
bool socEstimatorSetPercent(float percent, const BmsSettings &settings);
// Flush pending SOC and Ah totals before a software-requested reboot.
bool socEstimatorFlush();
// Return a snapshot with remaining Ah calculated from the current capacity setting.
SocStatus socEstimatorStatus(const BmsSettings &settings);

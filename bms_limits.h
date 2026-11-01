#pragma once

#include <Arduino.h>

#include "batterycomms.h"
#include "settings.h"

// Pack-wide limits derived from the most recent complete cell scan.
struct BatteryLimitStatus {
  bool readingsValid;
  bool overVoltage;
  bool underVoltage;
  bool dischargeBlocked;
  bool chargePausedByHysteresis;
  float lowestCellV;
  float highestCellV;
  float chargeCurrentRequestA;
  float packChargeTargetV;
};

// Update cell extrema, validity, discharge/charge hysteresis and requested
// charge current. The request is for display/future control, not CAN output.
BatteryLimitStatus batteryLimitsUpdate(const BatterySnapshot &snapshot,
                                       const BmsSettings &settings);

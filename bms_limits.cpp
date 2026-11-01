/*
  Pack-level cell checks and charge-request calculations.

  Discharge and charge limits use separate hysteresis latches so a state does
  not chatter when a cell sits close to a threshold. This module computes a
  requested current value; it does not transmit a charger command.
*/
#include "bms_limits.h"

namespace {
bool chargePausedLatch = false;
bool dischargeBlockedLatch = false;
}

BatteryLimitStatus batteryLimitsUpdate(const BatterySnapshot &snapshot,
                                       const BmsSettings &settings) {
  BatteryLimitStatus status = {};
  status.lowestCellV = 1000.0f;
  status.highestCellV = -1000.0f;
  status.packChargeTargetV = settings.chargeVoltageV * settings.seriesCells;

  // A partial stack or a missing cell is never considered safe input for the
  // state machine, even if the other modules happened to read successfully.
  status.readingsValid = snapshot.allCellReadsOk &&
      snapshot.moduleCount == settings.stackModules &&
      snapshot.validCellCount ==
          static_cast<uint16_t>(settings.stackModules) * settings.cellsPerModule;

  if (status.readingsValid) {
    for (uint8_t moduleIndex = 0; moduleIndex < snapshot.moduleCount;
         ++moduleIndex) {
      const ModuleReading &module = snapshot.modules[moduleIndex];
      if (module.validCellCount != settings.cellsPerModule) {
        status.readingsValid = false;
        break;
      }
      for (uint8_t cell = 0; cell < settings.cellsPerModule; ++cell) {
        if (module.cellRaw[cell] == 0x8000U) {
          status.readingsValid = false;
          break;
        }
        const float cellV = module.cellMv[cell] / 1000.0f;
        if (cellV < status.lowestCellV) status.lowestCellV = cellV;
        if (cellV > status.highestCellV) status.highestCellV = cellV;
      }
      if (!status.readingsValid) break;
    }
  }

  if (!status.readingsValid) {
    status.lowestCellV = 0.0f;
    status.highestCellV = 0.0f;
    status.dischargeBlocked = true;
    dischargeBlockedLatch = true;
    status.chargeCurrentRequestA = 0.0f;
    status.chargePausedByHysteresis = chargePausedLatch;
    return status;
  }

  status.overVoltage = status.highestCellV >= settings.cellOverVoltageV;
  status.underVoltage = status.lowestCellV <= settings.cellUnderVoltageV;
  // Latch discharge inhibition until the cell recovers past the configured
  // cut-off plus hysteresis; this prevents rapid contactor/limit toggling.
  if (status.underVoltage ||
      status.lowestCellV <= settings.dischargeCutoffV) {
    dischargeBlockedLatch = true;
  } else if (dischargeBlockedLatch &&
             status.lowestCellV >= settings.dischargeCutoffV +
                                       settings.dischargeHysteresisV) {
    dischargeBlockedLatch = false;
  }
  status.dischargeBlocked = dischargeBlockedLatch;

  // Charge pause latches at target and releases only below target-hysteresis.
  if (status.highestCellV >= settings.chargeVoltageV) {
    chargePausedLatch = true;
  } else if (chargePausedLatch &&
             status.highestCellV <=
                 settings.chargeVoltageV - settings.chargeHysteresisV) {
    chargePausedLatch = false;
  }
  status.chargePausedByHysteresis = chargePausedLatch;

  if (status.overVoltage || chargePausedLatch ||
      settings.maxChargeCurrentA <= 0.0f) {
    status.chargeCurrentRequestA = 0.0f;
  } else {
    const float taperStart =
        settings.chargeVoltageV - settings.chargeHysteresisV;
    if (status.highestCellV <= taperStart) {
      status.chargeCurrentRequestA = settings.maxChargeCurrentA;
    } else {
      float fraction = (settings.chargeVoltageV - status.highestCellV) /
                       settings.chargeHysteresisV;
      if (fraction < 0.0f) fraction = 0.0f;
      if (fraction > 1.0f) fraction = 1.0f;
      status.chargeCurrentRequestA = settings.endChargeCurrentA +
          fraction * (settings.maxChargeCurrentA - settings.endChargeCurrentA);
    }
  }
  return status;
}

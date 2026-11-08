#pragma once

#include <Arduino.h>

#include "settings.h"

// Recent Outlander OBC measurements. Values remain available after a timeout,
// while the accompanying freshness flags show whether they are current.
struct OutlanderChargerStatus {
  bool canBusReady;
  bool telemetryFresh;
  bool charger389Fresh;
  bool charger38AFresh;
  bool mainsPresent;
  bool charging;
  bool chargerError;
  bool dcdcRequestActive;
  bool pilotPresent;
  bool waitForMains;
  bool readyForCharge;
  uint16_t hvBatteryVoltageV;
  uint16_t acMainsVoltageV;
  float dcChargeCurrentA;
  float dcChargeCurrent2A;
  float acMainsCurrentA;
  int16_t chargerTemp1C;
  int16_t chargerTemp2C;
  int16_t chargerTemp3C;
  int16_t chargerTemp4C;
  uint16_t dcBusVoltageV;
  uint8_t evsePilotPercent;
  uint8_t status389;
  uint8_t status38A;
  uint32_t lastFrameAgeMs;
  uint32_t frameCount;
  uint32_t invalidFrameCount;
  uint32_t txFrameCount;
  uint32_t txBusyCount;
  uint8_t lastTxMode;  // 0=none, 1=Ready wake, 2=charge setpoint, 3=zero-current stop
};

void outlanderChargerBegin();
// Called by the shared battery CAN reader for standard IDs 0x389 and 0x38A.
void outlanderChargerAcceptCanFrame(uint16_t id, uint8_t length,
                                    const uint8_t *data);
OutlanderChargerStatus outlanderChargerGetStatus(bool canBusReady);
// Charge may start only after a fresh 0x389 reports mains present without error.
bool outlanderChargerAvailableForCharge(const BmsSettings &settings,
                                        bool canBusReady);
// Queue the SimpBMS-style 0x286 command; current is in A and capped to 12 A.
bool outlanderChargerSendSetpoint(const BmsSettings &settings,
                                  float requestedCurrentA,
                                  bool canBusReady);
// Periodically send a 0x285 wake in Ready/charge precharge, or the active
// 0x286 voltage/current request while Charge is active.
void outlanderChargerTask(const BmsSettings &settings, bool readyState,
                          bool chargePrechargeActive, bool chargeState,
                          bool chargeRequested, float requestedCurrentA,
                          bool canBusReady);

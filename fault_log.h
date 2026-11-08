#pragma once

#include <Arduino.h>

#include "bms_state_machine.h"

enum class BmsFaultCode : uint8_t {
  None = 0,
  StackCommunication = 1,
  StackRecoveryFailed,
  CellDataStale,
  CellDataInvalid,
  CellOverVoltage,
  CellUnderVoltage,
  CurrentSensor,
  PrechargeOutput,
  PrechargeTimeout,
  ContactorOutput,
  Charger,
  IsolationMonitor,
  Other
};

struct FaultLogEntry {
  uint32_t sequence;
  uint32_t bootCount;
  uint32_t uptimeSeconds;
  uint32_t driveCycle;
  BmsFaultCode code;
  BmsOperatingState state;
  bool occurredDuringDriveCycle;
  bool belongsToCurrentDriveCycle;
};

// Initialize the persistent fault history. The records use a reserved EEPROM
// range checked against settings and SOC storage before any writes are made.
void faultLogBegin();
bool faultLogAvailable();
uint8_t faultLogCount();
uint32_t faultLogCurrentDriveCycle();
bool faultLogHasPersistentFault();
BmsFaultCode faultLogPersistentFaultCode();
bool faultLogSetPersistentFault(BmsFaultCode code);
bool faultLogClearPersistentFault();
bool faultLogReadNewest(uint8_t newestIndex, FaultLogEntry &entry);
const char *faultLogCodeName(BmsFaultCode code);

// Drive cycles begin when Drive precharge starts and become previous when the
// state machine leaves that Drive cycle.
void faultLogStartDriveCycle();
void faultLogSetDriveCycleActive(bool active);
void faultLogRecord(BmsFaultCode code, BmsOperatingState state);

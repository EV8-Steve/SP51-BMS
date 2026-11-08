#pragma once

#include <Arduino.h>

enum class CurrentSensorType : uint8_t {
  Off = 0,
  LemCab500Can = 1,
  SmePlaceholder = 2
};

enum class ContactorControlType : uint8_t {
  NanoLowSideFet = 0,
  SmePlaceholder = 1
};

enum class IsolationMonitorType : uint8_t {
  Bender = 0,
  SmePlaceholder = 1
};

enum class ChargerType : uint8_t {
  None = 0,
  ExternalRelay = 1,
  BrusaNlg5 = 2,
  Volt = 3,
  Eltek = 4,
  Elcon = 5,
  VictronSma = 6,
  Outlander = 7
};

enum class SocMode : uint8_t {
  CoulombCount = 0,  // Signed pack-current integration; positive means charging.
  VoltageOnly = 1   // SimpBMS-style linear estimate from the lowest cell voltage.
};

enum class BmsOutputRole : uint8_t {
  MainPositive = 0,
  MainNegative = 1,
  Precharge = 2,
  ChargerRelay = 3
};

struct BmsSettings {
  // Stack devices at addresses 1..N; base address 0 is not a cell module.
  uint8_t stackModules;
  uint8_t cellsPerModule;     // This SP51 module uses 12 active cell channels.
  uint16_t samplePeriodMs;    // Target start-to-start monitor interval.
  float ntcPullupOhms;        // Provisional until the BMW resistor network is confirmed.
  float ntcR25Ohms;           // Assumed NTC resistance at 25 C.
  float ntcBetaKelvin;        // Assumed beta parameter used by the Beta equation.
  float ntcRatioMin;          // Ignore divider ratios below this NTC-estimate range.
  float ntcRatioMax;          // Ignore divider ratios above this NTC-estimate range.

  // Per-cell limits and charge taper settings, based on SimpBMS defaults.
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;    // Used to calculate/display a future charge request.
  float endChargeCurrentA;   // Stored setting; not yet used for charger control.
  float batteryCapacityAh;   // Total pack Ah for SOC integration; set to pack capacity.

  // LEM is a read-only CAN input. SME is a saved placeholder only; its data
  // decoder is not implemented in this build.
  CurrentSensorType currentSensor;
  bool invertCurrent;         // Reverse decoded current sign when needed.
  uint32_t currentCanBitrate; // LEM input bitrate; only 250 or 500 kbit/s accepted.
  uint16_t currentCanId;      // Standard (11-bit) LEM frame identifier.
  uint16_t currentSensorTimeoutMs; // Age after which a current frame is stale.

  // Outlander OBC and LEM current frames share the Nano R4's one CAN bus.
  ChargerType chargerType;
  uint16_t seriesCells;       // Total pack cell count used for charger voltage requests.
  uint32_t chargerCanBitrate; // Must match currentCanBitrate on the shared bus.
  uint16_t chargerMessagePeriodMs; // Disarmed/Ready wake and Charge request cadence.
  bool chargerTxEnabled;      // Outlander wake/command transmission enable.
  // Nano low-side FET gates are active HIGH. Mappings default unassigned;
  // main-negative/precharge can instead select the unimplemented SME placeholder.
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;

  // SimpBMS-style current-based precharge, with a timeout fail-safe.
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;    // Run input active level; defaults LOW for inverted isolator.
  bool chargeInputActiveHigh; // 240 V input active level; defaults LOW for inverted isolator.
  uint8_t runInputPin;        // Defaults to D2; D0/D1 are reserved for BQ UART.
  uint8_t chargeInputPin;     // Defaults to D3; must not overlap any mapped output.

  // SOC uses pack Ah capacity (including any parallel strings), so a separate
  // parallel-string multiplier is not needed for this 108S pack.
  SocMode socMode;
  uint16_t socLowCellMv;      // Low anchor for voltage-only SOC and first-start seed.
  uint8_t socLowPercent;
  uint16_t socHighCellMv;     // High anchor for voltage-only SOC and first-start seed.
  uint8_t socHighPercent;

  // true: charger is downstream of contactors; precharge and close them.
  // false: charger is wired directly to the pack; keep contactors open.
  bool chargerBehindContactors;

  // SME options are persistent placeholders only; no SME hardware protocol is
  // implemented. Selecting SME blocks use of the corresponding contactor.
  ContactorControlType mainNegativeControl;
  ContactorControlType prechargeControl;

  // Isolation-monitor support is a saved placeholder only. No input is read
  // until a Bender or SME signal implementation is added and validated.
  bool isolationMonitorEnabled;
  IsolationMonitorType isolationMonitorType;
  uint8_t isolationMonitorPin;
  bool isolationFaultActiveHigh;

  // Temperature/GPIO ADCs change slowly; poll them on this slower cadence
  // while cell voltages continue at samplePeriodMs.
  uint16_t temperatureSamplePeriodMs;

  // Passive cell balancing uses the BQ79616 internal CBFETs. This remains
  // disabled in saved defaults until the board's resistor power rating and
  // exact bleed path have been verified.
  bool cellBalancingEnabled;
  uint16_t balancingTargetDeltaMv;  // Pack max-min delta that initiates balancing.
  uint16_t balancingHysteresisMv;   // Extra delta required to start/restart.
  uint16_t balancingMinCellMv;      // Inhibit balancing below this pack minimum.
  uint16_t balancingMaxOnSeconds;  // BQ timer choice: 10, 30, or 60 seconds.
  uint8_t balancingConservativeLevel; // 0=standard, 1=moderate, 2=high.
  uint32_t balancingCoolOffMs;     // Cooling time after each timed pulse.
};

// CRC-protected retained SOC and cumulative charge/discharge throughput.
// SOC is stored as hundredths of a percent; throughput counters are mAh.
struct PersistentSocState {
  uint16_t percentBasisPoints;
  uint32_t chargedMilliAh;
  uint32_t dischargedMilliAh;
};

// Load validated versioned EEPROM settings. Accepted setters write to the
// alternate CRC-protected record and verify it before reporting success.
void settingsBegin();
const BmsSettings &settingsGet();
bool settingsAreStored();
// All setters validate limits and persist accepted values immediately.
bool settingsSetSamplePeriod(uint16_t periodMs);
bool settingsSetTemperatureSamplePeriod(uint16_t periodMs);
bool settingsSetCellBalancingEnabled(bool enabled);
bool settingsSetBalancingTargetDeltaMv(uint16_t millivolts);
bool settingsSetBalancingHysteresisMv(uint16_t millivolts);
bool settingsSetBalancingMinCellMv(uint16_t millivolts);
bool settingsSetBalancingMaxOnSeconds(uint16_t seconds);
bool settingsSetBalancingConservativeLevel(uint8_t level);
bool settingsSetBalancingCoolOffMs(uint32_t periodMs);
bool settingsSetVoltageLimit(const char *name, float value);
bool settingsSetChargeCurrent(const char *name, float value);
bool settingsSetBatteryCapacity(float value);
bool settingsSetSocMode(SocMode mode);
bool settingsSetSocVoltageAnchor(const char *name, float value);
bool settingsSetCurrentSensorType(CurrentSensorType type);
bool settingsSetContactorControlType(BmsOutputRole role,
                                     ContactorControlType type);
ContactorControlType settingsGetContactorControlType(BmsOutputRole role);
bool settingsSetIsolationMonitorEnabled(bool enabled);
bool settingsSetIsolationMonitorType(IsolationMonitorType type);
bool settingsSetIsolationMonitorPolarity(bool activeHigh);
bool settingsSetCurrentCanBitrate(uint32_t bitrate);
bool settingsSetCurrentCanId(uint16_t id);
bool settingsSetCurrentInvert(bool invert);
bool settingsSetChargerType(ChargerType type);
bool settingsSetChargerCanTx(bool enabled);
bool settingsSetChargerBehindContactors(bool behindContactors);
bool settingsSetSeriesCells(uint16_t count);
bool settingsSetPrecharge(const char *name, float value);
bool settingsSetInputPolarity(const char *name, bool activeHigh);
bool settingsSetInputPin(const char *name, uint8_t pin);
bool settingsSetOutputPin(BmsOutputRole role, uint8_t pin);
bool settingsSetChargerRelayPin(uint8_t pin);
uint8_t settingsGetOutputPin(BmsOutputRole role);
bool settingsLoadSocState(PersistentSocState &state);
bool settingsSaveSocState(const PersistentSocState &state);
// Return true only when a requested EEPROM range is inside storage and does
// not overlap either settings slot or the rotating SOC journals.
bool settingsCanUseEepromRange(int address, size_t length);

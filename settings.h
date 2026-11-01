#pragma once

#include <Arduino.h>

enum class CurrentSensorType : uint8_t {
  Off = 0,
  LemCab500Can = 1
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

  // LEM CAB500-C/SP7 is read-only CAN input; no sensor configuration frames
  // are sent by this build.
  CurrentSensorType currentSensor;
  bool invertCurrent;         // Reverse decoded current sign when needed.
  uint32_t currentCanBitrate; // LEM input bitrate; only 250 or 500 kbit/s accepted.
  uint16_t currentCanId;      // Standard (11-bit) LEM frame identifier.
  uint16_t currentSensorTimeoutMs; // Age after which a current frame is stale.

  // Charger profile is selected in the menu. CAN command transmission stays
  // disabled until pack series count and the actual charger wiring are verified.
  ChargerType chargerType;    // ExternalRelay is the only active charge output path.
  uint16_t seriesCells;       // Pack cell count for displayed voltage calculations.
  uint32_t chargerCanBitrate; // Reserved for future charger CAN implementation.
  uint16_t chargerMessagePeriodMs; // Reserved for future charger CAN implementation.
  bool chargerTxEnabled;      // Kept false; this build does not transmit charger CAN.
  // Low-side FET gates are active HIGH. All four mappings default unassigned.
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
bool settingsSetVoltageLimit(const char *name, float value);
bool settingsSetChargeCurrent(const char *name, float value);
bool settingsSetBatteryCapacity(float value);
bool settingsSetSocMode(SocMode mode);
bool settingsSetSocVoltageAnchor(const char *name, float value);
bool settingsSetCurrentSensorType(CurrentSensorType type);
bool settingsSetCurrentCanBitrate(uint32_t bitrate);
bool settingsSetCurrentCanId(uint16_t id);
bool settingsSetCurrentInvert(bool invert);
bool settingsSetChargerType(ChargerType type);
bool settingsSetSeriesCells(uint16_t count);
bool settingsSetPrecharge(const char *name, float value);
bool settingsSetInputPolarity(const char *name, bool activeHigh);
bool settingsSetInputPin(const char *name, uint8_t pin);
bool settingsSetOutputPin(BmsOutputRole role, uint8_t pin);
bool settingsSetChargerRelayPin(uint8_t pin);
uint8_t settingsGetOutputPin(BmsOutputRole role);
bool settingsLoadSocState(PersistentSocState &state);
bool settingsSaveSocState(const PersistentSocState &state);

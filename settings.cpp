/*
  Persistent BMS configuration.

  The current settings payload is written to one of two EEPROM slots with a
  generation counter and CRC32. A change is committed only after a readback
  verifies; prior version-1 through version-9 records are migrated field by
  field so pin maps and user limits survive firmware upgrades. Runtime SOC and
  Ah totals use a separate CRC-protected rotating journal.
*/
#include "settings.h"

#include <EEPROM.h>
#include <stddef.h>
#include <string.h>

namespace {
constexpr uint16_t MIN_SAMPLE_PERIOD_MS = 500;
constexpr uint16_t MAX_SAMPLE_PERIOD_MS = 5000;
constexpr uint16_t MIN_TEMPERATURE_SAMPLE_PERIOD_MS = 500;
constexpr uint16_t MAX_TEMPERATURE_SAMPLE_PERIOD_MS = 10000;
constexpr uint16_t DEFAULT_TEMPERATURE_SAMPLE_PERIOD_MS = 2000;
constexpr uint32_t SETTINGS_MAGIC = 0x53503531UL;  // "SP51"
constexpr uint16_t SETTINGS_VERSION = 11;
constexpr uint16_t LEGACY_SETTINGS_VERSION = 1;
constexpr uint16_t VERSION2_SETTINGS_VERSION = 2;
constexpr uint16_t VERSION3_SETTINGS_VERSION = 3;
constexpr uint16_t VERSION4_SETTINGS_VERSION = 4;
constexpr uint16_t VERSION5_SETTINGS_VERSION = 5;
constexpr uint16_t VERSION6_SETTINGS_VERSION = 6;
constexpr uint16_t VERSION7_SETTINGS_VERSION = 7;
constexpr uint16_t VERSION8_SETTINGS_VERSION = 8;
constexpr uint16_t VERSION9_SETTINGS_VERSION = 9;
constexpr uint16_t VERSION10_SETTINGS_VERSION = 10;
constexpr int EEPROM_SLOT_SPACING = 4096;
constexpr uint32_t SOC_MAGIC = 0x534F4331UL;  // "SOC1"
constexpr uint16_t SOC_RECORD_VERSION = 1;
constexpr int SOC_REGION0_ADDRESS = EEPROM_SLOT_SPACING / 2;
constexpr int SOC_REGION1_ADDRESS = EEPROM_SLOT_SPACING + EEPROM_SLOT_SPACING / 2;
constexpr size_t SOC_REGION_SIZE = EEPROM_SLOT_SPACING / 2;

struct SettingsRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettings payload;
  uint32_t crc;
};

// Version 1 record layout, retained so settings saved by the prior build can
// be migrated without discarding the user's configured limits and pin map.
struct BmsSettingsV1 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
};

struct SettingsRecordV1 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV1 payload;
  uint32_t crc;
};

// Version 2 added precharge settings and input polarity. Version 3 adds
// configurable Run and 240 V input pins.
struct BmsSettingsV2 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;
  bool chargeInputActiveHigh;
};

struct SettingsRecordV2 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV2 payload;
  uint32_t crc;
};

// Version 3 added persistent Run/240 V input pin assignments. Version 4
// changes the installed isolated inputs to active-LOW and migrates that level.
struct BmsSettingsV3 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;
  bool chargeInputActiveHigh;
  uint8_t runInputPin;
  uint8_t chargeInputPin;
};

struct SettingsRecordV3 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV3 payload;
  uint32_t crc;
};

// Version 4 changed only the installed input-polarity defaults. Its payload
// layout is identical to version 3; keep a distinct record version for clarity.
using BmsSettingsV4 = BmsSettingsV3;

struct SettingsRecordV4 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV4 payload;
  uint32_t crc;
};

// Version 5 added the SOC mode and voltage anchors to the version-4 payload.
struct BmsSettingsV5 {
  BmsSettingsV4 previous;
  SocMode socMode;
  uint16_t socLowCellMv;
  uint8_t socLowPercent;
  uint16_t socHighCellMv;
  uint8_t socHighPercent;
};

struct SettingsRecordV5 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV5 payload;
  uint32_t crc;
};

// Versions 6 and 7 have the same payload shape. Retain it for migration.
struct BmsSettingsV7 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;
  bool chargeInputActiveHigh;
  uint8_t runInputPin;
  uint8_t chargeInputPin;
  SocMode socMode;
  uint16_t socLowCellMv;
  uint8_t socLowPercent;
  uint16_t socHighCellMv;
  uint8_t socHighPercent;
  bool chargerBehindContactors;
};

using BmsSettingsV6 = BmsSettingsV7;

struct SettingsRecordV7 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV7 payload;
  uint32_t crc;
};

// Version 8 added the main-negative and precharge control selections.
struct BmsSettingsV8 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;
  bool chargeInputActiveHigh;
  uint8_t runInputPin;
  uint8_t chargeInputPin;
  SocMode socMode;
  uint16_t socLowCellMv;
  uint8_t socLowPercent;
  uint16_t socHighCellMv;
  uint8_t socHighPercent;
  bool chargerBehindContactors;
  ContactorControlType mainNegativeControl;
  ContactorControlType prechargeControl;
};

struct SettingsRecordV8 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV8 payload;
  uint32_t crc;
};

// Version 9 is the prior settings payload before the temperature/GPIO period.
struct BmsSettingsV9 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;
  bool chargeInputActiveHigh;
  uint8_t runInputPin;
  uint8_t chargeInputPin;
  SocMode socMode;
  uint16_t socLowCellMv;
  uint8_t socLowPercent;
  uint16_t socHighCellMv;
  uint8_t socHighPercent;
  bool chargerBehindContactors;
  ContactorControlType mainNegativeControl;
  ContactorControlType prechargeControl;
  bool isolationMonitorEnabled;
  IsolationMonitorType isolationMonitorType;
  uint8_t isolationMonitorPin;
  bool isolationFaultActiveHigh;
};

struct SettingsRecordV9 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV9 payload;
  uint32_t crc;
};

// Version 10 is the prior current settings payload. Keep its exact flat
// layout so EEPROM records can migrate without losing user settings.
struct BmsSettingsV10 {
  uint8_t stackModules;
  uint8_t cellsPerModule;
  uint16_t samplePeriodMs;
  float ntcPullupOhms;
  float ntcR25Ohms;
  float ntcBetaKelvin;
  float ntcRatioMin;
  float ntcRatioMax;
  float cellOverVoltageV;
  float cellUnderVoltageV;
  float dischargeCutoffV;
  float dischargeHysteresisV;
  float chargeVoltageV;
  float chargeHysteresisV;
  float maxChargeCurrentA;
  float endChargeCurrentA;
  float batteryCapacityAh;
  CurrentSensorType currentSensor;
  bool invertCurrent;
  uint32_t currentCanBitrate;
  uint16_t currentCanId;
  uint16_t currentSensorTimeoutMs;
  ChargerType chargerType;
  uint16_t seriesCells;
  uint32_t chargerCanBitrate;
  uint16_t chargerMessagePeriodMs;
  bool chargerTxEnabled;
  uint8_t mainPositivePin;
  uint8_t mainNegativePin;
  uint8_t prechargePin;
  uint8_t chargerRelayPin;
  uint16_t prechargeMinimumMs;
  uint16_t prechargeTimeoutMs;
  float prechargeFinishCurrentA;
  bool runInputActiveHigh;
  bool chargeInputActiveHigh;
  uint8_t runInputPin;
  uint8_t chargeInputPin;
  SocMode socMode;
  uint16_t socLowCellMv;
  uint8_t socLowPercent;
  uint16_t socHighCellMv;
  uint8_t socHighPercent;
  bool chargerBehindContactors;
  ContactorControlType mainNegativeControl;
  ContactorControlType prechargeControl;
  bool isolationMonitorEnabled;
  IsolationMonitorType isolationMonitorType;
  uint8_t isolationMonitorPin;
  bool isolationFaultActiveHigh;
  uint16_t temperatureSamplePeriodMs;
};

struct SettingsRecordV10 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV10 payload;
  uint32_t crc;
};

struct SettingsRecordV6 {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  BmsSettingsV6 payload;
  uint32_t crc;
};

static_assert(offsetof(BmsSettings, stackModules) ==
                  offsetof(BmsSettingsV9, stackModules) &&
              offsetof(BmsSettings, isolationFaultActiveHigh) ==
                  offsetof(BmsSettingsV9, isolationFaultActiveHigh) &&
              sizeof(BmsSettingsV9) <= sizeof(BmsSettings),
              "Version 9 settings prefix layout changed; update its migration.");
static_assert(offsetof(BmsSettings, stackModules) ==
                  offsetof(BmsSettingsV10, stackModules) &&
              offsetof(BmsSettings, temperatureSamplePeriodMs) ==
                  offsetof(BmsSettingsV10, temperatureSamplePeriodMs) &&
              sizeof(BmsSettingsV10) <= sizeof(BmsSettings),
              "Version 10 settings prefix changed; update its migration.");

struct SocRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t generation;
  PersistentSocState payload;
  uint32_t crc;
};

BmsSettings currentSettings = {
    9,       // Nine stack devices plus base address 0: ten total.
    12,      // Twelve active cell channels per module.
    500,     // Requested nominal monitor period.
    22000.0f,
    10000.0f,
    3380.0f,
    0.38f,
    0.50f,
    4.20f,   // Cell over-voltage limit.
    3.00f,   // Cell under-voltage limit.
    3.20f,   // Discharge cut-off target.
    0.20f,   // Discharge hysteresis.
    4.10f,   // Charge target per cell.
    0.20f,   // Charge hysteresis and taper band.
    30.0f,   // Maximum configured charge current.
    5.0f,    // End-of-charge current limit.
    50.0f,   // Placeholder total pack capacity; set this to the actual pack Ah.
    CurrentSensorType::LemCab500Can,
    false,         // Current sign follows the sensor's factory orientation.
    500000UL,      // LEM CAB 500-C/SP7 default CAN rate.
    0x3C2,          // LEM CAB 500-C default current frame ID.
    100,            // Sensor data older than this is stale.
    ChargerType::Outlander,
    108,            // User confirmed all nine 12-cell devices are in series.
    500000UL,
    100,            // SimpBMS Outlander profile cadence.
    true,           // Enable Outlander CAN wake and charge commands by default.
    0xFF,           // Main positive contactor: unassigned.
    0xFF,           // Main negative contactor: unassigned.
    0xFF,           // Precharge contactor: unassigned.
    0xFF,           // Charger relay: unassigned.
    5000,           // SimpBMS precharge minimum time.
    15000,          // Fail-safe timeout if current never falls below threshold.
    1.0f,            // SimpBMS Precurrent default of 1000 mA.
    false,           // The installed isolated Run input is electrically inverted.
    false,           // The installed isolated 240 V input is electrically inverted.
    2,               // Run input default pin.
    3,               // Charge/240 V input default pin.
    SocMode::CoulombCount,
    3100,            // SimpBMS voltage-SOC lower anchor: 3.10 V/cell.
    10,
    4100,            // SimpBMS voltage-SOC upper anchor: 4.10 V/cell.
    90,
    true,            // Preserve current behavior: charge through precharge/contactors.
    ContactorControlType::NanoLowSideFet,
    ContactorControlType::NanoLowSideFet,
    false,           // Isolation monitoring remains disabled until implemented.
    IsolationMonitorType::Bender,
    0xFF,            // Bender fault input is not assigned yet.
    true,            // Provisional fault polarity; not used until implemented.
    DEFAULT_TEMPERATURE_SAMPLE_PERIOD_MS,
    false,   // Balancing is locked out until resistor wattage is confirmed.
    20,      // Start when pack max-min exceeds target plus hysteresis.
    10,      // Extra spread required before starting or restarting a pulse.
    3800,    // Do not balance unless every cell is at least 3.8 V.
    10,      // BQ hardware timer provides a 10-second minimum pulse.
    2,       // High-conservatism preset caps effective on-time at 10 seconds.
    60000UL  // Minimum cooling period between balancing pulses.
};

BmsSettings savedSettings = {};
bool storageAvailable = false;
bool savedRecordAvailable = false;
int8_t activeRecordSlot = -1;
uint32_t settingsGeneration = 0;

uint32_t calculateCrc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 1UL) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
  }
  return ~crc;
}

bool sameSettings(const BmsSettings &a, const BmsSettings &b) {
  return a.stackModules == b.stackModules &&
         a.cellsPerModule == b.cellsPerModule &&
         a.samplePeriodMs == b.samplePeriodMs &&
         a.ntcPullupOhms == b.ntcPullupOhms &&
         a.ntcR25Ohms == b.ntcR25Ohms &&
         a.ntcBetaKelvin == b.ntcBetaKelvin &&
         a.ntcRatioMin == b.ntcRatioMin && a.ntcRatioMax == b.ntcRatioMax &&
         a.cellOverVoltageV == b.cellOverVoltageV &&
         a.cellUnderVoltageV == b.cellUnderVoltageV &&
         a.dischargeCutoffV == b.dischargeCutoffV &&
         a.dischargeHysteresisV == b.dischargeHysteresisV &&
         a.chargeVoltageV == b.chargeVoltageV &&
         a.chargeHysteresisV == b.chargeHysteresisV &&
         a.maxChargeCurrentA == b.maxChargeCurrentA &&
         a.endChargeCurrentA == b.endChargeCurrentA &&
         a.batteryCapacityAh == b.batteryCapacityAh &&
         a.currentSensor == b.currentSensor &&
         a.invertCurrent == b.invertCurrent &&
         a.currentCanBitrate == b.currentCanBitrate &&
         a.currentCanId == b.currentCanId &&
         a.currentSensorTimeoutMs == b.currentSensorTimeoutMs &&
         a.chargerType == b.chargerType && a.seriesCells == b.seriesCells &&
         a.chargerCanBitrate == b.chargerCanBitrate &&
         a.chargerMessagePeriodMs == b.chargerMessagePeriodMs &&
         a.chargerTxEnabled == b.chargerTxEnabled &&
         a.mainPositivePin == b.mainPositivePin &&
         a.mainNegativePin == b.mainNegativePin &&
         a.prechargePin == b.prechargePin &&
         a.chargerRelayPin == b.chargerRelayPin &&
         a.prechargeMinimumMs == b.prechargeMinimumMs &&
         a.prechargeTimeoutMs == b.prechargeTimeoutMs &&
         a.prechargeFinishCurrentA == b.prechargeFinishCurrentA &&
         a.runInputActiveHigh == b.runInputActiveHigh &&
         a.chargeInputActiveHigh == b.chargeInputActiveHigh &&
         a.runInputPin == b.runInputPin &&
         a.chargeInputPin == b.chargeInputPin &&
         a.socMode == b.socMode &&
         a.socLowCellMv == b.socLowCellMv &&
         a.socLowPercent == b.socLowPercent &&
         a.socHighCellMv == b.socHighCellMv &&
         a.socHighPercent == b.socHighPercent &&
         a.chargerBehindContactors == b.chargerBehindContactors &&
         a.mainNegativeControl == b.mainNegativeControl &&
         a.prechargeControl == b.prechargeControl &&
         a.isolationMonitorEnabled == b.isolationMonitorEnabled &&
         a.isolationMonitorType == b.isolationMonitorType &&
         a.isolationMonitorPin == b.isolationMonitorPin &&
         a.isolationFaultActiveHigh == b.isolationFaultActiveHigh &&
         a.temperatureSamplePeriodMs == b.temperatureSamplePeriodMs &&
         a.cellBalancingEnabled == b.cellBalancingEnabled &&
         a.balancingTargetDeltaMv == b.balancingTargetDeltaMv &&
         a.balancingHysteresisMv == b.balancingHysteresisMv &&
         a.balancingMinCellMv == b.balancingMinCellMv &&
         a.balancingMaxOnSeconds == b.balancingMaxOnSeconds &&
         a.balancingConservativeLevel == b.balancingConservativeLevel &&
         a.balancingCoolOffMs == b.balancingCoolOffMs;
}

bool allowedPin(uint8_t pin) {
  return pin == 0xFFU || pin == 6U || pin == 13U || pin == 11U || pin == 10U;
}

bool allowedInputPin(uint8_t pin) {
  return pin >= 2U && pin <= 13U;
}

bool validSettings(const BmsSettings &s) {
  // Central validation is shared by EEPROM load, migration, and every setter.
  // Pin uniqueness applies across four outputs and three configurable inputs.
  if (s.stackModules < 1U || s.stackModules > 12U ||
      s.cellsPerModule != 12U || s.samplePeriodMs < MIN_SAMPLE_PERIOD_MS ||
      s.samplePeriodMs > MAX_SAMPLE_PERIOD_MS ||
      s.temperatureSamplePeriodMs < MIN_TEMPERATURE_SAMPLE_PERIOD_MS ||
      s.temperatureSamplePeriodMs > MAX_TEMPERATURE_SAMPLE_PERIOD_MS ||
      s.balancingTargetDeltaMv < 5U || s.balancingTargetDeltaMv > 300U ||
      s.balancingHysteresisMv < 1U || s.balancingHysteresisMv > 100U ||
      s.balancingMinCellMv < 3000U || s.balancingMinCellMv > 4200U ||
      (s.balancingMaxOnSeconds != 10U &&
       s.balancingMaxOnSeconds != 30U &&
       s.balancingMaxOnSeconds != 60U) ||
      s.balancingConservativeLevel > 2U ||
      s.balancingCoolOffMs < 60000UL ||
      s.balancingCoolOffMs > 3600000UL) return false;
  if (!(s.ntcPullupOhms > 0.0f && s.ntcR25Ohms > 0.0f &&
        s.ntcBetaKelvin > 0.0f && s.ntcRatioMin >= 0.0f &&
        s.ntcRatioMin < s.ntcRatioMax && s.ntcRatioMax <= 1.0f)) return false;
  if (!(s.cellOverVoltageV >= 2.5f && s.cellOverVoltageV <= 4.5f &&
        s.cellUnderVoltageV >= 2.0f && s.cellUnderVoltageV <= 4.0f &&
        s.dischargeCutoffV >= 2.0f && s.dischargeCutoffV <= 4.3f &&
        s.dischargeHysteresisV >= 0.01f && s.dischargeHysteresisV <= 1.0f &&
        s.chargeVoltageV >= 2.5f && s.chargeVoltageV <= 4.5f &&
        s.chargeHysteresisV >= 0.01f && s.chargeHysteresisV <= 1.0f &&
        s.cellUnderVoltageV < s.dischargeCutoffV &&
        s.dischargeCutoffV < s.chargeVoltageV &&
        s.chargeVoltageV < s.cellOverVoltageV &&
        s.dischargeHysteresisV < s.cellOverVoltageV - s.dischargeCutoffV &&
        s.chargeHysteresisV < s.chargeVoltageV - s.cellUnderVoltageV)) return false;
  if (!(s.maxChargeCurrentA >= 0.0f && s.maxChargeCurrentA <= 500.0f &&
        s.endChargeCurrentA >= 0.0f &&
        s.endChargeCurrentA <= s.maxChargeCurrentA &&
        s.batteryCapacityAh >= 0.1f && s.batteryCapacityAh <= 2000.0f)) return false;
  if (static_cast<uint8_t>(s.currentSensor) >
          static_cast<uint8_t>(CurrentSensorType::SmePlaceholder) ||
      (s.currentCanBitrate != 250000UL && s.currentCanBitrate != 500000UL) ||
      s.currentCanId == 0U || s.currentCanId > 0x7FFU ||
      s.currentSensorTimeoutMs < 10U || s.currentSensorTimeoutMs > 5000U) return false;
  if (static_cast<uint8_t>(s.mainNegativeControl) >
          static_cast<uint8_t>(ContactorControlType::SmePlaceholder) ||
      static_cast<uint8_t>(s.prechargeControl) >
          static_cast<uint8_t>(ContactorControlType::SmePlaceholder)) return false;
  if (static_cast<uint8_t>(s.isolationMonitorType) >
          static_cast<uint8_t>(IsolationMonitorType::SmePlaceholder) ||
      (s.isolationMonitorPin != 0xFFU &&
       !allowedInputPin(s.isolationMonitorPin))) return false;
  if (static_cast<uint8_t>(s.chargerType) >
          static_cast<uint8_t>(ChargerType::Outlander) ||
      s.seriesCells < 1U || s.seriesCells > 160U ||
      (s.chargerCanBitrate != 250000UL && s.chargerCanBitrate != 500000UL) ||
      s.chargerMessagePeriodMs < 20U || s.chargerMessagePeriodMs > 5000U) return false;
  if (static_cast<uint8_t>(s.socMode) >
          static_cast<uint8_t>(SocMode::VoltageOnly) ||
      s.socLowCellMv < 2000U || s.socLowCellMv > 4500U ||
      s.socHighCellMv < 2000U || s.socHighCellMv > 4500U ||
      s.socHighCellMv <= s.socLowCellMv ||
      s.socLowPercent > 100U || s.socHighPercent > 100U ||
      s.socHighPercent <= s.socLowPercent) return false;
  if (s.prechargeMinimumMs < 100U || s.prechargeMinimumMs > 30000U ||
      s.prechargeTimeoutMs <= s.prechargeMinimumMs ||
      s.prechargeTimeoutMs > 60000U ||
      !(s.prechargeFinishCurrentA >= 0.1f &&
        s.prechargeFinishCurrentA <= 50.0f)) return false;
  const uint8_t pins[7] = {s.mainPositivePin, s.mainNegativePin,
                           s.prechargePin, s.chargerRelayPin,
                           s.runInputPin, s.chargeInputPin,
                           s.isolationMonitorPin};
  for (uint8_t i = 0; i < 7U; ++i) {
    if (i < 4U) {
      if (!allowedPin(pins[i])) return false;
    } else if (i < 6U) {
      if (!allowedInputPin(pins[i])) return false;
    } else if (pins[i] != 0xFFU && !allowedInputPin(pins[i])) {
      return false;
    }
    for (uint8_t j = 0; j < i; ++j) {
      if (pins[i] != 0xFFU && pins[i] == pins[j]) return false;
    }
  }
  return true;
}

bool validRecord(const SettingsRecord &record) {
  if (record.magic != SETTINGS_MAGIC || record.version != SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettings)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecord, crc));
  return record.crc == expected && validSettings(record.payload);
}

bool readRecord(int slot, SettingsRecord &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  return validRecord(record);
}

bool readVersion9Record(int slot, SettingsRecordV9 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION9_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV9)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV9, crc));
  return record.crc == expected;
}

bool readVersion10Record(int slot, SettingsRecordV10 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION10_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV10)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV10, crc));
  return record.crc == expected;
}

bool readLegacyRecord(int slot, SettingsRecordV1 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != LEGACY_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV1)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV1, crc));
  return record.crc == expected;
}

bool readVersion2Record(int slot, SettingsRecordV2 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION2_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV2)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV2, crc));
  return record.crc == expected;
}

bool readVersion3Record(int slot, SettingsRecordV3 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION3_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV3)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV3, crc));
  return record.crc == expected;
}

bool readVersion4Record(int slot, SettingsRecordV4 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION4_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV4)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV4, crc));
  return record.crc == expected;
}

bool readVersion5Record(int slot, SettingsRecordV5 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION5_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV5)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV5, crc));
  return record.crc == expected;
}

bool readVersion6Record(int slot, SettingsRecordV6 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION6_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV6)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV6, crc));
  return record.crc == expected;
}

bool readVersion7Record(int slot, SettingsRecordV7 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION7_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV7)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV7, crc));
  return record.crc == expected;
}

bool readVersion8Record(int slot, SettingsRecordV8 &record) {
  const int address = slot * EEPROM_SLOT_SPACING;
  if (address < 0 || address + static_cast<int>(sizeof(record)) > EEPROM.length()) {
    return false;
  }
  EEPROM.get(address, record);
  if (record.magic != SETTINGS_MAGIC ||
      record.version != VERSION8_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV8)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV8, crc));
  return record.crc == expected;
}

bool validSocRecord(const SocRecord &record) {
  if (record.magic != SOC_MAGIC || record.version != SOC_RECORD_VERSION ||
      record.payloadSize != sizeof(PersistentSocState) ||
      record.payload.percentBasisPoints > 10000U) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SocRecord, crc));
  return record.crc == expected;
}

bool readSocRecord(int address, SocRecord &record) {
  if (address < 0 ||
      address + static_cast<int>(sizeof(record)) > EEPROM.length()) return false;
  EEPROM.get(address, record);
  return validSocRecord(record);
}

uint16_t socJournalSlotCount() {
  // Use the two gaps between settings slots as a wear-levelled SOC journal.
  // On Nano R4 this distributes periodic writes over many record locations.
  const size_t slotsPerRegion = SOC_REGION_SIZE / sizeof(SocRecord);
  size_t firstRegionSlots = 0;
  if (EEPROM.length() > SOC_REGION0_ADDRESS) {
    const size_t available = static_cast<size_t>(EEPROM.length() - SOC_REGION0_ADDRESS);
    firstRegionSlots = available / sizeof(SocRecord);
    if (firstRegionSlots > slotsPerRegion) firstRegionSlots = slotsPerRegion;
  }
  size_t secondRegionSlots = 0;
  if (EEPROM.length() > SOC_REGION1_ADDRESS) {
    const size_t available = static_cast<size_t>(EEPROM.length() - SOC_REGION1_ADDRESS);
    secondRegionSlots = available / sizeof(SocRecord);
    if (secondRegionSlots > slotsPerRegion) secondRegionSlots = slotsPerRegion;
  }
  return static_cast<uint16_t>(firstRegionSlots + secondRegionSlots);
}

int socJournalAddress(uint16_t slot) {
  const size_t slotsPerRegion = SOC_REGION_SIZE / sizeof(SocRecord);
  size_t firstRegionSlots = 0;
  if (EEPROM.length() > SOC_REGION0_ADDRESS) {
    const size_t available = static_cast<size_t>(EEPROM.length() - SOC_REGION0_ADDRESS);
    firstRegionSlots = available / sizeof(SocRecord);
    if (firstRegionSlots > slotsPerRegion) firstRegionSlots = slotsPerRegion;
  }
  if (slot < firstRegionSlots) {
    const int address = SOC_REGION0_ADDRESS +
        static_cast<int>(slot * sizeof(SocRecord));
    return address + static_cast<int>(sizeof(SocRecord)) <= EEPROM.length()
        ? address : -1;
  }
  slot = static_cast<uint16_t>(slot - firstRegionSlots);
  if (slot >= slotsPerRegion) return -1;
  const int address = SOC_REGION1_ADDRESS +
      static_cast<int>(slot * sizeof(SocRecord));
  return address + static_cast<int>(sizeof(SocRecord)) <= EEPROM.length()
      ? address : -1;
}

bool readNewestSocRecord(SocRecord &newest, uint16_t &newestSlot) {
  const uint16_t slotCount = socJournalSlotCount();
  bool found = false;
  for (uint16_t slot = 0; slot < slotCount; ++slot) {
    SocRecord candidate = {};
    const int address = socJournalAddress(slot);
    if (address < 0 || !readSocRecord(address, candidate)) continue;
    if (!found || static_cast<int32_t>(candidate.generation - newest.generation) > 0) {
      newest = candidate;
      newestSlot = slot;
      found = true;
    }
  }
  return found;
}

BmsSettings migrateLegacySettings(const BmsSettingsV1 &old) {
  BmsSettings migrated = currentSettings;
  migrated.stackModules = old.stackModules;
  migrated.cellsPerModule = old.cellsPerModule;
  migrated.samplePeriodMs = old.samplePeriodMs;
  migrated.ntcPullupOhms = old.ntcPullupOhms;
  migrated.ntcR25Ohms = old.ntcR25Ohms;
  migrated.ntcBetaKelvin = old.ntcBetaKelvin;
  migrated.ntcRatioMin = old.ntcRatioMin;
  migrated.ntcRatioMax = old.ntcRatioMax;
  migrated.cellOverVoltageV = old.cellOverVoltageV;
  migrated.cellUnderVoltageV = old.cellUnderVoltageV;
  migrated.dischargeCutoffV = old.dischargeCutoffV;
  migrated.dischargeHysteresisV = old.dischargeHysteresisV;
  migrated.chargeVoltageV = old.chargeVoltageV;
  migrated.chargeHysteresisV = old.chargeHysteresisV;
  migrated.maxChargeCurrentA = old.maxChargeCurrentA;
  migrated.endChargeCurrentA = old.endChargeCurrentA;
  migrated.batteryCapacityAh = old.batteryCapacityAh;
  migrated.currentSensor = old.currentSensor;
  migrated.invertCurrent = old.invertCurrent;
  migrated.currentCanBitrate = old.currentCanBitrate;
  migrated.currentCanId = old.currentCanId;
  migrated.currentSensorTimeoutMs = old.currentSensorTimeoutMs;
  migrated.chargerType = old.chargerType;
  migrated.seriesCells = old.seriesCells;
  migrated.chargerCanBitrate = old.chargerCanBitrate;
  migrated.chargerMessagePeriodMs = old.chargerMessagePeriodMs;
  migrated.chargerTxEnabled = old.chargerTxEnabled;
  migrated.mainPositivePin = old.mainPositivePin;
  migrated.mainNegativePin = old.mainNegativePin;
  migrated.prechargePin = old.prechargePin;
  migrated.chargerRelayPin = old.chargerRelayPin;
  return migrated;
}

BmsSettings migrateVersion2Settings(const BmsSettingsV2 &old) {
  BmsSettings migrated = currentSettings;
  migrated.stackModules = old.stackModules;
  migrated.cellsPerModule = old.cellsPerModule;
  migrated.samplePeriodMs = old.samplePeriodMs;
  migrated.ntcPullupOhms = old.ntcPullupOhms;
  migrated.ntcR25Ohms = old.ntcR25Ohms;
  migrated.ntcBetaKelvin = old.ntcBetaKelvin;
  migrated.ntcRatioMin = old.ntcRatioMin;
  migrated.ntcRatioMax = old.ntcRatioMax;
  migrated.cellOverVoltageV = old.cellOverVoltageV;
  migrated.cellUnderVoltageV = old.cellUnderVoltageV;
  migrated.dischargeCutoffV = old.dischargeCutoffV;
  migrated.dischargeHysteresisV = old.dischargeHysteresisV;
  migrated.chargeVoltageV = old.chargeVoltageV;
  migrated.chargeHysteresisV = old.chargeHysteresisV;
  migrated.maxChargeCurrentA = old.maxChargeCurrentA;
  migrated.endChargeCurrentA = old.endChargeCurrentA;
  migrated.batteryCapacityAh = old.batteryCapacityAh;
  migrated.currentSensor = old.currentSensor;
  migrated.invertCurrent = old.invertCurrent;
  migrated.currentCanBitrate = old.currentCanBitrate;
  migrated.currentCanId = old.currentCanId;
  migrated.currentSensorTimeoutMs = old.currentSensorTimeoutMs;
  migrated.chargerType = old.chargerType;
  migrated.seriesCells = old.seriesCells;
  migrated.chargerCanBitrate = old.chargerCanBitrate;
  migrated.chargerMessagePeriodMs = old.chargerMessagePeriodMs;
  migrated.chargerTxEnabled = old.chargerTxEnabled;
  migrated.mainPositivePin = old.mainPositivePin;
  migrated.mainNegativePin = old.mainNegativePin;
  migrated.prechargePin = old.prechargePin;
  migrated.chargerRelayPin = old.chargerRelayPin;
  migrated.prechargeMinimumMs = old.prechargeMinimumMs;
  migrated.prechargeTimeoutMs = old.prechargeTimeoutMs;
  migrated.prechargeFinishCurrentA = old.prechargeFinishCurrentA;
  migrated.runInputActiveHigh = old.runInputActiveHigh;
  migrated.chargeInputActiveHigh = old.chargeInputActiveHigh;
  return migrated;
}

BmsSettings migrateVersion3Settings(const BmsSettingsV3 &old) {
  BmsSettings migrated = currentSettings;
  migrated.stackModules = old.stackModules;
  migrated.cellsPerModule = old.cellsPerModule;
  migrated.samplePeriodMs = old.samplePeriodMs;
  migrated.ntcPullupOhms = old.ntcPullupOhms;
  migrated.ntcR25Ohms = old.ntcR25Ohms;
  migrated.ntcBetaKelvin = old.ntcBetaKelvin;
  migrated.ntcRatioMin = old.ntcRatioMin;
  migrated.ntcRatioMax = old.ntcRatioMax;
  migrated.cellOverVoltageV = old.cellOverVoltageV;
  migrated.cellUnderVoltageV = old.cellUnderVoltageV;
  migrated.dischargeCutoffV = old.dischargeCutoffV;
  migrated.dischargeHysteresisV = old.dischargeHysteresisV;
  migrated.chargeVoltageV = old.chargeVoltageV;
  migrated.chargeHysteresisV = old.chargeHysteresisV;
  migrated.maxChargeCurrentA = old.maxChargeCurrentA;
  migrated.endChargeCurrentA = old.endChargeCurrentA;
  migrated.batteryCapacityAh = old.batteryCapacityAh;
  migrated.currentSensor = old.currentSensor;
  migrated.invertCurrent = old.invertCurrent;
  migrated.currentCanBitrate = old.currentCanBitrate;
  migrated.currentCanId = old.currentCanId;
  migrated.currentSensorTimeoutMs = old.currentSensorTimeoutMs;
  migrated.chargerType = old.chargerType;
  migrated.seriesCells = old.seriesCells;
  migrated.chargerCanBitrate = old.chargerCanBitrate;
  migrated.chargerMessagePeriodMs = old.chargerMessagePeriodMs;
  migrated.chargerTxEnabled = old.chargerTxEnabled;
  migrated.mainPositivePin = old.mainPositivePin;
  migrated.mainNegativePin = old.mainNegativePin;
  migrated.prechargePin = old.prechargePin;
  migrated.chargerRelayPin = old.chargerRelayPin;
  migrated.prechargeMinimumMs = old.prechargeMinimumMs;
  migrated.prechargeTimeoutMs = old.prechargeTimeoutMs;
  migrated.prechargeFinishCurrentA = old.prechargeFinishCurrentA;
  migrated.runInputPin = old.runInputPin;
  migrated.chargeInputPin = old.chargeInputPin;
  // The installed isolated interfaces invert their 12 V inputs. Apply the
  // corrected active-LOW defaults during upgrade even if v3 stored HIGH.
  migrated.runInputActiveHigh = false;
  migrated.chargeInputActiveHigh = false;
  return migrated;
}

BmsSettings migrateVersion4Settings(const BmsSettingsV4 &old) {
  // Version 4 already allowed polarity overrides; preserve those choices.
  BmsSettings migrated = migrateVersion3Settings(old);
  migrated.runInputActiveHigh = old.runInputActiveHigh;
  migrated.chargeInputActiveHigh = old.chargeInputActiveHigh;
  return migrated;
}

BmsSettings migrateVersion5Settings(const BmsSettingsV5 &old) {
  BmsSettings migrated = migrateVersion4Settings(old.previous);
  migrated.socMode = old.socMode;
  migrated.socLowCellMv = old.socLowCellMv;
  migrated.socLowPercent = old.socLowPercent;
  migrated.socHighCellMv = old.socHighCellMv;
  migrated.socHighPercent = old.socHighPercent;
  // Version 5 always closed the contactors for charging; preserve that setup.
  migrated.chargerBehindContactors = true;
  return migrated;
}

BmsSettings migrateVersion8Settings(const BmsSettingsV8 &old) {
  BmsSettings migrated = currentSettings;
  migrated.stackModules = old.stackModules;
  migrated.cellsPerModule = old.cellsPerModule;
  migrated.samplePeriodMs = old.samplePeriodMs;
  migrated.ntcPullupOhms = old.ntcPullupOhms;
  migrated.ntcR25Ohms = old.ntcR25Ohms;
  migrated.ntcBetaKelvin = old.ntcBetaKelvin;
  migrated.ntcRatioMin = old.ntcRatioMin;
  migrated.ntcRatioMax = old.ntcRatioMax;
  migrated.cellOverVoltageV = old.cellOverVoltageV;
  migrated.cellUnderVoltageV = old.cellUnderVoltageV;
  migrated.dischargeCutoffV = old.dischargeCutoffV;
  migrated.dischargeHysteresisV = old.dischargeHysteresisV;
  migrated.chargeVoltageV = old.chargeVoltageV;
  migrated.chargeHysteresisV = old.chargeHysteresisV;
  migrated.maxChargeCurrentA = old.maxChargeCurrentA;
  migrated.endChargeCurrentA = old.endChargeCurrentA;
  migrated.batteryCapacityAh = old.batteryCapacityAh;
  migrated.currentSensor = old.currentSensor;
  migrated.invertCurrent = old.invertCurrent;
  migrated.currentCanBitrate = old.currentCanBitrate;
  migrated.currentCanId = old.currentCanId;
  migrated.currentSensorTimeoutMs = old.currentSensorTimeoutMs;
  migrated.chargerType = old.chargerType;
  migrated.seriesCells = old.seriesCells;
  migrated.chargerCanBitrate = old.chargerCanBitrate;
  migrated.chargerMessagePeriodMs = old.chargerMessagePeriodMs;
  migrated.chargerTxEnabled = old.chargerTxEnabled;
  migrated.mainPositivePin = old.mainPositivePin;
  migrated.mainNegativePin = old.mainNegativePin;
  migrated.prechargePin = old.prechargePin;
  migrated.chargerRelayPin = old.chargerRelayPin;
  migrated.prechargeMinimumMs = old.prechargeMinimumMs;
  migrated.prechargeTimeoutMs = old.prechargeTimeoutMs;
  migrated.prechargeFinishCurrentA = old.prechargeFinishCurrentA;
  migrated.runInputActiveHigh = old.runInputActiveHigh;
  migrated.chargeInputActiveHigh = old.chargeInputActiveHigh;
  migrated.runInputPin = old.runInputPin;
  migrated.chargeInputPin = old.chargeInputPin;
  migrated.socMode = old.socMode;
  migrated.socLowCellMv = old.socLowCellMv;
  migrated.socLowPercent = old.socLowPercent;
  migrated.socHighCellMv = old.socHighCellMv;
  migrated.socHighPercent = old.socHighPercent;
  migrated.chargerBehindContactors = old.chargerBehindContactors;
  migrated.mainNegativeControl = old.mainNegativeControl;
  migrated.prechargeControl = old.prechargeControl;
  return migrated;
}

BmsSettings migrateVersion9Settings(const BmsSettingsV9 &old) {
  BmsSettings migrated = currentSettings;
  // Version 10 only appends the temperature sample period. The prefix is an
  // exact copy of v9, so retain its complete user configuration verbatim.
  memcpy(&migrated, &old, sizeof(old));
  migrated.temperatureSamplePeriodMs = DEFAULT_TEMPERATURE_SAMPLE_PERIOD_MS;
  return migrated;
}

BmsSettings migrateVersion10Settings(const BmsSettingsV10 &old) {
  BmsSettings migrated = currentSettings;
  // Version 11 appends balancing controls. Preserve every v10 setting and
  // inherit the safe, disabled balancing defaults from currentSettings.
  memcpy(&migrated, &old, sizeof(old));
  migrated.cellBalancingEnabled = false;
  return migrated;
}

BmsSettings migrateVersion7Settings(const BmsSettingsV7 &old) {
  BmsSettings migrated = currentSettings;
  migrated.stackModules = old.stackModules;
  migrated.cellsPerModule = old.cellsPerModule;
  migrated.samplePeriodMs = old.samplePeriodMs;
  migrated.ntcPullupOhms = old.ntcPullupOhms;
  migrated.ntcR25Ohms = old.ntcR25Ohms;
  migrated.ntcBetaKelvin = old.ntcBetaKelvin;
  migrated.ntcRatioMin = old.ntcRatioMin;
  migrated.ntcRatioMax = old.ntcRatioMax;
  migrated.cellOverVoltageV = old.cellOverVoltageV;
  migrated.cellUnderVoltageV = old.cellUnderVoltageV;
  migrated.dischargeCutoffV = old.dischargeCutoffV;
  migrated.dischargeHysteresisV = old.dischargeHysteresisV;
  migrated.chargeVoltageV = old.chargeVoltageV;
  migrated.chargeHysteresisV = old.chargeHysteresisV;
  migrated.maxChargeCurrentA = old.maxChargeCurrentA;
  migrated.endChargeCurrentA = old.endChargeCurrentA;
  migrated.batteryCapacityAh = old.batteryCapacityAh;
  migrated.currentSensor = old.currentSensor;
  migrated.invertCurrent = old.invertCurrent;
  migrated.currentCanBitrate = old.currentCanBitrate;
  migrated.currentCanId = old.currentCanId;
  migrated.currentSensorTimeoutMs = old.currentSensorTimeoutMs;
  migrated.chargerType = old.chargerType;
  migrated.seriesCells = old.seriesCells;
  migrated.chargerCanBitrate = old.chargerCanBitrate;
  migrated.chargerMessagePeriodMs = old.chargerMessagePeriodMs;
  migrated.chargerTxEnabled = old.chargerTxEnabled;
  migrated.mainPositivePin = old.mainPositivePin;
  migrated.mainNegativePin = old.mainNegativePin;
  migrated.prechargePin = old.prechargePin;
  migrated.chargerRelayPin = old.chargerRelayPin;
  migrated.prechargeMinimumMs = old.prechargeMinimumMs;
  migrated.prechargeTimeoutMs = old.prechargeTimeoutMs;
  migrated.prechargeFinishCurrentA = old.prechargeFinishCurrentA;
  migrated.runInputActiveHigh = old.runInputActiveHigh;
  migrated.chargeInputActiveHigh = old.chargeInputActiveHigh;
  migrated.runInputPin = old.runInputPin;
  migrated.chargeInputPin = old.chargeInputPin;
  migrated.socMode = old.socMode;
  migrated.socLowCellMv = old.socLowCellMv;
  migrated.socLowPercent = old.socLowPercent;
  migrated.socHighCellMv = old.socHighCellMv;
  migrated.socHighPercent = old.socHighPercent;
  migrated.chargerBehindContactors = old.chargerBehindContactors;
  return migrated;
}

BmsSettings migrateVersion6Settings(const BmsSettingsV6 &old) {
  return migrateVersion7Settings(old);
}

bool writeCurrentSettings() {
  if (!storageAvailable || !validSettings(currentSettings)) return false;
  if (savedRecordAvailable && sameSettings(currentSettings, savedSettings)) return true;

  // Write the inactive slot first, leaving the previous verified record intact
  // if power is lost before the new record passes readback verification.
  const int destinationSlot = activeRecordSlot == 0 ? 1 : 0;
  const int address = destinationSlot * EEPROM_SLOT_SPACING;
  if (address + static_cast<int>(sizeof(SettingsRecord)) > EEPROM.length()) return false;

  SettingsRecord record = {};
  record.magic = SETTINGS_MAGIC;
  record.version = SETTINGS_VERSION;
  record.payloadSize = sizeof(BmsSettings);
  record.generation = settingsGeneration + 1U;
  record.payload = currentSettings;
  record.crc = calculateCrc32(reinterpret_cast<const uint8_t *>(&record),
                              offsetof(SettingsRecord, crc));
  EEPROM.put(address, record);

  SettingsRecord verify = {};
  if (!readRecord(destinationSlot, verify) ||
      !sameSettings(verify.payload, currentSettings)) return false;
  savedSettings = verify.payload;
  savedRecordAvailable = true;
  activeRecordSlot = static_cast<int8_t>(destinationSlot);
  settingsGeneration = verify.generation;
  return true;
}

bool commitSettings(const BmsSettings &candidate) {
  if (!validSettings(candidate)) return false;
  if (sameSettings(candidate, currentSettings)) return settingsAreStored();
  const BmsSettings previous = currentSettings;
  currentSettings = candidate;
  if (!writeCurrentSettings()) {
    currentSettings = previous;
    return false;
  }
  return true;
}

void writeMigratedSettings(const BmsSettings &settings, int sourceSlot,
                           uint32_t sourceGeneration) {
  currentSettings = settings;
  if (currentSettings.chargerType == ChargerType::Outlander) {
    // The OBC and LEM share a physical bus, so use the legacy verified 500 kbit/s
    // rate and enable the newly implemented OBC control after firmware upgrade.
    currentSettings.currentCanBitrate = 500000UL;
    currentSettings.chargerCanBitrate = 500000UL;
    currentSettings.chargerTxEnabled = true;
  }
  activeRecordSlot = static_cast<int8_t>(sourceSlot);
  settingsGeneration = sourceGeneration;
  // The old record remains valid until the new version verifies.
  writeCurrentSettings();
}
}

void settingsBegin() {
  storageAvailable =
      EEPROM.length() >= EEPROM_SLOT_SPACING + static_cast<int>(sizeof(SettingsRecord));
  savedRecordAvailable = false;
  activeRecordSlot = -1;
  settingsGeneration = 0;
  if (!storageAvailable) return;

  // Load the newest valid record. For older formats, migrate fields forward
  // and write a new record without invalidating the source until verification.
  SettingsRecord first = {};
  SettingsRecord second = {};
  const bool firstValid = readRecord(0, first);
  const bool secondValid = readRecord(1, second);
  if (firstValid || secondValid) {
    const bool chooseSecond = secondValid &&
        (!firstValid || static_cast<int32_t>(second.generation - first.generation) > 0);
    const SettingsRecord &selected = chooseSecond ? second : first;
    currentSettings = selected.payload;
    savedSettings = selected.payload;
    savedRecordAvailable = true;
    activeRecordSlot = chooseSecond ? 1 : 0;
    settingsGeneration = selected.generation;
    return;
  }

  SettingsRecordV10 v10First = {};
  SettingsRecordV10 v10Second = {};
  const bool v10FirstValid = readVersion10Record(0, v10First);
  const bool v10SecondValid = readVersion10Record(1, v10Second);
  if (v10FirstValid || v10SecondValid) {
    const bool chooseSecond = v10SecondValid &&
        (!v10FirstValid || static_cast<int32_t>(v10Second.generation -
                                                  v10First.generation) > 0);
    const SettingsRecordV10 &previous = chooseSecond ? v10Second : v10First;
    const BmsSettings migrated = migrateVersion10Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV9 v9First = {};
  SettingsRecordV9 v9Second = {};
  const bool v9FirstValid = readVersion9Record(0, v9First);
  const bool v9SecondValid = readVersion9Record(1, v9Second);
  if (v9FirstValid || v9SecondValid) {
    const bool chooseSecond = v9SecondValid &&
        (!v9FirstValid || static_cast<int32_t>(v9Second.generation -
                                                v9First.generation) > 0);
    const SettingsRecordV9 &previous = chooseSecond ? v9Second : v9First;
    const BmsSettings migrated = migrateVersion9Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV8 v8First = {};
  SettingsRecordV8 v8Second = {};
  const bool v8FirstValid = readVersion8Record(0, v8First);
  const bool v8SecondValid = readVersion8Record(1, v8Second);
  if (v8FirstValid || v8SecondValid) {
    const bool chooseSecond = v8SecondValid &&
        (!v8FirstValid || static_cast<int32_t>(v8Second.generation -
                                                v8First.generation) > 0);
    const SettingsRecordV8 &previous = chooseSecond ? v8Second : v8First;
    const BmsSettings migrated = migrateVersion8Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV7 v7First = {};
  SettingsRecordV7 v7Second = {};
  const bool v7FirstValid = readVersion7Record(0, v7First);
  const bool v7SecondValid = readVersion7Record(1, v7Second);
  if (v7FirstValid || v7SecondValid) {
    const bool chooseSecond = v7SecondValid &&
        (!v7FirstValid || static_cast<int32_t>(v7Second.generation -
                                                v7First.generation) > 0);
    const SettingsRecordV7 &previous = chooseSecond ? v7Second : v7First;
    const BmsSettings migrated = migrateVersion7Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV6 v6First = {};
  SettingsRecordV6 v6Second = {};
  const bool v6FirstValid = readVersion6Record(0, v6First);
  const bool v6SecondValid = readVersion6Record(1, v6Second);
  if (v6FirstValid || v6SecondValid) {
    const bool chooseSecond = v6SecondValid &&
        (!v6FirstValid || static_cast<int32_t>(v6Second.generation -
                                                v6First.generation) > 0);
    const SettingsRecordV6 &previous = chooseSecond ? v6Second : v6First;
    const BmsSettings migrated = migrateVersion6Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV5 v5First = {};
  SettingsRecordV5 v5Second = {};
  const bool v5FirstValid = readVersion5Record(0, v5First);
  const bool v5SecondValid = readVersion5Record(1, v5Second);
  if (v5FirstValid || v5SecondValid) {
    const bool chooseSecond = v5SecondValid &&
        (!v5FirstValid || static_cast<int32_t>(v5Second.generation -
                                                v5First.generation) > 0);
    const SettingsRecordV5 &previous = chooseSecond ? v5Second : v5First;
    const BmsSettings migrated = migrateVersion5Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV4 v4First = {};
  SettingsRecordV4 v4Second = {};
  const bool v4FirstValid = readVersion4Record(0, v4First);
  const bool v4SecondValid = readVersion4Record(1, v4Second);
  if (v4FirstValid || v4SecondValid) {
    const bool chooseSecond = v4SecondValid &&
        (!v4FirstValid || static_cast<int32_t>(v4Second.generation -
                                                v4First.generation) > 0);
    const SettingsRecordV4 &previous = chooseSecond ? v4Second : v4First;
    const BmsSettings migrated = migrateVersion4Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV3 v3First = {};
  SettingsRecordV3 v3Second = {};
  const bool v3FirstValid = readVersion3Record(0, v3First);
  const bool v3SecondValid = readVersion3Record(1, v3Second);
  if (v3FirstValid || v3SecondValid) {
    const bool chooseSecond = v3SecondValid &&
        (!v3FirstValid || static_cast<int32_t>(v3Second.generation -
                                                v3First.generation) > 0);
    const SettingsRecordV3 &previous = chooseSecond ? v3Second : v3First;
    const BmsSettings migrated = migrateVersion3Settings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV2 v2First = {};
  SettingsRecordV2 v2Second = {};
  const bool v2FirstValid = readVersion2Record(0, v2First);
  const bool v2SecondValid = readVersion2Record(1, v2Second);
  if (v2FirstValid || v2SecondValid) {
    const bool chooseSecond = v2SecondValid &&
        (!v2FirstValid || static_cast<int32_t>(v2Second.generation -
                                                v2First.generation) > 0);
    const SettingsRecordV2 &previous = chooseSecond ? v2Second : v2First;
    BmsSettings migrated = migrateVersion2Settings(previous.payload);
    // The installed isolated inputs invert their 12 V sense signals.
    migrated.runInputActiveHigh = false;
    migrated.chargeInputActiveHigh = false;
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  SettingsRecordV1 v1First = {};
  SettingsRecordV1 v1Second = {};
  const bool v1FirstValid = readLegacyRecord(0, v1First);
  const bool v1SecondValid = readLegacyRecord(1, v1Second);
  if (v1FirstValid || v1SecondValid) {
    const bool chooseSecond = v1SecondValid &&
        (!v1FirstValid || static_cast<int32_t>(v1Second.generation -
                                                v1First.generation) > 0);
    const SettingsRecordV1 &previous = chooseSecond ? v1Second : v1First;
    const BmsSettings migrated = migrateLegacySettings(previous.payload);
    if (validSettings(migrated)) {
      writeMigratedSettings(migrated, chooseSecond ? 1 : 0,
                            previous.generation);
      return;
    }
  }

  // First boot or no recognized valid record: save compiled defaults.
  writeCurrentSettings();
}

const BmsSettings &settingsGet() {
  return currentSettings;
}

bool settingsAreStored() {
  return storageAvailable && savedRecordAvailable &&
         sameSettings(currentSettings, savedSettings);
}

bool settingsSetSamplePeriod(uint16_t periodMs) {
  if (periodMs < MIN_SAMPLE_PERIOD_MS || periodMs > MAX_SAMPLE_PERIOD_MS) {
    return false;
  }
  BmsSettings candidate = currentSettings;
  candidate.samplePeriodMs = periodMs;
  return commitSettings(candidate);
}

bool settingsSetTemperatureSamplePeriod(uint16_t periodMs) {
  if (periodMs < MIN_TEMPERATURE_SAMPLE_PERIOD_MS ||
      periodMs > MAX_TEMPERATURE_SAMPLE_PERIOD_MS) {
    return false;
  }
  BmsSettings candidate = currentSettings;
  candidate.temperatureSamplePeriodMs = periodMs;
  return commitSettings(candidate);
}

bool settingsSetCellBalancingEnabled(bool enabled) {
  BmsSettings candidate = currentSettings;
  candidate.cellBalancingEnabled = enabled;
  return commitSettings(candidate);
}

bool settingsSetBalancingTargetDeltaMv(uint16_t millivolts) {
  BmsSettings candidate = currentSettings;
  candidate.balancingTargetDeltaMv = millivolts;
  return commitSettings(candidate);
}

bool settingsSetBalancingHysteresisMv(uint16_t millivolts) {
  BmsSettings candidate = currentSettings;
  candidate.balancingHysteresisMv = millivolts;
  return commitSettings(candidate);
}

bool settingsSetBalancingMinCellMv(uint16_t millivolts) {
  BmsSettings candidate = currentSettings;
  candidate.balancingMinCellMv = millivolts;
  return commitSettings(candidate);
}

bool settingsSetBalancingMaxOnSeconds(uint16_t seconds) {
  BmsSettings candidate = currentSettings;
  candidate.balancingMaxOnSeconds = seconds;
  return commitSettings(candidate);
}

bool settingsSetBalancingConservativeLevel(uint8_t level) {
  BmsSettings candidate = currentSettings;
  candidate.balancingConservativeLevel = level;
  return commitSettings(candidate);
}

bool settingsSetBalancingCoolOffMs(uint32_t periodMs) {
  BmsSettings candidate = currentSettings;
  candidate.balancingCoolOffMs = periodMs;
  return commitSettings(candidate);
}

bool settingsSetVoltageLimit(const char *name, float value) {
  if (!name || value < 0.0f || value > 5.0f) return false;
  BmsSettings candidate = currentSettings;
  if (strcmp(name, "ov") == 0) {
    if (value < 2.5f || value > 4.5f) return false;
    candidate.cellOverVoltageV = value;
  } else if (strcmp(name, "uv") == 0) {
    if (value < 2.0f || value > 4.0f) return false;
    candidate.cellUnderVoltageV = value;
  } else if (strcmp(name, "dv") == 0) {
    if (value < 2.0f || value > 4.3f) return false;
    candidate.dischargeCutoffV = value;
  } else if (strcmp(name, "cv") == 0) {
    if (value < 2.5f || value > 4.5f) return false;
    candidate.chargeVoltageV = value;
  } else if (strcmp(name, "hys") == 0) {
    if (value < 0.01f || value > 1.0f) return false;
    candidate.chargeHysteresisV = value;
  } else if (strcmp(name, "dhy") == 0) {
    if (value < 0.01f || value > 1.0f) return false;
    candidate.dischargeHysteresisV = value;
  } else {
    return false;
  }

  if (!(candidate.cellUnderVoltageV < candidate.dischargeCutoffV &&
        candidate.dischargeCutoffV < candidate.chargeVoltageV &&
        candidate.chargeVoltageV < candidate.cellOverVoltageV &&
        candidate.chargeHysteresisV <
            candidate.chargeVoltageV - candidate.cellUnderVoltageV &&
        candidate.dischargeHysteresisV <
            candidate.cellOverVoltageV - candidate.dischargeCutoffV)) {
    return false;
  }

  return commitSettings(candidate);
}

bool settingsSetChargeCurrent(const char *name, float value) {
  if (!name || value < 0.0f || value > 500.0f) return false;
  if (strcmp(name, "imax") == 0) {
    if (value < currentSettings.endChargeCurrentA) return false;
    BmsSettings candidate = currentSettings;
    candidate.maxChargeCurrentA = value;
    return commitSettings(candidate);
  }
  if (strcmp(name, "iend") == 0) {
    if (value > currentSettings.maxChargeCurrentA) return false;
    BmsSettings candidate = currentSettings;
    candidate.endChargeCurrentA = value;
    return commitSettings(candidate);
  }
  return false;
}

bool settingsSetBatteryCapacity(float value) {
  if (value < 0.1f || value > 2000.0f) return false;
  BmsSettings candidate = currentSettings;
  candidate.batteryCapacityAh = value;
  return commitSettings(candidate);
}

bool settingsSetSocMode(SocMode mode) {
  if (mode != SocMode::CoulombCount && mode != SocMode::VoltageOnly) return false;
  BmsSettings candidate = currentSettings;
  candidate.socMode = mode;
  return commitSettings(candidate);
}

bool settingsSetSocVoltageAnchor(const char *name, float value) {
  if (!name || value != value) return false;
  BmsSettings candidate = currentSettings;
  if (strcmp(name, "soclowv") == 0 || strcmp(name, "sochighv") == 0) {
    if (value < 2.0f || value > 4.5f) return false;
    const uint16_t millivolts = static_cast<uint16_t>(value * 1000.0f + 0.5f);
    if (strcmp(name, "soclowv") == 0) candidate.socLowCellMv = millivolts;
    else candidate.socHighCellMv = millivolts;
  } else if (strcmp(name, "soclowpct") == 0 ||
             strcmp(name, "sochighpct") == 0) {
    if (value < 0.0f || value > 100.0f ||
        value != static_cast<float>(static_cast<uint8_t>(value))) return false;
    if (strcmp(name, "soclowpct") == 0) {
      candidate.socLowPercent = static_cast<uint8_t>(value);
    } else {
      candidate.socHighPercent = static_cast<uint8_t>(value);
    }
  } else {
    return false;
  }
  return commitSettings(candidate);
}

bool settingsSetCurrentSensorType(CurrentSensorType type) {
  if (static_cast<uint8_t>(type) >
      static_cast<uint8_t>(CurrentSensorType::SmePlaceholder)) return false;
  BmsSettings candidate = currentSettings;
  candidate.currentSensor = type;
  return commitSettings(candidate);
}

bool settingsSetContactorControlType(BmsOutputRole role,
                                     ContactorControlType type) {
  if (type != ContactorControlType::NanoLowSideFet &&
      type != ContactorControlType::SmePlaceholder) return false;
  BmsSettings candidate = currentSettings;
  if (role == BmsOutputRole::MainNegative) {
    candidate.mainNegativeControl = type;
  } else if (role == BmsOutputRole::Precharge) {
    candidate.prechargeControl = type;
  } else {
    return false;
  }
  return commitSettings(candidate);
}

ContactorControlType settingsGetContactorControlType(BmsOutputRole role) {
  if (role == BmsOutputRole::MainNegative) return currentSettings.mainNegativeControl;
  if (role == BmsOutputRole::Precharge) return currentSettings.prechargeControl;
  return ContactorControlType::NanoLowSideFet;
}

bool settingsSetCurrentCanBitrate(uint32_t bitrate) {
  if (bitrate != 250000UL && bitrate != 500000UL) return false;
  if (currentSettings.chargerType == ChargerType::Outlander &&
      bitrate != 500000UL) return false;
  BmsSettings candidate = currentSettings;
  candidate.currentCanBitrate = bitrate;
  if (candidate.chargerType == ChargerType::Outlander) {
    candidate.chargerCanBitrate = bitrate;
  }
  return commitSettings(candidate);
}

bool settingsSetCurrentCanId(uint16_t id) {
  if (!id || id > 0x7FFU) return false;
  BmsSettings candidate = currentSettings;
  candidate.currentCanId = id;
  return commitSettings(candidate);
}

bool settingsSetCurrentInvert(bool invert) {
  BmsSettings candidate = currentSettings;
  candidate.invertCurrent = invert;
  return commitSettings(candidate);
}

bool settingsSetChargerType(ChargerType type) {
  if (static_cast<uint8_t>(type) >
      static_cast<uint8_t>(ChargerType::Outlander)) return false;
  BmsSettings candidate = currentSettings;
  candidate.chargerType = type;
  if (type == ChargerType::Outlander) {
    // The OBC and LEM must share the reference SimpBMS 500 kbit/s bus rate.
    candidate.currentCanBitrate = 500000UL;
    candidate.chargerCanBitrate = 500000UL;
    candidate.chargerTxEnabled = true;
  }
  return commitSettings(candidate);
}

bool settingsSetChargerCanTx(bool enabled) {
  BmsSettings candidate = currentSettings;
  candidate.chargerTxEnabled = enabled;
  return commitSettings(candidate);
}

bool settingsSetChargerBehindContactors(bool behindContactors) {
  BmsSettings candidate = currentSettings;
  candidate.chargerBehindContactors = behindContactors;
  return commitSettings(candidate);
}

bool settingsSetSeriesCells(uint16_t count) {
  if (count < 1 || count > 160) return false;
  BmsSettings candidate = currentSettings;
  candidate.seriesCells = count;
  return commitSettings(candidate);
}

bool settingsSetPrecharge(const char *name, float value) {
  if (!name || value != value || value < 0.0f || value > 60000.0f) return false;
  BmsSettings candidate = currentSettings;
  if (strcmp(name, "premin") == 0) {
    if (value < 100.0f || value > 30000.0f ||
        value != static_cast<float>(static_cast<uint16_t>(value))) return false;
    candidate.prechargeMinimumMs = static_cast<uint16_t>(value);
  } else if (strcmp(name, "pretimeout") == 0) {
    if (value <= candidate.prechargeMinimumMs || value > 60000.0f ||
        value != static_cast<float>(static_cast<uint16_t>(value))) return false;
    candidate.prechargeTimeoutMs = static_cast<uint16_t>(value);
  } else if (strcmp(name, "precurrent") == 0) {
    if (value < 0.1f || value > 50.0f) return false;
    candidate.prechargeFinishCurrentA = value;
  } else {
    return false;
  }
  return commitSettings(candidate);
}

bool settingsSetInputPolarity(const char *name, bool activeHigh) {
  if (!name) return false;
  BmsSettings candidate = currentSettings;
  if (strcmp(name, "runpol") == 0) {
    candidate.runInputActiveHigh = activeHigh;
  } else if (strcmp(name, "chargepol") == 0) {
    candidate.chargeInputActiveHigh = activeHigh;
  } else if (strcmp(name, "isolationpol") == 0) {
    candidate.isolationFaultActiveHigh = activeHigh;
  } else {
    return false;
  }
  return commitSettings(candidate);
}

bool settingsSetInputPin(const char *name, uint8_t pin) {
  if (!name) return false;
  const bool isolationPin = strcmp(name, "isolationpin") == 0;
  if (isolationPin) {
    if (pin != 0xFFU &&
        (!allowedInputPin(pin) || pin == 4U || pin == 5U)) return false;
  } else if (!allowedInputPin(pin) || pin == 4U || pin == 5U) {
    return false;
  }
  BmsSettings candidate = currentSettings;
  if (strcmp(name, "runpin") == 0) {
    candidate.runInputPin = pin;
  } else if (strcmp(name, "chargepin") == 0) {
    candidate.chargeInputPin = pin;
  } else if (isolationPin) {
    candidate.isolationMonitorPin = pin;
  } else {
    return false;
  }
  return commitSettings(candidate);
}

bool settingsSetIsolationMonitorEnabled(bool enabled) {
  BmsSettings candidate = currentSettings;
  candidate.isolationMonitorEnabled = enabled;
  return commitSettings(candidate);
}

bool settingsSetIsolationMonitorType(IsolationMonitorType type) {
  if (static_cast<uint8_t>(type) >
      static_cast<uint8_t>(IsolationMonitorType::SmePlaceholder)) return false;
  BmsSettings candidate = currentSettings;
  candidate.isolationMonitorType = type;
  return commitSettings(candidate);
}

bool settingsSetIsolationMonitorPolarity(bool activeHigh) {
  return settingsSetInputPolarity("isolationpol", activeHigh);
}

bool settingsSetChargerRelayPin(uint8_t pin) {
  return settingsSetOutputPin(BmsOutputRole::ChargerRelay, pin);
}

uint8_t settingsGetOutputPin(BmsOutputRole role) {
  switch (role) {
    case BmsOutputRole::MainPositive: return currentSettings.mainPositivePin;
    case BmsOutputRole::MainNegative: return currentSettings.mainNegativePin;
    case BmsOutputRole::Precharge: return currentSettings.prechargePin;
    case BmsOutputRole::ChargerRelay: return currentSettings.chargerRelayPin;
    default: return 0xFFU;
  }
}

bool settingsSetOutputPin(BmsOutputRole role, uint8_t pin) {
  if (pin != 0xFFU && pin != 6U && pin != 13U && pin != 11U && pin != 10U) {
    return false;
  }

  for (uint8_t roleIndex = 0; roleIndex < 4U; ++roleIndex) {
    const BmsOutputRole otherRole = static_cast<BmsOutputRole>(roleIndex);
    if (otherRole != role && pin != 0xFFU &&
        settingsGetOutputPin(otherRole) == pin) return false;
  }

  BmsSettings candidate = currentSettings;
  switch (role) {
    case BmsOutputRole::MainPositive: candidate.mainPositivePin = pin; break;
    case BmsOutputRole::MainNegative: candidate.mainNegativePin = pin; break;
    case BmsOutputRole::Precharge: candidate.prechargePin = pin; break;
    case BmsOutputRole::ChargerRelay: candidate.chargerRelayPin = pin; break;
    default: return false;
  }
  return commitSettings(candidate);
}

bool settingsLoadSocState(PersistentSocState &state) {
  if (sizeof(SettingsRecord) >= static_cast<size_t>(SOC_REGION0_ADDRESS)) {
    return false;
  }
  SocRecord newest = {};
  uint16_t newestSlot = 0;
  if (!readNewestSocRecord(newest, newestSlot)) return false;
  (void)newestSlot;
  state = newest.payload;
  return true;
}

bool settingsSaveSocState(const PersistentSocState &state) {
  if (state.percentBasisPoints > 10000U ||
      sizeof(SettingsRecord) >= static_cast<size_t>(SOC_REGION0_ADDRESS) ||
      socJournalSlotCount() == 0U) {
    return false;
  }

  SocRecord newest = {};
  uint16_t newestSlot = 0;
  const bool haveNewest = readNewestSocRecord(newest, newestSlot);
  if (haveNewest &&
      newest.payload.percentBasisPoints == state.percentBasisPoints &&
      newest.payload.chargedMilliAh == state.chargedMilliAh &&
      newest.payload.dischargedMilliAh == state.dischargedMilliAh) return true;

  const uint16_t slotCount = socJournalSlotCount();
  const uint16_t destinationSlot = haveNewest
      ? static_cast<uint16_t>((newestSlot + 1U) % slotCount) : 0U;
  const int destinationAddress = socJournalAddress(destinationSlot);
  if (destinationAddress < 0) return false;
  SocRecord record = {};
  record.magic = SOC_MAGIC;
  record.version = SOC_RECORD_VERSION;
  record.payloadSize = sizeof(PersistentSocState);
  record.generation = haveNewest ? newest.generation + 1U : 1U;
  record.payload = state;
  record.crc = calculateCrc32(reinterpret_cast<const uint8_t *>(&record),
                              offsetof(SocRecord, crc));
  EEPROM.put(destinationAddress, record);

  SocRecord verify = {};
  if (!readSocRecord(destinationAddress, verify) ||
      verify.generation != record.generation ||
      verify.payload.percentBasisPoints != state.percentBasisPoints ||
      verify.payload.chargedMilliAh != state.chargedMilliAh ||
      verify.payload.dischargedMilliAh != state.dischargedMilliAh) return false;
  return true;
}

bool settingsCanUseEepromRange(int address, size_t length) {
  if (!storageAvailable || address < 0 || length == 0U) return false;
  const size_t start = static_cast<size_t>(address);
  const size_t end = start + length;
  if (end < start || end > static_cast<size_t>(EEPROM.length())) return false;

  const size_t settingsRecordSize = sizeof(SettingsRecord);
  const size_t slot0End = settingsRecordSize;
  const size_t slot1Start = static_cast<size_t>(EEPROM_SLOT_SPACING);
  const size_t slot1End = slot1Start + settingsRecordSize;
  const size_t soc0Start = static_cast<size_t>(SOC_REGION0_ADDRESS);
  const size_t soc0End = soc0Start + SOC_REGION_SIZE;
  const size_t soc1Start = static_cast<size_t>(SOC_REGION1_ADDRESS);
  const size_t soc1End = soc1Start + SOC_REGION_SIZE;
  const auto overlaps = [start, end](size_t reservedStart,
                                     size_t reservedEnd) {
    return start < reservedEnd && end > reservedStart;
  };

  return !overlaps(0U, slot0End) &&
         !overlaps(slot1Start, slot1End) &&
         !overlaps(soc0Start, soc0End) &&
         !overlaps(soc1Start, soc1End);
}

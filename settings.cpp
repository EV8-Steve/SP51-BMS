/*
  Persistent BMS configuration.

  The current settings payload is written to one of two EEPROM slots with a
  generation counter and CRC32. A change is committed only after a readback
  verifies; prior version-1 through version-4 records are migrated field by
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
constexpr uint32_t SETTINGS_MAGIC = 0x53503531UL;  // "SP51"
constexpr uint16_t SETTINGS_VERSION = 5;
constexpr uint16_t LEGACY_SETTINGS_VERSION = 1;
constexpr uint16_t VERSION2_SETTINGS_VERSION = 2;
constexpr uint16_t VERSION3_SETTINGS_VERSION = 3;
constexpr uint16_t PREVIOUS_SETTINGS_VERSION = 4;
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
    false,          // Keep charger CAN transmission disabled until verified.
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
    90
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
         a.socHighPercent == b.socHighPercent;
}

bool allowedPin(uint8_t pin) {
  return pin == 0xFFU || pin == 6U || pin == 13U || pin == 11U || pin == 10U;
}

bool allowedInputPin(uint8_t pin) {
  return pin >= 2U && pin <= 13U;
}

bool validSettings(const BmsSettings &s) {
  // Central validation is shared by EEPROM load, migration, and every setter.
  // Pin uniqueness applies across the four outputs and two isolated inputs.
  if (s.stackModules < 1U || s.stackModules > 12U ||
      s.cellsPerModule != 12U || s.samplePeriodMs < MIN_SAMPLE_PERIOD_MS ||
      s.samplePeriodMs > MAX_SAMPLE_PERIOD_MS) return false;
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
  if ((s.currentSensor != CurrentSensorType::Off &&
       s.currentSensor != CurrentSensorType::LemCab500Can) ||
      (s.currentCanBitrate != 250000UL && s.currentCanBitrate != 500000UL) ||
      s.currentCanId == 0U || s.currentCanId > 0x7FFU ||
      s.currentSensorTimeoutMs < 10U || s.currentSensorTimeoutMs > 5000U) return false;
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
  const uint8_t pins[6] = {s.mainPositivePin, s.mainNegativePin,
                           s.prechargePin, s.chargerRelayPin,
                           s.runInputPin, s.chargeInputPin};
  for (uint8_t i = 0; i < 6U; ++i) {
    if (i < 4U ? !allowedPin(pins[i]) : !allowedInputPin(pins[i])) return false;
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
      record.version != PREVIOUS_SETTINGS_VERSION ||
      record.payloadSize != sizeof(BmsSettingsV4)) return false;
  const uint32_t expected = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record), offsetof(SettingsRecordV4, crc));
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
}

void settingsBegin() {
  storageAvailable =
      EEPROM.length() >= EEPROM_SLOT_SPACING + static_cast<int>(sizeof(SettingsRecord));
  savedRecordAvailable = false;
  activeRecordSlot = -1;
  settingsGeneration = 0;
  if (!storageAvailable) return;

  // Prefer the newest valid current-version record; fall back through saved
  // formats in descending order while preserving each prior record on disk.
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
  } else {
    SettingsRecordV4 v4First = {};
    SettingsRecordV4 v4Second = {};
    const bool v4FirstValid = readVersion4Record(0, v4First);
    const bool v4SecondValid = readVersion4Record(1, v4Second);
    if (v4FirstValid || v4SecondValid) {
      const bool chooseSecondV4 = v4SecondValid &&
          (!v4FirstValid || static_cast<int32_t>(v4Second.generation -
                                                  v4First.generation) > 0);
      const SettingsRecordV4 &previous = chooseSecondV4 ? v4Second : v4First;
      const int previousSlot = chooseSecondV4 ? 1 : 0;
      const BmsSettings migrated = migrateVersion4Settings(previous.payload);
      if (validSettings(migrated)) {
        currentSettings = migrated;
        activeRecordSlot = static_cast<int8_t>(previousSlot);
        settingsGeneration = previous.generation;
        // The v4 record remains valid until the new v5 settings verify.
        writeCurrentSettings();
      }
    } else {
    SettingsRecordV3 v3First = {};
    SettingsRecordV3 v3Second = {};
    const bool v3FirstValid = readVersion3Record(0, v3First);
    const bool v3SecondValid = readVersion3Record(1, v3Second);
    if (v3FirstValid || v3SecondValid) {
      const bool chooseSecondV3 = v3SecondValid &&
          (!v3FirstValid || static_cast<int32_t>(v3Second.generation -
                                                  v3First.generation) > 0);
      const SettingsRecordV3 &previous = chooseSecondV3 ? v3Second : v3First;
      const int previousSlot = chooseSecondV3 ? 1 : 0;
      const BmsSettings migrated = migrateVersion3Settings(previous.payload);
      if (validSettings(migrated)) {
        currentSettings = migrated;
        activeRecordSlot = static_cast<int8_t>(previousSlot);
        settingsGeneration = previous.generation;
        // Keep v3 intact until the new active-LOW v5 record verifies.
        writeCurrentSettings();
      }
    } else {
      SettingsRecordV2 v2First = {};
      SettingsRecordV2 v2Second = {};
      const bool v2FirstValid = readVersion2Record(0, v2First);
      const bool v2SecondValid = readVersion2Record(1, v2Second);
      if (v2FirstValid || v2SecondValid) {
        const bool chooseSecondV2 = v2SecondValid &&
            (!v2FirstValid || static_cast<int32_t>(v2Second.generation -
                                                    v2First.generation) > 0);
        const SettingsRecordV2 &previous = chooseSecondV2 ? v2Second : v2First;
        const int previousSlot = chooseSecondV2 ? 1 : 0;
        BmsSettings migrated = migrateVersion2Settings(previous.payload);
        // Version 4 changed both installed isolated inputs to active-LOW.
        migrated.runInputActiveHigh = false;
        migrated.chargeInputActiveHigh = false;
        if (validSettings(migrated)) {
          currentSettings = migrated;
          activeRecordSlot = static_cast<int8_t>(previousSlot);
          settingsGeneration = previous.generation;
          writeCurrentSettings();
        }
      } else {
        SettingsRecordV1 v1First = {};
        SettingsRecordV1 v1Second = {};
        const bool v1FirstValid = readLegacyRecord(0, v1First);
        const bool v1SecondValid = readLegacyRecord(1, v1Second);
        if (v1FirstValid || v1SecondValid) {
          const bool chooseSecondV1 = v1SecondValid &&
              (!v1FirstValid || static_cast<int32_t>(v1Second.generation -
                                                      v1First.generation) > 0);
          const SettingsRecordV1 &previous = chooseSecondV1 ? v1Second : v1First;
          const int previousSlot = chooseSecondV1 ? 1 : 0;
          const BmsSettings migrated = migrateLegacySettings(previous.payload);
          if (validSettings(migrated)) {
            currentSettings = migrated;
            activeRecordSlot = static_cast<int8_t>(previousSlot);
            settingsGeneration = previous.generation;
            writeCurrentSettings();
          }
        } else {
          // First boot: save compiled defaults so they survive reset/power loss.
          writeCurrentSettings();
        }
      }
    }
    }
  }
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
  if (type != CurrentSensorType::Off &&
      type != CurrentSensorType::LemCab500Can) return false;
  BmsSettings candidate = currentSettings;
  candidate.currentSensor = type;
  return commitSettings(candidate);
}

bool settingsSetCurrentCanBitrate(uint32_t bitrate) {
  if (bitrate != 250000UL && bitrate != 500000UL) return false;
  BmsSettings candidate = currentSettings;
  candidate.currentCanBitrate = bitrate;
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
  } else {
    return false;
  }
  return commitSettings(candidate);
}

bool settingsSetInputPin(const char *name, uint8_t pin) {
  if (!name || !allowedInputPin(pin)) return false;
  BmsSettings candidate = currentSettings;
  if (strcmp(name, "runpin") == 0) {
    candidate.runInputPin = pin;
  } else if (strcmp(name, "chargepin") == 0) {
    candidate.chargeInputPin = pin;
  } else {
    return false;
  }
  return commitSettings(candidate);
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

/*
  Persistent fault/event history for SP51_basic_BMS.

  The log is a small rotating EEPROM journal. Its range is validated by the
  settings module so it cannot overlap the settings records or SOC journal.
  Records contain a boot counter and uptime seconds because this target has no
  wall-clock source. Fault entries keep the drive-cycle number at occurrence;
  the display labels a record CURRENT only while that same cycle is active.
*/
#include "fault_log.h"

#include <EEPROM.h>
#include <stddef.h>

#include "settings.h"

namespace {
constexpr int FAULT_LOG_ADDRESS = 1024;
constexpr size_t FAULT_LOG_REGION_BYTES = 1024U;
constexpr uint32_t FAULT_LOG_MAGIC = 0x464C4731UL;  // "FLG1"
constexpr uint32_t FAULT_LOG_VERSION = 2U;

struct FaultLogMetadata {
  uint32_t magic;
  uint32_t version;
  uint32_t generation;
  uint32_t nextSequence;
  uint32_t driveCycle;
  uint32_t bootCount;
  uint32_t latchedFaultCode;
  uint32_t crc;
};

struct StoredFaultRecord {
  uint32_t sequence;
  uint32_t bootCount;
  uint32_t uptimeSeconds;
  uint32_t driveCycle;
  uint8_t code;
  uint8_t state;
  uint8_t flags;
  uint8_t reserved;
  uint32_t crc;
};

static_assert(sizeof(FaultLogMetadata) == 32U,
              "Fault log metadata EEPROM layout changed");
static_assert(sizeof(StoredFaultRecord) == 24U,
              "Fault log record EEPROM layout changed");

constexpr size_t METADATA_AREA_BYTES = 2U * sizeof(FaultLogMetadata);
constexpr uint8_t LOG_CAPACITY = static_cast<uint8_t>(
    (FAULT_LOG_REGION_BYTES - METADATA_AREA_BYTES) / sizeof(StoredFaultRecord));
constexpr uint8_t DRIVE_CYCLE_FLAG = 0x01U;

FaultLogMetadata metadata = {};
StoredFaultRecord records[LOG_CAPACITY] = {};
uint8_t recordCount = 0;
int8_t activeMetadataSlot = -1;
bool storageReady = false;
bool driveCycleActive = false;

uint32_t calculateCrc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
  }
  return ~crc;
}

bool sequenceNewer(uint32_t lhs, uint32_t rhs) {
  return static_cast<int32_t>(lhs - rhs) > 0;
}

int metadataAddress(uint8_t slot) {
  return FAULT_LOG_ADDRESS +
      static_cast<int>(slot * sizeof(FaultLogMetadata));
}

int recordAddress(uint8_t slot) {
  return FAULT_LOG_ADDRESS + static_cast<int>(METADATA_AREA_BYTES) +
      static_cast<int>(slot * sizeof(StoredFaultRecord));
}

bool validMetadata(const FaultLogMetadata &candidate) {
  if (candidate.magic != FAULT_LOG_MAGIC ||
      candidate.version != FAULT_LOG_VERSION ||
      candidate.nextSequence == 0U ||
      candidate.latchedFaultCode >
          static_cast<uint32_t>(BmsFaultCode::Other)) return false;
  return candidate.crc == calculateCrc32(
      reinterpret_cast<const uint8_t *>(&candidate),
      offsetof(FaultLogMetadata, crc));
}

bool readMetadata(uint8_t slot, FaultLogMetadata &out) {
  if (slot > 1U) return false;
  EEPROM.get(metadataAddress(slot), out);
  return validMetadata(out);
}

bool writeMetadata() {
  const uint8_t destination = activeMetadataSlot == 0 ? 1U : 0U;
  metadata.magic = FAULT_LOG_MAGIC;
  metadata.version = FAULT_LOG_VERSION;
  ++metadata.generation;
  metadata.crc = calculateCrc32(
      reinterpret_cast<const uint8_t *>(&metadata),
      offsetof(FaultLogMetadata, crc));
  EEPROM.put(metadataAddress(destination), metadata);

  FaultLogMetadata verify = {};
  if (!readMetadata(destination, verify) ||
      verify.generation != metadata.generation ||
      verify.driveCycle != metadata.driveCycle ||
      verify.nextSequence != metadata.nextSequence ||
      verify.bootCount != metadata.bootCount ||
      verify.latchedFaultCode != metadata.latchedFaultCode) return false;
  activeMetadataSlot = static_cast<int8_t>(destination);
  return true;
}

bool validRecord(const StoredFaultRecord &record) {
  if (record.code < static_cast<uint8_t>(BmsFaultCode::StackCommunication) ||
      record.code > static_cast<uint8_t>(BmsFaultCode::Other) ||
      record.state > static_cast<uint8_t>(BmsOperatingState::Fault)) {
    return false;
  }
  return record.crc == calculateCrc32(
      reinterpret_cast<const uint8_t *>(&record),
      offsetof(StoredFaultRecord, crc));
}

void sortRecordsOldestFirst() {
  for (uint8_t i = 1U; i < recordCount; ++i) {
    const StoredFaultRecord item = records[i];
    uint8_t j = i;
    while (j > 0U && sequenceNewer(records[j - 1U].sequence,
                                   item.sequence)) {
      records[j] = records[j - 1U];
      --j;
    }
    records[j] = item;
  }
}

void loadRecords() {
  recordCount = 0;
  for (uint8_t slot = 0; slot < LOG_CAPACITY; ++slot) {
    StoredFaultRecord candidate = {};
    EEPROM.get(recordAddress(slot), candidate);
    if (!validRecord(candidate)) continue;
    if (recordCount < LOG_CAPACITY) records[recordCount++] = candidate;
  }
  sortRecordsOldestFirst();
  if (recordCount > 0U) {
    const uint32_t newestSequence = records[recordCount - 1U].sequence;
    if (sequenceNewer(newestSequence, metadata.nextSequence - 1U)) {
      metadata.nextSequence = newestSequence + 1U;
    }
  }
}
}  // namespace

void faultLogBegin() {
  storageReady = settingsCanUseEepromRange(FAULT_LOG_ADDRESS,
                                           FAULT_LOG_REGION_BYTES);
  recordCount = 0;
  activeMetadataSlot = -1;
  driveCycleActive = false;
  if (!storageReady) return;

  FaultLogMetadata first = {};
  FaultLogMetadata second = {};
  const bool firstValid = readMetadata(0U, first);
  const bool secondValid = readMetadata(1U, second);
  if (firstValid || secondValid) {
    const bool chooseSecond = secondValid &&
        (!firstValid || sequenceNewer(second.generation, first.generation));
    metadata = chooseSecond ? second : first;
    activeMetadataSlot = chooseSecond ? 1 : 0;
  } else {
    metadata = {};
    metadata.nextSequence = 1U;
    metadata.bootCount = 0U;
  }

  ++metadata.bootCount;
  if (metadata.bootCount == 0U) metadata.bootCount = 1U;
  loadRecords();
  writeMetadata();
}

bool faultLogAvailable() {
  return storageReady;
}

uint8_t faultLogCount() {
  return recordCount;
}

uint32_t faultLogCurrentDriveCycle() {
  return metadata.driveCycle;
}

bool faultLogHasPersistentFault() {
  return storageReady && metadata.latchedFaultCode !=
      static_cast<uint32_t>(BmsFaultCode::None);
}

BmsFaultCode faultLogPersistentFaultCode() {
  return faultLogHasPersistentFault()
      ? static_cast<BmsFaultCode>(metadata.latchedFaultCode)
      : BmsFaultCode::None;
}

bool faultLogSetPersistentFault(BmsFaultCode code) {
  if (!storageReady || code == BmsFaultCode::None ||
      static_cast<uint8_t>(code) >
          static_cast<uint8_t>(BmsFaultCode::Other)) return false;
  if (metadata.latchedFaultCode == static_cast<uint32_t>(code)) return true;
  const uint32_t previous = metadata.latchedFaultCode;
  metadata.latchedFaultCode = static_cast<uint32_t>(code);
  if (writeMetadata()) return true;
  metadata.latchedFaultCode = previous;
  return false;
}

bool faultLogClearPersistentFault() {
  if (!storageReady) return false;
  if (metadata.latchedFaultCode ==
      static_cast<uint32_t>(BmsFaultCode::None)) return true;
  const uint32_t previous = metadata.latchedFaultCode;
  metadata.latchedFaultCode = static_cast<uint32_t>(BmsFaultCode::None);
  if (writeMetadata()) return true;
  metadata.latchedFaultCode = previous;
  return false;
}

bool faultLogReadNewest(uint8_t newestIndex, FaultLogEntry &entry) {
  if (!storageReady || newestIndex >= recordCount) return false;
  const StoredFaultRecord &stored = records[recordCount - 1U - newestIndex];
  entry.sequence = stored.sequence;
  entry.bootCount = stored.bootCount;
  entry.uptimeSeconds = stored.uptimeSeconds;
  entry.driveCycle = stored.driveCycle;
  entry.code = static_cast<BmsFaultCode>(stored.code);
  entry.state = static_cast<BmsOperatingState>(stored.state);
  entry.occurredDuringDriveCycle = (stored.flags & DRIVE_CYCLE_FLAG) != 0U;
  entry.belongsToCurrentDriveCycle = driveCycleActive &&
      stored.driveCycle != 0U && stored.driveCycle == metadata.driveCycle;
  return true;
}

const char *faultLogCodeName(BmsFaultCode code) {
  switch (code) {
    case BmsFaultCode::None: return "No latched fault";
    case BmsFaultCode::StackCommunication: return "Stack communication";
    case BmsFaultCode::StackRecoveryFailed: return "Stack recovery failed";
    case BmsFaultCode::CellDataStale: return "Cell data stale";
    case BmsFaultCode::CellDataInvalid: return "Cell data invalid";
    case BmsFaultCode::CellOverVoltage: return "Cell over-voltage";
    case BmsFaultCode::CellUnderVoltage: return "Cell under-voltage";
    case BmsFaultCode::CurrentSensor: return "Current sensor";
    case BmsFaultCode::PrechargeOutput: return "Precharge output";
    case BmsFaultCode::PrechargeTimeout: return "Precharge timeout";
    case BmsFaultCode::ContactorOutput: return "Contactor output";
    case BmsFaultCode::Charger: return "Charger";
    case BmsFaultCode::IsolationMonitor: return "Isolation monitor";
    case BmsFaultCode::Other: return "Other fault";
    default: return "Unknown fault";
  }
}

void faultLogStartDriveCycle() {
  if (!storageReady) return;
  ++metadata.driveCycle;
  if (metadata.driveCycle == 0U) metadata.driveCycle = 1U;
  driveCycleActive = true;
  writeMetadata();
}

void faultLogSetDriveCycleActive(bool active) {
  driveCycleActive = active;
}

void faultLogRecord(BmsFaultCode code, BmsOperatingState state) {
  if (!storageReady) return;
  StoredFaultRecord record = {};
  record.sequence = metadata.nextSequence;
  record.bootCount = metadata.bootCount;
  record.uptimeSeconds = millis() / 1000UL;
  record.driveCycle = metadata.driveCycle;
  record.code = static_cast<uint8_t>(code);
  record.state = static_cast<uint8_t>(state);
  record.flags = driveCycleActive ? DRIVE_CYCLE_FLAG : 0U;
  record.crc = calculateCrc32(reinterpret_cast<const uint8_t *>(&record),
                              offsetof(StoredFaultRecord, crc));
  const uint8_t slot = static_cast<uint8_t>(record.sequence % LOG_CAPACITY);
  EEPROM.put(recordAddress(slot), record);
  StoredFaultRecord verify = {};
  EEPROM.get(recordAddress(slot), verify);
  if (!validRecord(verify) || verify.sequence != record.sequence) return;

  metadata.nextSequence = record.sequence + 1U;
  if (metadata.nextSequence == 0U) metadata.nextSequence = 1U;
  writeMetadata();

  if (recordCount < LOG_CAPACITY) {
    records[recordCount++] = record;
  } else {
    for (uint8_t i = 1U; i < LOG_CAPACITY; ++i) records[i - 1U] = records[i];
    records[LOG_CAPACITY - 1U] = record;
  }
  sortRecordsOldestFirst();
}

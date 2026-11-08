/*
  BQ79616 stack communication and sequential ADC scanner.

  This module owns Serial1, the WAKE pulse, DIR0 addressing/role setup, the
  previously verified manual base-COMH override, ADC/TSREF preparation, and
  register reads for stack addresses 1..N. It validates UART frames with
  CRC16-Modbus. Scans advance through one asynchronous register transaction
  at a time so UART waits and ADC conversion waits do not block the main loop.
  Cell results are read every scan; GPIO/temperature blocks use their own
  slower configurable interval and cached values between refreshes.
*/
#include "batterycomms.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {
// Nano R4 Serial1 routing and BQ link timing used by this installation.
constexpr uint8_t BASE_ADDRESS = 0;
constexpr uint8_t BQ_RX_PIN = 0;
constexpr uint8_t BQ_TX_PIN = 1;
constexpr uint32_t BQ_BAUD = 1000000UL;
constexpr uint16_t RESPONSE_TIMEOUT_MS = 200;
constexpr uint16_t ADC_READY_TIMEOUT_MS = 100;
constexpr uint16_t RX_QUIET_MS = 4;
constexpr uint16_t RX_QUIET_MAX_MS = 35;
// Reuse only a very recent quiet interval already proven after the prior frame.
constexpr uint16_t QUIET_REUSE_MAX_AGE_MS = 20;
constexpr uint16_t NO_REPLY_GAP_MS = 5;
constexpr uint16_t WAKE_PULSE_US = 2500;
constexpr uint16_t WAKE_PROPAGATION_MS = 250;
constexpr size_t RX_BUFFER_SIZE = 128;
constexpr size_t MAX_DATA_BYTES = 32;
constexpr size_t ASYNC_RESPONSE_SIZE = MAX_DATA_BYTES + 6U;

// Physical balancing remains locked until the fitted resistor wattage and
// exact CB resistor path are confirmed from the module or its schematic.
// A 2 x 50-ohm path dissipates about 0.176 W total per cell at 4.2 V.
constexpr bool BALANCING_HARDWARE_CONFIRMED = false;

// BQ79616 register addresses used by startup and the monitor.
constexpr uint16_t DEV_CONF = 0x0002;
constexpr uint16_t DIR0_ADDR = 0x0306;
constexpr uint16_t COMM_CTRL = 0x0308;
constexpr uint16_t CONTROL1 = 0x0309;
constexpr uint16_t CONTROL2 = 0x030A;
constexpr uint16_t ADC_CTRL1 = 0x030D;
constexpr uint16_t GPIO_CONF1 = 0x000E;
constexpr uint16_t PARTID = 0x0500;
constexpr uint16_t ADC_STAT1 = 0x0527;
constexpr uint16_t BAL_STAT = 0x052B;
constexpr uint8_t BAL_STAT_INVALID_CBCONF = 0x80U;
constexpr uint8_t BAL_STAT_CB_RUN = 0x08U;
constexpr uint16_t TSREF_HI = 0x058C;
constexpr uint16_t GPIO1_HI = 0x058E;
constexpr uint16_t VCELL1_HI = 0x0586;
constexpr uint16_t DEBUG_COMM_STAT = 0x0780;
constexpr uint16_t DEBUG_CTRL_UNLOCK = 0x0700;
constexpr uint16_t DEBUG_COMM_CTRL1 = 0x0701;
constexpr uint16_t DEBUG_COMM_CTRL2 = 0x0702;
constexpr uint16_t OTP_ECC_TEST = 0x034C;
constexpr uint16_t CB_CELL1_CTRL = 0x0327;
constexpr uint16_t BAL_CTRL2 = 0x032F;

// Field values used by the established forward DIR0 setup sequence.
constexpr uint8_t MULTIDROP_EN_MASK = 0x20;
constexpr uint8_t CONTROL1_ADDR_WR_DIR0 = 0x01;
constexpr uint8_t COMM_BASE = 0x00;
constexpr uint8_t COMM_STACK = 0x02;
constexpr uint8_t COMM_TOP = 0x03;
constexpr uint8_t ADC_MODE_MASK = 0x03;
constexpr uint8_t ADC_MODE_8RR = 0x01;
constexpr uint8_t ADC_MODE_CONTINUOUS = 0x02;
constexpr uint8_t ADC_GO = 0x04;
constexpr uint8_t ADC_READY = 0x01;
constexpr uint8_t TSREF_EN = 0x01;
constexpr uint8_t GPIO_SPI_EN = 0x40;
constexpr uint8_t GPIO_FAULT_IN_EN = 0x80;

constexpr uint8_t CMD_SINGLE_READ = 0x80;
constexpr uint8_t CMD_SINGLE_WRITE = 0x90;
constexpr uint8_t CMD_BROADCAST_READ = 0xC0;
constexpr uint8_t CMD_BROADCAST_WRITE = 0xD0;

constexpr float VCELL_LSB_MV = 0.19073f;
constexpr float GPIO_LSB_MV = 0.15259f;
constexpr float TSREF_LSB_MV = 0.16954f;
constexpr float NTC_T0_K = 298.15f;

uint8_t rxBuffer[RX_BUFFER_SIZE];
bool communicationReady = false;
bool scanActive = false;
void (*backgroundService)() = nullptr;
uint32_t lastBackgroundServiceAtMs = 0;
uint8_t configuredStackModules = 0;
uint8_t scanModuleIndex = 0;
bool scanTemperatureDue = true;
bool temperaturePollStarted = false;
uint32_t lastTemperaturePollAtMs = 0;

struct CachedGpioData {
  bool valid;
  uint8_t adcGpioMask;
  uint8_t validGpioMask;
  uint8_t ntcEstimateMask;
  uint16_t gpioRaw[BATTERY_GPIO_COUNT];
  float gpioMv[BATTERY_GPIO_COUNT];
  float gpioRatio[BATTERY_GPIO_COUNT];
  float temperatureC[BATTERY_GPIO_COUNT];
  float tsrefMv;
};
CachedGpioData gpioCache[BATTERY_MAX_STACK_MODULES] = {};
enum class ScanPhase : uint8_t {
  ReadAdcControl,
  ReadAdcStatus,
  StartAdc,
  WaitAdcReady,
  ReadCells,
  ReadGpio
};
ScanPhase scanPhase = ScanPhase::ReadAdcControl;
uint8_t scanAdcControl = 0;
uint8_t scanAdcStatus = 0;
uint8_t scanAdcStartValue = 0;
uint32_t scanAdcReadyDeadlineMs = 0;
uint32_t scanAdcNextPollMs = 0;

enum class AsyncState : uint8_t {
  Idle,
  WaitTxQuiet,
  WaitResponse,
  WaitResponseQuiet,
  WaitNoReply
};
enum class AsyncResult : uint8_t { Inactive, Pending, Success, Failure };
AsyncState asyncState = AsyncState::Idle;
uint8_t asyncFrame[7] = {};
uint8_t asyncResponse[ASYNC_RESPONSE_SIZE] = {};
size_t asyncFrameLength = 0;
size_t asyncExpectedLength = 0;
size_t asyncReceivedLength = 0;
uint32_t asyncStartedAtMs = 0;
uint32_t asyncTxAtMs = 0;
uint32_t asyncLastByteAtMs = 0;
uint16_t asyncTimeoutMs = RESPONSE_TIMEOUT_MS;
bool asyncLastByteSeen = false;
bool asyncUnexpectedByte = false;
bool uartLineQuietConfirmed = false;
uint32_t uartLineQuietConfirmedAtMs = 0;
bool asyncCanSkipTxQuiet = false;
bool scanTransactionPending = false;
enum class BalancePhase : uint8_t {
  Idle,
  StartTimer,
  StartGo,
  VerifyStart,
  StopTimer,
  StopGo,
  VerifyStop
};
BalancePhase balancePhase = BalancePhase::Idle;
bool balanceAsyncPending = false;
bool balancePulseActive = false;
bool balanceCandidateValid = false;
uint8_t balanceAddress = 0;
uint8_t balanceCell = 0;
uint8_t balanceCandidateAddress = 0;
uint8_t balanceCandidateCell = 0;
uint8_t balanceTimerCode = 0;
uint16_t balanceCandidateMv = 0;
uint32_t balancePulseStartedAtMs = 0;
uint32_t balancePulseMaxOnMs = 10000UL;
uint32_t balancePulseCoolOffMs = 60000UL;
uint32_t balanceCoolOffUntilMs = 0;
char balanceStatusText[64] = "disabled";
uint8_t gpioConfig[BATTERY_MAX_STACK_MODULES][4] = {};
uint8_t gpioAdcMasks[BATTERY_MAX_STACK_MODULES] = {};
bool gpioConfigReady[BATTERY_MAX_STACK_MODULES] = {};
bool tsrefReady[BATTERY_MAX_STACK_MODULES] = {};
char lastError[96] = "not initialized";

void copyText(char *destination, size_t capacity, const char *source) {
  if (!destination || !capacity) return;
  snprintf(destination, capacity, "%s", source ? source : "");
}

void setLastError(const char *message) {
  copyText(lastError, sizeof(lastError), message);
}

uint16_t crc16Modbus(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001U)
                       : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

void appendCrc(uint8_t *frame, size_t payloadLength) {
  const uint16_t crc = crc16Modbus(frame, payloadLength);
  frame[payloadLength] = static_cast<uint8_t>(crc & 0xFFU);
  frame[payloadLength + 1] = static_cast<uint8_t>(crc >> 8);
}

bool crcValid(const uint8_t *frame, size_t length) {
  return length >= 3 && crc16Modbus(frame, length) == 0;
}

void serviceBackground() {
  if (!backgroundService) return;
  const uint32_t now = millis();
  if (now - lastBackgroundServiceAtMs < 1U) return;
  lastBackgroundServiceAtMs = now;
  backgroundService();
}

void cooperativeDelay(uint16_t durationMs) {
  const uint32_t started = millis();
  while (millis() - started < durationMs) {
    serviceBackground();
    delay(1);
  }
}

void drainRx() {
  while (Serial1.available()) {
    if (Serial1.read() < 0) break;
  }
}

void waitForRxQuietAndDrain() {
  const uint32_t started = millis();
  uint32_t lastByteAt = started;
  while (millis() - started < RX_QUIET_MAX_MS &&
         millis() - lastByteAt < RX_QUIET_MS) {
    while (Serial1.available()) {
      if (Serial1.read() < 0) continue;
      lastByteAt = millis();
    }
    serviceBackground();
    delayMicroseconds(100);
  }
}

bool prepareTx() {
  uartLineQuietConfirmed = false;
  drainRx();
  waitForRxQuietAndDrain();
  return true;
}

bool sendNoReply(const uint8_t *frame, size_t length) {
  if (!frame || !length) return false;
  prepareTx();
  if (Serial1.write(frame, length) != length) return false;
  Serial1.flush();
  cooperativeDelay(NO_REPLY_GAP_MS);
  waitForRxQuietAndDrain();
  return true;
}

bool sendAndReceive(const uint8_t *frame, size_t frameLength,
                    size_t expectedBytes, uint8_t *response,
                    size_t responseCapacity, uint16_t timeoutMs) {
  if (!frame || !frameLength || !response || !expectedBytes ||
      expectedBytes > responseCapacity) {
    return false;
  }

  prepareTx();
  if (Serial1.write(frame, frameLength) != frameLength) return false;
  Serial1.flush();

  size_t received = 0;
  const uint32_t started = millis();
  while (millis() - started < timeoutMs && received < expectedBytes) {
    while (Serial1.available() && received < expectedBytes) {
      const int byteValue = Serial1.read();
      if (byteValue >= 0) response[received++] = static_cast<uint8_t>(byteValue);
    }
    serviceBackground();
    if (received < expectedBytes) delayMicroseconds(100);
  }

  if (received != expectedBytes) {
    waitForRxQuietAndDrain();
    return false;
  }
  waitForRxQuietAndDrain();
  return true;
}

bool writeRegister(uint8_t address, uint16_t reg, uint8_t value) {
  uint8_t frame[7] = {CMD_SINGLE_WRITE, address,
                      static_cast<uint8_t>(reg >> 8),
                      static_cast<uint8_t>(reg & 0xFFU), value, 0, 0};
  appendCrc(frame, 5);
  return sendNoReply(frame, sizeof(frame));
}

bool writeBroadcast(uint16_t reg, uint8_t value) {
  uint8_t frame[6] = {CMD_BROADCAST_WRITE,
                      static_cast<uint8_t>(reg >> 8),
                      static_cast<uint8_t>(reg & 0xFFU), value, 0, 0};
  appendCrc(frame, 4);
  return sendNoReply(frame, sizeof(frame));
}

bool readBlock(uint8_t address, uint16_t reg, uint8_t dataLength,
               uint8_t *data) {
  // BQ single-read replies contain a length byte, address, register, payload
  // and CRC; reject malformed frames before copying register data to callers.
  if (!data || !dataLength || dataLength > MAX_DATA_BYTES) return false;
  uint8_t frame[7] = {CMD_SINGLE_READ, address,
                      static_cast<uint8_t>(reg >> 8),
                      static_cast<uint8_t>(reg & 0xFFU),
                      static_cast<uint8_t>(dataLength - 1U), 0, 0};
  appendCrc(frame, 5);

  uint8_t response[MAX_DATA_BYTES + 6U] = {};
  const size_t responseLength = static_cast<size_t>(dataLength) + 6U;
  if (!sendAndReceive(frame, sizeof(frame), responseLength, response,
                      sizeof(response), RESPONSE_TIMEOUT_MS)) {
    return false;
  }
  if (response[0] != static_cast<uint8_t>(dataLength - 1U) ||
      response[1] != address ||
      response[2] != static_cast<uint8_t>(reg >> 8) ||
      response[3] != static_cast<uint8_t>(reg & 0xFFU) ||
      !crcValid(response, responseLength)) {
    return false;
  }
  memcpy(data, &response[4], dataLength);
  return true;
}

bool readRegister(uint8_t address, uint16_t reg, uint8_t &value) {
  return readBlock(address, reg, 1, &value);
}

bool beginAsyncRequest(const uint8_t *frame, size_t frameLength,
                       size_t expectedLength, uint16_t timeoutMs) {
  if (!frame || !frameLength || frameLength > sizeof(asyncFrame) ||
      expectedLength > sizeof(asyncResponse) || asyncState != AsyncState::Idle) {
    return false;
  }
  memcpy(asyncFrame, frame, frameLength);
  asyncFrameLength = frameLength;
  asyncExpectedLength = expectedLength;
  asyncReceivedLength = 0;
  asyncTimeoutMs = timeoutMs;
  asyncStartedAtMs = millis();
  asyncCanSkipTxQuiet = uartLineQuietConfirmed &&
      asyncStartedAtMs - uartLineQuietConfirmedAtMs <=
          QUIET_REUSE_MAX_AGE_MS &&
      Serial1.available() == 0;
  uartLineQuietConfirmed = false;
  asyncTxAtMs = 0;
  asyncLastByteAtMs = 0;
  asyncLastByteSeen = false;
  asyncUnexpectedByte = false;
  asyncState = AsyncState::WaitTxQuiet;
  return true;
}

bool beginAsyncRead(uint8_t address, uint16_t reg, uint8_t dataLength) {
  if (!dataLength || dataLength > MAX_DATA_BYTES) return false;
  uint8_t frame[7] = {CMD_SINGLE_READ, address,
                      static_cast<uint8_t>(reg >> 8),
                      static_cast<uint8_t>(reg & 0xFFU),
                      static_cast<uint8_t>(dataLength - 1U), 0, 0};
  appendCrc(frame, 5);
  return beginAsyncRequest(frame, sizeof(frame),
                           static_cast<size_t>(dataLength) + 6U,
                           RESPONSE_TIMEOUT_MS);
}

bool beginAsyncWrite(uint8_t address, uint16_t reg, uint8_t value) {
  uint8_t frame[7] = {CMD_SINGLE_WRITE, address,
                      static_cast<uint8_t>(reg >> 8),
                      static_cast<uint8_t>(reg & 0xFFU), value, 0, 0};
  appendCrc(frame, 5);
  return beginAsyncRequest(frame, sizeof(frame), 0U, NO_REPLY_GAP_MS);
}

AsyncResult finishAsync(bool success) {
  const uint32_t now = millis();
  bool lineQuiet = false;
  if (success && asyncState == AsyncState::WaitResponseQuiet) {
    lineQuiet = true;  // This state finishes only after a full quiet interval.
  } else if (success && asyncState == AsyncState::WaitNoReply) {
    lineQuiet = asyncLastByteSeen
        ? now - asyncLastByteAtMs >= RX_QUIET_MS
        : now - asyncTxAtMs >= RX_QUIET_MS;
  }
  uartLineQuietConfirmed = lineQuiet;
  if (lineQuiet) uartLineQuietConfirmedAtMs = now;
  asyncState = AsyncState::Idle;
  return success ? AsyncResult::Success : AsyncResult::Failure;
}

AsyncResult pollAsyncRequest() {
  if (asyncState == AsyncState::Idle) return AsyncResult::Inactive;

  uint32_t now = millis();
  if (asyncState == AsyncState::WaitTxQuiet) {
    while (Serial1.available()) {
      if (Serial1.read() >= 0) {
        asyncLastByteAtMs = millis();
        asyncLastByteSeen = true;
        asyncCanSkipTxQuiet = false;
      }
    }
    now = millis();
    const bool quiet = asyncCanSkipTxQuiet
        ? true
        : (asyncLastByteSeen
        ? now - asyncLastByteAtMs >= RX_QUIET_MS
        : now - asyncStartedAtMs >= RX_QUIET_MS);
    if (!quiet && now - asyncStartedAtMs < RX_QUIET_MAX_MS) {
      return AsyncResult::Pending;
    }
    if (Serial1.write(asyncFrame, asyncFrameLength) != asyncFrameLength) {
      return finishAsync(false);
    }
    asyncTxAtMs = millis();
    asyncState = asyncExpectedLength ? AsyncState::WaitResponse
                                     : AsyncState::WaitNoReply;
    return AsyncResult::Pending;
  }

  if (asyncState == AsyncState::WaitResponse) {
    while (Serial1.available()) {
      const int byteValue = Serial1.read();
      if (byteValue < 0) continue;
      asyncLastByteAtMs = millis();
      asyncLastByteSeen = true;
      if (asyncReceivedLength < asyncExpectedLength) {
        asyncResponse[asyncReceivedLength++] = static_cast<uint8_t>(byteValue);
      } else {
        asyncUnexpectedByte = true;
      }
    }
    if (asyncReceivedLength == asyncExpectedLength) {
      asyncState = AsyncState::WaitResponseQuiet;
      return AsyncResult::Pending;
    }
    if (millis() - asyncTxAtMs >= asyncTimeoutMs) {
      return finishAsync(false);
    }
    return AsyncResult::Pending;
  }

  if (asyncState == AsyncState::WaitResponseQuiet) {
    while (Serial1.available()) {
      if (Serial1.read() >= 0) {
        asyncLastByteAtMs = millis();
        asyncLastByteSeen = true;
        asyncUnexpectedByte = true;
      }
    }
    now = millis();
    if (asyncLastByteSeen && now - asyncLastByteAtMs >= RX_QUIET_MS) {
      return finishAsync(!asyncUnexpectedByte);
    }
    if (now - asyncTxAtMs >= asyncTimeoutMs) return finishAsync(false);
    return AsyncResult::Pending;
  }

  // Single writes have no response. Observe the minimum inter-frame gap and a
  // short RX-quiet interval without making the main loop wait.
  while (Serial1.available()) {
    if (Serial1.read() >= 0) {
      asyncLastByteAtMs = millis();
      asyncLastByteSeen = true;
    }
  }
  now = millis();
  const bool quiet = asyncLastByteSeen
      ? now - asyncLastByteAtMs >= RX_QUIET_MS
      : now - asyncTxAtMs >= RX_QUIET_MS;
  if (now - asyncTxAtMs >= asyncTimeoutMs && quiet) {
    return finishAsync(true);
  }
  if (now - asyncTxAtMs >= RX_QUIET_MAX_MS) return finishAsync(true);
  return AsyncResult::Pending;
}

bool asyncReadPayload(uint8_t address, uint16_t reg, uint8_t dataLength,
                      const uint8_t *&payload) {
  const size_t responseLength = static_cast<size_t>(dataLength) + 6U;
  if (!dataLength || asyncReceivedLength != responseLength ||
      asyncResponse[0] != static_cast<uint8_t>(dataLength - 1U) ||
      asyncResponse[1] != address ||
      asyncResponse[2] != static_cast<uint8_t>(reg >> 8) ||
      asyncResponse[3] != static_cast<uint8_t>(reg & 0xFFU) ||
      !crcValid(asyncResponse, responseLength)) {
    return false;
  }
  payload = &asyncResponse[4];
  return true;
}

bool dummyBroadcastRead(uint8_t totalDevices) {
  // TI DLL synchronization/readback step. Validate every expected response
  // frame and address before declaring the daisy-chain link ready.
  uint8_t frame[6] = {CMD_BROADCAST_READ,
                      static_cast<uint8_t>(OTP_ECC_TEST >> 8),
                      static_cast<uint8_t>(OTP_ECC_TEST & 0xFFU),
                      0, 0, 0};
  appendCrc(frame, 4);

  const size_t expectedBytes = static_cast<size_t>(totalDevices) * 7U;
  uint8_t response[RX_BUFFER_SIZE] = {};
  if (!sendAndReceive(frame, sizeof(frame), expectedBytes, response,
                      sizeof(response), RESPONSE_TIMEOUT_MS)) {
    return false;
  }

  bool seen[13] = {};
  for (uint8_t index = 0; index < totalDevices; ++index) {
    const uint8_t *entry = &response[static_cast<size_t>(index) * 7U];
    if (entry[0] != 0 || entry[2] != static_cast<uint8_t>(OTP_ECC_TEST >> 8) ||
        entry[3] != static_cast<uint8_t>(OTP_ECC_TEST & 0xFFU) ||
        !crcValid(entry, 7)) {
      return false;
    }
    if (entry[1] >= totalDevices || seen[entry[1]]) return false;
    seen[entry[1]] = true;
  }
  for (uint8_t address = 0; address < totalDevices; ++address) {
    if (!seen[address]) return false;
  }
  return true;
}

void wakeBase() {
  // WAKE is generated on host TX while Serial1 is temporarily detached. RX
  // remains an input; the BQ base propagates the wake tone through the stack.
  Serial1.flush();
  drainRx();
  Serial1.end();
  pinMode(BQ_TX_PIN, OUTPUT);
  digitalWrite(BQ_TX_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(BQ_TX_PIN, LOW);
  delayMicroseconds(WAKE_PULSE_US);
  digitalWrite(BQ_TX_PIN, HIGH);
  cooperativeDelay(12);
  pinMode(BQ_TX_PIN, INPUT_PULLUP);
  Serial1.begin(BQ_BAUD);
  cooperativeDelay(100);
  drainRx();
}

bool ensureDaisyMode(bool &changed) {
  changed = false;
  uint8_t conf = 0;
  if (!readRegister(BASE_ADDRESS, DEV_CONF, conf)) {
    setLastError("base DEV_CONF read failed after WAKE");
    return false;
  }
  if (conf & MULTIDROP_EN_MASK) {
    const uint8_t desired = static_cast<uint8_t>(conf & ~MULTIDROP_EN_MASK);
    if (!writeRegister(BASE_ADDRESS, DEV_CONF, desired)) {
      setLastError("could not clear DEV_CONF.MULTIDROP_EN");
      return false;
    }
    cooperativeDelay(2);
    uint8_t verify = 0;
    if (!readRegister(BASE_ADDRESS, DEV_CONF, verify) ||
        (verify & MULTIDROP_EN_MASK)) {
      setLastError("DEV_CONF daisy-chain mode did not verify");
      return false;
    }
    changed = true;
  }
  return true;
}

bool configureAddressesAndRoles(uint8_t stackModules) {
  const uint8_t totalDevices = static_cast<uint8_t>(stackModules + 1U);
  const uint8_t topAddress = stackModules;

  // DIR_SEL=0 is the previously verified forward direction using base COMH.
  if (!writeRegister(BASE_ADDRESS, CONTROL1, 0x00)) {
    setLastError("could not select forward DIR_SEL=0");
    return false;
  }

  // Reopen the prior top transmitter before broadcasting a fresh address pass.
  // The current stack count is the previously successful ToS address (9).
  if (!writeRegister(topAddress, COMM_CTRL, COMM_STACK)) {
    setLastError("could not release the previous top-of-stack role");
    return false;
  }
  if (!writeBroadcast(OTP_ECC_TEST, 0x00)) {
    setLastError("TI DLL dummy broadcast write failed");
    return false;
  }
  if (!writeBroadcast(CONTROL1, CONTROL1_ADDR_WR_DIR0)) {
    setLastError("DIR0 auto-address enable broadcast failed");
    return false;
  }
  for (uint8_t address = 0; address < totalDevices; ++address) {
    if (!writeBroadcast(DIR0_ADDR, address)) {
      setLastError("DIR0 address assignment broadcast failed");
      return false;
    }
  }
  if (!writeBroadcast(COMM_CTRL, COMM_STACK) ||
      !writeRegister(BASE_ADDRESS, COMM_CTRL, COMM_BASE) ||
      !writeRegister(topAddress, COMM_CTRL, COMM_TOP)) {
    setLastError("stack/base/top role assignment failed");
    return false;
  }
  return true;
}

bool enableManualBaseComh() {
  // Preserve the known-good base COMH TX/RX override. These are volatile
  // debug-register writes, not customer OTP programming.
  if (!writeRegister(BASE_ADDRESS, DEBUG_CTRL_UNLOCK, 0xA5) ||
      !writeRegister(BASE_ADDRESS, DEBUG_COMM_CTRL2, 0x03) ||
      !writeRegister(BASE_ADDRESS, DEBUG_COMM_CTRL1, 0x05)) {
    setLastError("base COMH manual override writes failed");
    return false;
  }
  uint8_t status = 0;
  if (!readRegister(BASE_ADDRESS, DEBUG_COMM_STAT, status) ||
      (status & 0x1FU) != 0x03U) {
    setLastError("base DEBUG_COMM_STAT did not confirm COMH TX/RX override");
    return false;
  }
  return true;
}

bool cacheGpioConfigAndEnableTsref(uint8_t moduleIndex,
                                   uint8_t address) {
  // Preserve the device's existing GPIO modes. Only channels configured for
  // ADC use are scanned; bus/fault-input functions are not reconfigured here.
  uint8_t config[4] = {};
  if (!readBlock(address, GPIO_CONF1, sizeof(config), config)) {
    gpioConfigReady[moduleIndex] = false;
    gpioAdcMasks[moduleIndex] = 0;
    return false;
  }
  memcpy(gpioConfig[moduleIndex], config, sizeof(config));
  gpioConfigReady[moduleIndex] = true;

  uint8_t adcMask = 0;
  for (uint8_t gpio = 1; gpio <= BATTERY_GPIO_COUNT; ++gpio) {
    const uint8_t configByte = config[(gpio - 1U) / 2U];
    const uint8_t shift = static_cast<uint8_t>(((gpio - 1U) % 2U) * 3U);
    const uint8_t mode = static_cast<uint8_t>((configByte >> shift) & 0x07U);
    bool adcEnabled = mode == 0x01 || mode == 0x02 || mode == 0x06 || mode == 0x07;
    if (gpio >= 4 && gpio <= 7 && (config[0] & GPIO_SPI_EN)) adcEnabled = false;
    if (gpio == 8 && (config[0] & GPIO_FAULT_IN_EN)) adcEnabled = false;
    if (adcEnabled) adcMask |= static_cast<uint8_t>(1U << (gpio - 1U));
  }
  gpioAdcMasks[moduleIndex] = adcMask;
  tsrefReady[moduleIndex] = false;
  if (!adcMask) return true;

  uint8_t control2 = 0;
  if (!readRegister(address, CONTROL2, control2)) return false;
  if (!(control2 & TSREF_EN)) {
    // CONTROL2 bit 1 is SEND_HW_RESET. Write only TSREF_EN, keeping reset low.
    if (!writeRegister(address, CONTROL2, TSREF_EN)) return false;
    cooperativeDelay(2);  // TI requires at least 1.35 ms TSREF startup time.
    if (!readRegister(address, CONTROL2, control2) || !(control2 & TSREF_EN)) {
      return false;
    }
  }
  tsrefReady[moduleIndex] = true;
  return true;
}

int32_t signedAdcCounts(uint16_t raw) {
  return (raw & 0x8000U) ? static_cast<int32_t>(raw) - 0x10000L
                          : static_cast<int32_t>(raw);
}

void setModuleError(ModuleReading &module, const char *message) {
  if (module.error[0] == '\0') {
    snprintf(module.error, sizeof(module.error), "%s", message);
  }
}

bool parseCellResults(const uint8_t *data, ModuleReading &module) {
  // Convert the twelve adjacent HI/LO cell-result pairs to millivolts.
  // 0x8000 is the BQ invalid-result sentinel.
  constexpr uint16_t FIRST_CELL_HI = VCELL1_HI - 2U * (BATTERY_CELLS_PER_MODULE - 1U);
  (void)FIRST_CELL_HI;
  if (!data) return false;

  module.minCellMv = 1000000.0f;
  module.maxCellMv = -1000000.0f;
  module.validCellCount = 0;
  for (uint8_t cell = 0; cell < BATTERY_CELLS_PER_MODULE; ++cell) {
    const uint16_t cellHi = static_cast<uint16_t>(
        VCELL1_HI - 2U * cell);
    const uint8_t offset = static_cast<uint8_t>(cellHi - FIRST_CELL_HI);
    const uint16_t raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1U]);
    module.cellRaw[cell] = raw;
    if (raw == 0x8000U) continue;

    const float millivolts = signedAdcCounts(raw) * VCELL_LSB_MV;
    module.cellMv[cell] = millivolts;
    if (millivolts < module.minCellMv) module.minCellMv = millivolts;
    if (millivolts > module.maxCellMv) module.maxCellMv = millivolts;
    ++module.validCellCount;
  }
  if (!module.validCellCount) return false;
  module.deltaCellMv = module.maxCellMv - module.minCellMv;
  return module.validCellCount == BATTERY_CELLS_PER_MODULE;
}

bool parseGpioResults(uint8_t moduleIndex, const BmsSettings &settings,
                      ModuleReading &module, const uint8_t *data) {
  // Derive GPIO volts and (for enabled NTC modes) provisional temperatures
  // from the already-read TSREF and GPIO result block.
  module.adcGpioMask = gpioAdcMasks[moduleIndex];
  if (!gpioConfigReady[moduleIndex]) return false;
  if (!module.adcGpioMask) {
    module.gpioDataFresh = true;
    return true;
  }
  if (!tsrefReady[moduleIndex]) return false;
  if (!data) return false;

  const uint16_t tsrefRaw = static_cast<uint16_t>(
      (static_cast<uint16_t>(data[0]) << 8) | data[1]);
  if (tsrefRaw == 0x8000U) return false;
  const int32_t tsrefCounts = signedAdcCounts(tsrefRaw);
  if (tsrefCounts <= 0) return false;
  module.tsrefMv = tsrefCounts * TSREF_LSB_MV;

  bool allConfiguredChannelsValid = true;
  for (uint8_t gpio = 1; gpio <= BATTERY_GPIO_COUNT; ++gpio) {
    const uint8_t bit = static_cast<uint8_t>(1U << (gpio - 1U));
    if (!(module.adcGpioMask & bit)) continue;

    const uint16_t gpioHi = static_cast<uint16_t>(GPIO1_HI + 2U * (gpio - 1U));
    const uint8_t offset = static_cast<uint8_t>(gpioHi - TSREF_HI);
    const uint16_t raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1U]);
    module.gpioRaw[gpio - 1U] = raw;
    if (raw == 0x8000U) {
      allConfiguredChannelsValid = false;
      continue;
    }

    const int32_t counts = signedAdcCounts(raw);
    module.validGpioMask |= bit;
    module.gpioMv[gpio - 1U] = counts * GPIO_LSB_MV;
    const float ratio = (static_cast<float>(counts) * GPIO_LSB_MV) /
                        (static_cast<float>(tsrefCounts) * TSREF_LSB_MV);
    module.gpioRatio[gpio - 1U] = ratio;

    const uint8_t configByte = gpioConfig[moduleIndex][(gpio - 1U) / 2U];
    const uint8_t shift = static_cast<uint8_t>(((gpio - 1U) % 2U) * 3U);
    const uint8_t mode = static_cast<uint8_t>((configByte >> shift) & 0x07U);
    if ((mode == 0x01 || mode == 0x02) && ratio >= settings.ntcRatioMin &&
        ratio <= settings.ntcRatioMax && settings.ntcPullupOhms > 0.0f &&
        settings.ntcR25Ohms > 0.0f && settings.ntcBetaKelvin > 0.0f) {
      const float resistance = settings.ntcPullupOhms * ratio / (1.0f - ratio);
      const float inverseKelvin = (1.0f / NTC_T0_K) +
          logf(resistance / settings.ntcR25Ohms) / settings.ntcBetaKelvin;
      if (resistance > 0.0f && inverseKelvin > 0.0f) {
        module.temperatureC[gpio - 1U] = (1.0f / inverseKelvin) - 273.15f;
        module.ntcEstimateMask |= bit;
      }
    }
  }
  module.gpioDataFresh = allConfiguredChannelsValid;
  return allConfiguredChannelsValid;
}

void loadCachedGpioData(uint8_t moduleIndex, ModuleReading &module) {
  const CachedGpioData &cached = gpioCache[moduleIndex];
  module.adcGpioMask = cached.adcGpioMask;
  module.validGpioMask = cached.validGpioMask;
  module.ntcEstimateMask = cached.ntcEstimateMask;
  memcpy(module.gpioRaw, cached.gpioRaw, sizeof(module.gpioRaw));
  memcpy(module.gpioMv, cached.gpioMv, sizeof(module.gpioMv));
  memcpy(module.gpioRatio, cached.gpioRatio, sizeof(module.gpioRatio));
  memcpy(module.temperatureC, cached.temperatureC,
         sizeof(module.temperatureC));
  module.tsrefMv = cached.tsrefMv;
  module.gpioReadOk = cached.valid;
  module.gpioDataFresh = false;
}

void saveCachedGpioData(uint8_t moduleIndex, const ModuleReading &module) {
  CachedGpioData &cached = gpioCache[moduleIndex];
  cached.valid = module.gpioReadOk;
  cached.adcGpioMask = module.adcGpioMask;
  cached.validGpioMask = module.validGpioMask;
  cached.ntcEstimateMask = module.ntcEstimateMask;
  memcpy(cached.gpioRaw, module.gpioRaw, sizeof(module.gpioRaw));
  memcpy(cached.gpioMv, module.gpioMv, sizeof(module.gpioMv));
  memcpy(cached.gpioRatio, module.gpioRatio, sizeof(module.gpioRatio));
  memcpy(cached.temperatureC, module.temperatureC,
         sizeof(module.temperatureC));
  cached.tsrefMv = module.tsrefMv;
}

void initializeScanModule(BatterySnapshot &snapshot) {
  if (scanModuleIndex >= snapshot.moduleCount) return;
  ModuleReading &module = snapshot.modules[scanModuleIndex];
  module.address = static_cast<uint8_t>(scanModuleIndex + 1U);
  module.cellReadOk = false;
  module.gpioReadOk = scanTemperatureDue
      ? false : gpioCache[scanModuleIndex].valid;
  module.gpioDataFresh = false;
  module.validCellCount = 0;
  if (scanTemperatureDue) {
    module.adcGpioMask = 0;
    module.validGpioMask = 0;
    module.ntcEstimateMask = 0;
  }
  module.error[0] = '\0';
  scanAdcControl = 0;
  scanAdcStatus = 0;
  scanPhase = ScanPhase::ReadAdcControl;
}

BatteryScanStepResult finishScanModule(BatterySnapshot &snapshot) {
  ModuleReading &module = snapshot.modules[scanModuleIndex];
  if (scanTemperatureDue) {
    if (module.gpioReadOk) {
      saveCachedGpioData(scanModuleIndex, module);
    } else if (gpioCache[scanModuleIndex].valid) {
      // Keep the last usable values visible if this slower refresh fails,
      // while preserving the failed-read flag for this scan.
      loadCachedGpioData(scanModuleIndex, module);
      module.gpioReadOk = false;
    }
  } else {
    module.gpioReadOk = gpioCache[scanModuleIndex].valid;
    module.gpioDataFresh = false;
  }
  snapshot.validCellCount = static_cast<uint16_t>(
      snapshot.validCellCount + module.validCellCount);
  if (!module.cellReadOk) snapshot.allCellReadsOk = false;
  if (!module.gpioReadOk) snapshot.allGpioReadsOk = false;
  ++snapshot.completedModules;
  ++scanModuleIndex;
  scanTransactionPending = false;

  if (scanModuleIndex >= snapshot.moduleCount) {
    snapshot.finishedAtMs = millis();
    snapshot.durationMs = snapshot.finishedAtMs - snapshot.startedAtMs;
    scanActive = false;
    return BatteryScanStepResult::Complete;
  }

  initializeScanModule(snapshot);
  return BatteryScanStepResult::InProgress;
}

BatteryScanStepResult failScanOperation(BatterySnapshot &snapshot,
                                        const char *detail = nullptr) {
  ModuleReading &module = snapshot.modules[scanModuleIndex];
  scanTransactionPending = false;
  if (scanPhase == ScanPhase::ReadCells) {
    module.cellReadOk = false;
    setModuleError(module, "cell ADC block read incomplete");
    scanPhase = ScanPhase::ReadGpio;
    return BatteryScanStepResult::InProgress;
  }
  if (scanPhase == ScanPhase::ReadGpio) {
    module.gpioReadOk = false;
    setModuleError(module, "GPIO/TSREF ADC read incomplete");
    return finishScanModule(snapshot);
  }

  if (detail) {
    setModuleError(module, detail);
  } else {
    switch (scanPhase) {
      case ScanPhase::ReadAdcControl:
        setModuleError(module, "ADC_CTRL1 read/response failed");
        break;
      case ScanPhase::ReadAdcStatus:
        setModuleError(module, "ADC_STAT1 read/response failed");
        break;
      case ScanPhase::StartAdc:
        setModuleError(module, "ADC_CTRL1 start write failed");
        break;
      case ScanPhase::WaitAdcReady:
        setModuleError(module, "ADC_STAT1 read/ADC_READY failed");
        break;
      case ScanPhase::ReadCells:
      case ScanPhase::ReadGpio:
        setModuleError(module, "unexpected ADC scan phase failure");
        break;
    }
  }
  return finishScanModule(snapshot);
}

bool launchScanOperation(ModuleReading &module, uint8_t moduleIndex,
                         const BmsSettings &settings) {
  if (scanPhase == ScanPhase::WaitAdcReady &&
      static_cast<int32_t>(millis() - scanAdcNextPollMs) < 0) {
    return true;  // Waiting between ADC status polls is not a failure.
  }

  bool started = false;
  switch (scanPhase) {
    case ScanPhase::ReadAdcControl:
      started = beginAsyncRead(module.address, ADC_CTRL1, 1);
      break;
    case ScanPhase::ReadAdcStatus:
    case ScanPhase::WaitAdcReady:
      started = beginAsyncRead(module.address, ADC_STAT1, 1);
      break;
    case ScanPhase::StartAdc:
      started = beginAsyncWrite(module.address, ADC_CTRL1, scanAdcStartValue);
      break;
    case ScanPhase::ReadCells:
      started = beginAsyncRead(module.address,
          static_cast<uint16_t>(VCELL1_HI -
              2U * (BATTERY_CELLS_PER_MODULE - 1U)),
          static_cast<uint8_t>(2U * BATTERY_CELLS_PER_MODULE));
      break;
    case ScanPhase::ReadGpio:
      module.adcGpioMask = gpioAdcMasks[moduleIndex];
      if (!gpioConfigReady[moduleIndex] ||
          (module.adcGpioMask && !tsrefReady[moduleIndex])) {
        return false;
      }
      if (!module.adcGpioMask) return true;
      started = beginAsyncRead(module.address, TSREF_HI, 18);
      break;
  }

  if (started) scanTransactionPending = true;
  return started;
}

BatteryScanStepResult scanStepInternal(BatterySnapshot &snapshot,
                                       const BmsSettings &settings) {
  // This state machine starts or services one UART transaction per call.
  // Receive waits and ADC conversion waits are timer-driven, so CAN and BMS
  // supervision continue to run while a BQ reply or conversion is pending.
  if (!communicationReady || !scanActive) return BatteryScanStepResult::NotReady;

  ModuleReading &module = snapshot.modules[scanModuleIndex];
  if (!scanTransactionPending) {
    if (scanPhase == ScanPhase::ReadGpio && !scanTemperatureDue) {
      module.gpioReadOk = gpioCache[scanModuleIndex].valid;
      module.gpioDataFresh = false;
      if (!module.gpioReadOk) {
        setModuleError(module, "no cached GPIO/TSREF ADC data");
      }
      return finishScanModule(snapshot);
    }
    if (scanPhase == ScanPhase::ReadGpio &&
        gpioConfigReady[scanModuleIndex] &&
        gpioAdcMasks[scanModuleIndex] == 0U) {
      module.adcGpioMask = 0;
      module.gpioReadOk = parseGpioResults(scanModuleIndex, settings, module,
                                           nullptr);
      if (!module.gpioReadOk) setModuleError(module, "GPIO/TSREF ADC read incomplete");
      return finishScanModule(snapshot);
    }
    if (!launchScanOperation(module, scanModuleIndex, settings)) {
      return failScanOperation(snapshot);
    }
    return BatteryScanStepResult::InProgress;
  }

  const AsyncResult io = pollAsyncRequest();
  if (io == AsyncResult::Pending) return BatteryScanStepResult::InProgress;
  scanTransactionPending = false;
  if (io != AsyncResult::Success) return failScanOperation(snapshot);

  const uint8_t *payload = nullptr;
  switch (scanPhase) {
    case ScanPhase::ReadAdcControl:
      if (!asyncReadPayload(module.address, ADC_CTRL1, 1, payload)) {
        return failScanOperation(snapshot);
      }
      scanAdcControl = payload[0];
      scanPhase = ScanPhase::ReadAdcStatus;
      break;

    case ScanPhase::ReadAdcStatus: {
      if (!asyncReadPayload(module.address, ADC_STAT1, 1, payload)) {
        return failScanOperation(snapshot);
      }
      scanAdcStatus = payload[0];
      const uint8_t mode = scanAdcControl & ADC_MODE_MASK;
      if (mode == 0 || mode == ADC_MODE_8RR) {
        scanAdcStartValue = static_cast<uint8_t>(
            (scanAdcControl & 0x18U) | ADC_MODE_8RR | ADC_GO);
        scanPhase = ScanPhase::StartAdc;
      } else if (mode == ADC_MODE_CONTINUOUS) {
        if (scanAdcStatus & ADC_READY) {
          scanPhase = ScanPhase::ReadCells;
        } else {
          scanAdcReadyDeadlineMs = millis() + ADC_READY_TIMEOUT_MS;
          scanAdcNextPollMs = millis() + 2U;
          scanPhase = ScanPhase::WaitAdcReady;
        }
      } else {
        return failScanOperation(snapshot, "unsupported ADC_CTRL1 mode");
      }
      break;
    }

    case ScanPhase::StartAdc:
      scanAdcReadyDeadlineMs = millis() + ADC_READY_TIMEOUT_MS;
      scanAdcNextPollMs = millis() + 2U;
      scanPhase = ScanPhase::WaitAdcReady;
      break;

    case ScanPhase::WaitAdcReady:
      if (!asyncReadPayload(module.address, ADC_STAT1, 1, payload)) {
        return failScanOperation(snapshot);
      }
      scanAdcStatus = payload[0];
      if (scanAdcStatus & ADC_READY) {
        scanPhase = ScanPhase::ReadCells;
      } else if (static_cast<int32_t>(millis() - scanAdcReadyDeadlineMs) >= 0) {
        return failScanOperation(snapshot,
                                 "ADC_READY timeout after 100 ms");
      } else {
        scanAdcNextPollMs = millis() + 2U;
      }
      break;

    case ScanPhase::ReadCells:
      if (!asyncReadPayload(module.address,
              static_cast<uint16_t>(VCELL1_HI -
                  2U * (BATTERY_CELLS_PER_MODULE - 1U)),
              static_cast<uint8_t>(2U * BATTERY_CELLS_PER_MODULE), payload)) {
        return failScanOperation(snapshot);
      }
      module.cellReadOk = parseCellResults(payload, module);
      if (!module.cellReadOk) setModuleError(module, "cell ADC block read incomplete");
      scanPhase = ScanPhase::ReadGpio;
      break;

    case ScanPhase::ReadGpio:
      if (module.adcGpioMask) {
        if (!asyncReadPayload(module.address, TSREF_HI, 18, payload)) {
          return failScanOperation(snapshot);
        }
      }
      module.gpioReadOk = parseGpioResults(scanModuleIndex, settings, module,
                                           module.adcGpioMask ? payload : nullptr);
      if (!module.gpioReadOk) setModuleError(module, "GPIO/TSREF ADC read incomplete");
      return finishScanModule(snapshot);
  }
  return BatteryScanStepResult::InProgress;
}

void setBalanceStatus(const char *message) {
  copyText(balanceStatusText, sizeof(balanceStatusText), message);
}

uint8_t balanceTimerEncoding(uint16_t seconds) {
  // TI CB_CELLx_CTRL timer encodings: 1=10 s, 2=30 s, 3=60 s.
  if (seconds == 30U) return 0x02U;
  if (seconds == 60U) return 0x03U;
  return 0x01U;
}

uint16_t effectiveBalanceOnSeconds(const BmsSettings &settings) {
  uint16_t seconds = settings.balancingMaxOnSeconds;
  if (settings.balancingConservativeLevel >= 1U && seconds > 30U) {
    seconds = 30U;
  }
  if (settings.balancingConservativeLevel >= 2U && seconds > 10U) {
    seconds = 10U;
  }
  return seconds;
}

uint32_t effectiveBalanceCoolOffMs(const BmsSettings &settings) {
  const uint8_t multiplier = settings.balancingConservativeLevel == 2U
      ? 4U : (settings.balancingConservativeLevel == 1U ? 2U : 1U);
  return settings.balancingCoolOffMs * multiplier;
}

uint16_t balanceCellControlRegister(uint8_t cell) {
  // CB_CELL1_CTRL is the highest register; subsequent cells descend by one.
  return static_cast<uint16_t>(CB_CELL1_CTRL - (cell - 1U));
}

bool beginBalanceWrite(uint16_t reg, uint8_t value) {
  if (!beginAsyncWrite(balanceAddress, reg, value)) return false;
  balanceAsyncPending = true;
  return true;
}

bool beginBalanceReadStatus() {
  if (!beginAsyncRead(balanceAddress, BAL_STAT, 1U)) return false;
  balanceAsyncPending = true;
  return true;
}

void failBalanceSequence() {
  balanceAsyncPending = false;
  balancePhase = BalancePhase::Idle;
  balancePulseActive = false;
  balanceCoolOffUntilMs = millis() + balancePulseCoolOffMs;
  setLastError("cell-balancing command/status verification failed");
  setBalanceStatus("ERROR: balancing command/status");
}

bool serviceBalanceSequence(const BmsSettings &settings, bool permitted) {
  const uint32_t now = millis();
  if (balanceAsyncPending) {
    const AsyncResult result = pollAsyncRequest();
    if (result == AsyncResult::Pending) return true;
    balanceAsyncPending = false;
    if (result != AsyncResult::Success) {
      failBalanceSequence();
      return false;
    }
    switch (balancePhase) {
      case BalancePhase::StartTimer:
        balancePhase = BalancePhase::StartGo;
        break;
      case BalancePhase::StartGo:
        balancePhase = BalancePhase::VerifyStart;
        break;
      case BalancePhase::VerifyStart: {
        const uint8_t *payload = nullptr;
        if (!asyncReadPayload(balanceAddress, BAL_STAT, 1U, payload) ||
            (payload[0] & BAL_STAT_INVALID_CBCONF) ||
            !(payload[0] & BAL_STAT_CB_RUN)) {
          failBalanceSequence();
          return false;
        }
        balancePhase = BalancePhase::Idle;
        balancePulseActive = true;
        balancePulseStartedAtMs = now;
        snprintf(balanceStatusText, sizeof(balanceStatusText),
                 "ACTIVE: addr %u cell %u",
                 static_cast<unsigned>(balanceAddress),
                 static_cast<unsigned>(balanceCell));
        break;
      }
      case BalancePhase::StopTimer:
        balancePhase = BalancePhase::StopGo;
        break;
      case BalancePhase::StopGo:
        balancePhase = BalancePhase::VerifyStop;
        break;
      case BalancePhase::VerifyStop: {
        const uint8_t *payload = nullptr;
        if (!asyncReadPayload(balanceAddress, BAL_STAT, 1U, payload) ||
            (payload[0] & BAL_STAT_CB_RUN)) {
          failBalanceSequence();
          return false;
        }
        balancePhase = BalancePhase::Idle;
        balancePulseActive = false;
        balanceCoolOffUntilMs = now + balancePulseCoolOffMs;
        setBalanceStatus("COOLING after balance pulse");
        break;
      }
      case BalancePhase::Idle:
        break;
    }
    return false;
  }

  if (balancePhase != BalancePhase::Idle) {
    if (balancePhase == BalancePhase::StartGo &&
        (!settings.cellBalancingEnabled || !permitted)) {
      // Do not start the timed CBFET after interlocks changed while the timer
      // register write was being sent.
      balancePhase = BalancePhase::StopTimer;
    }

    uint16_t reg = BAL_CTRL2;
    uint8_t value = 0x22U;  // FLTSTOP_EN plus BAL_GO; AUTO_BAL remains clear.
    switch (balancePhase) {
      case BalancePhase::StartTimer:
        reg = balanceCellControlRegister(balanceCell);
        value = balanceTimerCode;
        setBalanceStatus("STARTING timed balance pulse");
        break;
      case BalancePhase::StopTimer:
        reg = balanceCellControlRegister(balanceCell);
        value = 0x00U;
        setBalanceStatus("STOPPING balance pulse");
        break;
      case BalancePhase::StartGo:
        setBalanceStatus("STARTING timed balance pulse");
        break;
      case BalancePhase::VerifyStart:
        setBalanceStatus("VERIFYING BQ balance status");
        if (!beginBalanceReadStatus()) {
          failBalanceSequence();
          return false;
        }
        return true;
      case BalancePhase::StopGo:
        setBalanceStatus("STOPPING balance pulse");
        break;
      case BalancePhase::VerifyStop:
        setBalanceStatus("VERIFYING balance stopped");
        if (!beginBalanceReadStatus()) {
          failBalanceSequence();
          return false;
        }
        return true;
      case BalancePhase::Idle:
        return false;
    }
    if (!beginBalanceWrite(reg, value)) {
      failBalanceSequence();
      return false;
    }
    return true;
  }

  if (balancePulseActive) {
    if (!settings.cellBalancingEnabled || !permitted ||
        now - balancePulseStartedAtMs >= balancePulseMaxOnMs) {
      balancePhase = BalancePhase::StopTimer;
      setBalanceStatus("STOPPING balance pulse");
      return false;
    }
    snprintf(balanceStatusText, sizeof(balanceStatusText),
             "ACTIVE: addr %u cell %u",
             static_cast<unsigned>(balanceAddress),
             static_cast<unsigned>(balanceCell));
    return false;
  }

  if (!settings.cellBalancingEnabled) {
    setBalanceStatus("disabled in settings");
    return false;
  }
  if (!permitted) {
    setBalanceStatus("inhibited by BMS state/interlocks");
    return false;
  }
  if (!balanceCandidateValid) {
    setBalanceStatus("target met; no candidate cell");
    return false;
  }
  if (static_cast<int32_t>(now - balanceCoolOffUntilMs) < 0) {
    setBalanceStatus("COOLING after balance pulse");
    return false;
  }

  balanceAddress = balanceCandidateAddress;
  balanceCell = balanceCandidateCell;
  const uint16_t effectiveOnSeconds = effectiveBalanceOnSeconds(settings);
  balanceTimerCode = balanceTimerEncoding(effectiveOnSeconds);
  balancePulseMaxOnMs = static_cast<uint32_t>(effectiveOnSeconds) * 1000UL;
  balancePulseCoolOffMs = effectiveBalanceCoolOffMs(settings);
  balancePhase = BalancePhase::StartTimer;
  return serviceBalanceSequence(settings, permitted);
}

bool initializationFailed(char *error, size_t errorSize, const char *message) {
  communicationReady = false;
  scanActive = false;
  if (message != lastError) setLastError(message);
  copyText(error, errorSize, lastError);
  return false;
}
}

BatteryScanStepResult batteryCommsScanStep(BatterySnapshot &snapshot,
                                           const BmsSettings &settings) {
  return scanStepInternal(snapshot, settings);
}

bool batteryCommsBegin(const BmsSettings &settings, char *error,
                       size_t errorSize) {
  communicationReady = false;
  scanActive = false;
  scanTransactionPending = false;
  asyncState = AsyncState::Idle;
  balancePhase = BalancePhase::Idle;
  balanceAsyncPending = false;
  balancePulseActive = false;
  balanceCandidateValid = false;
  setBalanceStatus("waiting for a valid full-stack scan");
  uartLineQuietConfirmed = false;
  asyncCanSkipTxQuiet = false;
  scanTemperatureDue = true;
  temperaturePollStarted = false;
  lastTemperaturePollAtMs = 0U;
  memset(gpioCache, 0, sizeof(gpioCache));
  memset(gpioConfig, 0, sizeof(gpioConfig));
  memset(gpioAdcMasks, 0, sizeof(gpioAdcMasks));
  memset(gpioConfigReady, 0, sizeof(gpioConfigReady));
  memset(tsrefReady, 0, sizeof(tsrefReady));
  copyText(error, errorSize, "");

  if (settings.stackModules == 0 ||
      settings.stackModules > BATTERY_MAX_STACK_MODULES) {
    return initializationFailed(error, errorSize, "stack module count out of range");
  }
  configuredStackModules = settings.stackModules;

  // Follow the hardware-verified startup order: WAKE, addresses/roles, manual
  // COMH override, then DEV_CONF and stack-response verification.
  Serial1.begin(BQ_BAUD);
  cooperativeDelay(100);
  wakeBase();
  cooperativeDelay(WAKE_PROPAGATION_MS);

  if (!configureAddressesAndRoles(configuredStackModules)) {
    return initializationFailed(error, errorSize, lastError);
  }
  if (!enableManualBaseComh()) {
    return initializationFailed(error, errorSize, lastError);
  }

  // Read DEV_CONF only after the known-good DIR0/role setup and manual COMH
  // override. Earlier experiments showed this base read can time out directly
  // after WAKE before the Run 33 startup sequence has completed.
  bool daisyModeChanged = false;
  if (!ensureDaisyMode(daisyModeChanged)) {
    return initializationFailed(error, errorSize, lastError);
  }
  if (daisyModeChanged &&
      !configureAddressesAndRoles(configuredStackModules)) {
    return initializationFailed(error, errorSize,
        "stack re-addressing failed after clearing MULTIDROP_EN");
  }

  const uint8_t totalDevices = static_cast<uint8_t>(configuredStackModules + 1U);
  if (!dummyBroadcastRead(totalDevices)) {
    return initializationFailed(error, errorSize,
        "DLL dummy read did not return valid frames from all addressed devices");
  }

  uint8_t partId = 0;
  uint8_t role = 0;
  uint8_t conf = 0;
  if (!readRegister(BASE_ADDRESS, PARTID, partId) || partId != 0x21 ||
      !readRegister(BASE_ADDRESS, COMM_CTRL, role) ||
      (role & 0x03U) != COMM_BASE ||
      !readRegister(BASE_ADDRESS, DEV_CONF, conf) ||
      (conf & MULTIDROP_EN_MASK)) {
    return initializationFailed(error, errorSize,
        "base identity, role, or daisy-chain mode check failed");
  }

  // Cache channel configuration and leave TSREF enabled on ADC-configured
  // stack modules so the slower GPIO/temperature poll can reuse it.
  for (uint8_t index = 0; index < configuredStackModules; ++index) {
    cacheGpioConfigAndEnableTsref(index, static_cast<uint8_t>(index + 1U));
  }

  communicationReady = true;
  scanActive = false;
  setLastError("");
  return true;
}

void batteryCommsSetBackgroundService(void (*service)()) {
  backgroundService = service;
  lastBackgroundServiceAtMs = millis() - 1U;
}

bool batteryCommsIsReady() {
  return communicationReady;
}

const char *batteryCommsLastError() {
  return lastError;
}

void batteryCommsStartScan(BatterySnapshot &snapshot,
                           const BmsSettings &settings) {
  const uint32_t now = millis();
  scanTemperatureDue = !temperaturePollStarted ||
      now - lastTemperaturePollAtMs >= settings.temperatureSamplePeriodMs;
  if (scanTemperatureDue) {
    temperaturePollStarted = true;
    lastTemperaturePollAtMs = now;
  }
  memset(&snapshot, 0, sizeof(snapshot));
  snapshot.moduleCount = settings.stackModules;
  snapshot.allCellReadsOk = true;
  snapshot.allGpioReadsOk = true;
  snapshot.startedAtMs = millis();
  scanModuleIndex = 0;
  scanActive = communicationReady && settings.stackModules > 0 &&
               settings.stackModules <= BATTERY_MAX_STACK_MODULES;
  scanTransactionPending = false;
  asyncState = AsyncState::Idle;
  if (scanActive && !scanTemperatureDue) {
    for (uint8_t index = 0; index < snapshot.moduleCount; ++index) {
      loadCachedGpioData(index, snapshot.modules[index]);
    }
  }
  if (scanActive) initializeScanModule(snapshot);
}

void batteryCommsCancelScan() {
  scanActive = false;
  scanTransactionPending = false;
  asyncState = AsyncState::Idle;
  uartLineQuietConfirmed = false;
  asyncCanSkipTxQuiet = false;
}

void batteryCommsBalanceObserveScan(const BatterySnapshot &snapshot,
                                    const BmsSettings &settings) {
  balanceCandidateValid = false;
  if (!snapshot.allCellReadsOk ||
      snapshot.moduleCount != settings.stackModules ||
      snapshot.validCellCount !=
          static_cast<uint16_t>(settings.stackModules) *
              settings.cellsPerModule) {
    return;
  }

  uint16_t lowestMv = 0xFFFFU;
  uint16_t highestMv = 0;
  uint8_t highestAddress = 0;
  uint8_t highestCell = 0;
  for (uint8_t moduleIndex = 0; moduleIndex < snapshot.moduleCount;
       ++moduleIndex) {
    const ModuleReading &module = snapshot.modules[moduleIndex];
    if (!module.cellReadOk || module.validCellCount != settings.cellsPerModule) {
      balanceCandidateValid = false;
      return;
    }
    for (uint8_t cellIndex = 0; cellIndex < settings.cellsPerModule;
         ++cellIndex) {
      if (module.cellRaw[cellIndex] == 0x8000U) return;
      const float cellMv = module.cellMv[cellIndex];
      if (!(cellMv > 0.0f && cellMv <= 5000.0f)) return;
      const uint16_t measuredMv = static_cast<uint16_t>(cellMv + 0.5f);
      if (measuredMv < lowestMv) lowestMv = measuredMv;
      if (measuredMv > highestMv) {
        highestMv = measuredMv;
        highestAddress = module.address;
        highestCell = static_cast<uint8_t>(cellIndex + 1U);
      }
    }
  }

  const uint16_t startDelta = static_cast<uint16_t>(
      settings.balancingTargetDeltaMv + settings.balancingHysteresisMv);
  if (lowestMv >= settings.balancingMinCellMv &&
      highestMv >= lowestMv && highestMv - lowestMv >= startDelta) {
    balanceCandidateAddress = highestAddress;
    balanceCandidateCell = highestCell;
    balanceCandidateMv = highestMv;
    balanceCandidateValid = highestAddress != 0U && highestCell != 0U;
  }
}

void batteryCommsBalanceService(const BmsSettings &settings, bool permitted) {
  if (!BALANCING_HARDWARE_CONFIRMED) {
    if (!settings.cellBalancingEnabled) {
      setBalanceStatus("disabled in settings");
    } else if (balanceCandidateValid) {
      snprintf(balanceStatusText, sizeof(balanceStatusText),
               "LOCKED: check resistor rating; candidate %u/%u at %u mV",
               static_cast<unsigned>(balanceCandidateAddress),
               static_cast<unsigned>(balanceCandidateCell),
               static_cast<unsigned>(balanceCandidateMv));
    } else {
      setBalanceStatus("LOCKED: check resistor rating; no candidate");
    }
    return;
  }
  if (!communicationReady) {
    setBalanceStatus("inhibited: BQ stack unavailable");
    return;
  }
  if (scanActive) return;
  serviceBalanceSequence(settings, permitted);
}

bool batteryCommsBalanceBusy() {
  return balanceAsyncPending || balancePhase != BalancePhase::Idle;
}

bool batteryCommsBalanceIsActive() {
  return balancePulseActive;
}

const char *batteryCommsBalanceStatus() {
  return balanceStatusText;
}

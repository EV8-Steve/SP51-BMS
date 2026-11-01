/*
  BQ79616 stack communication and sequential ADC scanner.

  This module owns Serial1, the WAKE pulse, DIR0 addressing/role setup, the
  previously verified manual base-COMH override, ADC/TSREF preparation, and
  register reads for stack addresses 1..N. It validates UART frames with
  CRC16-Modbus and advances scans one device at a time.
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
constexpr uint16_t NO_REPLY_GAP_MS = 5;
constexpr uint16_t WAKE_PULSE_US = 2500;
constexpr uint16_t WAKE_PROPAGATION_MS = 250;
constexpr size_t RX_BUFFER_SIZE = 128;
constexpr size_t MAX_DATA_BYTES = 32;

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
constexpr uint16_t TSREF_HI = 0x058C;
constexpr uint16_t GPIO1_HI = 0x058E;
constexpr uint16_t VCELL1_HI = 0x0586;
constexpr uint16_t DEBUG_COMM_STAT = 0x0780;
constexpr uint16_t DEBUG_CTRL_UNLOCK = 0x0700;
constexpr uint16_t DEBUG_COMM_CTRL1 = 0x0701;
constexpr uint16_t DEBUG_COMM_CTRL2 = 0x0702;
constexpr uint16_t OTP_ECC_TEST = 0x034C;

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
uint8_t configuredStackModules = 0;
uint8_t scanModuleIndex = 0;
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
    delayMicroseconds(100);
  }
}

bool prepareTx() {
  drainRx();
  waitForRxQuietAndDrain();
  return true;
}

bool sendNoReply(const uint8_t *frame, size_t length) {
  if (!frame || !length) return false;
  prepareTx();
  if (Serial1.write(frame, length) != length) return false;
  Serial1.flush();
  delay(NO_REPLY_GAP_MS);
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
  delay(12);
  pinMode(BQ_TX_PIN, INPUT_PULLUP);
  Serial1.begin(BQ_BAUD);
  delay(100);
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
    delay(2);
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
    delay(2);  // TI requires at least 1.35 ms TSREF startup time.
    if (!readRegister(address, CONTROL2, control2) || !(control2 & TSREF_EN)) {
      return false;
    }
  }
  tsrefReady[moduleIndex] = true;
  return true;
}

bool configureAdcReady(uint8_t address) {
  // Start a one-shot 8RR conversion only when the ADC is stopped or already in
  // 8RR mode. Existing continuous mode is left running; other modes are rejected.
  uint8_t control = 0;
  uint8_t status = 0;
  if (!readRegister(address, ADC_CTRL1, control) ||
      !readRegister(address, ADC_STAT1, status)) {
    return false;
  }

  const uint8_t mode = control & ADC_MODE_MASK;
  if (mode == 0 || mode == ADC_MODE_8RR) {
    const uint8_t startValue = static_cast<uint8_t>(
        (control & 0x18U) | ADC_MODE_8RR | ADC_GO);
    if (!writeRegister(address, ADC_CTRL1, startValue)) return false;
    status &= static_cast<uint8_t>(~ADC_READY);
  } else if (mode != ADC_MODE_CONTINUOUS) {
    return false;
  }

  const uint32_t started = millis();
  while (!(status & ADC_READY) && millis() - started < ADC_READY_TIMEOUT_MS) {
    delay(2);
    if (!readRegister(address, ADC_STAT1, status)) return false;
  }
  return (status & ADC_READY) != 0;
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

bool readCellResults(uint8_t address, ModuleReading &module) {
  // Read the twelve adjacent HI/LO cell-result pairs in one block and convert
  // signed ADC counts to millivolts. 0x8000 is the BQ invalid-result sentinel.
  constexpr uint16_t FIRST_CELL_HI = VCELL1_HI - 2U * (BATTERY_CELLS_PER_MODULE - 1U);
  constexpr uint8_t CELL_DATA_BYTES = 2U * BATTERY_CELLS_PER_MODULE;
  uint8_t data[CELL_DATA_BYTES] = {};
  if (!readBlock(address, FIRST_CELL_HI, CELL_DATA_BYTES, data)) return false;

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

bool readGpioResults(uint8_t moduleIndex, uint8_t address,
                     const BmsSettings &settings, ModuleReading &module) {
  // Read TSREF and all GPIO result pairs as one register block, then derive
  // GPIO volts and (for enabled NTC modes) provisional temperatures.
  module.adcGpioMask = gpioAdcMasks[moduleIndex];
  if (!gpioConfigReady[moduleIndex]) return false;
  if (!module.adcGpioMask) return true;
  if (!tsrefReady[moduleIndex]) return false;

  constexpr uint8_t RESULT_BYTES = 18;  // TSREF plus GPIO1..GPIO8, HI/LO.
  uint8_t data[RESULT_BYTES] = {};
  if (!readBlock(address, TSREF_HI, RESULT_BYTES, data)) return false;

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
  return allConfiguredChannelsValid;
}

void readOneModule(uint8_t moduleIndex, const BmsSettings &settings,
                   ModuleReading &module) {
  // Finish one device's ADC status, cell block and configured GPIO/TSREF block
  // before advancing the scan to the next stack address.
  module.address = static_cast<uint8_t>(moduleIndex + 1U);
  module.cellReadOk = false;
  module.gpioReadOk = false;
  module.error[0] = '\0';

  if (!configureAdcReady(module.address)) {
    setModuleError(module, "main ADC not ready or control/status read failed");
    return;
  }

  module.cellReadOk = readCellResults(module.address, module);
  if (!module.cellReadOk) setModuleError(module, "cell ADC block read incomplete");

  module.gpioReadOk = readGpioResults(moduleIndex, module.address, settings, module);
  if (!module.gpioReadOk) setModuleError(module, "GPIO/TSREF ADC read incomplete");
}

bool initializationFailed(char *error, size_t errorSize, const char *message) {
  communicationReady = false;
  scanActive = false;
  if (message != lastError) setLastError(message);
  copyText(error, errorSize, lastError);
  return false;
}
}

bool batteryCommsBegin(const BmsSettings &settings, char *error,
                       size_t errorSize) {
  communicationReady = false;
  scanActive = false;
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
  delay(100);
  wakeBase();
  delay(WAKE_PROPAGATION_MS);

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
  // stack modules so the 500 ms monitor can sample GPIO thermistors each pass.
  for (uint8_t index = 0; index < configuredStackModules; ++index) {
    cacheGpioConfigAndEnableTsref(index, static_cast<uint8_t>(index + 1U));
  }

  communicationReady = true;
  scanActive = false;
  setLastError("");
  return true;
}

bool batteryCommsIsReady() {
  return communicationReady;
}

const char *batteryCommsLastError() {
  return lastError;
}

void batteryCommsStartScan(BatterySnapshot &snapshot,
                           const BmsSettings &settings) {
  memset(&snapshot, 0, sizeof(snapshot));
  snapshot.moduleCount = settings.stackModules;
  snapshot.allCellReadsOk = true;
  snapshot.allGpioReadsOk = true;
  snapshot.startedAtMs = millis();
  scanModuleIndex = 0;
  scanActive = communicationReady && settings.stackModules > 0 &&
               settings.stackModules <= BATTERY_MAX_STACK_MODULES;
}

BatteryScanStepResult batteryCommsScanStep(BatterySnapshot &snapshot,
                                           const BmsSettings &settings) {
  // One call reads at most one module, leaving the main loop opportunities to
  // service USB commands, CAN frames, and state-machine checks between modules.
  if (!communicationReady || !scanActive) return BatteryScanStepResult::NotReady;

  if (scanModuleIndex < snapshot.moduleCount) {
    ModuleReading &module = snapshot.modules[scanModuleIndex];
    readOneModule(scanModuleIndex, settings, module);
    snapshot.validCellCount = static_cast<uint16_t>(
        snapshot.validCellCount + module.validCellCount);
    if (!module.cellReadOk) snapshot.allCellReadsOk = false;
    if (!module.gpioReadOk) snapshot.allGpioReadsOk = false;
    ++snapshot.completedModules;
    ++scanModuleIndex;
  }

  if (scanModuleIndex >= snapshot.moduleCount) {
    snapshot.finishedAtMs = millis();
    snapshot.durationMs = snapshot.finishedAtMs - snapshot.startedAtMs;
    scanActive = false;
    return BatteryScanStepResult::Complete;
  }
  return BatteryScanStepResult::InProgress;
}

void batteryCommsCancelScan() {
  scanActive = false;
}

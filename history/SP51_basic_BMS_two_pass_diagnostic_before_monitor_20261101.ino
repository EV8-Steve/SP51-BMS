/*
  SP51_basic_BMS — target BMS firmware for the Arduino Nano R4

  PROJECT INTENT
  This project is a state-machine BMS that reads cell-voltage and temperature
  data from the BQ79616 daisy-chain. It does not yet initialize or use the
  S-box functions of the original SP51 hardware. Contactor switching and pack
  current sensing are handled by external hardware: the Nano drives external
  low-side contactor drivers, and current is measured by an external
  LEM CAB500-C/SP7 sensor.

  TARGET STATE MACHINE AND HARDWARE
  - States: Ready, Precharge, Drive, and Charge.
  - BQ79616 UART/daisy-chain setup, retaining the proven manual base COMH
    TX/RX debug override while stack communications need it.
  - USB serial monitoring and serial-configurable BMS settings.
  - Low-side contactor-driver outputs on D6, D13, D11, and D10.
  - Isolated digital inputs on D2 and D3 for Run and 240 V sensing.
  - A non-blocking main-loop task scheduler to sample stack cell voltages and
    module temperatures every 500 ms, and read current from the external LEM.
  - Cell-voltage delta and charge/discharge-current calculations, SOC, charger
    control over CAN, and a wide, readable USB serial status display.
  - Retain the SOC calculation and serial-configurable settings from Jamie's
    Simp BMS after review and adaptation. Do not carry forward its legacy
    Victron, Tesla, BMW/Mini CAN protocols, or old CSC control/CRC handling.
  - Use the user's modified Nano R4 R7FA4M1_CAN library for CAN, including its
    txReady() transmit-buffer status before queuing another message.

  CURRENT SOURCE STATUS
  This file is still the stack/ADC bring-up test: it wakes and addresses the
  ten-device chain, reads cell and configured GPIO/TSREF ADC results twice,
  and intentionally leaves the manual base COMH override enabled. It does not
  yet implement the four-state controller, contactor outputs, isolated inputs,
  LEM current acquisition, charger CAN control, or the final status display.
  Those are the planned SP51_basic_BMS implementation steps.

  CURRENT TEST WIRING
  Host UART: Nano R4 D1/TX -> base BQ RX; D0/RX <- base BQ TX.
  UART: 1 Mbps, 8N1. The host UART connects to the base only.
  The stack uses the BQ differential COMH/COML daisy-chain interface.
  The base board has COMH connected and COML unconnected.
  No SPI transactions, HW_RESET, customer OTP programming, fault clears, or
  GPIO/pyro writes are included. The volatile CONTROL2[TSREF_EN] bit may be
  toggled for the duration of an ADC read.
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>

// ---------------- Hardware ----------------
constexpr uint8_t BQ_RX_PIN = 0;      // Nano D0 / Serial1 RX
constexpr uint8_t BQ_TX_PIN = 1;      // Nano D1 / Serial1 TX
constexpr uint32_t BQ_BAUD = 1000000UL;

// ---------------- Stack ----------------
constexpr uint8_t MIN_CANDIDATE_COUNT = 2;
constexpr uint8_t MAX_CANDIDATE_COUNT = 13;
constexpr uint8_t MIN_SWEEP_COUNT = 10;
constexpr uint8_t BASE_ADDR = 0;

// ---------------- Registers ----------------
constexpr uint16_t DEV_CONF       = 0x0002;
constexpr uint16_t ACTIVE_CELL    = 0x0003;
constexpr uint16_t DIR0_ADDR      = 0x0306;
constexpr uint16_t DIR1_ADDR      = 0x0307;
constexpr uint16_t COMM_CTRL      = 0x0308;
constexpr uint16_t CONTROL1       = 0x0309;
constexpr uint16_t CONTROL2       = 0x030A;
constexpr uint16_t ADC_CTRL1      = 0x030D;
constexpr uint16_t GPIO_CONF1     = 0x000E;
constexpr uint16_t TSREF_HI       = 0x058C;
constexpr uint16_t GPIO1_HI       = 0x058E;
constexpr uint16_t PARTID         = 0x0500;
constexpr uint16_t ADC_STAT1      = 0x0527;
constexpr uint16_t FAULT_COMM1    = 0x0530;
constexpr uint16_t FAULT_COMM2    = 0x0531;
constexpr uint16_t FAULT_COMM3    = 0x0532;
constexpr uint16_t DEBUG_COMM_STAT = 0x0780;
constexpr uint16_t DEBUG_CTRL_UNLOCK = 0x0700;
constexpr uint16_t DEBUG_COMM_CTRL1 = 0x0701;
constexpr uint16_t DEBUG_COMM_CTRL2 = 0x0702;
constexpr uint16_t DEBUG_UART_RC  = 0x0781;
constexpr uint16_t DEBUG_UART_RR_TR = 0x0782;
constexpr uint16_t DEBUG_COMH_BIT = 0x0783;
constexpr uint16_t DEBUG_COMH_RC  = 0x0784;
constexpr uint16_t DEBUG_COMH_RR_TR = 0x0785;
constexpr uint16_t DEBUG_COML_BIT = 0x0786;
constexpr uint16_t DEBUG_COML_RC  = 0x0787;
constexpr uint16_t DEBUG_COML_RR_TR = 0x0788;
constexpr uint16_t DEBUG_COMH_DISCARD = 0x078A;
constexpr uint16_t DEBUG_COMH_VALID_HI = 0x078E;
constexpr uint16_t DEBUG_COMH_VALID_LO = 0x078F;
constexpr uint16_t OTP_ECC_DATAIN1 = 0x0343;
constexpr uint16_t OTP_ECC_TEST = 0x034C;
constexpr uint16_t VCELL1_HI     = 0x0586;


constexpr uint8_t MULTIDROP_EN_MASK = 0x20;
constexpr uint8_t ADC_MAIN_GO_MASK = 0x04;
constexpr uint8_t ADC_MAIN_MODE_MASK = 0x03;
constexpr uint8_t ADC_MAIN_8RR_MODE = 0x01;
constexpr uint8_t ADC_LPF_PRESERVE_MASK = 0x18;
constexpr uint8_t ADC_MAIN_READY_MASK = 0x01;
constexpr uint8_t TSREF_EN_MASK = 0x01;
constexpr uint8_t GPIO_SPI_EN_MASK = 0x40;
constexpr uint8_t GPIO_FAULT_IN_EN_MASK = 0x80;
constexpr uint8_t MODULE_CELL_COUNT = 12;
constexpr float VCELL_LSB_UV = 190.73f;
constexpr float GPIO_LSB_UV = 152.59f;
constexpr float TSREF_LSB_UV = 169.54f;
// Provisional thermistor model inferred from Run43 and the reported ~12 C
// module temperature. These BMW board values are not confirmed: assumed
// R1=22 kOhm, R25=10 kOhm, and NTC beta=3380 K. Only candidate channels inside
// the broad 0.38..0.50 GPIO/TSREF window get a model-based temperature estimate.
// All raw ADC values are always printed and no readings are dropped.
constexpr float NTC_CANDIDATE_RATIO_MIN = 0.38f;
constexpr float NTC_CANDIDATE_RATIO_MAX = 0.50f;
constexpr float NTC_PULLUP_OHMS = 22000.0f;
constexpr float NTC_R25_OHMS = 10000.0f;
constexpr float NTC_BETA_K = 3380.0f;
constexpr float NTC_T0_K = 298.15f;
constexpr uint8_t CONTROL1_DIR_SEL  = 0x80;
constexpr uint8_t CONTROL1_ADDR_WR_DIR1 = 0x81;
constexpr uint8_t CONTROL1_ADDR_WR_DIR0 = 0x01;
constexpr uint8_t COMM_BASE         = 0x00;
constexpr uint8_t COMM_STACK        = 0x02;
constexpr uint8_t COMM_TOP          = 0x03;

constexpr uint8_t SINGLE_READ      = 0x80;
constexpr uint8_t SINGLE_WRITE     = 0x90;
constexpr uint8_t BROADCAST_READ   = 0xC0;
constexpr uint8_t STACK_READ       = 0xA0;
constexpr uint8_t BROADCAST_WRITE  = 0xD0;
constexpr uint8_t BROADCAST_WRITE_REVERSE = 0xE0;

constexpr uint32_t RESPONSE_TIMEOUT_MS = 200;
constexpr uint16_t SERIAL_STARTUP_DELAY_MS = 6000;
constexpr uint16_t WAKE_PULSE_US = 2500;
constexpr uint16_t WAKE_PROPAGATION_MS = 250;
constexpr uint16_t ADC_READY_TIMEOUT_MS = 100;
constexpr uint16_t PARTIAL_FRAME_QUIET_MS = 3;
constexpr uint16_t RX_QUIET_GUARD_MS = 5;
constexpr uint16_t NO_REPLY_WRITE_GAP_MS = 5;
constexpr size_t RX_BUFFER_SIZE = 512;
constexpr size_t ONE_BYTE_RESPONSE_SIZE = 7;

uint8_t rxBuf[RX_BUFFER_SIZE];
bool testSucceeded = false;
unsigned long lastHeartbeat = 0;

// ---------------- CRC-16/IBM (MODBUS), low byte first ----------------
uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;

  while (n--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
  }
  return crc;
}

void addCRC(uint8_t *frame, size_t payloadLength) {
  const uint16_t crc = crc16(frame, payloadLength);
  frame[payloadLength] = crc & 0xFF;
  frame[payloadLength + 1] = crc >> 8;
}

bool validCRC(const uint8_t *frame, size_t length) {
  return length >= 3 && crc16(frame, length) == 0;
}

// ---------------- Logging / RX handling ----------------
void dumpHex(const char *tag, const uint8_t *data, size_t n) {
  Serial.print(tag);
  for (size_t i = 0; i < n; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void drainAndReportRX(const char *reason) {
  uint8_t local[RX_BUFFER_SIZE];
  size_t stored = 0;
  size_t total = 0;

  while (Serial1.available()) {
    const int c = Serial1.read();
    if (c < 0) break;
    if (stored < sizeof(local)) local[stored++] = static_cast<uint8_t>(c);
    ++total;
  }

  if (total) {
    Serial.print(F("RX NOTICE ["));
    Serial.print(reason);
    Serial.print(F("]: drained "));
    Serial.print(total);
    Serial.println(F(" unexpected/stale byte(s):"));
    dumpHex("  BYTES: ", local, stored);
  }
}

void waitForRxQuiet(uint16_t quietMs = RX_QUIET_GUARD_MS,
                    const char *reason = "quiet guard") {
  uint32_t lastByte = millis();
  uint8_t local[RX_BUFFER_SIZE];
  size_t stored = 0;
  size_t total = 0;

  while (millis() - lastByte < quietMs) {
    while (Serial1.available()) {
      const int c = Serial1.read();
      if (c < 0) continue;
      if (stored < sizeof(local)) local[stored++] = static_cast<uint8_t>(c);
      ++total;
      lastByte = millis();
    }
    delayMicroseconds(100);
  }

  if (total) {
    Serial.print(F("RX NOTICE ["));
    Serial.print(reason);
    Serial.print(F("]: received "));
    Serial.print(total);
    Serial.println(F(" trailing byte(s):"));
    dumpHex("  BYTES: ", local, stored);
  }
}

void prepareTX() {
  drainAndReportRX("before TX");
  waitForRxQuiet(RX_QUIET_GUARD_MS, "pre-TX quiet guard");
}

void beginBQ() {
  Serial1.begin(BQ_BAUD);
  delay(100);
}

// ---------------- Base RX WAKE ping ----------------
void wakeBQOnly() {
  Serial.println(F("WAKE-only recovery: 2.5 ms active-low on Nano D1/TX -> base RX."));
  Serial1.flush();
  drainAndReportRX("before WAKE");
  Serial1.end();

  pinMode(BQ_TX_PIN, OUTPUT);
  digitalWrite(BQ_TX_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(BQ_TX_PIN, LOW);
  delayMicroseconds(WAKE_PULSE_US);
  digitalWrite(BQ_TX_PIN, HIGH);

  delay(12);
  pinMode(BQ_TX_PIN, INPUT_PULLUP);
  beginBQ();
  drainAndReportRX("after WAKE");
}

// ---------------- Frame builders ----------------
size_t buildSingleRead(uint8_t dev, uint16_t reg, uint8_t *frame) {
  frame[0] = SINGLE_READ;
  frame[1] = dev;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = 0x00;  // one data byte
  addCRC(frame, 5);
  return 7;
}

size_t buildSingleWrite(uint8_t dev, uint16_t reg, uint8_t value,
                        uint8_t *frame) {
  frame[0] = SINGLE_WRITE;
  frame[1] = dev;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = value;
  addCRC(frame, 5);
  return 7;
}

size_t buildBroadcastWrite(uint16_t reg, uint8_t value, uint8_t *frame) {
  frame[0] = BROADCAST_WRITE;
  frame[1] = reg >> 8;
  frame[2] = reg & 0xFF;
  frame[3] = value;
  addCRC(frame, 4);
  return 6;
}
size_t buildBroadcastWriteReverse(uint16_t reg, uint8_t value, uint8_t *frame) {
  frame[0] = BROADCAST_WRITE_REVERSE;
  frame[1] = reg >> 8;
  frame[2] = reg & 0xFF;
  frame[3] = value;
  addCRC(frame, 4);
  return 6;
}

size_t buildBroadcastWriteBlock(uint16_t reg, const uint8_t *data,
                                uint8_t dataLength, uint8_t *frame) {
  if (!dataLength || dataLength > 8) return 0;

  frame[0] = BROADCAST_WRITE | ((dataLength - 1) & 0x07);
  frame[1] = reg >> 8;
  frame[2] = reg & 0xFF;
  memcpy(&frame[3], data, dataLength);
  addCRC(frame, 3 + dataLength);
  return 3 + dataLength + 2;
}

size_t buildBroadcastRead(uint16_t reg, uint8_t dataLength, uint8_t *frame) {
  if (!dataLength || dataLength > 8) return 0;

  frame[0] = BROADCAST_READ;
  frame[1] = reg >> 8;
  frame[2] = reg & 0xFF;
  frame[3] = dataLength - 1;
  addCRC(frame, 4);
  return 6;
}

size_t buildStackRead(uint16_t reg, uint8_t dataLength, uint8_t *frame) {
  if (!dataLength || dataLength > 128) return 0;

  frame[0] = STACK_READ;
  frame[1] = reg >> 8;
  frame[2] = reg & 0xFF;
  frame[3] = dataLength - 1;
  addCRC(frame, 4);
  return 6;
}

// ---------------- Command / response helpers ----------------
bool sendNoReply(const uint8_t *frame, size_t length, const char *tag) {
  prepareTX();
  dumpHex(tag, frame, length);

  const size_t written = Serial1.write(frame, length);
  Serial1.flush();

  if (written != length) {
    Serial.println(F("ERROR: UART did not accept the complete command."));
    return false;
  }

  delay(NO_REPLY_WRITE_GAP_MS);
  waitForRxQuiet(RX_QUIET_GUARD_MS, "after no-reply write");
  return true;
}

bool broadcastWrite(uint16_t reg, uint8_t value) {
  uint8_t frame[6];
  const size_t n = buildBroadcastWrite(reg, value, frame);
  return sendNoReply(frame, n, "TX(BCAST): ");
}
bool broadcastWriteReverse(uint16_t reg, uint8_t value) {
  uint8_t frame[6];
  const size_t n = buildBroadcastWriteReverse(reg, value, frame);
  return sendNoReply(frame, n, "TX(BCAST REVERSE): ");
}

bool broadcastWriteBlock(uint16_t reg, const uint8_t *data,
                         uint8_t dataLength) {
  uint8_t frame[14];
  const size_t n = buildBroadcastWriteBlock(reg, data, dataLength, frame);
  if (!n) return false;
  return sendNoReply(frame, n, "TX(BCAST BLOCK): ");
}

bool singleWrite(uint8_t dev, uint16_t reg, uint8_t value) {
  uint8_t frame[7];
  const size_t n = buildSingleWrite(dev, reg, value, frame);
  return sendNoReply(frame, n, "TX(SINGLE WRITE): ");
}

bool collectFixedResponse(size_t expected, size_t &stored) {
  stored = 0;
  const uint32_t start = millis();

  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    while (Serial1.available()) {
      const int c = Serial1.read();
      if (c < 0) continue;
      if (stored < RX_BUFFER_SIZE) rxBuf[stored++] = static_cast<uint8_t>(c);
      // A single read expects exactly one response frame. Leave any extra
      // bytes for the post-response quiet/drain step instead of folding them
      // into this frame. Partial frames still wait until the full timeout.
      if (stored >= expected) return true;
    }
    delayMicroseconds(100);
  }
  return stored >= expected;
}

bool readByte(uint8_t dev, uint16_t reg, uint8_t &value) {
  uint8_t frame[7];
  const size_t n = buildSingleRead(dev, reg, frame);

  prepareTX();
  dumpHex("TX(SINGLE READ): ", frame, n);
  if (Serial1.write(frame, n) != n) {
    Serial.println(F("ERROR: UART did not accept the complete read command."));
    return false;
  }
  Serial1.flush();

  size_t got = 0;
  collectFixedResponse(ONE_BYTE_RESPONSE_SIZE, got);

  if (!got) {
    Serial.println(F("ERROR: no response before timeout."));
    return false;
  }

  dumpHex("RX: ", rxBuf, got);

  if (got != ONE_BYTE_RESPONSE_SIZE) {
    Serial.print(F("ERROR: partial/extra response; received "));
    Serial.print(got);
    Serial.println(F(" byte(s), expected 7."));
    waitForRxQuiet(RX_QUIET_GUARD_MS, "after bad single response");
    return false;
  }
  if (rxBuf[0] != 0x00 || !validCRC(rxBuf, got)) {
    Serial.println(F("ERROR: response INIT or CRC check failed."));
    waitForRxQuiet(RX_QUIET_GUARD_MS, "after invalid single response");
    return false;
  }
  if (rxBuf[1] != dev || rxBuf[2] != (reg >> 8) ||
      rxBuf[3] != (reg & 0xFF)) {
    Serial.println(F("ERROR: response device/register does not match request."));
    waitForRxQuiet(RX_QUIET_GUARD_MS, "after mismatched single response");
    return false;
  }

  value = rxBuf[4];
  waitForRxQuiet(RX_QUIET_GUARD_MS, "after single response");
  return true;
}

// Read a contiguous register block from one addressed device only. The BQ
// response INIT byte reports dataLength-1; e.g. 0x17 means 24 data bytes.
bool readBytes(uint8_t dev, uint16_t reg, uint8_t dataLength,
               uint8_t *data) {
  if (!data || !dataLength || dataLength > 128) {
    Serial.println(F("ERROR: invalid single-device read length or destination."));
    return false;
  }

  uint8_t frame[7];
  frame[0] = SINGLE_READ;  // DATA_SIZE remains 000 for all read commands.
  frame[1] = dev;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = dataLength - 1;
  addCRC(frame, 5);
  const size_t expected = static_cast<size_t>(dataLength) + 6U;

  prepareTX();
  dumpHex("TX(SINGLE READ BURST): ", frame, sizeof(frame));
  if (Serial1.write(frame, sizeof(frame)) != sizeof(frame)) {
    Serial.println(F("ERROR: UART did not accept the complete single-device burst read."));
    return false;
  }
  Serial1.flush();

  size_t got = 0;
  collectFixedResponse(expected, got);
  if (!got) {
    Serial.println(F("ERROR: no response before timeout."));
    return false;
  }
  dumpHex("RX(SINGLE READ BURST): ", rxBuf, got);

  if (got != expected) {
    Serial.print(F("ERROR: burst response has "));
    Serial.print(got);
    Serial.print(F(" bytes; expected "));
    Serial.println(expected);
    waitForRxQuiet(RX_QUIET_GUARD_MS, "after bad burst response");
    return false;
  }
  if (rxBuf[0] != static_cast<uint8_t>(dataLength - 1) ||
      !validCRC(rxBuf, got)) {
    Serial.println(F("ERROR: burst response length INIT or CRC check failed."));
    waitForRxQuiet(RX_QUIET_GUARD_MS, "after invalid burst response");
    return false;
  }
  if (rxBuf[1] != dev || rxBuf[2] != (reg >> 8) ||
      rxBuf[3] != (reg & 0xFF)) {
    Serial.println(F("ERROR: burst response device/register does not match request."));
    waitForRxQuiet(RX_QUIET_GUARD_MS, "after mismatched burst response");
    return false;
  }

  memcpy(data, &rxBuf[4], dataLength);
  waitForRxQuiet(RX_QUIET_GUARD_MS, "after single-device burst response");
  return true;
}

struct CommDiagReg {
  const char *name;
  uint16_t address;
};

void printBaseCommDiagnostics(const char *stage) {
  // Read-only fault/debug snapshots. Do not clear flags; compare snapshots.
  const CommDiagReg regs[] = {
    {"DEV_CONF", DEV_CONF},
    {"CONTROL1", CONTROL1},
    {"COMM_CTRL", COMM_CTRL},
    {"DIR0_ADDR", DIR0_ADDR},
    {"DEBUG_COMM_CTRL1", DEBUG_COMM_CTRL1},
    {"DEBUG_COMM_CTRL2", DEBUG_COMM_CTRL2},
    {"DEBUG_COMM_STAT", DEBUG_COMM_STAT},
    {"FAULT_COMM1", FAULT_COMM1},
    {"FAULT_COMM2", FAULT_COMM2},
    {"FAULT_COMM3", FAULT_COMM3},
    {"DEBUG_UART_RC", DEBUG_UART_RC},
    {"DEBUG_UART_RR_TR", DEBUG_UART_RR_TR},
    {"DEBUG_COMH_BIT", DEBUG_COMH_BIT},
    {"DEBUG_COMH_RC", DEBUG_COMH_RC},
    {"DEBUG_COMH_RR_TR", DEBUG_COMH_RR_TR},
    {"DEBUG_COML_BIT", DEBUG_COML_BIT},
    {"DEBUG_COML_RC", DEBUG_COML_RC},
    {"DEBUG_COML_RR_TR", DEBUG_COML_RR_TR}
  };

  Serial.print(F("\n=== BASE COMMUNICATION DIAGNOSTICS: "));
  Serial.print(stage);
  Serial.println(F(" ==="));
  for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
    uint8_t value = 0;
    const bool success = readByte(BASE_ADDR, regs[i].address, value);

    Serial.print(F("  "));
    Serial.print(regs[i].name);
    Serial.print(F(" [0x"));
    if (regs[i].address < 0x1000) Serial.print('0');
    Serial.print(regs[i].address, HEX);
    Serial.print(F("] = "));
    if (!success) {
      Serial.println(F("<read failed>"));
      continue;
    }
    Serial.print(F("0x"));
    if (value < 0x10) Serial.print('0');
    Serial.println(value, HEX);
  }
  Serial.println(F("Read-only snapshot; no fault/debug flags were cleared."));
}

void printDeviceCommDiagnostics(uint8_t dev, const char *stage) {
  const CommDiagReg regs[] = {
    {"DEBUG_COMM_STAT", DEBUG_COMM_STAT},
    {"FAULT_COMM1", FAULT_COMM1},
    {"FAULT_COMM2", FAULT_COMM2},
    {"FAULT_COMM3", FAULT_COMM3},
    {"DEBUG_UART_RC", DEBUG_UART_RC},
    {"DEBUG_UART_RR_TR", DEBUG_UART_RR_TR},
    {"DEBUG_COMH_BIT", DEBUG_COMH_BIT},
    {"DEBUG_COMH_RC", DEBUG_COMH_RC},
    {"DEBUG_COMH_RR_TR", DEBUG_COMH_RR_TR},
    {"DEBUG_COML_BIT", DEBUG_COML_BIT},
    {"DEBUG_COML_RC", DEBUG_COML_RC},
    {"DEBUG_COML_RR_TR", DEBUG_COML_RR_TR}
  };

  Serial.print(F("\n=== DEVICE "));
  Serial.print(dev);
  Serial.print(F(" COMMUNICATION SNAPSHOT: "));
  Serial.print(stage);
  Serial.println(F(" ==="));
  for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
    uint8_t value = 0;
    const bool success = readByte(dev, regs[i].address, value);
    Serial.print(F("  "));
    Serial.print(regs[i].name);
    Serial.print(F(" [0x"));
    if (regs[i].address < 0x1000) Serial.print('0');
    Serial.print(regs[i].address, HEX);
    Serial.print(F("] = "));
    if (!success) {
      Serial.println(F("<read failed>"));
      continue;
    }
    Serial.print(F("0x"));
    if (value < 0x10) Serial.print('0');
    Serial.println(value, HEX);
  }
  Serial.println(F("Stack communication snapshot is read-only; no flags were cleared."));
}

void probeBaseStage(const char *stage) {
  const CommDiagReg regs[] = {
    {"DEV_CONF", DEV_CONF},
    {"CONTROL1", CONTROL1},
    {"DEBUG_COMM_STAT", DEBUG_COMM_STAT}
  };

  Serial.print(F("\n=== BASE CHECK: "));
  Serial.print(stage);
  Serial.println(F(" ==="));
  for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
    uint8_t value = 0;
    const bool success = readByte(BASE_ADDR, regs[i].address, value);
    Serial.print(F("  "));
    Serial.print(regs[i].name);
    Serial.print(F(" = "));
    if (!success) {
      Serial.println(F("<no response>"));
    } else {
      Serial.print(F("0x"));
      if (value < 0x10) Serial.print('0');
      Serial.println(value, HEX);
    }
  }
}

// TI requires the host to wait for all expected response frames, or for the
// response timeout, before starting another transaction. Do not end a burst
// merely because there was a short gap between response frames.
bool broadcastReadBurst(uint16_t reg, uint8_t dataLength,
                        uint8_t expectedFrames, size_t &got) {
  uint8_t frame[6];
  const size_t n = buildBroadcastRead(reg, dataLength, frame);
  if (!n || !expectedFrames) {
    got = 0;
    return false;
  }
  const size_t expectedBytes = static_cast<size_t>(expectedFrames) *
                               (static_cast<size_t>(dataLength) + 6);

  prepareTX();
  dumpHex("TX(BCAST READ): ", frame, n);
  if (Serial1.write(frame, n) != n) {
    Serial.println(F("ERROR: UART did not accept the complete broadcast read."));
    got = 0;
    return false;
  }
  Serial1.flush();

  got = 0;
  const uint32_t start = millis();
  uint32_t lastByte = start;

  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    while (Serial1.available()) {
      const int c = Serial1.read();
      if (c < 0) continue;
      if (got < RX_BUFFER_SIZE) rxBuf[got++] = static_cast<uint8_t>(c);
      lastByte = millis();
    }

    if (got >= expectedBytes && millis() - lastByte >= PARTIAL_FRAME_QUIET_MS) break;
    delayMicroseconds(100);
  }

  if (got) dumpHex("RX(BCAST): ", rxBuf, got);
  else Serial.println(F("RX(BCAST): no response bytes."));
  if (got < expectedBytes) {
    Serial.print(F("WARNING: received "));
    Serial.print(got);
    Serial.print(F(" of "));
    Serial.print(expectedBytes);
    Serial.println(F(" expected response bytes before timeout."));
  }
  waitForRxQuiet(RX_QUIET_GUARD_MS, "after broadcast response burst");
  return true;
}

// Stack Read (INIT=0xA0) returns only STACK_DEV devices, with the ToS
// responding first. The caller supplies the expected number of stack frames.
bool stackReadBurst(uint16_t reg, uint8_t dataLength,
                    uint8_t expectedFrames, size_t &got) {
  uint8_t frame[6];
  const size_t n = buildStackRead(reg, dataLength, frame);
  if (!n || !expectedFrames) {
    got = 0;
    return false;
  }
  const size_t expectedBytes = static_cast<size_t>(expectedFrames) *
                               (static_cast<size_t>(dataLength) + 6);

  prepareTX();
  dumpHex("TX(STACK READ): ", frame, n);
  if (Serial1.write(frame, n) != n) {
    Serial.println(F("ERROR: UART did not accept the complete Stack Read."));
    got = 0;
    return false;
  }
  Serial1.flush();

  got = 0;
  const uint32_t start = millis();
  uint32_t lastByte = start;
  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    while (Serial1.available()) {
      const int c = Serial1.read();
      if (c < 0) continue;
      if (got < RX_BUFFER_SIZE) rxBuf[got++] = static_cast<uint8_t>(c);
      lastByte = millis();
    }
    if (got >= expectedBytes && millis() - lastByte >= PARTIAL_FRAME_QUIET_MS) break;
    delayMicroseconds(100);
  }

  if (got) dumpHex("RX(STACK): ", rxBuf, got);
  else Serial.println(F("RX(STACK): no response bytes."));
  if (got < expectedBytes) {
    Serial.print(F("WARNING: received "));
    Serial.print(got);
    Serial.print(F(" of "));
    Serial.print(expectedBytes);
    Serial.println(F(" expected Stack Read response bytes before timeout."));
  }
  waitForRxQuiet(RX_QUIET_GUARD_MS, "after Stack Read response burst");
  return true;
}

// ---------------- Required mode check ----------------
bool checkBaseDaisyChainMode(bool &changed) {
  changed = false;
  uint8_t conf = 0;

  Serial.println(F("\n=== BASE DEV_CONF MODE CHECK ==="));
  if (!readByte(BASE_ADDR, DEV_CONF, conf)) {
    Serial.println(F("Could not read base DEV_CONF after initial setup; stopping before address verification."));
    return false;
  }

  Serial.print(F("DEV_CONF[0x0002] = 0x"));
  if (conf < 0x10) Serial.print('0');
  Serial.println(conf, HEX);

  if (conf & MULTIDROP_EN_MASK) {
    const uint8_t desired = static_cast<uint8_t>(conf & ~MULTIDROP_EN_MASK);
    Serial.println(F("MULTIDROP_EN=1 disables the daisy-chain COMH/COML links."));
    Serial.print(F("Writing DEV_CONF shadow 0x"));
    if (desired < 0x10) Serial.print('0');
    Serial.println(desired, HEX);
    Serial.println(F("Only bit 5 is cleared; all other DEV_CONF bits are preserved."));
    Serial.println(F("This is a volatile shadow-register write, not OTP programming."));

    if (!singleWrite(BASE_ADDR, DEV_CONF, desired)) {
      Serial.println(F("Could not send DEV_CONF update; stopping."));
      return false;
    }

    uint8_t verify = 0;
    if (!readByte(BASE_ADDR, DEV_CONF, verify)) {
      Serial.println(F("Could not read DEV_CONF back after update; stopping."));
      return false;
    }

    Serial.print(F("DEV_CONF readback = 0x"));
    if (verify < 0x10) Serial.print('0');
    Serial.println(verify, HEX);

    if (verify & MULTIDROP_EN_MASK) {
      Serial.println(F("ERROR: MULTIDROP_EN is still set; stopping."));
      return false;
    }
    Serial.println(F("MULTIDROP_EN cleared and verified; the stack will be addressed again."));
    changed = true;
  } else {
    Serial.println(F("MULTIDROP_EN=0: base is already configured for daisy-chain operation."));
  }

  return true;
}

// A BQ broadcast write has no acknowledgement phase. Test whether it reaches
// the base by writing a reversible pattern to the volatile ECC_DATAIN1
// register, then reading it through the known-good single-device path.
bool probeBroadcastWriteAtBase() {
  constexpr uint8_t SINGLE_TEST_VALUE = 0x5A;
  constexpr uint8_t BROADCAST_TEST_VALUE = 0xA5;
  bool broadcastReachedBase = false;

  Serial.println(F("\n=== SAFE BROADCAST-WRITE READBACK TEST ==="));
  Serial.println(F("CONTROL1.ADDR_WR is a one-transaction auto-address command, so it is not read back here."));
  Serial.println(F("Using volatile ECC_DATAIN1 (0x0343); no ECC test or OTP programming is enabled."));

  if (!singleWrite(BASE_ADDR, OTP_ECC_DATAIN1, SINGLE_TEST_VALUE)) {
    Serial.println(F("ERROR: could not prepare the base ECC_DATAIN1 single-write control."));
    return false;
  }

  uint8_t value = 0;
  if (!readByte(BASE_ADDR, OTP_ECC_DATAIN1, value)) {
    Serial.println(F("INCONCLUSIVE: could not read back the ECC_DATAIN1 single-write control."));
    singleWrite(BASE_ADDR, OTP_ECC_DATAIN1, 0x00);
    return false;
  }
  Serial.print(F("Single-write control readback = 0x"));
  if (value < 0x10) Serial.print('0');
  Serial.println(value, HEX);
  if (value != SINGLE_TEST_VALUE) {
    Serial.println(F("INCONCLUSIVE: ECC_DATAIN1 single-write control did not match."));
    singleWrite(BASE_ADDR, OTP_ECC_DATAIN1, 0x00);
    return false;
  }

  if (broadcastWrite(OTP_ECC_DATAIN1, BROADCAST_TEST_VALUE)) {
    if (readByte(BASE_ADDR, OTP_ECC_DATAIN1, value)) {
      Serial.print(F("Base ECC_DATAIN1 after broadcast write = 0x"));
      if (value < 0x10) Serial.print('0');
      Serial.println(value, HEX);
      broadcastReachedBase = (value == BROADCAST_TEST_VALUE);
    } else {
      Serial.println(F("INCONCLUSIVE: base ECC_DATAIN1 read failed after broadcast write."));
    }
  } else {
    Serial.println(F("ERROR: host UART did not accept the broadcast ECC_DATAIN1 write."));
  }

  // Restore the scratch register on every device; also force the base to zero
  // with a single write in case the broadcast write path did not reach it.
  const bool broadcastCleanupSent = broadcastWrite(OTP_ECC_DATAIN1, 0x00);
  const bool baseCleanupSent = singleWrite(BASE_ADDR, OTP_ECC_DATAIN1, 0x00);
  uint8_t cleanupValue = 0xFF;
  const bool cleanupRead = readByte(BASE_ADDR, OTP_ECC_DATAIN1, cleanupValue);
  Serial.print(F("Base ECC_DATAIN1 cleanup readback = 0x"));
  if (cleanupValue < 0x10) Serial.print('0');
  if (cleanupRead) Serial.println(cleanupValue, HEX);
  else Serial.println(F("unavailable"));

  if (!broadcastCleanupSent || !baseCleanupSent || !cleanupRead || cleanupValue != 0x00) {
    Serial.println(F("WARNING: scratch-register cleanup was not fully verified."));
  }

  if (broadcastReachedBase) {
    Serial.println(F("PASS: broadcast write reached the base; this does not prove it traversed the stack."));
  } else {
    Serial.println(F("FAIL: base did not show the broadcast pattern; single-write control passed."));
  }
  return broadcastReachedBase;
}
bool primeForwardDirectionForBaseRead(uint8_t deviceCount) {
  if (deviceCount < 2 || deviceCount > MAX_CANDIDATE_COUNT) return false;
  const uint8_t topAddr = deviceCount - 1;

  Serial.println(F("\n=== FORWARD DIR0 ADDRESS / ROLE PRIME ==="));
  Serial.println(F("Replaying the previously successful DIR0 auto-address/role order before base register reads."));
  if (!broadcastWrite(CONTROL1, CONTROL1_ADDR_WR_DIR0)) return false;
  for (uint8_t addr = 0; addr < deviceCount; ++addr) {
    Serial.print(F("  forward address "));
    Serial.println(addr);
    if (!broadcastWrite(DIR0_ADDR, addr)) return false;
  }
  if (!broadcastWrite(COMM_CTRL, COMM_STACK)) return false;
  if (!singleWrite(BASE_ADDR, COMM_CTRL, COMM_BASE)) return false;
  if (!singleWrite(topAddr, COMM_CTRL, COMM_TOP)) return false;
  return true;
}
bool switchToReverseDirection() {
  uint8_t control1 = 0;

  Serial.println(F("\n=== SWITCHING TO REVERSE DAISY-CHAIN DIRECTION ==="));
  Serial.println(F("TI sequence: single-write base DIR_SEL=1; reverse-broadcast clear COMM_CTRL; reverse-broadcast DIR_SEL=1."));
  if (!readByte(BASE_ADDR, CONTROL1, control1)) {
    Serial.println(F("Could not read base CONTROL1 before direction switch."));
    return false;
  }
  Serial.print(F("Base CONTROL1 before switch = 0x"));
  if (control1 < 0x10) Serial.print('0');
  Serial.println(control1, HEX);

  if (!(control1 & CONTROL1_DIR_SEL)) {
    if (!singleWrite(BASE_ADDR, CONTROL1, CONTROL1_DIR_SEL)) {
      Serial.println(F("Could not send base DIR_SEL=1 write."));
      return false;
    }
  } else {
    Serial.println(F("Base already has DIR_SEL=1; continuing the TI stack setup."));
  }

  if (!broadcastWriteReverse(COMM_CTRL, 0x00)) return false;
  if (!broadcastWriteReverse(CONTROL1, CONTROL1_DIR_SEL)) return false;
  delay(2);  // TI requires at least 100 us for COMH/COML reconfiguration.

  uint8_t verify = 0;
  if (!readByte(BASE_ADDR, CONTROL1, verify)) {
    Serial.println(F("Could not read base CONTROL1 after direction switch."));
    return false;
  }
  Serial.print(F("Base CONTROL1 after switch = 0x"));
  if (verify < 0x10) Serial.print('0');
  Serial.println(verify, HEX);
  if (!(verify & CONTROL1_DIR_SEL)) {
    Serial.println(F("ERROR: base DIR_SEL did not read back as 1."));
    return false;
  }
  Serial.println(F("Base DIR_SEL=1 verified; stack direction changes were sent by reverse broadcast."));
  return true;
}

// Return the base and stack to the connected COMH/forward path if the base
// still reports DIR_SEL=1 after WAKE. This mirrors TI's documented direction
// switch procedure in reverse; if any step fails, stop before auto-addressing.
bool switchToForwardDirection() {
  uint8_t control1 = 0;

  Serial.println(F("\n=== ENSURING FORWARD DIR_SEL=0 / COMH PATH ==="));
  if (!readByte(BASE_ADDR, CONTROL1, control1)) {
    Serial.println(F("Could not read base CONTROL1 before forward switch."));
    return false;
  }

  Serial.print(F("Base CONTROL1 before forward check = 0x"));
  if (control1 < 0x10) Serial.print('0');
  Serial.println(control1, HEX);

  if (control1 & CONTROL1_DIR_SEL) {
    Serial.println(F("Base is DIR_SEL=1; switching it to DIR_SEL=0 so it uses connected COMH."));
    if (!singleWrite(BASE_ADDR, CONTROL1, 0x00)) {
      Serial.println(F("Could not switch base to DIR_SEL=0."));
      return false;
    }
    delay(2);

    Serial.println(F("Sending reverse-broadcast COMM_CTRL clear and stack DIR_SEL=0."));
    if (!broadcastWriteReverse(COMM_CTRL, 0x00)) return false;
    if (!broadcastWriteReverse(CONTROL1, 0x00)) return false;
    delay(2);
  } else {
    Serial.println(F("Base is already DIR_SEL=0; COMH is the selected base port."));
  }

  uint8_t verify = 0;
  if (!readByte(BASE_ADDR, CONTROL1, verify)) {
    Serial.println(F("Could not read base CONTROL1 after forward direction setup."));
    return false;
  }
  if (verify & CONTROL1_DIR_SEL) {
    Serial.println(F("ERROR: base remains DIR_SEL=1; stopping."));
    return false;
  }
  Serial.println(F("Base CONTROL1 verifies DIR_SEL=0; continuing over COMH."));
  return true;
}
// ---------------- Auto-address and role setup ----------------
bool autoAddressAndConfigureRoles(uint8_t deviceCount) {
  if (deviceCount < 2 || deviceCount > MAX_CANDIDATE_COUNT) {
    Serial.println(F("ERROR: candidate device count is outside the supported test range."));
    return false;
  }

  const uint8_t topAddr = deviceCount - 1;

  Serial.println(F("\n=== AUTO-ADDRESS / STACK ROLE SETUP ==="));
  Serial.print(F("Trying total device count "));
  Serial.print(deviceCount);
  Serial.print(F(" (addresses 0.."));
  Serial.print(topAddr);
  Serial.println(F("; base included)."));
  Serial.println(F("DIR_SEL=0: writes DIR0_ADDR (0x0306); readback verifies the result."));
  Serial.println(F("Writes below have no per-device acknowledgement."));

  if (!broadcastWrite(CONTROL1, CONTROL1_ADDR_WR_DIR0)) return false;

  Serial.print(F("Sending addresses 0.."));
  Serial.println(topAddr);
  for (uint8_t addr = 0; addr < deviceCount; ++addr) {
    Serial.print(F("  address "));
    Serial.println(addr);
    if (!broadcastWrite(DIR0_ADDR, addr)) return false;
  }

  if (!broadcastWrite(COMM_CTRL, COMM_STACK)) return false;
  if (!singleWrite(BASE_ADDR, COMM_CTRL, COMM_BASE)) return false;
  if (!singleWrite(topAddr, COMM_CTRL, COMM_TOP)) return false;

  return true;
}

// Read back the three role boundaries through single-device UART commands.
// TI defines base=0x00, intermediate stack devices=0x02, and ToS=0x03.
bool verifyRoleReadbacks(uint8_t deviceCount) {
  const uint8_t topAddr = deviceCount - 1;
  bool ok = true;

  Serial.println(F("\n=== COMM_CTRL ROLE READBACK ==="));
  Serial.println(F("Expected COMM_CTRL: base 0x00, intermediate 0x02, ToS 0x03."));

  const uint8_t checkAddr[] = {BASE_ADDR, static_cast<uint8_t>(deviceCount > 2 ? 1 : topAddr), topAddr};
  const uint8_t expected[] = {COMM_BASE, static_cast<uint8_t>(deviceCount > 2 ? COMM_STACK : COMM_TOP), COMM_TOP};
  const char *labels[] = {"base", "first stack device / ToS", "candidate ToS"};
  const size_t checkCount = deviceCount > 2 ? 3 : 2;

  for (size_t i = 0; i < checkCount; ++i) {
    uint8_t value = 0;
    const uint8_t dev = checkAddr[i];
    if (!readByte(dev, COMM_CTRL, value)) {
      Serial.print(F("  "));
      Serial.print(labels[i]);
      Serial.print(F(" address "));
      Serial.print(dev);
      Serial.println(F(": read failed"));
      ok = false;
      continue;
    }

    Serial.print(F("  "));
    Serial.print(labels[i]);
    Serial.print(F(" address "));
    Serial.print(dev);
    Serial.print(F(": COMM_CTRL=0x"));
    if (value < 0x10) Serial.print('0');
    Serial.print(value, HEX);
    Serial.print(F("; expected 0x"));
    if (expected[i] < 0x10) Serial.print('0');
    Serial.println(expected[i], HEX);
    if (value != expected[i]) ok = false;
  }

  Serial.println(ok ? F("COMM_CTRL role boundary readback passed.")
                    : F("COMM_CTRL role boundary readback failed; continue collecting evidence."));
  return ok;
}
// ---------------- TI broadcast address readback ----------------
bool verifyAddressesBroadcast(uint8_t expectedCount) {
  constexpr size_t RESPONSE_FRAME_SIZE = 7;
  bool seen[MAX_CANDIDATE_COUNT] = {false};
  bool ok = true;

  Serial.println(F("\n=== BROADCAST DIR0_ADDR READBACK (DIR_SEL=0) ==="));
  Serial.print(F("Expected: "));
  Serial.print(expectedCount);
  Serial.print(F(" valid 7-byte response frames for addresses 0.."));
  Serial.println(expectedCount - 1);

  size_t got = 0;
  if (!broadcastReadBurst(DIR0_ADDR, 1, expectedCount, got)) return false;

  if (got % RESPONSE_FRAME_SIZE != 0) {
    Serial.print(F("ERROR: response burst has "));
    Serial.print(got % RESPONSE_FRAME_SIZE);
    Serial.println(F(" trailing byte(s), not a whole number of frames."));
    ok = false;
  }

  const size_t frameCount = got / RESPONSE_FRAME_SIZE;
  Serial.print(F("Response frames received: "));
  Serial.println(frameCount);

  for (size_t i = 0; i < frameCount; ++i) {
    const uint8_t *f = &rxBuf[i * RESPONSE_FRAME_SIZE];

    if (!validCRC(f, RESPONSE_FRAME_SIZE)) {
      Serial.print(F("Frame "));
      Serial.print(i);
      Serial.println(F(": CRC FAIL"));
      ok = false;
      continue;
    }
    if (f[0] != 0x00 || f[2] != (DIR0_ADDR >> 8) ||
        f[3] != (DIR0_ADDR & 0xFF)) {
      Serial.print(F("Frame "));
      Serial.print(i);
      Serial.println(F(": INIT/register mismatch"));
      ok = false;
      continue;
    }

    const uint8_t dev = f[1];
    const uint8_t value = f[4];

    Serial.print(F("  device address "));
    Serial.print(dev);
    Serial.print(F(" returned DIR0_ADDR=0x"));
    if (value < 0x10) Serial.print('0');
    Serial.println(value, HEX);

    if (dev >= expectedCount) {
      Serial.println(F("    ERROR: response address is outside this candidate count."));
      ok = false;
      continue;
    }
    if (seen[dev]) {
      Serial.println(F("    ERROR: duplicate response address."));
      ok = false;
      continue;
    }

    seen[dev] = true;
    if (value != dev) {
      Serial.println(F("    ERROR: DIR0_ADDR value does not match response address."));
      ok = false;
    }
  }

  if (frameCount != expectedCount) {
    Serial.print(F("ERROR: expected "));
    Serial.print(expectedCount);
    Serial.println(F(" response frames."));
    ok = false;
  }

  for (uint8_t dev = 0; dev < expectedCount; ++dev) {
    if (!seen[dev]) {
      Serial.print(F("MISSING response from address "));
      Serial.println(dev);
      ok = false;
    }
  }

  if (ok) {
    Serial.print(F("ADDRESS VERIFICATION PASSED for total count "));
    Serial.println(expectedCount);
  } else {
    Serial.print(F("ADDRESS VERIFICATION FAILED for candidate count "));
    Serial.println(expectedCount);
  }
  return ok;
}

uint8_t probeAddressesIndividual(uint8_t scanCount) {
  uint8_t responders = 0;

  Serial.println(F("\n=== INDIVIDUAL ADDRESS / ROLE DIAGNOSTIC ==="));
  Serial.print(F("Probing addresses 0.."));
  Serial.println(scanCount - 1);

  for (uint8_t dev = 0; dev < scanCount; ++dev) {
    Serial.print(F("\nAddress "));
    Serial.println(dev);

    uint8_t dir0 = 0;
    if (!readByte(dev, DIR0_ADDR, dir0)) {
      Serial.println(F("  No valid DIR0_ADDR response."));
      continue;
    }

    ++responders;
    Serial.print(F("  DIR0_ADDR=0x"));
    if (dir0 < 0x10) Serial.print('0');
    Serial.println(dir0, HEX);
    if (dir0 != dev) {
      Serial.println(F("  WARNING: DIR0_ADDR does not match the requested address."));
    }

    uint8_t commCtrl = 0;
    if (readByte(dev, COMM_CTRL, commCtrl)) {
      Serial.print(F("  COMM_CTRL=0x"));
      if (commCtrl < 0x10) Serial.print('0');
      Serial.print(commCtrl, HEX);
      Serial.print(F("; role bits=0x"));
      Serial.println(commCtrl & 0x03, HEX);
    }
  }

  Serial.print(F("\nValid individual DIR0_ADDR responders: "));
  Serial.println(responders);
  return responders;
}
void printSingleDevicePartID(uint8_t dev) {
  uint8_t partid = 0;

  Serial.println(F("\n=== SINGLE-DEVICE PARTID CHECK ==="));
  if (!readByte(dev, PARTID, partid)) {
    Serial.print(F("Device "));
    Serial.print(dev);
    Serial.println(F(" PARTID read failed."));
    return;
  }

  Serial.print(F("Device "));
  Serial.print(dev);
  Serial.print(F(" PARTID = 0x"));
  if (partid < 0x10) Serial.print('0');
  Serial.println(partid, HEX);
  if (partid != 0x21) {
    Serial.println(F("WARNING: expected 0x21 for BQ79616."));
  } else {
    testSucceeded = true;
  }
}

// ---------------- Stack setup / prior diagnostic helpers ----------------
bool enableBaseManualComh() {
  Serial.println(F("Enabling the known-good temporary base COMH override."));
  if (!singleWrite(BASE_ADDR, DEBUG_CTRL_UNLOCK, 0xA5)) return false;
  if (!singleWrite(BASE_ADDR, DEBUG_COMM_CTRL2, 0x03)) return false;
  if (!singleWrite(BASE_ADDR, DEBUG_COMM_CTRL1, 0x05)) return false;

  uint8_t stat = 0;
  if (!readByte(BASE_ADDR, DEBUG_COMM_STAT, stat)) {
    Serial.println(F("Could not verify base COMH override status."));
    return false;
  }
  Serial.print(F("Base DEBUG_COMM_STAT = 0x"));
  if (stat < 0x10) Serial.print('0');
  Serial.println(stat, HEX);
  Serial.println(F("Expected lower bits 0x03: COMH TX/RX on, COML TX/RX off."));
  return (stat & 0x1F) == 0x03;
}

bool sendDllDummyWrite() {
  Serial.println(F("DLL synchronization: TI dummy broadcast write OTP_ECC_TEST (0x034C)=0x00."));
  return broadcastWrite(OTP_ECC_TEST, 0x00);
}

void sendDllDummyRead(uint8_t expectedDevices) {
  size_t got = 0;
  Serial.println(F("DLL synchronization: TI dummy broadcast read OTP_ECC_TEST (0x034C), one byte per device."));
  broadcastReadBurst(OTP_ECC_TEST, 1, expectedDevices, got);
  Serial.print(F("OTP_ECC_TEST DLL dummy-read bytes observed: "));
  Serial.println(got);
}

bool readIdentity(uint8_t dev, uint8_t expectedRole, const char *label) {
  uint8_t address = 0;
  uint8_t role = 0;
  uint8_t partid = 0;
  const bool addressOk = readByte(dev, DIR0_ADDR, address);
  const bool roleOk = readByte(dev, COMM_CTRL, role);
  const bool partidOk = readByte(dev, PARTID, partid);

  Serial.print(F("\n"));
  Serial.println(label);
  Serial.print(F("  DIR0_ADDR: "));
  if (addressOk) Serial.println(address, HEX);
  else Serial.println(F("<no response>"));
  Serial.print(F("  COMM_CTRL: "));
  if (roleOk) Serial.println(role, HEX);
  else Serial.println(F("<no response>"));
  Serial.print(F("  PARTID: "));
  if (partidOk) Serial.println(partid, HEX);
  else Serial.println(F("<no response>"));

  return addressOk && address == dev && roleOk && role == expectedRole &&
         partidOk && partid == 0x21;
}

bool readDevice1RoleAndComhTx(uint8_t expectedRole, bool expectedComhTx,
                              const char *stage) {
  uint8_t role = 0;
  uint8_t stat = 0;
  Serial.print(F("\n=== DEVICE 1 ROLE / COMH STATUS: "));
  Serial.print(stage);
  Serial.println(F(" ==="));

  const bool roleOk = readByte(1, COMM_CTRL, role);
  const bool statOk = readByte(1, DEBUG_COMM_STAT, stat);
  if (roleOk) {
    Serial.print(F("COMM_CTRL=0x"));
    if (role < 0x10) Serial.print('0');
    Serial.println(role, HEX);
  } else {
    Serial.println(F("COMM_CTRL=<read failed>"));
  }
  if (statOk) {
    Serial.print(F("DEBUG_COMM_STAT=0x"));
    if (stat < 0x10) Serial.print('0');
    Serial.print(stat, HEX);
    Serial.print(F("; COMH_TX_ON="));
    Serial.println((stat & 0x02) ? '1' : '0');
  } else {
    Serial.println(F("DEBUG_COMM_STAT=<read failed>"));
  }

  return roleOk && (role & 0x03) == expectedRole && statOk &&
         (((stat & 0x02) != 0) == expectedComhTx);
}

bool latchComhCounterWindow(uint8_t dev, const char *stage,
                            uint8_t &discarded, uint16_t &valid) {
  uint8_t validHi = 0;
  uint8_t validLo = 0;
  Serial.print(F("\n=== COMH COUNTER WINDOW: "));
  Serial.print(stage);
  Serial.println(F(" ==="));

  if (!readByte(dev, DEBUG_COMH_DISCARD, discarded)) {
    Serial.println(F("Could not read DEBUG_COMH_DISCARD."));
    return false;
  }
  if (!readByte(dev, DEBUG_COMH_VALID_HI, validHi) ||
      !readByte(dev, DEBUG_COMH_VALID_LO, validLo)) {
    Serial.println(F("Could not read the latched DEBUG_COMH_VALID count."));
    return false;
  }
  valid = (static_cast<uint16_t>(validHi) << 8) | validLo;

  Serial.print(F("COMH discarded frames = "));
  Serial.println(discarded);
  Serial.print(F("COMH valid frames = "));
  Serial.println(valid);
  Serial.println(F("Counter window was reset by the DEBUG_COMH_DISCARD read; fault flags were not cleared."));
  return true;
}

bool waitForMainAdc(uint8_t &finalStatus, bool &startedOneShot) {
  uint8_t adcCtrl = 0;
  uint8_t adcStatus = 0;
  startedOneShot = false;

  Serial.println(F("\n=== ADDRESS-1 MAIN ADC STATUS ==="));
  if (!readByte(1, ADC_CTRL1, adcCtrl) || !readByte(1, ADC_STAT1, adcStatus)) {
    Serial.println(F("Could not read ADC control/status from address 1."));
    return false;
  }

  const uint8_t mode = adcCtrl & ADC_MAIN_MODE_MASK;
  Serial.print(F("ADC_CTRL1=0x"));
  if (adcCtrl < 0x10) Serial.print('0');
  Serial.print(adcCtrl, HEX);
  Serial.print(F("; MAIN_MODE="));
  Serial.print(mode);
  Serial.print(F("; DRDY_MAIN_ADC="));
  Serial.println((adcStatus & ADC_MAIN_READY_MASK) ? F("1") : F("0"));

  if (mode == 0 || mode == ADC_MAIN_8RR_MODE) {
    const uint8_t startValue = static_cast<uint8_t>(
        (adcCtrl & ADC_LPF_PRESERVE_MASK) |
        ADC_MAIN_8RR_MODE | ADC_MAIN_GO_MASK);
    Serial.println(mode == 0
        ? F("Main ADC is stopped; starting one 8-round-robin conversion.")
        : F("Main ADC is configured for 8RR; starting one fresh 8-round-robin conversion."));
    Serial.println(F("This volatile ADC_CTRL1 write preserves the two LPF enable bits and does not touch OTP."));
    if (!singleWrite(1, ADC_CTRL1, startValue)) {
      Serial.println(F("Nano UART did not accept the Main ADC start command."));
      return false;
    }
    startedOneShot = true;
    if (adcCtrl & 0x08) {
      Serial.println(F("Note: TI says filtering is not effective in 8RR mode; the returned values are raw single-run samples."));
    }
  } else if (mode == 0x02) {
    Serial.println(F("Main ADC is configured for continuous mode; leaving its run configuration unchanged."));
  } else {
    Serial.println(F("ERROR: MAIN_MODE=3 is reserved; no ADC control write will be sent."));
    return false;
  }

  const uint32_t start = millis();
  do {
    if (!readByte(1, ADC_STAT1, adcStatus)) {
      Serial.println(F("ADC_STAT1 read failed while waiting for cell data."));
      return false;
    }
    if (adcStatus & ADC_MAIN_READY_MASK) {
      finalStatus = adcStatus;
      Serial.print(F("Main ADC data ready; ADC_STAT1=0x"));
      if (adcStatus < 0x10) Serial.print('0');
      Serial.println(adcStatus, HEX);
      return true;
    }
    delay(2);
  } while (millis() - start < ADC_READY_TIMEOUT_MS);

  Serial.println(F("ERROR: DRDY_MAIN_ADC did not set before the 100 ms wait expired."));
  return false;
}

bool readAddress1CellResults(uint8_t cellCount, uint8_t &numericResults) {
  bool allReadsOk = true;
  numericResults = 0;
  Serial.println(F("\n=== ADDRESS-1 CELL VOLTAGE RESULTS ==="));
  Serial.println(F("VCELL results are signed 16-bit counts; voltage = count * 190.73 uV."));
  Serial.println(F("Each cell is read HI then LO so the device's low-byte latch is released immediately."));

  for (uint8_t cell = 1; cell <= cellCount; ++cell) {
    const uint16_t highReg = static_cast<uint16_t>(VCELL1_HI - 2U * (cell - 1U));
    uint8_t high = 0;
    uint8_t low = 0;
    Serial.print(F("Cell "));
    Serial.print(cell);
    Serial.print(F(" (VCELL HI/LO 0x"));
    Serial.print(highReg, HEX);
    Serial.print(F("/0x"));
    Serial.print(highReg + 1, HEX);
    Serial.println(')');

    if (!readByte(1, highReg, high)) {
      Serial.println(F("  ERROR: high-byte read failed."));
      allReadsOk = false;
      continue;
    }
    if (!readByte(1, highReg + 1, low)) {
      Serial.println(F("  ERROR: low-byte read failed."));
      allReadsOk = false;
      continue;
    }

    const uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;
    Serial.print(F("  raw=0x"));
    if (raw < 0x1000) Serial.print('0');
    if (raw < 0x0100) Serial.print('0');
    if (raw < 0x0010) Serial.print('0');
    Serial.print(raw, HEX);
    if (raw == 0x8000) {
      Serial.println(F("; 0x8000 sentinel (inactive, not yet converted, or unavailable)"));
      continue;
    }

    int32_t signedCounts = raw;
    if (raw & 0x8000) signedCounts -= 0x10000L;
    const float millivolts = signedCounts * (VCELL_LSB_UV / 1000.0f);
    Serial.print(F("; signed counts="));
    Serial.print(signedCounts);
    Serial.print(F("; "));
    Serial.print(millivolts, 2);
    Serial.println(F(" mV"));
    ++numericResults;
  }
  return allReadsOk;
}

// Stack Read returns one frame per configured stack device, ToS first.
// The base device is not part of this response burst.
bool verifyStackReadDir0(uint8_t totalDevices, const char *label) {
  constexpr size_t FRAME_SIZE = ONE_BYTE_RESPONSE_SIZE;
  bool seen[MAX_CANDIDATE_COUNT] = {false};
  bool ok = true;
  size_t got = 0;

  Serial.print(F("\n=== STACK READ DIR0_ADDR: "));
  Serial.print(label);
  Serial.println(F(" ==="));
  Serial.print(F("Expected "));
  Serial.print(totalDevices - 1);
  Serial.println(F(" response frame(s), ordered top-of-stack down to address 1."));

  if (totalDevices < 2 || totalDevices > MAX_CANDIDATE_COUNT) {
    Serial.println(F("ERROR: invalid total device count for Stack Read check."));
    return false;
  }
  if (!stackReadBurst(DIR0_ADDR, 1, totalDevices - 1, got)) return false;

  if (got % FRAME_SIZE != 0) {
    Serial.print(F("ERROR: Stack Read burst has "));
    Serial.print(got % FRAME_SIZE);
    Serial.println(F(" trailing byte(s), not a whole number of frames."));
    ok = false;
  }

  const size_t frameCount = got / FRAME_SIZE;
  Serial.print(F("Stack response frames received: "));
  Serial.println(frameCount);

  for (size_t i = 0; i < frameCount; ++i) {
    const uint8_t *f = &rxBuf[i * FRAME_SIZE];
    if (!validCRC(f, FRAME_SIZE)) {
      Serial.print(F("Frame "));
      Serial.print(i);
      Serial.println(F(": CRC FAIL"));
      ok = false;
      continue;
    }
    if (f[0] != 0x00 || f[2] != (DIR0_ADDR >> 8) ||
        f[3] != (DIR0_ADDR & 0xFF)) {
      Serial.print(F("Frame "));
      Serial.print(i);
      Serial.println(F(": INIT/register mismatch"));
      ok = false;
      continue;
    }

    const uint8_t addr = f[1];
    const uint8_t value = f[4];
    Serial.print(F("  frame "));
    Serial.print(i);
    Serial.print(F(": response address="));
    Serial.print(addr);
    Serial.print(F(", DIR0_ADDR=0x"));
    if (value < 0x10) Serial.print('0');
    Serial.println(value, HEX);

    if (addr == 0 || addr >= totalDevices || addr >= MAX_CANDIDATE_COUNT) {
      Serial.println(F("    ERROR: response address is outside the candidate stack."));
      ok = false;
      continue;
    }
    if (value != addr) {
      Serial.println(F("    ERROR: DIR0_ADDR value does not match response address."));
      ok = false;
    }
    if (seen[addr]) {
      Serial.println(F("    ERROR: duplicate response address."));
      ok = false;
    }
    seen[addr] = true;

    const uint8_t expectedAddr = static_cast<uint8_t>(totalDevices - 1 - i);
    if (addr != expectedAddr) {
      Serial.print(F("    ERROR: expected top-first address "));
      Serial.println(expectedAddr);
      ok = false;
    }
  }

  if (frameCount != totalDevices - 1) {
    Serial.print(F("ERROR: received "));
    Serial.print(frameCount);
    Serial.print(F(" frame(s); expected "));
    Serial.println(totalDevices - 1);
    ok = false;
  }
  for (uint8_t addr = 1; addr < totalDevices; ++addr) {
    if (!seen[addr]) {
      Serial.print(F("MISSING Stack Read response from address "));
      Serial.println(addr);
      ok = false;
    }
  }

  Serial.println(ok ? F("Stack Read address/order check passed.")
                    : F("Stack Read address/order check failed."));
  return ok;
}

bool verifyStackReadRegister(uint16_t reg, uint8_t totalDevices,
                             const char *label) {
  constexpr size_t FRAME_SIZE = ONE_BYTE_RESPONSE_SIZE;
  const uint8_t expectedFrames = totalDevices - 1;
  bool seen[MAX_CANDIDATE_COUNT] = {false};
  bool ok = true;
  size_t got = 0;

  Serial.print(F("\n=== STACK READ REGISTER 0x"));
  if (reg < 0x1000) Serial.print('0');
  Serial.print(reg, HEX);
  Serial.print(F(": "));
  Serial.println(label);

  if (totalDevices < 2 || totalDevices > MAX_CANDIDATE_COUNT ||
      !stackReadBurst(reg, 1, expectedFrames, got)) {
    Serial.println(F("ERROR: invalid Stack Read request or transfer failure."));
    return false;
  }
  if (got % FRAME_SIZE != 0) {
    Serial.print(F("ERROR: Stack Read burst has "));
    Serial.print(got % FRAME_SIZE);
    Serial.println(F(" trailing byte(s)."));
    ok = false;
  }

  const size_t frameCount = got / FRAME_SIZE;
  Serial.print(F("Stack response frames received: "));
  Serial.println(frameCount);
  for (size_t i = 0; i < frameCount; ++i) {
    const uint8_t *f = &rxBuf[i * FRAME_SIZE];
    if (!validCRC(f, FRAME_SIZE)) {
      Serial.print(F("Frame "));
      Serial.print(i);
      Serial.println(F(": CRC FAIL"));
      ok = false;
      continue;
    }
    if (f[0] != 0x00 || f[2] != (reg >> 8) || f[3] != (reg & 0xFF)) {
      Serial.print(F("Frame "));
      Serial.print(i);
      Serial.println(F(": INIT/register mismatch"));
      ok = false;
      continue;
    }

    const uint8_t addr = f[1];
    const uint8_t value = f[4];
    Serial.print(F("  address "));
    Serial.print(addr);
    Serial.print(F(" value=0x"));
    if (value < 0x10) Serial.print('0');
    Serial.print(value, HEX);

    if (reg == GPIO_CONF1) {
      Serial.print(F("; SPI_EN="));
      Serial.print((value & 0x40) ? '1' : '0');
    }
    Serial.println();

    if (addr == 0 || addr >= totalDevices || addr >= MAX_CANDIDATE_COUNT) {
      Serial.println(F("    ERROR: response address is outside the stack."));
      ok = false;
      continue;
    }
    if (seen[addr]) {
      Serial.println(F("    ERROR: duplicate response address."));
      ok = false;
    }
    seen[addr] = true;

    if (i >= expectedFrames) {
      Serial.println(F("    ERROR: unexpected extra Stack Read frame."));
      ok = false;
      continue;
    }
    const uint8_t expectedAddr = static_cast<uint8_t>(totalDevices - 1 - i);
    if (addr != expectedAddr) {
      Serial.print(F("    ERROR: expected top-first address "));
      Serial.println(expectedAddr);
      ok = false;
    }

    if (reg == PARTID && value != 0x21) {
      Serial.println(F("    ERROR: PARTID is not 0x21."));
      ok = false;
    } else if (reg == COMM_CTRL) {
      const uint8_t expectedRole = (addr == totalDevices - 1) ? COMM_TOP : COMM_STACK;
      if ((value & 0x03) != expectedRole) {
        Serial.print(F("    ERROR: expected COMM_CTRL role bits 0x"));
        Serial.println(expectedRole, HEX);
        ok = false;
      }
    }
  }

  if (frameCount != expectedFrames) {
    Serial.print(F("ERROR: expected "));
    Serial.print(expectedFrames);
    Serial.println(F(" stack frames."));
    ok = false;
  }
  for (uint8_t addr = 1; addr < totalDevices; ++addr) {
    if (!seen[addr]) {
      Serial.print(F("MISSING response from address "));
      Serial.println(addr);
      ok = false;
    }
  }

  Serial.println(ok ? F("Stack register check passed.")
                    : F("Stack register check failed."));
  return ok;
}

bool prepareOneDeviceMainAdc(uint8_t addr, uint8_t &activeCell,
                             uint8_t &adcControl) {
  uint8_t status = 0;
  if (!readByte(addr, ACTIVE_CELL, activeCell) ||
      !readByte(addr, ADC_CTRL1, adcControl) ||
      !readByte(addr, ADC_STAT1, status)) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": failed to read ACTIVE_CELL/ADC_CTRL1/ADC_STAT1."));
    return false;
  }

  const uint8_t mode = adcControl & ADC_MAIN_MODE_MASK;
  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.print(F(": ACTIVE_CELL=0x"));
  if (activeCell < 0x10) Serial.print('0');
  Serial.print(activeCell, HEX);
  Serial.print(F(" ADC_CTRL1=0x"));
  if (adcControl < 0x10) Serial.print('0');
  Serial.print(adcControl, HEX);
  Serial.print(F(" MAIN_MODE="));
  Serial.print(mode);
  Serial.print(F(" DRDY_MAIN_ADC="));
  Serial.println((status & ADC_MAIN_READY_MASK) ? '1' : '0');

  if (mode == 0 || mode == ADC_MAIN_8RR_MODE) {
    const uint8_t startValue = static_cast<uint8_t>(
        (adcControl & ADC_LPF_PRESERVE_MASK) |
        ADC_MAIN_8RR_MODE | ADC_MAIN_GO_MASK);
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": starting one 8-round-robin Main ADC run."));
    if (!singleWrite(addr, ADC_CTRL1, startValue)) {
      Serial.println(F("UART did not accept the ADC_CTRL1 start frame."));
      return false;
    }
    if (adcControl & 0x08) {
      Serial.println(F("LPF_VCELL_EN is set; TI says filtering is ineffective in 8RR mode."));
    }
    status &= static_cast<uint8_t>(~ADC_MAIN_READY_MASK);  // MAIN_GO clears DRDY.
  } else if (mode == 0x02) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": continuous mode left unchanged; waiting for DRDY_MAIN_ADC."));
  } else {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": MAIN_MODE=3 is reserved; no ADC write sent."));
    return false;
  }

  const uint32_t waitStart = millis();
  while (!(status & ADC_MAIN_READY_MASK) &&
         millis() - waitStart < ADC_READY_TIMEOUT_MS) {
    delay(2);
    if (!readByte(addr, ADC_STAT1, status)) {
      Serial.print(F("Address "));
      Serial.print(addr);
      Serial.println(F(": ADC_STAT1 poll failed."));
      return false;
    }
  }
  if (!(status & ADC_MAIN_READY_MASK)) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": Main ADC did not become ready before timeout."));
    return false;
  }

  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.println(F(": Main ADC ready; proceeding to this address only."));
  return true;
}

bool readOneDeviceCellBlock(uint8_t addr, float &sumMillivolts,
                            uint16_t &validResults) {
  constexpr uint16_t FIRST_VCELL_HI = static_cast<uint16_t>(
      VCELL1_HI - 2U * (MODULE_CELL_COUNT - 1U));
  constexpr uint8_t DATA_LENGTH = 2U * MODULE_CELL_COUNT;
  uint8_t data[DATA_LENGTH] = {};
  bool ok = true;

  Serial.print(F("\n=== SINGLE-DEVICE VCELL READ, ADDRESS "));
  Serial.print(addr);
  Serial.print(F(", START 0x"));
  Serial.print(FIRST_VCELL_HI, HEX);
  Serial.println(F(" ==="));
  Serial.println(F("Only the addressed device returns its 24-byte VCELL12_HI..VCELL1_LO block."));

  if (!readBytes(addr, FIRST_VCELL_HI, DATA_LENGTH, data)) return false;

  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.print(F(": "));
  for (uint8_t cell = 1; cell <= MODULE_CELL_COUNT; ++cell) {
    if (cell > 1) Serial.print(F(" | "));
    Serial.print('C');
    if (cell < 10) Serial.print('0');
    Serial.print(cell);
    Serial.print('=');

    const uint16_t highReg = static_cast<uint16_t>(
        VCELL1_HI - 2U * (cell - 1U));
    const uint8_t dataOffset = static_cast<uint8_t>(highReg - FIRST_VCELL_HI);
    const uint16_t raw =
        (static_cast<uint16_t>(data[dataOffset]) << 8) |
        data[dataOffset + 1];
    if (raw == 0x8000) {
      Serial.print(F("0x8000_SENTINEL"));
      ok = false;
      continue;
    }

    int32_t signedCounts = raw;
    if (raw & 0x8000) signedCounts -= 0x10000L;
    const float millivolts = signedCounts * 0.19073f;
    sumMillivolts += millivolts;
    ++validResults;

    Serial.print(millivolts, 2);
    Serial.print(F("mV[0x"));
    if (raw < 0x1000) Serial.print('0');
    if (raw < 0x0100) Serial.print('0');
    if (raw < 0x0010) Serial.print('0');
    Serial.print(raw, HEX);
    Serial.print(']');
  }
  Serial.println();
  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.print(F(" 12-cell sum = "));
  Serial.print(sumMillivolts / 1000.0f, 3);
  Serial.println(F(" V"));
  return ok;
}

bool gpioModeSupportsAdc(uint8_t mode) {
  return mode == 0x01 || mode == 0x02 || mode == 0x06 || mode == 0x07;
}

bool readOneDeviceGpioConfig(uint8_t addr, uint8_t config[4],
                             uint8_t &adcMask) {
  adcMask = 0;
  if (!readBytes(addr, GPIO_CONF1, 4, config)) return false;

  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.print(F(" GPIO config:"));
  for (uint8_t gpio = 1; gpio <= 8; ++gpio) {
    const uint8_t confByte = config[(gpio - 1) / 2];
    const uint8_t shift = ((gpio - 1) % 2) * 3;
    const uint8_t mode = (confByte >> shift) & 0x07;
    bool adcEnabled = gpioModeSupportsAdc(mode);

    // GPIO4..7 are taken over when SPI_EN is set; GPIO8 is taken over by
    // FAULT_IN_EN. Do not report those pins as thermistor ADC inputs then.
    if (gpio >= 4 && gpio <= 7 && (config[0] & GPIO_SPI_EN_MASK)) {
      adcEnabled = false;
    }
    if (gpio == 8 && (config[0] & GPIO_FAULT_IN_EN_MASK)) {
      adcEnabled = false;
    }

    Serial.print(F(" GPIO"));
    Serial.print(gpio);
    Serial.print('=' );
    if (mode == 0x01) Serial.print(F("ADC_OTUT"));
    else if (mode == 0x02) Serial.print(F("ADC_ONLY"));
    else if (mode == 0x06) Serial.print(F("ADC_PULLUP"));
    else if (mode == 0x07) Serial.print(F("ADC_PULLDOWN"));
    else if (mode == 0x00) Serial.print(F("DISABLED"));
    else Serial.print(F("NON_ADC"));
    if (adcEnabled) adcMask |= static_cast<uint8_t>(1U << (gpio - 1));
  }
  Serial.println();
  return true;
}

uint8_t countConfiguredGpios(uint8_t adcMask) {
  uint8_t count = 0;
  for (uint8_t gpio = 0; gpio < 8; ++gpio) {
    if (adcMask & static_cast<uint8_t>(1U << gpio)) ++count;
  }
  return count;
}

bool enableTsrefForNtcRead(uint8_t addr, bool &changedByThisRun) {
  changedByThisRun = false;

  uint8_t control2 = 0;
  if (!readByte(addr, CONTROL2, control2)) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": could not read CONTROL2/TSREF_EN."));
    return false;
  }

  if (control2 & TSREF_EN_MASK) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": TSREF was already enabled."));
    return true;
  }

  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.println(F(": temporarily enabling TSREF; SEND_HW_RESET remains 0."));
  // CONTROL2 bit 1 is SEND_HW_RESET. Write only TSREF_EN=1 so no reset tone
  // can be requested and no other reserved/control bits are replayed.
  if (!singleWrite(addr, CONTROL2, TSREF_EN_MASK)) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": TSREF enable write was not accepted by the UART."));
    return false;
  }
  changedByThisRun = true;
  delay(2);  // TI specifies at least 1.35 ms for TSREF startup.
  uint8_t verify = 0;
  if (!readByte(addr, CONTROL2, verify) || !(verify & TSREF_EN_MASK)) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": TSREF_EN did not verify after the write."));
    return false;
  }
  return true;
}

bool restoreTsrefAfterNtcRead(uint8_t addr, bool changedByThisRun) {
  if (!changedByThisRun) return true;
  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.println(F(": restoring TSREF_EN=0 to its original state."));
  // The original value was disabled, so write 0 to both TSREF_EN and the
  // SEND_HW_RESET command bit. Other CONTROL2 bits are reserved.
  if (!singleWrite(addr, CONTROL2, 0x00)) {
    Serial.print(F("WARNING: could not restore TSREF_EN on address "));
    Serial.println(addr);
    return false;
  }
  uint8_t verify = 0;
  if (!readByte(addr, CONTROL2, verify) || (verify & TSREF_EN_MASK)) {
    Serial.print(F("WARNING: TSREF_EN=0 did not verify on address "));
    Serial.println(addr);
    return false;
  }
  return true;
}

int32_t decodeSignedAdcResult(uint16_t raw) {
  return (raw & 0x8000U) ? static_cast<int32_t>(raw) - 0x10000L
                          : static_cast<int32_t>(raw);
}

void printNtcTemperatureEstimate(float gpioToTsrefRatio) {
  if (NTC_PULLUP_OHMS <= 0.0f || NTC_R25_OHMS <= 0.0f || NTC_BETA_K <= 0.0f) {
  Serial.print(F(" temp_estimate=UNCONFIGURED"));
    return;
  }
  if (gpioToTsrefRatio <= 0.0f || gpioToTsrefRatio >= 1.0f) {
    Serial.print(F(" temp_estimate=INVALID_RATIO"));
    return;
  }

  // TI's Figure 8-6 divider: TSREF -> R1 -> GPIO node -> NTC -> GND,
  // assuming the optional parallel R2 is not populated.
  const float rNtc = NTC_PULLUP_OHMS * gpioToTsrefRatio /
                     (1.0f - gpioToTsrefRatio);
  const float invKelvin = (1.0f / NTC_T0_K) +
      logf(rNtc / NTC_R25_OHMS) / NTC_BETA_K;
  if (rNtc <= 0.0f || invKelvin <= 0.0f) {
    Serial.print(F(" temp_estimate=INVALID_CALC"));
    return;
  }
  Serial.print(F(" temp_estimate="));
  Serial.print((1.0f / invKelvin) - 273.15f, 2);
  Serial.print(F("C"));
}

bool readOneDeviceNtcResults(uint8_t addr, const uint8_t gpioConfig[4],
                             uint8_t adcGpioMask,
                             uint8_t &validChannels,
                             uint8_t &ntcCandidateChannels,
                             uint8_t &outsideNtcBandChannels,
                             uint8_t &nonstandardModeChannels) {
  if (!adcGpioMask) {
    Serial.print(F("Address "));
    Serial.print(addr);
    Serial.println(F(": no GPIO channels are configured for ADC."));
    return true;
  }

  // One contiguous Main ADC result read: TSREF, then GPIO1 through GPIO8.
  // The eight-round-robin conversion already performed for the cell pass
  // samples all GPIO slots without starting a separate AUX diagnostic.
  constexpr uint8_t RESULT_LENGTH = 18;
  uint8_t data[RESULT_LENGTH] = {};
  Serial.print(F("\n=== MAIN ADC TSREF/GPIO READ, ADDRESS "));
  Serial.print(addr);
  Serial.println(F(" ==="));
  if (!readBytes(addr, TSREF_HI, RESULT_LENGTH, data)) return false;

  const uint16_t tsrefRaw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  if (tsrefRaw == 0x8000U) {
    Serial.println(F("TSREF result is the 0x8000 reset/inactive sentinel."));
    return false;
  }
  const int32_t tsrefCounts = decodeSignedAdcResult(tsrefRaw);
  const float tsrefMillivolts = tsrefCounts * (TSREF_LSB_UV / 1000.0f);
  Serial.print(F("TSREF="));
  Serial.print(tsrefMillivolts, 2);
  Serial.print(F("mV[0x"));
  Serial.print(tsrefRaw, HEX);
  Serial.println(']');
  if (tsrefCounts <= 0) {
    Serial.println(F("TSREF result is not a positive reference; GPIO ratios cannot be calculated."));
    return false;
  }

  bool ok = true;
  for (uint8_t gpio = 1; gpio <= 8; ++gpio) {
    const uint8_t gpioMask = static_cast<uint8_t>(1U << (gpio - 1));
    if (!(adcGpioMask & gpioMask)) continue;

    const uint16_t gpioHiReg = static_cast<uint16_t>(
        GPIO1_HI + 2U * (gpio - 1U));
    const uint8_t offset = static_cast<uint8_t>(gpioHiReg - TSREF_HI);
    const uint16_t raw =
        (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
    Serial.print(F("GPIO"));
    Serial.print(gpio);
    Serial.print('=' );
    if (raw == 0x8000U) {
      Serial.print(F("0x8000_SENTINEL (inactive/unavailable)"));
      Serial.println();
      ok = false;
      continue;
    }

    const int32_t counts = decodeSignedAdcResult(raw);
    const float gpioMillivolts = counts * (GPIO_LSB_UV / 1000.0f);
    const float ratio = (static_cast<float>(counts) * GPIO_LSB_UV) /
                        (static_cast<float>(tsrefCounts) * TSREF_LSB_UV);
    Serial.print(gpioMillivolts, 2);
    Serial.print(F("mV[0x"));
    if (raw < 0x1000U) Serial.print('0');
    if (raw < 0x0100U) Serial.print('0');
    if (raw < 0x0010U) Serial.print('0');
    Serial.print(raw, HEX);
    Serial.print(F("] ratio="));
    Serial.print(ratio, 5);
    const uint8_t configByte = gpioConfig[(gpio - 1) / 2];
    const uint8_t configShift = ((gpio - 1) % 2) * 3;
    const uint8_t mode = (configByte >> configShift) & 0x07;
    if (mode == 0x01 || mode == 0x02) {
      if (ratio >= NTC_CANDIDATE_RATIO_MIN &&
          ratio <= NTC_CANDIDATE_RATIO_MAX) {
        Serial.print(F(" classification=NTC_CANDIDATE_PROVISIONAL"));
        printNtcTemperatureEstimate(ratio);
        ++ntcCandidateChannels;
      } else {
        Serial.print(F(" temp_estimate=SKIPPED_OUTSIDE_PROVISIONAL_NTC_BAND"));
        Serial.print(F(" classification=OUTSIDE_PROVISIONAL_NTC_BAND (raw retained)"));
        ++outsideNtcBandChannels;
      }
    } else {
      Serial.print(F(" temp_estimate=SKIPPED_NONSTANDARD_GPIO_CONFIG"));
      Serial.print(F(" classification=NONSTANDARD_CONFIG (raw retained)"));
      ++nonstandardModeChannels;
    }
    Serial.println();
    // A non-sentinel result proves the ADC returned data. An out-of-range
    // divider ratio is sensor/open/short information, not a UART read failure.
    ++validChannels;
  }
  return ok;
}

bool acquireStackAdcData(uint8_t totalDevices, const char *passLabel,
                         bool &allAdcReady, bool &cellReadPassed,
                         bool &ntcReadPassed, float sumMillivolts[],
                         uint16_t &validCellResults,
                         uint8_t &expectedNtcChannels,
                         uint8_t &validNtcChannels,
                         uint8_t &ntcCandidateChannels,
                         uint8_t &outsideNtcBandChannels,
                         uint8_t &nonstandardModeChannels) {
  if (totalDevices < 2 || totalDevices > MAX_CANDIDATE_COUNT) return false;

  allAdcReady = true;
  cellReadPassed = true;
  ntcReadPassed = true;
  validCellResults = 0;
  expectedNtcChannels = 0;
  validNtcChannels = 0;
  ntcCandidateChannels = 0;
  outsideNtcBandChannels = 0;
  nonstandardModeChannels = 0;
  for (uint8_t addr = 0; addr < totalDevices; ++addr) {
    sumMillivolts[addr] = 0.0f;
  }

  Serial.print(F("\n=== CELL AND GPIO ACQUISITION: "));
  Serial.print(passLabel);
  Serial.println(F(" ==="));
  Serial.println(F("For each address, read its GPIO config, start only its Main ADC, then read cells and GPIO/TSREF before moving on."));

  for (uint8_t addr = 1; addr < totalDevices; ++addr) {
    Serial.print(F("\n--- Processing address "));
    Serial.print(addr);
    Serial.print(F(" during "));
    Serial.print(passLabel);
    Serial.println(F(" ---"));

    uint8_t gpioConfig[4] = {};
    uint8_t adcGpioMask = 0;
    const bool gpioConfigOk = readOneDeviceGpioConfig(
        addr, gpioConfig, adcGpioMask);
    if (!gpioConfigOk) {
      ntcReadPassed = false;
      Serial.print(F("Address "));
      Serial.print(addr);
      Serial.println(F(": GPIO configuration read failed; NTC pass cannot validate this device."));
    }

    uint8_t activeCell = 0;
    uint8_t adcControl = 0;
    bool tsrefChangedByRun = false;
    bool tsrefReady = false;
    if (gpioConfigOk && adcGpioMask) {
      expectedNtcChannels += countConfiguredGpios(adcGpioMask);
      tsrefReady = enableTsrefForNtcRead(addr, tsrefChangedByRun);
      if (!tsrefReady) ntcReadPassed = false;
    }

    const bool adcReady = prepareOneDeviceMainAdc(addr, activeCell, adcControl);
    if (!adcReady) {
      allAdcReady = false;
      cellReadPassed = false;
      if (!restoreTsrefAfterNtcRead(addr, tsrefChangedByRun)) {
        ntcReadPassed = false;
      }
      continue;
    }
    if (!readOneDeviceCellBlock(addr, sumMillivolts[addr],
                                validCellResults)) {
      cellReadPassed = false;
    }
    if (gpioConfigOk && adcGpioMask) {
      if (tsrefReady) {
        if (!readOneDeviceNtcResults(addr, gpioConfig, adcGpioMask,
                                     validNtcChannels,
                                     ntcCandidateChannels,
                                     outsideNtcBandChannels,
                                     nonstandardModeChannels)) {
          ntcReadPassed = false;
        }
      } else {
        Serial.print(F("Address "));
        Serial.print(addr);
        Serial.println(F(": skipping GPIO/NTC ratio because TSREF is unavailable."));
      }
    }
    if (!restoreTsrefAfterNtcRead(addr, tsrefChangedByRun)) {
      ntcReadPassed = false;
    }
  }

  const uint16_t expectedCells =
      static_cast<uint16_t>(totalDevices - 1) * MODULE_CELL_COUNT;
  return allAdcReady && cellReadPassed && validCellResults == expectedCells &&
         ntcReadPassed && expectedNtcChannels > 0 &&
         validNtcChannels == expectedNtcChannels;
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  delay(SERIAL_STARTUP_DELAY_MS);

  Serial.println(F("\nSP51_basic_BMS | BQ79616-Q1 Nano R4 Stack/ADC Bring-up"));
  Serial.println(F("Assumes the successful Run 33 setup: ten devices, base 0 and stack addresses 1..9."));
  Serial.println(F("Uses Stack Read to verify the chain and ToS=9, then reads one addressed device at a time."));
  Serial.println(F("Reads 12 cell voltages plus every ADC-configured GPIO input and TSREF per stack device."));
  Serial.println(F("Provisional NTC model: R1=22k, R25=10k, beta=3380 K; values are estimates, not confirmed calibration."));
  Serial.println(F("Only ratios 0.38..0.50 get a temperature estimate; all raw ADC readings remain printed."));
  Serial.println(F("Replays Run 33 startup, then reads all stack data twice with the manual base COMH override held on."));
  Serial.println(F("The second pass rechecks the stack but does not reapply the override, replay addresses/roles, or DLL-resynchronize."));
  Serial.println(F("The base COMH override stays enabled; automatic daisy control is not restored."));
  Serial.println(F("No HW_RESET, customer OTP programming, fault clear, or GPIO/pyro writes."));
  Serial.println(F("ADC_CTRL1 may receive a volatile single 8RR start on devices not already in continuous mode."));
  Serial.println(F("CONTROL2[TSREF_EN] is enabled only if needed for a configured GPIO read, then restored to its original state."));
  Serial.println(F("GPIO and TSREF use TI's separate LSB scales; temperature estimates use the provisional NTC model."));
  Serial.println(F("Temporary base COMH override stays on across both passes and remains on after this test."));
  Serial.println(F("Do not run after BQ power cycle; use Run 33 to initialize if power was removed."));

  constexpr uint8_t TOTAL_DEVICES = 10;
  beginBQ();
  wakeBQOnly();
  Serial.print(F("Waiting "));
  Serial.print(WAKE_PROPAGATION_MS);
  Serial.println(F(" ms for the WAKE tone and startup, matching Run 33."));
  delay(WAKE_PROPAGATION_MS);

  bool manualComhOverrideAttempted = false;
  bool manualComhOverrideEnabled = false;
  bool manualComhOverrideStillOn = false;
  bool forwardSetupPassed = false;
  bool baseChecksPassed = false;
  bool dirReadPassed = false;
  bool roleReadPassed = false;
  bool partIdReadPassed = false;
  bool gpioReadPassed = false;
  bool allAdcReady = false;
  bool cellReadPassed = false;
  bool ntcReadPassed = true;
  bool firstAcquisitionPassed = false;
  bool secondDirReadPassed = false;
  bool secondRoleReadPassed = false;
  bool secondPartIdReadPassed = false;
  bool secondGpioReadPassed = false;
  bool secondAllAdcReady = false;
  bool secondCellReadPassed = false;
  bool secondNtcReadPassed = true;
  bool secondAcquisitionPassed = false;
  float sumMillivolts[TOTAL_DEVICES] = {0.0f};
  uint16_t validCellResults = 0;
  uint8_t expectedNtcChannels = 0;
  uint8_t validNtcChannels = 0;
  uint8_t ntcCandidateChannels = 0;
  uint8_t outsideNtcBandChannels = 0;
  uint8_t nonstandardModeChannels = 0;
  float secondSumMillivolts[TOTAL_DEVICES] = {0.0f};
  uint16_t secondValidCellResults = 0;
  uint8_t secondExpectedNtcChannels = 0;
  uint8_t secondValidNtcChannels = 0;
  uint8_t secondNtcCandidateChannels = 0;
  uint8_t secondOutsideNtcBandChannels = 0;
  uint8_t secondNonstandardModeChannels = 0;

  do {
    Serial.println(F("\n=== REPLAY VERIFIED FORWARD STARTUP BEFORE BASE READS ==="));
    Serial.println(F("Run 33 established that base reads follow forward DIR0 address/role setup."));
    if (!singleWrite(BASE_ADDR, CONTROL1, 0x00)) {
      Serial.println(F("Could not send base CONTROL1=0x00 to select forward DIR0/COMH."));
      break;
    }
    if (!singleWrite(9, COMM_CTRL, COMM_STACK)) {
      Serial.println(F("Could not send address 9 COMM_CTRL=STACK to reopen the known prior top."));
      break;
    }
    if (!sendDllDummyWrite()) {
      Serial.println(F("Could not send TI OTP_ECC_TEST DLL dummy write."));
      break;
    }
    if (!primeForwardDirectionForBaseRead(TOTAL_DEVICES)) {
      Serial.println(F("Could not send the ten-device DIR0 address/role prime."));
      break;
    }
    manualComhOverrideAttempted = true;
    manualComhOverrideEnabled = enableBaseManualComh();
    if (!manualComhOverrideEnabled) {
      Serial.println(F("Could not verify the temporary base COMH override."));
      break;
    }
    sendDllDummyRead(TOTAL_DEVICES);
    forwardSetupPassed = true;

    Serial.println(F("\n=== BASE SINGLE-DEVICE IDENTITY / CONFIG ==="));
    uint8_t basePartId = 0;
    uint8_t baseRole = 0;
    uint8_t baseDevConf = 0;
    uint8_t baseGpioConf = 0;
    const bool basePartOk = readByte(BASE_ADDR, PARTID, basePartId);
    const bool baseRoleOk = readByte(BASE_ADDR, COMM_CTRL, baseRole);
    const bool baseDevOk = readByte(BASE_ADDR, DEV_CONF, baseDevConf);
    const bool baseGpioOk = readByte(BASE_ADDR, GPIO_CONF1, baseGpioConf);
    baseChecksPassed = basePartOk && basePartId == 0x21 &&
                       baseRoleOk && ((baseRole & 0x03) == COMM_BASE) &&
                       baseDevOk && baseGpioOk;
    Serial.print(F("Base PARTID: "));
    if (basePartOk) Serial.println(basePartId, HEX);
    else Serial.println(F("read failed"));
    Serial.print(F("Base COMM_CTRL: "));
    if (baseRoleOk) Serial.println(baseRole, HEX);
    else Serial.println(F("read failed"));
    if (baseDevOk) {
      Serial.print(F("Base DEV_CONF=0x"));
      if (baseDevConf < 0x10) Serial.print('0');
      Serial.print(baseDevConf, HEX);
      Serial.print(F("; MULTIDROP_EN="));
      Serial.println((baseDevConf & MULTIDROP_EN_MASK) ? '1' : '0');
    }
    if (baseGpioOk) {
      Serial.print(F("Base GPIO_CONF1=0x"));
      if (baseGpioConf < 0x10) Serial.print('0');
      Serial.print(baseGpioConf, HEX);
      Serial.print(F("; SPI_EN="));
      Serial.println((baseGpioConf & 0x40) ? '1' : '0');
    }
    if (!baseChecksPassed) {
      Serial.println(F("Base identity/role did not verify; stopping without changing the stack."));
      break;
    }

    dirReadPassed = verifyStackReadDir0(TOTAL_DEVICES,
        "verify addresses 9..1 remain in the successful Run 33 stack");
    roleReadPassed = verifyStackReadRegister(COMM_CTRL, TOTAL_DEVICES,
        "expect addresses 1..8 as STACK and address 9 as ToS");
    partIdReadPassed = verifyStackReadRegister(PARTID, TOTAL_DEVICES,
        "expect PARTID=0x21 from every stack device");
    gpioReadPassed = verifyStackReadRegister(GPIO_CONF1, TOTAL_DEVICES,
        "read GPIO_CONF1/SPI_EN from each stack device; configuration only");

    const bool stackPreflightPassed = dirReadPassed && roleReadPassed &&
                                     partIdReadPassed && gpioReadPassed;
    if (!stackPreflightPassed) {
      Serial.println(F("Stack preflight failed; skipping ADC starts and voltage reads."));
      break;
    }

    firstAcquisitionPassed = acquireStackAdcData(
        TOTAL_DEVICES, "PASS 1 WITH TEMPORARY COMH OVERRIDE",
        allAdcReady, cellReadPassed, ntcReadPassed, sumMillivolts,
        validCellResults, expectedNtcChannels, validNtcChannels,
        ntcCandidateChannels, outsideNtcBandChannels,
        nonstandardModeChannels);

    if (!firstAcquisitionPassed) {
      Serial.println(F("Pass 1 did not fully pass; skipping the repeated acquisition."));
      break;
    }

    Serial.println(F("\n=== SECOND STACK PREFLIGHT WITH SAME COMH OVERRIDE ==="));
    secondDirReadPassed = verifyStackReadDir0(TOTAL_DEVICES,
        "recheck addresses 9..1 before the second data pass");
    if (!secondDirReadPassed) {
      Serial.println(F("Second-pass Stack Read failed; skipping remaining second-pass reads."));
      break;
    }
    secondRoleReadPassed = verifyStackReadRegister(COMM_CTRL, TOTAL_DEVICES,
        "recheck addresses 1..8 as STACK and address 9 as ToS");
    if (!secondRoleReadPassed) {
      Serial.println(F("Second-pass role check failed; skipping remaining second-pass reads."));
      break;
    }
    secondPartIdReadPassed = verifyStackReadRegister(PARTID, TOTAL_DEVICES,
        "recheck PARTID=0x21 from every stack device");
    if (!secondPartIdReadPassed) {
      Serial.println(F("Second-pass PARTID check failed; skipping remaining second-pass reads."));
      break;
    }
    secondGpioReadPassed = verifyStackReadRegister(GPIO_CONF1, TOTAL_DEVICES,
        "recheck GPIO_CONF1/SPI_EN from each stack device");
    if (!secondGpioReadPassed) {
      Serial.println(F("Second-pass GPIO configuration check failed; skipping ADC acquisition."));
      break;
    }

    secondAcquisitionPassed = acquireStackAdcData(
        TOTAL_DEVICES, "PASS 2 WITH SAME TEMPORARY COMH OVERRIDE",
        secondAllAdcReady, secondCellReadPassed, secondNtcReadPassed,
        secondSumMillivolts, secondValidCellResults,
        secondExpectedNtcChannels, secondValidNtcChannels,
        secondNtcCandidateChannels, secondOutsideNtcBandChannels,
        secondNonstandardModeChannels);
  } while (false);

  if (manualComhOverrideAttempted) {
    Serial.println(F("\n=== VERIFY BASE COMH OVERRIDE LEFT ENABLED ==="));
    uint8_t stat = 0;
    if (readByte(BASE_ADDR, DEBUG_COMM_STAT, stat)) {
      Serial.print(F("Base DEBUG_COMM_STAT at end = 0x"));
      if (stat < 0x10) Serial.print('0');
      Serial.println(stat, HEX);
      manualComhOverrideStillOn = (stat & 0x1F) == 0x03;
      Serial.println(manualComhOverrideStillOn
          ? F("Manual mode confirmed: COMH TX/RX on; automatic control remains overridden.")
          : F("WARNING: final status does not confirm manual COMH TX/RX on."));
    } else {
      Serial.println(F("Could not read final DEBUG_COMM_STAT; override state is unconfirmed."));
    }
  }

  testSucceeded = forwardSetupPassed && baseChecksPassed && dirReadPassed && roleReadPassed &&
                  partIdReadPassed && gpioReadPassed &&
                  firstAcquisitionPassed && manualComhOverrideEnabled &&
                  manualComhOverrideStillOn && secondDirReadPassed && secondRoleReadPassed &&
                  secondPartIdReadPassed && secondGpioReadPassed &&
                  secondAcquisitionPassed;
  Serial.println(F("\n=== SP51_basic_BMS RESULT ==="));
  Serial.print(F("Forward startup commands sent and base COMH verified: "));
  Serial.println(forwardSetupPassed ? F("yes") : F("no"));
  Serial.print(F("Base identity/role/config reads passed: "));
  Serial.println(baseChecksPassed ? F("yes") : F("no"));
  Serial.print(F("Stack identity/role/config preflight passed: "));
  Serial.println(dirReadPassed && roleReadPassed && partIdReadPassed && gpioReadPassed ? F("yes") : F("no"));
  Serial.print(F("Main ADC ready on addresses 1..9: "));
  Serial.println(allAdcReady ? F("yes") : F("no"));
  Serial.print(F("Non-sentinel VCELL results: "));
  Serial.print(validCellResults);
  Serial.print(F(" of "));
  Serial.println((TOTAL_DEVICES - 1) * MODULE_CELL_COUNT);
  Serial.print(F("All 108 VCELL values read: "));
  Serial.println(validCellResults == (TOTAL_DEVICES - 1) * MODULE_CELL_COUNT && cellReadPassed ? F("yes") : F("no"));
  Serial.print(F("Non-sentinel configured GPIO results: "));
  Serial.print(validNtcChannels);
  Serial.print(F(" of "));
  Serial.println(expectedNtcChannels);
  Serial.print(F("All configured GPIO ADC results were received: "));
  Serial.println(ntcReadPassed && expectedNtcChannels > 0 &&
                 validNtcChannels == expectedNtcChannels ? F("yes") : F("no"));
  Serial.print(F("Provisional NTC-ratio candidates: "));
  Serial.println(ntcCandidateChannels);
  Serial.print(F("Outside provisional NTC ratio band (still retained): "));
  Serial.println(outsideNtcBandChannels);
  Serial.print(F("Nonstandard GPIO configurations (raw retained): "));
  Serial.println(nonstandardModeChannels);
  Serial.print(F("Manual base COMH override enabled: "));
  Serial.println(manualComhOverrideEnabled ? F("yes") : F("no"));
  Serial.print(F("Manual base COMH override still enabled at test end: "));
  Serial.println(manualComhOverrideStillOn ? F("yes") : F("no"));
  Serial.println(F("Automatic base daisy control restored: no (intentionally left in manual mode)."));
  Serial.print(F("Second-pass Stack Read preflight with manual COMH held on passed: "));
  Serial.println(secondDirReadPassed && secondRoleReadPassed &&
                 secondPartIdReadPassed && secondGpioReadPassed ? F("yes") : F("no"));
  Serial.print(F("Second-pass cell results: "));
  Serial.print(secondValidCellResults);
  Serial.print(F(" of "));
  Serial.println((TOTAL_DEVICES - 1) * MODULE_CELL_COUNT);
  Serial.print(F("Second-pass configured GPIO results: "));
  Serial.print(secondValidNtcChannels);
  Serial.print(F(" of "));
  Serial.println(secondExpectedNtcChannels);
  Serial.print(F("Second-pass Main ADC ready on addresses 1..9: "));
  Serial.println(secondAllAdcReady ? F("yes") : F("no"));
  Serial.print(F("Second-pass all 108 cell values read: "));
  Serial.println(secondValidCellResults == (TOTAL_DEVICES - 1) * MODULE_CELL_COUNT &&
                 secondCellReadPassed ? F("yes") : F("no"));
  Serial.print(F("Second-pass all configured GPIO ADC results received: "));
  Serial.println(secondNtcReadPassed && secondExpectedNtcChannels > 0 &&
                 secondValidNtcChannels == secondExpectedNtcChannels
                     ? F("yes") : F("no"));
  Serial.print(F("Second-pass provisional NTC candidates: "));
  Serial.println(secondNtcCandidateChannels);
  Serial.print(F("Second-pass outside-band readings retained: "));
  Serial.println(secondOutsideNtcBandChannels);
  Serial.print(F("Second full acquisition with manual COMH still on passed: "));
  Serial.println(secondAcquisitionPassed ? F("yes") : F("no"));
  Serial.println(F("Ratio flags do not affect the ADC/UART pass result; candidate temperatures are model-only estimates."));
  Serial.println(F("Confirm the BMW pull-up, NTC curve, and channel mapping before treating estimates as calibrated temperatures."));
  Serial.println(F("=== SP51_basic_BMS COMPLETE ==="));
}

void loop() {
  const unsigned long now = millis();
  if (now - lastHeartbeat >= 1000) {
    lastHeartbeat = now;
    Serial.print(F("NANO ALIVE | uptime="));
    Serial.print(now / 1000);
    Serial.print(F("s | test: "));
    Serial.println(testSucceeded ? F("passed") : F("not passed"));
  }
  delay(5);
}


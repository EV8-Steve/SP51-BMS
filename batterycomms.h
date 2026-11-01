#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "settings.h"

// Capacity limits for the addressable BQ79616 monitor stack.
constexpr uint8_t BATTERY_MAX_STACK_MODULES = 12;
constexpr uint8_t BATTERY_CELLS_PER_MODULE = 12;
constexpr uint8_t BATTERY_GPIO_COUNT = 8;

struct ModuleReading {
  // BQ address (the base is address 0; cell modules are read from 1 upward).
  uint8_t address;
  // Read success and validity counts are independent for cell and GPIO ADCs.
  bool cellReadOk;
  bool gpioReadOk;
  uint8_t validCellCount;
  uint8_t adcGpioMask;
  uint8_t validGpioMask;
  uint8_t ntcEstimateMask;
  // Raw signed ADC words and converted engineering values, indexed by cell 1..12.
  uint16_t cellRaw[BATTERY_CELLS_PER_MODULE];
  float cellMv[BATTERY_CELLS_PER_MODULE];
  // GPIO ADC results; temperatureC is only an estimate for configured NTC pins.
  uint16_t gpioRaw[BATTERY_GPIO_COUNT];
  float gpioMv[BATTERY_GPIO_COUNT];
  float gpioRatio[BATTERY_GPIO_COUNT];
  float temperatureC[BATTERY_GPIO_COUNT];
  float tsrefMv;
  float minCellMv;
  float maxCellMv;
  float deltaCellMv;
  char error[64];
};

struct BatterySnapshot {
  // One complete sequential pass across the configured stack.
  uint8_t moduleCount;
  uint8_t completedModules;
  uint16_t validCellCount;
  bool allCellReadsOk;
  bool allGpioReadsOk;
  uint32_t startedAtMs;
  uint32_t finishedAtMs;
  uint32_t durationMs;
  ModuleReading modules[BATTERY_MAX_STACK_MODULES];
};

enum class BatteryScanStepResult : uint8_t {
  NotReady,
  InProgress,
  Complete
};

// Initialize UART, wake and configure the stack, and verify address/role/DLL
// responses. Returns false and fills error when any startup check fails.
bool batteryCommsBegin(const BmsSettings &settings, char *error,
                       size_t errorSize);
bool batteryCommsIsReady();
const char *batteryCommsLastError();

// Start one full pass, then call batteryCommsScanStep() until Complete or
// NotReady. Each step performs no more than one module's ADC reads.
void batteryCommsStartScan(BatterySnapshot &snapshot,
                           const BmsSettings &settings);
BatteryScanStepResult batteryCommsScanStep(BatterySnapshot &snapshot,
                                           const BmsSettings &settings);
// Abort an in-progress pass before reinitializing the stack.
void batteryCommsCancelScan();

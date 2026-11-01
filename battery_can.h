#pragma once

#include <Arduino.h>

#include "settings.h"

// Snapshot of LEM CAN link state and the most recently decoded current frame.
// currentMilliamp is valid only while valid is true; errorCode is the sensor's
// low seven status bits and sensorFault reflects its high status bit.
struct CurrentSensorStatus {
  bool canReady;
  bool frameSeen;
  bool valid;
  bool sensorFault;
  int32_t currentMilliamp;
  uint8_t errorCode;
  uint32_t lastFrameAtMs;
  uint32_t frameCount;
  uint32_t invalidFrameCount;
};

// Initialize or reconfigure the CAN controller from the current settings.
void batteryCanBegin(const BmsSettings &settings);
// Drain available CAN frames; call frequently from loop() to avoid backlog.
void batteryCanPoll(const BmsSettings &settings);
// Return current sensor status with stale-frame and sign settings applied.
CurrentSensorStatus batteryCanCurrent(const BmsSettings &settings);

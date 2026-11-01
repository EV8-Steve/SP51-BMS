/*
  USB serial user interface.

  This file owns the line-oriented configuration menus and fixed-width live
  battery display. Menu visibility suppresses display writes only; stack scans
  and state-machine supervision continue in the main loop.
*/
#include <Arduino.h>

#include "serial_console.h"

#include "charger_relay.h"
#include "bms_outputs.h"
#include "bms_state_machine.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr size_t COMMAND_BUFFER_SIZE = 64;
constexpr size_t DISPLAY_BUFFER_SIZE = 6144;
// Stable padded width reduces column movement in ordinary serial terminals.
constexpr size_t DISPLAY_FRAME_WIDTH = 160;
char commandBuffer[COMMAND_BUFFER_SIZE] = {};
char displayBuffer[DISPLAY_BUFFER_SIZE] = {};
size_t commandLength = 0;
bool menuOpen = false;
bool retryRequested = false;

enum class MenuPage : uint8_t {
  Main = 0,
  BatteryLimits,
  Charging,
  CurrentSensor,
  SocEstimation,
  Outputs,
  Monitor,
  StateControl
};

MenuPage menuPage = MenuPage::Main;

void appendText(char *buffer, size_t capacity, size_t &length, bool &ok,
                const char *text) {
  if (!ok) return;
  const size_t textLength = strlen(text);
  if (textLength >= capacity - length) {
    ok = false;
    return;
  }
  memcpy(buffer + length, text, textLength);
  length += textLength;
  buffer[length] = '\0';
}

void appendChar(char *buffer, size_t capacity, size_t &length, bool &ok,
                char value) {
  if (!ok) return;
  if (length + 1U >= capacity) {
    ok = false;
    return;
  }
  buffer[length++] = value;
  buffer[length] = '\0';
}

void appendRepeated(char *buffer, size_t capacity, size_t &length, bool &ok,
                    char value, size_t count) {
  while (ok && count) {
    appendChar(buffer, capacity, length, ok, value);
    --count;
  }
}

void appendFormat(char *buffer, size_t capacity, size_t &length, bool &ok,
                  const char *format, ...) {
  if (!ok || length >= capacity) return;
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buffer + length, capacity - length, format, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= capacity - length) {
    ok = false;
    return;
  }
  length += static_cast<size_t>(written);
}

void appendFixed(char *buffer, size_t capacity, size_t &length, bool &ok,
                 float value, uint8_t decimals, uint8_t width) {
  if (!ok) return;
  if (value != value || value > 1000000.0f || value < -1000000.0f) {
    for (uint8_t i = 3; i < width; ++i) {
      appendChar(buffer, capacity, length, ok, ' ');
    }
    appendText(buffer, capacity, length, ok, "---");
    return;
  }

  uint32_t scale = 1;
  for (uint8_t i = 0; i < decimals; ++i) scale *= 10U;
  const bool negative = value < 0.0f;
  const float magnitude = negative ? -value : value;
  const uint32_t scaled = static_cast<uint32_t>(magnitude * scale + 0.5f);
  const uint32_t whole = scaled / scale;
  const uint32_t fraction = scaled % scale;
  char number[24];
  if (decimals == 1) {
    snprintf(number, sizeof(number), "%s%lu.%01lu", negative ? "-" : "",
             static_cast<unsigned long>(whole),
             static_cast<unsigned long>(fraction));
  } else if (decimals == 3) {
    snprintf(number, sizeof(number), "%s%lu.%03lu", negative ? "-" : "",
             static_cast<unsigned long>(whole),
             static_cast<unsigned long>(fraction));
  } else {
    snprintf(number, sizeof(number), "%s%lu", negative ? "-" : "",
             static_cast<unsigned long>(scaled));
  }

  const size_t numberLength = strlen(number);
  for (size_t i = numberLength; i < width; ++i) {
    appendChar(buffer, capacity, length, ok, ' ');
  }
  appendText(buffer, capacity, length, ok, number);
}

void appendVoltage(char *buffer, size_t capacity, size_t &length, bool &ok,
                   float millivolts) {
  appendFixed(buffer, capacity, length, ok, millivolts / 1000.0f, 3, 5);
}

void appendCellValue(char *buffer, size_t capacity, size_t &length, bool &ok,
                     const ModuleReading &module, uint8_t cellIndex) {
  if (!module.validCellCount || module.cellRaw[cellIndex] == 0x8000U) {
    appendText(buffer, capacity, length, ok, "  ---");
    return;
  }
  appendVoltage(buffer, capacity, length, ok, module.cellMv[cellIndex]);
}

const char *currentSensorName(CurrentSensorType type) {
  return type == CurrentSensorType::LemCab500Can ? "LEM CAB500-C/SP7 CAN" : "OFF";
}

const char *chargerName(ChargerType type) {
  switch (type) {
    case ChargerType::ExternalRelay: return "External relay";
    case ChargerType::BrusaNlg5: return "Brusa NLG5";
    case ChargerType::Volt: return "Volt";
    case ChargerType::Eltek: return "Eltek";
    case ChargerType::Elcon: return "Elcon";
    case ChargerType::VictronSma: return "Victron/SMA";
    case ChargerType::Outlander: return "Outlander OBC";
    default: return "None";
  }
}

void printOutputPin(const char *label, uint8_t pin) {
  Serial.print(label);
  if (pin == 0xFFU) Serial.println(F("unassigned"));
  else { Serial.print('D'); Serial.println(pin); }
}

void printFloatSetting(const __FlashStringHelper *label, float value,
                       uint8_t decimals, const __FlashStringHelper *unit) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.print(value, decimals);
  Serial.print(' ');
  Serial.println(unit);
}

void printSettings(const BmsSettings &settings) {
  Serial.println(F("\nSP51_basic_BMS configuration:"));
  Serial.print(F("  Nonvolatile settings record: "));
  Serial.println(settingsAreStored() ? F("SAVED / VERIFIED") : F("NOT SAVED"));
  Serial.print(F("  Stack modules: "));
  Serial.print(settings.stackModules);
  Serial.print(F("; cells/module: "));
  Serial.println(settings.cellsPerModule);
  Serial.print(F("  Monitor period target: "));
  Serial.print(settings.samplePeriodMs);
  Serial.println(F(" ms start-to-start; minimum 500 ms, maximum 5000 ms."));
  Serial.print(F("  Provisional NTC: pull-up "));
  Serial.print(settings.ntcPullupOhms, 0);
  Serial.print(F(" ohm, R25 "));
  Serial.print(settings.ntcR25Ohms, 0);
  Serial.print(F(" ohm, beta "));
  Serial.print(settings.ntcBetaKelvin, 0);
  Serial.println(F(" K; temperature estimates are uncalibrated."));
  Serial.print(F("  Cell limits V: OV=")); Serial.print(settings.cellOverVoltageV, 3);
  Serial.print(F(" UV=")); Serial.print(settings.cellUnderVoltageV, 3);
  Serial.print(F(" discharge cutoff=")); Serial.print(settings.dischargeCutoffV, 3);
  Serial.print(F(" +/- hysteresis=")); Serial.println(settings.dischargeHysteresisV, 3);
  Serial.print(F("  Charge: cell target=")); Serial.print(settings.chargeVoltageV, 3);
  Serial.print(F(" V; hysteresis=")); Serial.print(settings.chargeHysteresisV, 3);
  Serial.print(F(" V; current=")); Serial.print(settings.maxChargeCurrentA, 1);
  Serial.print(F(" A max / ")); Serial.print(settings.endChargeCurrentA, 1);
  Serial.println(F(" A end threshold."));
  Serial.print(F("  Pack: ")); Serial.print(settings.seriesCells);
  Serial.print(F("S; calculated target="));
  Serial.print(settings.chargeVoltageV * settings.seriesCells, 1);
  Serial.println(F(" V. Confirm this is within the actual charger's rating."));
  Serial.print(F("  Pack capacity: ")); Serial.print(settings.batteryCapacityAh, 1);
  Serial.println(F(" Ah total (set this to the actual pack capacity)."));
  const SocStatus soc = socEstimatorStatus(settings);
  Serial.print(F("  SOC: "));
  if (soc.initialized) Serial.print(soc.percent, 2);
  else Serial.print(F("not initialized"));
  Serial.print(F("%; remaining=")); Serial.print(soc.remainingAh, 2);
  Serial.print(F(" Ah; journal="));
  Serial.print(soc.stateStored ? F("VALID") : F("EMPTY"));
  Serial.print(F("; pending save=")); Serial.println(soc.pendingSave ? F("YES") : F("NO"));
  Serial.print(F("  Ah throughput: charged in=")); Serial.print(soc.chargedAh, 3);
  Serial.print(F(" Ah; discharged out=")); Serial.print(soc.dischargedAh, 3);
  Serial.println(F(" Ah."));
  Serial.print(F("  Current sensor: ")); Serial.print(currentSensorName(settings.currentSensor));
  Serial.print(F("; CAN ")); Serial.print(settings.currentCanBitrate / 1000UL);
  Serial.print(F(" kbit/s; ID 0x")); Serial.print(settings.currentCanId, HEX);
  Serial.print(F("; invert=")); Serial.println(settings.invertCurrent ? F("YES") : F("NO"));
  Serial.print(F("  Charger profile: ")); Serial.print(chargerName(settings.chargerType));
  Serial.print(F("; Outlander CAN TX="));
  Serial.println(settings.chargerTxEnabled ? F("ENABLED") : F("DISABLED"));
  Serial.println(F("  Output pin map (active HIGH into low-side FETs):"));
  printOutputPin("    Main positive: ", settings.mainPositivePin);
  printOutputPin("    Main negative: ", settings.mainNegativePin);
  printOutputPin("    Precharge:     ", settings.prechargePin);
  printOutputPin("    Charger relay: ", settings.chargerRelayPin);
  printOutputPin("    Run input:     ", settings.runInputPin);
  printOutputPin("    240 V input:   ", settings.chargeInputPin);
  Serial.print(F("    Run active level: "));
  Serial.println(settings.runInputActiveHigh ? F("HIGH") : F("LOW"));
  Serial.print(F("    240 V active level: "));
  Serial.println(settings.chargeInputActiveHigh ? F("HIGH") : F("LOW"));
  const BmsStateStatus state = bmsStateMachineStatus();
  Serial.print(F("  Controller state: "));
  Serial.print(bmsOperatingStateName(state.state));
  Serial.print(F("; armed="));
  Serial.println(state.armed ? F("YES") : F("NO"));
  Serial.println(F("    Output gates are active HIGH; controller may operate from the two isolated inputs."));
}

void printMenu() {
  // Each page prints the current persisted value beside its SET command.
  Serial.println(F("\n=== SP51 BMS CONFIGURATION ==="));
  Serial.println(F("Live table is hidden; stack scans and state safety checks continue in the background."));
  Serial.print(F("Settings persistence: "));
  Serial.println(settingsAreStored()
                     ? F("EEPROM record is valid; accepted changes save immediately.")
                     : F("EEPROM record is not verified; settings changes will be rejected."));
  if (menuPage == MenuPage::Main) {
    Serial.println();
    Serial.println(F("MAIN MENU - type a number or the section name:"));
    Serial.println(F("  1 battery  - cell voltage limits and discharge cut-off settings"));
    Serial.println(F("  2 charging - charge target, hysteresis, current, capacity and charger profile"));
    Serial.println(F("  3 current  - LEM current sensor CAN input and polarity settings"));
    Serial.println(F("  4 SOC      - coulomb counting, voltage seed curve and SOC calibration"));
    Serial.println(F("  5 outputs  - map main contactor, precharge and charger relay FET pins"));
    Serial.println(F("  6 monitor  - scan interval and BQ stack retry"));
    Serial.println(F("  7 state    - precharge settings, input polarity, status and fault clear"));
    Serial.println(F("  s/settings - show all current settings, input/output pins and state"));
    Serial.println(F("  reboot     - restart the Nano and rerun BQ startup; does not hardware-reset BQ devices"));
    Serial.println(F("  disarm     - immediately open all mapped outputs; automatic restart waits for input release"));
    Serial.println(F("  help       - explain this menu; x exits, reinitializes stack, resumes live data"));
  } else if (menuPage == MenuPage::BatteryLimits) {
    const BmsSettings &settings = settingsGet();
    Serial.println(F("BATTERY LIMITS"));
    Serial.println(F("Current settings (volts per cell):"));
    printFloatSetting(F("Cell over-voltage trip"), settings.cellOverVoltageV, 3, F("V/cell"));
    printFloatSetting(F("Cell under-voltage trip"), settings.cellUnderVoltageV, 3, F("V/cell"));
    printFloatSetting(F("Discharge cut-off"), settings.dischargeCutoffV, 3, F("V/cell"));
    printFloatSetting(F("Discharge release hysteresis"), settings.dischargeHysteresisV, 3, F("V/cell"));
    Serial.println();
    Serial.println(F("Commands (every settings change must begin with SET):"));
    Serial.println(F("  set ov <V>   set cell over-voltage limit (2.50..4.50 V)"));
    Serial.println(F("  set uv <V>   set cell under-voltage limit (2.00..4.00 V)"));
    Serial.println(F("  set dv <V>   set discharge cut-off (2.00..4.30 V; must exceed UV)"));
    Serial.println(F("  set dhy <V>  set discharge release hysteresis (0.01..1.00 V)"));
    Serial.println(F("  Limits must remain ordered: UV < discharge cut-off < charge target < OV."));
    Serial.print(F("  UV must be below the current discharge cut-off: "));
    Serial.print(settings.dischargeCutoffV, 3);
    Serial.println(F(" V/cell. Change DV first if the intended UV is higher."));
    Serial.println(F("  Example syntax: set uv <value below the displayed discharge cut-off>"));
  } else if (menuPage == MenuPage::Charging) {
    const BmsSettings &settings = settingsGet();
    Serial.println(F("CHARGING"));
    Serial.println(F("Current settings:"));
    printFloatSetting(F("Charge target"), settings.chargeVoltageV, 3, F("V/cell"));
    printFloatSetting(F("Charge hysteresis"), settings.chargeHysteresisV, 3, F("V/cell"));
    printFloatSetting(F("Maximum charge current"), settings.maxChargeCurrentA, 1, F("A"));
    printFloatSetting(F("End-of-charge current threshold"), settings.endChargeCurrentA, 1, F("A"));
    printFloatSetting(F("Battery capacity"), settings.batteryCapacityAh, 1, F("Ah"));
    Serial.print(F("  Pack series cells: ")); Serial.print(settings.seriesCells);
    Serial.println(F("S"));
    Serial.print(F("  Calculated pack target: "));
    Serial.print(settings.chargeVoltageV * settings.seriesCells, 1);
    Serial.println(F(" V (cell target x series count)"));
    Serial.print(F("  Selected charger profile: "));
    Serial.println(chargerName(settings.chargerType));
    Serial.print(F("  Charger CAN transmit: "));
    Serial.println(settings.chargerTxEnabled ? F("ENABLED") : F("DISABLED"));
    Serial.println();
    Serial.println(F("Commands (voltages in V/cell, currents in A, capacity in Ah):"));
    Serial.println(F("  set cv <V>       set charge target per cell (2.50..4.50 V; below OV)"));
    Serial.println(F("  set hys <V>      set charge taper/pause hysteresis (0.01..1.00 V)"));
    Serial.println(F("  set imax <A>     set maximum requested charge current (0..500 A)"));
    Serial.println(F("  set iend <A>     set end-of-charge current (0..imax A)"));
    Serial.println(F("  set cap <Ah>     set total pack capacity used by SOC (0.1..2000 Ah)"));
    Serial.println(F("  set series <N>   set pack series-cell count (1..160; current pack is 108S)"));
    Serial.println(F("  set charger <type>  none|relay|outlander|brusa|volt|eltek|elcon|victron"));
    Serial.println(F("  Outlander CAN transmission is disabled; this menu selects a profile only."));
  } else if (menuPage == MenuPage::CurrentSensor) {
    const BmsSettings &settings = settingsGet();
    Serial.println(F("CURRENT SENSOR - LEM CAB500-C/SP7 reports current over CAN."));
    Serial.println(F("Current settings:"));
    Serial.print(F("  Sensor type: ")); Serial.println(currentSensorName(settings.currentSensor));
    Serial.print(F("  CAN bitrate: ")); Serial.print(settings.currentCanBitrate / 1000UL);
    Serial.println(F(" kbit/s"));
    Serial.print(F("  CAN frame ID: 0x")); Serial.println(settings.currentCanId, HEX);
    Serial.print(F("  Invert current sign: ")); Serial.println(settings.invertCurrent ? F("YES") : F("NO"));
    Serial.print(F("  Stale-data timeout: ")); Serial.print(settings.currentSensorTimeoutMs);
    Serial.println(F(" ms"));
    Serial.println();
    Serial.println(F("Commands:"));
    Serial.println(F("  set sensor lem|off  select LEM CAN sensor or disable current input"));
    Serial.println(F("  set canrate 250|500  set CAN bitrate in kbit/s (250 or 500)"));
    Serial.println(F("  set canid <ID>  set standard CAN ID 1..0x7FF (decimal or hex, e.g. 0x3C2)"));
    Serial.println(F("  set invert 0|1  leave current sign unchanged (0) or reverse it (1)"));
    Serial.println(F("  Fresh LEM current is a precharge/interlock input; current regulation and charger CAN TX are not active."));
  } else if (menuPage == MenuPage::SocEstimation) {
    const BmsSettings &settings = settingsGet();
    const SocStatus soc = socEstimatorStatus(settings);
    Serial.println(F("STATE OF CHARGE - signed current integration and retained SOC"));
    Serial.print(F("  Mode: "));
    Serial.println(settings.socMode == SocMode::CoulombCount
                       ? F("COULOMB COUNT") : F("VOLTAGE ONLY"));
    Serial.print(F("  Pack capacity for coulomb mode: "));
    Serial.print(settings.batteryCapacityAh, 1);
    Serial.println(F(" Ah total; change it in the Charging submenu with SET CAP."));
    Serial.print(F("  SOC: "));
    if (soc.initialized) Serial.print(soc.percent, 2);
    else Serial.print(F("not initialized"));
    Serial.print(F(" %; remaining: ")); Serial.print(soc.remainingAh, 2);
    Serial.println(F(" Ah"));
    Serial.print(F("  Cumulative charged: ")); Serial.print(soc.chargedAh, 3);
    Serial.print(F(" Ah; discharged: ")); Serial.print(soc.dischargedAh, 3);
    Serial.println(F(" Ah"));
    Serial.print(F("  Retained SOC record: "));
    Serial.print(soc.stateStored ? F("VALID") : F("NOT SAVED"));
    Serial.print(F("; latest changes pending: "));
    Serial.println(soc.pendingSave ? F("YES") : F("NO"));
    Serial.print(F("  Voltage seed lower anchor: "));
    Serial.print(settings.socLowCellMv / 1000.0f, 3);
    Serial.print(F(" V = ")); Serial.print(settings.socLowPercent);
    Serial.println(F(" %"));
    Serial.print(F("  Voltage seed upper anchor: "));
    Serial.print(settings.socHighCellMv / 1000.0f, 3);
    Serial.print(F(" V = ")); Serial.print(settings.socHighPercent);
    Serial.println(F(" %"));
    Serial.println(F("  Current sign: positive charges/increases SOC; negative discharges/decreases SOC."));
    Serial.println(F("  The existing current invert setting changes this sign if the LEM is reversed."));
    Serial.println();
    Serial.println(F("Commands (settings save immediately; SOC calibration saves to its own CRC record):"));
    Serial.println(F("  set socmode coulomb|voltage  choose signed Ah counting or voltage-only estimate"));
    Serial.println(F("  set soc <0..100>             calibrate SOC percent (coulomb mode only)"));
    Serial.println(F("  set soclowv <V>              voltage anchor paired with lower SOC percent"));
    Serial.println(F("  set soclowpct <0..100>       lower voltage-anchor SOC percent"));
    Serial.println(F("  set sochighv <V>             voltage anchor paired with upper SOC percent"));
    Serial.println(F("  set sochighpct <0..100>      upper voltage-anchor SOC percent"));
    Serial.println(F("  In coulomb mode, a valid cell scan seeds SOC from the voltage curve only when no retained SOC exists."));
    Serial.println(F("  Ah charged/discharged totals are counted in both modes; voltage mode uses cell voltage for SOC."));
    Serial.println(F("  Coulomb-count SOC can drift; calibrate it periodically with a known pack SOC using SET SOC."));
    Serial.println(F("  The default 50 Ah capacity is a placeholder; set the actual pack Ah before relying on SOC."));
    Serial.println(F("  SOC and Ah totals save every 60 seconds; an abrupt reset can lose up to one minute of new counts."));
  } else if (menuPage == MenuPage::Outputs) {
    const BmsSettings &settings = settingsGet();
    Serial.println(F("I/O PIN MAP - assign each logical output/input to a unique Nano pin."));
    Serial.println(F("Current output mapping (active HIGH into low-side FET gates):"));
    printOutputPin("  Main positive contactor: ", settings.mainPositivePin);
    printOutputPin("  Main negative contactor: ", settings.mainNegativePin);
    printOutputPin("  Precharge contactor:     ", settings.prechargePin);
    printOutputPin("  External charger relay:  ", settings.chargerRelayPin);
    printOutputPin("  Isolated Run input:      ", settings.runInputPin);
    printOutputPin("  Isolated 240 V input:    ", settings.chargeInputPin);
    Serial.println();
    Serial.println(F("Commands (pins are Arduino digital pin numbers):"));
    Serial.println(F("  set pin mainpos <pin|none>  map/unmap positive main contactor output"));
    Serial.println(F("  set pin mainneg <pin|none>  map/unmap negative main contactor output"));
    Serial.println(F("  set pin precharge <pin|none>  map/unmap precharge contactor output"));
    Serial.println(F("  set pin charger <pin|none>  map/unmap external charger relay output"));
    Serial.println(F("  set pin run <pin>     map isolated 12 V Run input (default D2)"));
    Serial.println(F("  set pin charge <pin>  map isolated 240 V-present input (default D3)"));
    Serial.println(F("  Output pins: 6, 13, 11, 10, or none. Input pins: D2..D13."));
    Serial.println(F("  All assigned input/output pins must be unique; D0/D1 are reserved for BQ UART."));
    Serial.println(F("  Both isolated inputs default to active LOW; change polarity in State Control if needed."));
    Serial.println(F("  Outputs are active HIGH into low-side FET gates; DISARM commands all mapped outputs OFF."));
    Serial.println(F("  Example: set pin mainpos 6   |   set pin charger 10"));
  } else if (menuPage == MenuPage::StateControl) {
    const BmsSettings &settings = settingsGet();
    const BmsStateStatus state = bmsStateMachineStatus();
    Serial.println(F("STATE CONTROL AND SIMPBMS-STYLE PRECHARGE"));
    Serial.print(F("  State: ")); Serial.println(bmsOperatingStateName(state.state));
    Serial.print(F("  Automatic control: ")); Serial.println(state.armed ? F("ARMED") : F("DISARMED"));
    Serial.print(F("  Run request D")); Serial.print(settings.runInputPin);
    Serial.print(F(" (12 V isolated, active "));
    Serial.print(settings.runInputActiveHigh ? F("HIGH") : F("LOW"));
    Serial.print(F("), stable=")); Serial.println(state.runRequested ? F("YES") : F("NO"));
    Serial.print(F("  240 V request D")); Serial.print(settings.chargeInputPin);
    Serial.print(F(" (isolated, active "));
    Serial.print(settings.chargeInputActiveHigh ? F("HIGH") : F("LOW"));
    Serial.print(F("), stable=")); Serial.println(state.chargeRequested ? F("YES") : F("NO"));
    Serial.print(F("  Precharge elapsed: ")); Serial.print(state.prechargeElapsedMs);
    Serial.println(F(" ms"));
    Serial.print(F("  Current: "));
    if (state.currentValid) Serial.print(state.currentMilliamp / 1000.0f, 2);
    else Serial.print(F("invalid"));
    Serial.println(F(" A"));
    Serial.print(F("  State detail: ")); Serial.println(state.reason ? state.reason : "-");
    printFloatSetting(F("Precharge minimum time"), settings.prechargeMinimumMs, 0, F("ms"));
    printFloatSetting(F("Precharge timeout"), settings.prechargeTimeoutMs, 0, F("ms"));
    printFloatSetting(F("Precharge finish current"), settings.prechargeFinishCurrentA, 2, F("A"));
    Serial.println();
    Serial.println(F("Settings commands save immediately and are accepted only with outputs off:"));
    Serial.println(F("  set premin <ms>       minimum precharge dwell (100..30000 ms; Simp default 5000)"));
    Serial.println(F("  set pretimeout <ms>   timeout and fault if not complete (must exceed premin)"));
    Serial.println(F("  set precurrent <A>    finish when |LEM current| is at/below this value (0.1..50 A)"));
    Serial.println(F("  set runpol high|low   active level for the isolated Run input"));
    Serial.println(F("  set chargepol high|low active level for isolated 240 V-present input"));
    Serial.println(F("  clearfault            clear a latched fault only when both requests are inactive"));
    Serial.println(F("  Physical Run/240 V inputs request operation; no terminal ARM command is needed."));
    Serial.println(F("  Charge relay starts only with the External Relay charger profile; Outlander CAN TX is disabled."));
  } else if (menuPage == MenuPage::Monitor) {
    const BmsSettings &settings = settingsGet();
    Serial.println(F("MONITOR - settings are saved to EEPROM and restored at startup."));
    Serial.print(F("Current scan target: ")); Serial.print(settings.samplePeriodMs);
    Serial.println(F(" ms"));
    Serial.print(F("Stack configuration: ")); Serial.print(settings.stackModules);
    Serial.print(F(" stack devices x ")); Serial.print(settings.cellsPerModule);
    Serial.println(F(" cells/device"));
    Serial.println();
    Serial.println(F("Commands:"));
    Serial.println(F("  set p <ms>  set requested battery scan interval (500..5000 ms)"));
    Serial.println(F("  r       restart the BQ startup/communication initialization sequence"));
    Serial.println(F("  The stack scan is never overlapped; long scans can exceed the target interval."));
  }
  Serial.println();
  Serial.println(F("Command: back returns from this submenu to the main configuration menu."));
  Serial.println(F("Navigation: s/settings shows all settings; x exits, reinitializes stack, then resumes scans."));
  Serial.println(F("Type help to repeat this submenu; values are saved immediately when accepted."));
  Serial.print(F("BMS CFG> "));
}

bool commandEquals(const char *command, const char *expected) {
  while (*command && *expected) {
    char value = *command++;
    char target = *expected++;
    if (value >= 'A' && value <= 'Z') value += 'a' - 'A';
    if (target >= 'A' && target <= 'Z') target += 'a' - 'A';
    if (value != target) return false;
  }
  return *command == '\0' && *expected == '\0';
}

bool parseFloatArgument(const char *text, float &value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  value = strtof(text, &end);
  while (end && (*end == ' ' || *end == '\t')) ++end;
  return end != text && end && *end == '\0' && value == value &&
         value >= -1000000.0f && value <= 1000000.0f;
}

bool parseUnsignedArgument(const char *text, unsigned long &value) {
  if (!text || !*text || *text == '-') return false;
  char *end = nullptr;
  value = strtoul(text, &end, 0);
  while (end && (*end == ' ' || *end == '\t')) ++end;
  return end != text && end && *end == '\0';
}

void printCommandResult(bool accepted, const char *what) {
  Serial.print(what);
  Serial.println(accepted ? F(" saved; this value is now active.")
                          : F(" rejected or not saved; check limits and EEPROM status."));
}

bool commandAllowedInPage(MenuPage page, const char *command) {
  if (page == MenuPage::BatteryLimits) {
    return commandEquals(command, "ov") || commandEquals(command, "uv") ||
           commandEquals(command, "dv") || commandEquals(command, "dhy");
  }
  if (page == MenuPage::Charging) {
    return commandEquals(command, "cv") || commandEquals(command, "hys") ||
           commandEquals(command, "imax") || commandEquals(command, "iend") ||
           commandEquals(command, "cap") || commandEquals(command, "series") ||
           commandEquals(command, "charger");
  }
  if (page == MenuPage::SocEstimation) {
    return commandEquals(command, "soc") ||
           commandEquals(command, "socmode") ||
           commandEquals(command, "soclowv") ||
           commandEquals(command, "soclowpct") ||
           commandEquals(command, "sochighv") ||
           commandEquals(command, "sochighpct");
  }
  if (page == MenuPage::CurrentSensor) {
    return commandEquals(command, "sensor") || commandEquals(command, "canrate") ||
           commandEquals(command, "canid") || commandEquals(command, "invert");
  }
  if (page == MenuPage::Outputs) return commandEquals(command, "pin");
  if (page == MenuPage::Monitor) return commandEquals(command, "p");
  if (page == MenuPage::StateControl) {
    return commandEquals(command, "premin") ||
           commandEquals(command, "pretimeout") ||
           commandEquals(command, "precurrent") ||
           commandEquals(command, "runpol") ||
           commandEquals(command, "chargepol");
  }
  return false;
}

void printVoltageSettingRejection(const char *name, float value) {
  const BmsSettings &settings = settingsGet();
  if (commandEquals(name, "uv")) {
    if (value < 2.0f || value > 4.0f) {
      Serial.println(F("UV must be within 2.00..4.00 V/cell."));
    } else if (value >= settings.dischargeCutoffV) {
      Serial.print(F("Requested UV "));
      Serial.print(value, 3);
      Serial.print(F(" V/cell must be below discharge cut-off, currently "));
      Serial.print(settings.dischargeCutoffV, 3);
      Serial.println(F(" V/cell. Set a chemistry-appropriate DV above the requested UV, then repeat this UV command."));
    } else {
      Serial.println(F("UV was rejected by another limit-order rule or EEPROM save verification."));
    }
  } else if (commandEquals(name, "dv")) {
    if (value < 2.0f || value > 4.3f) {
      Serial.println(F("Discharge cut-off must be within 2.00..4.30 V/cell."));
    } else if (value <= settings.cellUnderVoltageV) {
      Serial.println(F("Discharge cut-off must be above the current UV setting."));
    } else if (value >= settings.chargeVoltageV) {
      Serial.println(F("Discharge cut-off must be below the charge target."));
    } else if (settings.dischargeHysteresisV >= settings.cellOverVoltageV - value) {
      Serial.println(F("That cut-off leaves too little range for the configured OV and discharge hysteresis."));
    } else {
      Serial.println(F("Discharge cut-off was rejected by EEPROM save verification."));
    }
  } else if (commandEquals(name, "cv")) {
    if (value < 2.5f || value > 4.5f) {
      Serial.println(F("Charge target must be within 2.50..4.50 V/cell."));
    } else if (value <= settings.dischargeCutoffV || value >= settings.cellOverVoltageV) {
      Serial.println(F("Charge target must be above discharge cut-off and below OV."));
    } else if (settings.chargeHysteresisV >= value - settings.cellUnderVoltageV) {
      Serial.println(F("That target leaves too little range for the configured UV and charge hysteresis."));
    } else {
      Serial.println(F("Charge target was rejected by EEPROM save verification."));
    }
  } else if (commandEquals(name, "ov")) {
    if (value < 2.5f || value > 4.5f) {
      Serial.println(F("OV must be within 2.50..4.50 V/cell."));
    } else if (value <= settings.chargeVoltageV) {
      Serial.println(F("OV must be above the current charge target."));
    } else if (settings.dischargeHysteresisV >= value - settings.dischargeCutoffV) {
      Serial.println(F("That OV leaves too little range for the configured discharge cut-off and hysteresis."));
    } else {
      Serial.println(F("OV was rejected by EEPROM save verification."));
    }
  } else if (commandEquals(name, "hys")) {
    if (value < 0.01f || value > 1.0f) {
      Serial.println(F("Charge hysteresis must be within 0.01..1.00 V/cell."));
    } else if (value >= settings.chargeVoltageV - settings.cellUnderVoltageV) {
      Serial.println(F("Charge hysteresis must be smaller than charge target minus UV."));
    } else {
      Serial.println(F("Charge hysteresis was rejected by EEPROM save verification."));
    }
  } else if (commandEquals(name, "dhy")) {
    if (value < 0.01f || value > 1.0f) {
      Serial.println(F("Discharge hysteresis must be within 0.01..1.00 V/cell."));
    } else if (value >= settings.cellOverVoltageV - settings.dischargeCutoffV) {
      Serial.println(F("Discharge hysteresis must be smaller than OV minus discharge cut-off."));
    } else {
      Serial.println(F("Discharge hysteresis was rejected by EEPROM save verification."));
    }
  }
}

void processCommand(char *command) {
  // Commands are parsed one complete USB-serial line at a time. Settings use
  // SET explicitly; DISARM and CLEARFAULT are state actions, not settings.
  while (*command == ' ' || *command == '\t') ++command;
  size_t length = strlen(command);
  while (length && (command[length - 1] == ' ' || command[length - 1] == '\t')) {
    command[--length] = '\0';
  }
  if (!length) return;

  if (commandEquals(command, "disarm")) {
    bmsStateMachineDisarm(settingsGet());
    Serial.println(F("DISARMED: mapped outputs commanded OFF; automatic control waits for Run and 240 V inputs to release."));
    if (menuOpen) {
      Serial.println();
      printMenu();
    }
    return;
  }

  if (!menuOpen) {
    if (commandEquals(command, "m") || commandEquals(command, "menu") ||
        commandEquals(command, "help") || commandEquals(command, "h") ||
        commandEquals(command, "?")) {
      menuOpen = true;
      menuPage = MenuPage::Main;
      printMenu();
    }
    return;
  }

  if (commandEquals(command, "x") || commandEquals(command, "q")) {
    bmsStateMachineDisarm(settingsGet());
    menuOpen = false;
    menuPage = MenuPage::Main;
    retryRequested = true;
    Serial.println(F("Leaving configuration menu; reinitializing the BQ stack before live scanning."));
    return;
  }
  if (commandEquals(command, "help") || commandEquals(command, "h") ||
      commandEquals(command, "?")) {
    printMenu();
    return;
  }
  if (commandEquals(command, "b") || commandEquals(command, "back")) {
    menuPage = MenuPage::Main;
    printMenu();
    return;
  }
  if (commandEquals(command, "m") || commandEquals(command, "menu") ||
      commandEquals(command, "home")) {
    menuPage = MenuPage::Main;
    printMenu();
    return;
  }
  if (commandEquals(command, "s") || commandEquals(command, "settings")) {
    printSettings(settingsGet());
    Serial.print(F("BMS CFG> "));
    return;
  }
  if (commandEquals(command, "r")) {
    if (menuPage != MenuPage::Monitor) {
      Serial.println(F("Stack retry is in the Monitor submenu; use b, then 5."));
      Serial.print(F("BMS CFG> "));
      return;
    }
    bmsStateMachineDisarm(settingsGet());
    menuOpen = false;
    menuPage = MenuPage::Main;
    retryRequested = true;
    Serial.println(F("Retrying the BQ stack startup sequence; live display will resume afterward."));
    return;
  }
  if (commandEquals(command, "reboot")) {
    if (menuPage != MenuPage::Main) {
      Serial.println(F("Reboot is available from the main menu; use back first."));
      Serial.print(F("BMS CFG> "));
      return;
    }
    if (!socEstimatorFlush()) {
      Serial.println(F("SOC/Ah save failed; reboot cancelled so the latest estimate is not discarded."));
      Serial.print(F("BMS CFG> "));
      return;
    }
    Serial.println(F("Rebooting the Nano; firmware will reload EEPROM settings and rerun BQ stack startup."));
    const BmsSettings &settings = settingsGet();
    bmsStateMachineDisarm(settings);
    Serial.flush();
    delay(50);
    NVIC_SystemReset();
    return;
  }
  if (menuPage == MenuPage::Main) {
    if (commandEquals(command, "1") || commandEquals(command, "battery") ||
        commandEquals(command, "limits")) menuPage = MenuPage::BatteryLimits;
    else if (commandEquals(command, "2") || commandEquals(command, "charging") ||
             commandEquals(command, "charge")) menuPage = MenuPage::Charging;
    else if (commandEquals(command, "3") || commandEquals(command, "current") ||
             commandEquals(command, "sensor")) menuPage = MenuPage::CurrentSensor;
    else if (commandEquals(command, "4") || commandEquals(command, "soc"))
      menuPage = MenuPage::SocEstimation;
    else if (commandEquals(command, "5") || commandEquals(command, "outputs") ||
             commandEquals(command, "pins")) menuPage = MenuPage::Outputs;
    else if (commandEquals(command, "6") || commandEquals(command, "monitor"))
      menuPage = MenuPage::Monitor;
    else if (commandEquals(command, "7") || commandEquals(command, "state") ||
             commandEquals(command, "precharge")) menuPage = MenuPage::StateControl;
    else {
      Serial.println(F("Choose a numbered section from the main menu, or type s to show settings."));
      Serial.print(F("BMS CFG> "));
      return;
    }
    printMenu();
    return;
  }

  if (menuPage == MenuPage::StateControl &&
      commandEquals(command, "clearfault")) {
    const bool cleared = bmsStateMachineClearFault();
    Serial.println(cleared
        ? F("Fault cleared; outputs remain OFF and the controller waits for fresh interlocks.")
        : F("Fault not cleared; ensure a fault is latched and both isolated inputs are inactive."));
    Serial.println();
    printMenu();
    return;
  }

  char *setToken = command;
  while (*setToken && *setToken != ' ' && *setToken != '\t') ++setToken;
  char *arguments = nullptr;
  if (*setToken) {
    *setToken++ = '\0';
    while (*setToken == ' ' || *setToken == '\t') ++setToken;
    arguments = setToken;
  }
  if (!commandEquals(command, "set")) {
    Serial.println(F("Use SET for configuration changes; state actions are DISARM and CLEARFAULT."));
    Serial.print(F("BMS CFG> "));
    return;
  }
  if (!bmsStateMachineCanEditSettings()) {
    Serial.println(F("Settings are locked while precharge, drive or charge is active. Use DISARM first."));
    Serial.print(F("BMS CFG> "));
    return;
  }
  if (!arguments || !*arguments) {
    Serial.println(F("Enter the setting name and value after SET; type help for examples."));
    Serial.print(F("BMS CFG> "));
    return;
  }

  char *settingKey = arguments;
  while (*settingKey && *settingKey != ' ' && *settingKey != '\t') ++settingKey;
  char *valueText = nullptr;
  if (*settingKey) {
    *settingKey++ = '\0';
    while (*settingKey == ' ' || *settingKey == '\t') ++settingKey;
    valueText = settingKey;
  }
  command = arguments;
  for (char *key = command; *key; ++key) {
    if (*key >= 'A' && *key <= 'Z') *key += 'a' - 'A';
  }

  if (!commandAllowedInPage(menuPage, command)) {
    Serial.println(F("That command is not in this submenu. Type help to see its options, or b to return."));
    Serial.print(F("BMS CFG> "));
    return;
  }

  if (commandEquals(command, "p")) {
    unsigned long value = 0;
    const bool parsed = parseUnsignedArgument(valueText, value);
    printCommandResult(parsed && value <= 65535UL &&
                           settingsSetSamplePeriod(static_cast<uint16_t>(value)),
                       "Monitor period");
  } else if (commandEquals(command, "ov") || commandEquals(command, "uv") ||
             commandEquals(command, "dv") || commandEquals(command, "dhy") ||
             commandEquals(command, "cv") || commandEquals(command, "hys")) {
    float value = 0.0f;
    const bool parsed = parseFloatArgument(valueText, value);
    const bool accepted = parsed && settingsSetVoltageLimit(command, value);
    printCommandResult(accepted, "Voltage setting");
    if (!accepted && parsed) printVoltageSettingRejection(command, value);
  } else if (commandEquals(command, "imax") || commandEquals(command, "iend")) {
    float value = 0.0f;
    printCommandResult(parseFloatArgument(valueText, value) &&
                           settingsSetChargeCurrent(command, value),
                       "Charge-current setting");
  } else if (commandEquals(command, "cap")) {
    float value = 0.0f;
    printCommandResult(parseFloatArgument(valueText, value) &&
                           settingsSetBatteryCapacity(value),
                       "Battery capacity");
  } else if (commandEquals(command, "socmode")) {
    const bool valid = valueText &&
        (commandEquals(valueText, "coulomb") || commandEquals(valueText, "cc") ||
         commandEquals(valueText, "voltage"));
    const SocMode mode = valueText && commandEquals(valueText, "voltage")
        ? SocMode::VoltageOnly : SocMode::CoulombCount;
    printCommandResult(valid && settingsSetSocMode(mode), "SOC mode");
    if (valid && mode == SocMode::CoulombCount) {
      Serial.println(F("Coulomb counting treats positive current as charge and negative current as discharge."));
    }
  } else if (commandEquals(command, "soc")) {
    float value = 0.0f;
    printCommandResult(parseFloatArgument(valueText, value) &&
                           socEstimatorSetPercent(value, settingsGet()),
                       "SOC calibration");
  } else if (commandEquals(command, "soclowv") ||
             commandEquals(command, "soclowpct") ||
             commandEquals(command, "sochighv") ||
             commandEquals(command, "sochighpct")) {
    float value = 0.0f;
    printCommandResult(parseFloatArgument(valueText, value) &&
                           settingsSetSocVoltageAnchor(command, value),
                       "SOC voltage-curve setting");
  } else if (commandEquals(command, "sensor")) {
    const bool valid = valueText &&
        (commandEquals(valueText, "off") || commandEquals(valueText, "lem"));
    const CurrentSensorType type =
        valueText && commandEquals(valueText, "lem")
            ? CurrentSensorType::LemCab500Can : CurrentSensorType::Off;
    printCommandResult(valid && settingsSetCurrentSensorType(type), "Current sensor");
  } else if (commandEquals(command, "canrate")) {
    unsigned long value = 0;
    const bool parsed = parseUnsignedArgument(valueText, value);
    printCommandResult(parsed && (value == 250UL || value == 500UL) &&
                           settingsSetCurrentCanBitrate(static_cast<uint32_t>(value * 1000UL)),
                       "Current-sensor CAN rate");
  } else if (commandEquals(command, "canid")) {
    unsigned long value = 0;
    const bool parsed = parseUnsignedArgument(valueText, value);
    printCommandResult(parsed && value <= 0x7FFUL &&
                           settingsSetCurrentCanId(static_cast<uint16_t>(value)),
                       "Current-sensor CAN ID");
  } else if (commandEquals(command, "invert")) {
    unsigned long value = 0;
    const bool parsed = parseUnsignedArgument(valueText, value);
    printCommandResult(parsed && value <= 1UL &&
                           settingsSetCurrentInvert(value != 0UL),
                       "Current polarity inversion");
  } else if (commandEquals(command, "series")) {
    unsigned long value = 0;
    const bool parsed = parseUnsignedArgument(valueText, value);
    printCommandResult(parsed && value <= 65535UL &&
                           settingsSetSeriesCells(static_cast<uint16_t>(value)),
                       "Pack series-cell count");
  } else if (commandEquals(command, "charger")) {
    bool valid = true;
    ChargerType type = ChargerType::None;
    if (!valueText || commandEquals(valueText, "none")) type = ChargerType::None;
    else if (commandEquals(valueText, "relay")) type = ChargerType::ExternalRelay;
    else if (commandEquals(valueText, "outlander")) type = ChargerType::Outlander;
    else if (commandEquals(valueText, "brusa")) type = ChargerType::BrusaNlg5;
    else if (commandEquals(valueText, "volt")) type = ChargerType::Volt;
    else if (commandEquals(valueText, "eltek")) type = ChargerType::Eltek;
    else if (commandEquals(valueText, "victron")) type = ChargerType::VictronSma;
    else valid = false;
    const bool accepted = valid && valueText && settingsSetChargerType(type);
    printCommandResult(accepted, "Charger profile");
    if (accepted && type == ChargerType::Outlander) {
      Serial.println(F("Outlander CAN TX remains disabled pending OBC/pack voltage compatibility."));
    }
  } else if (commandEquals(command, "pin")) {
    bool valid = valueText != nullptr;
    char *pinText = valueText;
    while (pinText && *pinText && *pinText != ' ' && *pinText != '\t') ++pinText;
    char *pinValue = nullptr;
    if (pinText && *pinText) {
      *pinText++ = '\0';
      while (*pinText == ' ' || *pinText == '\t') ++pinText;
      pinValue = pinText;
    } else {
      valid = false;
    }
    bool inputRole = false;
    const char *inputName = nullptr;
    BmsOutputRole role = BmsOutputRole::MainPositive;
    if (valueText && commandEquals(valueText, "mainpos")) role = BmsOutputRole::MainPositive;
    else if (valueText && commandEquals(valueText, "mainneg")) role = BmsOutputRole::MainNegative;
    else if (valueText && commandEquals(valueText, "precharge")) role = BmsOutputRole::Precharge;
    else if (valueText && commandEquals(valueText, "charger")) role = BmsOutputRole::ChargerRelay;
    else if (valueText && commandEquals(valueText, "run")) {
      inputRole = true;
      inputName = "runpin";
    } else if (valueText && commandEquals(valueText, "charge")) {
      inputRole = true;
      inputName = "chargepin";
    } else valid = false;

    uint8_t pin = 0xFFU;
    unsigned long numericPin = 0;
    if (pinValue && commandEquals(pinValue, "none") && !inputRole) pin = 0xFFU;
    else if (pinValue && commandEquals(pinValue, "none")) valid = false;
    else if (!parseUnsignedArgument(pinValue, numericPin) || numericPin > 255UL) valid = false;
    else pin = static_cast<uint8_t>(numericPin);
    const bool accepted = valid && (inputRole
        ? settingsSetInputPin(inputName, pin)
        : settingsSetOutputPin(role, pin));
    printCommandResult(accepted, inputRole ? "Input pin mapping" : "Output pin mapping");
    if (accepted) Serial.println(F("Mapping saved; duplicate input/output pin assignments are rejected."));
  } else if (commandEquals(command, "premin") ||
             commandEquals(command, "pretimeout") ||
             commandEquals(command, "precurrent")) {
    float value = 0.0f;
    const bool parsed = parseFloatArgument(valueText, value);
    printCommandResult(parsed && settingsSetPrecharge(command, value),
                       "Precharge setting");
  } else if (commandEquals(command, "runpol") ||
             commandEquals(command, "chargepol")) {
    const bool valid = valueText &&
        (commandEquals(valueText, "high") || commandEquals(valueText, "low"));
    const bool activeHigh = valueText && commandEquals(valueText, "high");
    printCommandResult(valid && settingsSetInputPolarity(command, activeHigh),
                       "Input active level");
  }
  else {
    Serial.println(F("Unknown menu command or missing value; enter h for help."));
  }

  Serial.println();
  printMenu();
}

}

void serialConsoleBegin() {
  Serial.begin(115200);
  delay(500);
}

void serialConsolePrintStartup() {
  Serial.println();
  Serial.println(F("SP51_basic_BMS | Nano R4 / BQ79616 continuous monitor"));
  Serial.println(F("Cell ADC data is printed each scan; type HELP + Enter for configuration."));
  Serial.println(F("Current monitor uses the manual base COMH TX/RX override."));
  Serial.print(F("Configuration storage: "));
  Serial.println(settingsAreStored() ? F("Nano R4 EEPROM settings verified.")
                                    : F("EEPROM unavailable or record invalid; defaults are active."));
  Serial.println(F("Contactor state machine starts disarmed and waits for valid stack/current interlocks."));
  Serial.println(F("Run and 240 V isolated inputs request operation; Outlander charger CAN TX remains disabled."));
}

void serialConsolePrintInitResult(bool ok, const char *error) {
  if (ok) {
    Serial.println(F("BQ stack startup and manual COMH override verified."));
    Serial.println(F("Starting the live cell/temperature display."));
  } else {
    Serial.print(F("BQ stack startup failed: "));
    Serial.println(error ? error : "unknown error");
    Serial.println(F("Type m + Enter, then r to retry, or x to return."));
  }
}

void serialConsolePollInput() {
  while (Serial.available()) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    const char value = static_cast<char>(incoming);
    if (value == '\r') continue;
    if (value == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
      continue;
    }
    if (commandLength + 1U < COMMAND_BUFFER_SIZE) {
      commandBuffer[commandLength++] = value;
    }
  }
}

bool serialConsoleMenuIsOpen() {
  return menuOpen;
}

bool serialConsoleTakeRetryRequest() {
  const bool requested = retryRequested;
  retryRequested = false;
  return requested;
}

void serialConsolePrintSnapshot(const BatterySnapshot &snapshot,
                                const BmsSettings &settings,
                                const CurrentSensorStatus &current,
                                const BatteryLimitStatus &limits,
                                const BmsStateStatus &state,
                                const SocStatus &soc) {
  // Assemble the complete frame in RAM and emit once, avoiding partial rows
  // and keeping the live table's columns aligned as values change.
  size_t length = 0;
  bool ok = true;
  displayBuffer[0] = '\0';

  appendRepeated(displayBuffer, sizeof(displayBuffer), length, ok, '=',
                 DISPLAY_FRAME_WIDTH);
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             "                    SP51_basic_BMS\n"
             "               LIVE STACK MONITOR\n");
  appendRepeated(displayBuffer, sizeof(displayBuffer), length, ok, '=',
                 DISPLAY_FRAME_WIDTH);
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');
  appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
               "TIME:%5lu s  SCAN:%4lu ms  TARGET:%4u ms  CELLS:%3u/%u  CELL:%s  GPIO:%s\n",
               static_cast<unsigned long>(snapshot.finishedAtMs / 1000UL),
               static_cast<unsigned long>(snapshot.durationMs),
               static_cast<unsigned>(settings.samplePeriodMs),
               static_cast<unsigned>(snapshot.validCellCount),
               static_cast<unsigned>(static_cast<uint16_t>(snapshot.moduleCount) * BATTERY_CELLS_PER_MODULE),
               snapshot.allCellReadsOk ? "OK" : "PARTIAL",
               snapshot.allGpioReadsOk ? "OK" : "PARTIAL");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "ADDR   |");
  for (uint8_t cell = 1; cell <= BATTERY_CELLS_PER_MODULE; ++cell) {
    appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "C%02u     ",
                 static_cast<unsigned>(cell));
  }
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             "| MIN     MAX     DELTA_mV T_EST_C / NTC\n"
             "--------+");
  appendRepeated(displayBuffer, sizeof(displayBuffer), length, ok, '-', 96);
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '+');
  appendRepeated(displayBuffer, sizeof(displayBuffer), length, ok, '-', 40);
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

  float packMin = 1000000.0f;
  float packMax = -1000000.0f;
  uint8_t packMinAddress = 0;
  uint8_t packMinCell = 0;
  uint8_t packMaxAddress = 0;
  uint8_t packMaxCell = 0;

  for (uint8_t index = 0; index < snapshot.moduleCount; ++index) {
    const ModuleReading &module = snapshot.modules[index];
    appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
                 "  %02u   |", static_cast<unsigned>(module.address));
    for (uint8_t cell = 0; cell < BATTERY_CELLS_PER_MODULE; ++cell) {
      appendCellValue(displayBuffer, sizeof(displayBuffer), length, ok,
                      module, cell);
      appendText(displayBuffer, sizeof(displayBuffer), length, ok, "   ");
    }
    appendText(displayBuffer, sizeof(displayBuffer), length, ok, "| ");

    if (module.validCellCount) {
      appendVoltage(displayBuffer, sizeof(displayBuffer), length, ok,
                    module.minCellMv);
      appendText(displayBuffer, sizeof(displayBuffer), length, ok, "   ");
      appendVoltage(displayBuffer, sizeof(displayBuffer), length, ok,
                    module.maxCellMv);
      appendText(displayBuffer, sizeof(displayBuffer), length, ok, "   ");
      appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                  module.deltaCellMv, 1, 5);
      appendText(displayBuffer, sizeof(displayBuffer), length, ok, "   ");
      for (uint8_t cell = 0; cell < BATTERY_CELLS_PER_MODULE; ++cell) {
        if (module.cellRaw[cell] == 0x8000U) continue;
        if (module.cellMv[cell] < packMin) {
          packMin = module.cellMv[cell];
          packMinAddress = module.address;
          packMinCell = cell + 1U;
        }
        if (module.cellMv[cell] > packMax) {
          packMax = module.cellMv[cell];
          packMaxAddress = module.address;
          packMaxCell = cell + 1U;
        }
      }
    } else {
      appendText(displayBuffer, sizeof(displayBuffer), length, ok,
                 "  ---     ---     ---   ");
    }

    float temperatureSum = 0.0f;
    uint8_t temperatureCount = 0;
    for (uint8_t gpio = 0; gpio < BATTERY_GPIO_COUNT; ++gpio) {
      if (module.ntcEstimateMask & static_cast<uint8_t>(1U << gpio)) {
        temperatureSum += module.temperatureC[gpio];
        ++temperatureCount;
      }
    }
    if (temperatureCount) {
      appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                  temperatureSum / temperatureCount, 1, 5);
      appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
                   "C NTC=%u", static_cast<unsigned>(temperatureCount));
    } else {
      appendText(displayBuffer, sizeof(displayBuffer), length, ok,
                 "  ---C NTC=0");
    }
    appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

    appendText(displayBuffer, sizeof(displayBuffer), length, ok,
               "     ADC mV | ");
    for (uint8_t gpio = 0; gpio < BATTERY_GPIO_COUNT; ++gpio) {
      const uint8_t bit = static_cast<uint8_t>(1U << gpio);
      appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "G%u=",
                   static_cast<unsigned>(gpio + 1U));
      if ((module.adcGpioMask & bit) && module.gpioReadOk &&
          (module.validGpioMask & bit)) {
        appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                    module.gpioMv[gpio], 1, 6);
      } else if (module.adcGpioMask & bit) {
        appendText(displayBuffer, sizeof(displayBuffer), length, ok, "   ---");
      } else {
        appendText(displayBuffer, sizeof(displayBuffer), length, ok, "      ");
      }
      appendText(displayBuffer, sizeof(displayBuffer), length, ok, "    ");
    }
    appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

    if (module.error[0]) {
      appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
                   "     ERROR: %s\n", module.error);
    }
  }

  if (packMinAddress && packMaxAddress) {
    appendText(displayBuffer, sizeof(displayBuffer), length, ok,
               "PACK RANGE: ");
    appendVoltage(displayBuffer, sizeof(displayBuffer), length, ok, packMin);
    appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
                 "V @%u/C%02u  to  ", static_cast<unsigned>(packMinAddress),
                 static_cast<unsigned>(packMinCell));
    appendVoltage(displayBuffer, sizeof(displayBuffer), length, ok, packMax);
    appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
                 "V @%u/C%02u  Δ", static_cast<unsigned>(packMaxAddress),
                 static_cast<unsigned>(packMaxCell));
    appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                packMax - packMin, 1, 5);
    appendText(displayBuffer, sizeof(displayBuffer), length, ok, "mV\n");
  }

  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "LIMITS: DATA=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             limits.readingsValid ? "VALID" : "INVALID");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " LOW=");
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
              limits.lowestCellV, 3, 6);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "V HIGH=");
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
              limits.highestCellV, 3, 6);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "V OV=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             limits.overVoltage ? "YES" : "NO ");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " UV=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             limits.underVoltage ? "YES" : "NO ");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " DISCHARGE=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             limits.dischargeBlocked ? "BLOCKED" : "READY  ");
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             "CHARGE: CELL_TARGET=");
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
              settings.chargeVoltageV, 3, 6);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "V PACK_TARGET=");
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
              limits.packChargeTargetV, 1, 7);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "V FOR ");
  appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "%uS  I_REQUEST=",
               static_cast<unsigned>(settings.seriesCells));
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
              limits.chargeCurrentRequestA, 1, 6);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "A PAUSED=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             limits.chargePausedByHysteresis ? "YES" : "NO ");
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "CURRENT: ");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             currentSensorName(settings.currentSensor));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " CAN=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             current.canReady ? "READY" : "OFF/ERR");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " DATA=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             current.valid ? "FRESH" : (current.frameSeen ? "STALE/FAULT" : "NONE"));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " I=");
  if (current.valid) {
    appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                current.currentMilliamp / 1000.0f, 2, 8);
  } else {
    appendText(displayBuffer, sizeof(displayBuffer), length, ok, "     ---");
  }
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "A ERR=");
  appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
               "%3u FRAMES=%lu\n", static_cast<unsigned>(current.errorCode),
               static_cast<unsigned long>(current.frameCount));

  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "SOC: MODE=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             settings.socMode == SocMode::CoulombCount ? "COULOMB" : "VOLTAGE");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " VALUE=");
  if (soc.initialized) appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                                   soc.percent, 2, 6);
  else appendText(displayBuffer, sizeof(displayBuffer), length, ok, "   ---");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "% REM=");
  if (soc.initialized) appendFixed(displayBuffer, sizeof(displayBuffer), length, ok,
                                   soc.remainingAh, 2, 7);
  else appendText(displayBuffer, sizeof(displayBuffer), length, ok, "    ---");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "Ah IN=");
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok, soc.chargedAh, 2, 9);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "Ah OUT=");
  appendFixed(displayBuffer, sizeof(displayBuffer), length, ok, soc.dischargedAh, 2, 9);
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "Ah SAVE=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             soc.pendingSave ? "PEND\n" : (soc.stateStored ? "OK\n" : "NONE\n"));

  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "BMS: STATE=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             bmsOperatingStateName(state.state));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " ARMED=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             state.armed ? "YES" : "NO ");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " RUN=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             state.runRequested ? "ON " : "OFF");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " 240V=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             state.chargeRequested ? "ON " : "OFF");
  appendFormat(displayBuffer, sizeof(displayBuffer), length, ok,
               " PRE=%lu ms\n", static_cast<unsigned long>(state.prechargeElapsedMs));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "STATE DETAIL: ");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             state.reason ? state.reason : "-");
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

  appendText(displayBuffer, sizeof(displayBuffer), length, ok, "CHARGER: ");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             chargerName(settings.chargerType));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " CAN_TX=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             settings.chargerTxEnabled ? "ENABLED" : "DISABLED");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " RELAY=");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             chargerRelayIsOn() ? "ON " : "OFF");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " PIN=");
  if (settings.chargerRelayPin == 0xFFU) {
    appendText(displayBuffer, sizeof(displayBuffer), length, ok, "UNASSIGNED");
  } else {
    appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "D%u",
                 static_cast<unsigned>(settings.chargerRelayPin));
  }
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             " ACTIVE_HIGH; STATE_MACHINE_DRIVEN\n");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             "OUTPUTS: MAIN+=");
  if (settings.mainPositivePin == 0xFFU) appendText(displayBuffer, sizeof(displayBuffer), length, ok, "---");
  else appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "D%u", static_cast<unsigned>(settings.mainPositivePin));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             bmsOutputIsActive(BmsOutputRole::MainPositive) ? " ON " : " OFF");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " MAIN-=");
  if (settings.mainNegativePin == 0xFFU) appendText(displayBuffer, sizeof(displayBuffer), length, ok, "---");
  else appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "D%u", static_cast<unsigned>(settings.mainNegativePin));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             bmsOutputIsActive(BmsOutputRole::MainNegative) ? " ON " : " OFF");
  appendText(displayBuffer, sizeof(displayBuffer), length, ok, " PRECHARGE=");
  if (settings.prechargePin == 0xFFU) appendText(displayBuffer, sizeof(displayBuffer), length, ok, "---");
  else appendFormat(displayBuffer, sizeof(displayBuffer), length, ok, "D%u", static_cast<unsigned>(settings.prechargePin));
  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             bmsOutputIsActive(BmsOutputRole::Precharge) ? " ON\n" : " OFF\n");

  appendText(displayBuffer, sizeof(displayBuffer), length, ok,
             "For configuration menu type HELP and press Enter.\n"
             "T_EST uses provisional 22k/10k/B3380 assumptions; ADC row shows raw GPIO millivolts.\n"
             "");
  appendRepeated(displayBuffer, sizeof(displayBuffer), length, ok, '=',
                 DISPLAY_FRAME_WIDTH);
  appendChar(displayBuffer, sizeof(displayBuffer), length, ok, '\n');

  if (!ok) {
    Serial.println(F("Display frame exceeded its buffer; no partial frame was sent."));
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(displayBuffer), length);
}

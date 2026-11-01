# SP51_basic_BMS

Experimental Nano R4 battery-management firmware for a 108-cell series pack
using a BQ79616 daisy-chain stack. The project reads cell and configured GPIO
ADC data, estimates thermistor temperatures from GPIO measurements, reads a
LEM CAB500-C/SP7 current sensor over CAN, and contains an input-driven BMS state
machine with precharge control.

> **Testing status: currently untested.** The integrated `SP51_basic_BMS`
> firmware has not yet been tested on the hardware. Earlier successful
> standalone BQ stack diagnostics do not validate this integrated firmware,
> its state machine, input polarity, or contactor outputs.

This is an in-development project. It has not been validated as production BMS
firmware. Treat the contactor and charger outputs as real hardware controls:
the state machine can energize mapped outputs automatically when its
interlocks pass and a physical Run or 240 V-present request is active.

## Acknowledgements

Thank you to [Tom_EVnut](https://github.com/Tom-evnut/SimpBMS) for creating and
sharing SimpBMS and for the work he has put into making the project useful to
the EV-conversion community. SimpBMS is a reference for this project's
configuration approach, precharge sequence and signed coulomb-counting method.
This project is an independent
Nano R4 implementation; it does not include SimpBMS firmware or its older
vehicle-specific CAN/CSC control code.

## Current scope

The current build includes:

- Sequential cell-voltage reads from BQ79616 devices at stack addresses
  `1..N` (nine stack devices and 108 cells by default; address 0 is the UART
  base device and is not included in the cell table).
- Reads of GPIO ADC channels enabled by each device's existing `GPIO_CONF`
  settings, plus TSREF. Temperature estimates are made only for GPIO modes
  treated as thermistor-divider inputs by the firmware.
- A fixed-width USB serial live display and line-oriented configuration menus.
- A read-only LEM current CAN receiver, with a configurable ID, bitrate,
  current sign and stale-frame timeout.
- SimpBMS-style signed coulomb counting from LEM current, reporting SOC,
  remaining Ah, cumulative Ah charged into the pack and Ah discharged out.
- Optional lowest-cell-voltage SOC estimate and first-start seed curve, with
  serial settings for the mode and both voltage/percent anchors.
- CRC-protected SOC and Ah retention in a rotating EEPROM journal, plus an
  immediate save before the console's software reboot command.
- Persistent battery, charger, precharge, input polarity and I/O pin settings.
- `Ready`, `Precharge`, `Drive` and `Charge` operating states, with additional
  `Disarmed` and latched `Fault` states.
- SimpBMS-style time-and-current precharge, with a separate timeout fault.
- Active-HIGH low-side FET gate control for main positive, main negative,
  precharge and an optional external charger relay.

The BMS state machine starts with all mapped outputs OFF. It enters `Ready`
automatically only after the stack and current readings are valid and the main
positive, main negative and precharge output pins are mapped. The physical Run
input requests Drive; the physical 240 V-present input requests Charge. No
serial `ARM` command is required.

## Hardware and pin assumptions

| Function | Default Nano R4 pin | Electrical behavior |
| --- | --- | --- |
| BQ UART RX | D0 / `Serial1` RX | Receives the base BQ79616 UART response |
| BQ UART TX | D1 / `Serial1` TX | Sends UART frames and the WAKE pulse |
| Isolated Run request | D2 | Active LOW by default; pin and polarity are configurable |
| Isolated 240 V-present request | D3 | Active LOW by default; pin and polarity are configurable |
| Main positive, main negative, precharge, charger relay | Unassigned by default | Active HIGH into low-side logic-level FET gates |

The isolated Run and 240 V input circuits invert their 12 V sense signals, so
both inputs default to active LOW. Their pin assignments and active levels can
be changed in the menu. Only the conditioned logic-level outputs of the
isolated input hardware should reach the Nano pins; do not connect a 12 V or
240 V signal directly to a Nano input.

Output pins may be assigned from D6, D13, D11 and D10. Input pins may be
assigned from D2 through D13. All six assigned input/output pins must be
unique, and D0/D1 are reserved for the BQ UART. Output pins default to
unassigned, so the state machine cannot energize a contactor until the required
three contactor outputs are mapped.

The current installation uses the base device's COMH differential pair; COML
is not connected on the base board. The base COMH manual debug override used by
the successful stack-read procedure remains enabled. Contactors, isolated
inputs and current sensing are external hardware. This firmware does not
initialize S-box features from the original SP51 hardware.

## BQ79616 startup and stack reads

`batterycomms.cpp` owns the BQ UART and startup sequence:

1. Starts `Serial1` at 1 Mbps, sends a 2.5 ms active-LOW WAKE pulse on D1, then
   waits for the stack wake propagation interval.
2. Selects the previously verified forward direction (`DIR_SEL=0`), releases
   the previous top-of-stack role, performs the DLL synchronization write to
   `OTP_ECC_TEST`, enables DIR0 auto-addressing, and assigns addresses `0..N`.
3. Sets the base role to `COMM_BASE`, stack-device roles to `COMM_STACK`, and
   the last device's role to `COMM_TOP`.
4. Enables and verifies the manual base COMH TX/RX override through the BQ
   debug registers. These are volatile register writes; customer OTP is not
   programmed.
5. Reads `DEV_CONF` and clears `MULTIDROP_EN` if it is set, then verifies the
   daisy-chain configuration. If the setting changed, it repeats addressing
   and role setup.
6. Performs the TI DLL dummy broadcast read and requires one valid response
   from every configured address, then checks the base PARTID and role.
7. Caches GPIO configuration and enables TSREF where configured ADC inputs
   need it. The monitor then reads modules at addresses `1..N` sequentially.

The DLL synchronization uses the BQ `OTP_ECC_TEST` register for its specified
test/readback sequence; this is not customer OTP programming. The startup
sequence does not issue a hardware reset pulse. Exiting the configuration
menu with `x` reruns the stack startup sequence, including WAKE and addressing.
The `reboot` command restarts the Nano; it does not hardware-reset the BQ
devices.

Each scan reads one module per `batteryCommsScanStep()` call. A complete
snapshot is marked valid only if all configured stack devices and all active
cell channels were read. GPIO/TSREF results have separate validity flags, so a
GPIO or thermistor read problem is shown separately from the cell-read result.
The requested scan period is a start-to-start target; a slow full-stack pass
is never overlapped with the next pass.

### ADC conversion and temperature estimates

Cell ADC results use the BQ result registers and the conversion factor in the
source (`0.19073 mV/count`). GPIO and TSREF readings use their corresponding
source conversion factors. Raw GPIO millivolts are printed even when a GPIO is
not configured as an NTC.

NTC estimates are provisional. Defaults assume a 22 kΩ pull-up, a 10 kΩ NTC at
25 °C, and a beta value of 3380 K; the firmware restricts estimates to the
configured divider-ratio range. These assumptions must be checked against the
actual BMW module resistor network and thermistors before treating the printed
temperatures as calibrated measurements.

## State machine and precharge

The state machine debounces both inputs for 30 ms. It requires all of the
following before automatically entering `Ready`:

- BQ stack communication initialized successfully.
- A fresh complete cell scan with valid readings and no cell over-voltage.
- A fresh, valid LEM current frame.
- Main positive, main negative and precharge output pins assigned.
- The LEM sensor selected in configuration.

When Run and 240 V-present are active together, starting is inhibited. A Run
request is also blocked by under-voltage or the discharge cut-off latch. Charge
can operate with under-voltage cells, but is blocked by invalid readings,
over-voltage, the charge target/hysteresis, a missing external-relay
configuration, or missing relay mapping.

The precharge sequence is:

1. Command all outputs OFF.
2. Close main negative and energize the precharge path.
3. Wait at least the configured minimum time and until the absolute LEM current
   is at or below the configured threshold.
4. Close main positive, then open the precharge path.
5. Enter `Drive`, or enable the external charger relay and enter `Charge`.

Defaults are 5,000 ms minimum precharge time, 1.0 A finish current and a
15,000 ms timeout. If the threshold is not reached before timeout, the state
machine latches `Fault` and commands every mapped output LOW. Loss of stack
communication, stale cell data, invalid/stale LEM data, over-voltage, or a
Drive under-voltage/discharge violation also faults an active state. Charge
stops when its request ends, Run becomes active, or the charge target latch is
reached.

The relay charge path is available only when the charger profile is set to
`External Relay` and its output pin is mapped. The default charger profile is
`Outlander OBC`, but Outlander CAN transmit remains disabled. No charger CAN
command or current regulation is implemented in this build. Charge-current
and capacity settings are stored and some values are calculated/displayed,
but they do not yet command a charger. SOC is active, but remains experimental
and depends on the configured pack capacity, current polarity, cell-voltage
seed curve and LEM sensor configuration being correct.

`disarm` commands all outputs OFF. A still-active physical request keeps the
controller inhibited until both requests return inactive. `clearfault` only
clears a latched fault when both debounced requests are inactive. There is no
contactor auxiliary-contact feedback in this version.

## USB serial console

Open the USB serial monitor at **115200 baud** and use newline/Carriage Return
to submit commands. Type `help` or `menu` to open the configuration menu. The
live table is hidden while a menu is open, but ADC scans and state-machine
safety checks continue. Accepted setting changes are written to EEPROM and the
current menu is printed again with the saved value.

Main menu sections:

| Menu | Purpose |
| --- | --- |
| `1` / `battery` | Cell over/under-voltage, discharge cut-off and hysteresis |
| `2` / `charging` | Charge target/hysteresis, current settings, series count and charger profile |
| `3` / `current` | LEM selection, CAN bitrate/ID and current sign |
| `4` / `soc` | Coulomb-count/voltage mode, SOC calibration and seed-curve anchors |
| `5` / `outputs` | Map all four FET outputs and both isolated inputs |
| `6` / `monitor` | Scan target and BQ stack retry |
| `7` / `state` | State status, precharge settings and input polarities |

All setting changes must start with `set`. Examples:

```text
help
5
set pin mainpos 6
set pin mainneg 13
set pin precharge 11
set pin charger 10
set pin run 2
set pin charge 3
back
7
set premin 5000
set pretimeout 15000
set precurrent 1.0
set runpol low
set chargepol low
```

The output example maps main positive to D6, main negative to D13, precharge
to D11 and the optional charger relay to D10. Adjust it to match the wiring;
the firmware rejects duplicate pin assignments. Input pin commands use bare
Arduino pin numbers, without the `D` prefix.

Other useful commands:

- `back` returns from a submenu to the main menu.
- `x` exits the menu and reinitializes BQ stack communication before resuming
  the live table.
- `r` in the Monitor submenu retries BQ startup.
- `disarm` commands mapped outputs OFF.
- `clearfault` in the State Control submenu clears a fault only after both
  requests have returned inactive.
- `reboot` in the main menu restarts the Nano and reloads EEPROM settings.

Battery-limit commands are `set ov`, `set uv`, `set dv` and `set dhy`. Charging
commands include `set cv`, `set hys`, `set imax`, `set iend`, `set cap`,
`set series` and `set charger <type>`. The current menu provides `set sensor`,
`set canrate`, `set canid` and `set invert`. The SOC menu provides `set socmode`,
`set soc`, `set soclowv`, `set soclowpct`, `set sochighv` and `set sochighpct`.
See each menu page for accepted ranges and current values.

Settings may be edited only while the state machine is Disarmed, Ready or
Fault (outputs are not in an active state). Input/output mappings are checked
for uniqueness together. The default active level for both isolated inputs is
LOW; the polarity commands allow `high` if the installed interface requires
it.

## Persistent settings

Settings are stored in two alternating EEPROM records with a generation number
and CRC32. A new record is read back and verified before it replaces the
previous active record. The current settings format is version 5; earlier
version-1 through version-4 records are migrated so existing limits,
charger choices, output maps and input pins are retained. Version-1 through
version-3 migrations set both isolated input polarities to active LOW; version
4 preserves the polarity choices already saved by the user.

SOC, cumulative Ah charged and cumulative Ah discharged use a separate
CRC-protected rotating journal in the EEPROM space reserved between settings
records. The coulomb counter integrates the average of adjacent fresh signed
LEM samples over their timestamp interval. Positive current means charge into
the pack; negative current means discharge out. The existing `set invert 1`
option reverses the sensor sign if needed. Coulomb mode restores the retained
SOC and Ah totals on startup; if there is no retained SOC yet, the first valid
complete cell scan seeds SOC from the configured lowest-cell-voltage curve.
Voltage-only mode continuously estimates SOC from that same two-point curve;
the Ah-in/Ah-out throughput counters still integrate fresh LEM current in this
mode. The curve is a rough estimate and needs chemistry-specific calibration.
Like any coulomb counter, the integrated estimate can drift with sensor offset
and missed/stale CAN data; periodically calibrate it with a known pack SOC
using `set soc <percent>` while in coulomb mode.

SOC/Ah records are saved every 60 seconds and on the `reboot` command; a sudden
power loss may therefore discard up to 60 seconds of new coulomb counts. The
manual `set soc <percent>` command saves immediately. The default 50 Ah pack
capacity is a placeholder and must be changed to the actual total pack capacity
before relying on displayed SOC. Because all 108 cells are in series, cell
count does not multiply pack Ah. This SOC logic follows the signed current
integration approach in [Tom_EVnut's SimpBMS source](https://github.com/Tom-evnut/BMWPhevBMS/blob/master/BMWPhevBMS.ino).

On a fresh EEPROM, defaults include nine stack devices, 12 cells per device,
500 ms target scan interval, 108 series cells, LEM CAN at 500 kbit/s with ID
`0x3C2`, and an Outlander profile with charger CAN TX disabled. All contactor
and relay output pins start unassigned. Defaults may differ after a saved
configuration has been loaded.

## Build in Arduino IDE

1. Open the `SP51_basic_BMS` sketch folder, ensuring
   `SP51_basic_BMS.ino` remains in a folder with the same name.
2. Select **Arduino Nano R4** and the connected serial port.
3. Compile and upload from the Arduino IDE.
4. Open Serial Monitor at 115200 baud and send `help` to configure the sketch.

The current LEM receiver source includes `Arduino_CAN.h`. The modified
`R7FA4M1_CAN` VCU library files are not part of this sketch snapshot. The
current project has no charger CAN transmit implementation. The present
state-machine/settings integration has not yet been compiled or tested on the
hardware; compile and review the output/input mapping before enabling physical
control outputs.

## Source map

| File | Responsibility |
| --- | --- |
| `SP51_basic_BMS.ino` | Startup order, cooperative scan scheduling, freshness checks and state updates |
| `batterycomms.cpp/.h` | BQ UART protocol, wake/address/role setup, manual COMH override and ADC reads |
| `bms_state_machine.cpp/.h` | Input debounce, Ready/Precharge/Drive/Charge transitions, interlocks and fault handling |
| `bms_outputs.cpp/.h` | Active-HIGH low-side FET output maps and safe pin changes |
| `charger_relay.cpp/.h` | Optional external charger relay adapter |
| `battery_can.cpp/.h` | Read-only LEM current CAN input and stale-frame detection |
| `soc_estimator.cpp/.h` | Signed Ah integration, voltage seed mode and SOC retention |
| `bms_limits.cpp/.h` | Cell validity, pack extrema, charge request values and hysteresis latches |
| `settings.cpp/.h` | Defaults, validation, EEPROM persistence and record migration |
| `serial_console.cpp/.h` | Configuration menus, command parsing and live display formatting |
| `history/` | Archived pre-monitor diagnostic sketch; not part of the active build |


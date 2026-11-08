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
interlocks pass and a physical Run or charge request is active.

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
- A shared LEM/Outlander CAN interface. It decodes LEM current frames and
  Outlander OBC status frames, and sends the Outlander wake/charge frames when
  enabled. Both devices share one 500 kbit/s bus while Outlander is selected.
- Saved `SME` selections for the current sensor, main-negative contactor and
  precharge contactor. These are placeholders only; SME decoding/control is not
  implemented. An SME current-sensor selection prevents the BMS from arming,
  and SME main-negative/precharge selections block Drive and behind-contactor
  charging.
- SimpBMS-style signed coulomb counting from LEM current, reporting SOC,
  remaining Ah, cumulative Ah charged into the pack and Ah discharged out.
- Optional lowest-cell-voltage SOC estimate and first-start seed curve, with
  serial settings for the mode and both voltage/percent anchors.
- CRC-protected SOC and Ah retention in a rotating EEPROM journal, plus an
  immediate save before the console's software reboot command.
- Persistent battery, charger, precharge, input polarity and I/O pin settings.
- Persistent isolation-monitor configuration for a future standalone Bender
  digital input or SME interface. Both sensing backends are placeholders and
  are not implemented.
- `Ready`, `Precharge`, `Drive` and `Charge` operating states, with additional
  `Disarmed` and latched `Fault` states.
- SimpBMS-style time-and-current precharge, with a separate timeout fault.
- Active-HIGH low-side FET gate control for main positive, main negative,
  precharge and an optional external charger relay.

The BMS state machine starts with all mapped outputs OFF. It enters `Ready`
automatically after the stack/current checks and the outputs needed for the
requested path are available. Drive and behind-contactors charging require the
main positive, main negative and precharge outputs. Direct-pack charging can
run with those contactors unmapped, but still requires the external charger
relay. The physical Run input requests Drive. A physical 240 V-present input
requests Charge for any configured charger; with the Outlander profile, fresh
OBC CAN telemetry reporting AC mains also requests Charge automatically. The
live display distinguishes the physical 240 V input, OBC mains detection and
the resulting charge request. Outlander wake frames are sent while Disarmed or
Ready so charger status can be acquired before contactors are operated. For the
Outlander profile, OBC CAN control replaces the external charger relay. No
serial `ARM` command is required.

## Hardware and pin assumptions

| Function | Default Nano R4 pin | Electrical behavior |
| --- | --- | --- |
| BQ UART RX | D0 / `Serial1` RX | Receives the base BQ79616 UART response |
| BQ UART TX | D1 / `Serial1` TX | Sends UART frames and the WAKE pulse |
| Isolated Run request | D2 | Active LOW by default; pin and polarity are configurable |
| Isolated 240 V-present request | D3 | Active LOW by default; pin and polarity are configurable |
| Nextion SoftwareSerial RX | D4 | Reserved for software UART RX; display input is not used |
| Nextion SoftwareSerial TX | D5 | Sends the live dashboard at 9600 baud |
| Isolation-monitor fault input | Unassigned | Bender placeholder; signal is not read by this build |
| Main positive, main negative, precharge, charger relay | Unassigned by default | Active HIGH into low-side logic-level FET gates |

The isolated Run and 240 V input circuits invert their 12 V sense signals, so
both inputs default to active LOW. Their pin assignments and active levels can
be changed in the menu. Only the conditioned logic-level outputs of the
isolated input hardware should reach the Nano pins; do not connect a 12 V or
240 V signal directly to a Nano input.

Output pins may be assigned from D6, D13, D11 and D10. Input pins may be
assigned from D2, D3 and D6 through D13. D0/D1 are reserved for the BQ UART;
D4/D5 are reserved for the Nextion SoftwareSerial link. Assigned inputs and
outputs must be unique. Output pins default to unassigned, so the state machine
cannot energize a contactor until the required three contactor outputs are
mapped. The optional isolation input defaults unassigned.

### Nextion dashboard

The firmware sends the legacy SimpBMS component names over SoftwareSerial at
9600 baud. D5 is the Nano TX line and connects to the Nextion RX; D4 is reserved
as the software UART RX line, but this build does not consume display input.
Connect signal ground as well, and check the display's UART input voltage
requirements before wiring. The display refreshes after each completed cell
stack scan, nominally every 500 ms. GPIO/temperature ADC blocks are polled at a
separate slower cadence (2 s by default); cached readings are marked with `~`
between polls. Each main-loop pass sends no more than one byte because the
Renesas SoftwareSerial implementation waits for that byte to finish.

The numeric fields keep the SimpBMS scales: `soc.val` and `soc1.val` are whole
percent; `current.val` and `chargercurrent.val` are tenths of an amp;
`volt.val` is tenths of a pack volt; `lowcell.val`, `highcell.val` and
`celldelta.val` are millivolts; temperatures are rounded degrees Celsius;
`chargervolts.val` is volts. `stat.txt` reports Boot/Ready/Precharge/Drive/
Charge/Error, with Disarmed shown as Boot and Fault shown as Error. The charger
fields use the latest Outlander measurements. `cellbal.txt` reports whether a
timed balance pulse is active; it remains NO while the balancing hardware lock
is in place. `ac.txt` is YES when the isolated 240 V input
or fresh Outlander mains status indicates AC. When no valid cell pack or NTC
estimate is available, the corresponding numeric display values are zero.
`firm.val` currently sends firmware ID 1 until a project version scheme is set.

The legacy snippet used `.val=YES/NO` for `cellbal` and `ac`. Since those are
text indicators, this implementation keeps the same component IDs and sends
valid Nextion text commands (`.txt="YES"` / `.txt="NO"`).

Main-negative and precharge control can each be selected as `Nano low-side FET`
or `SME placeholder` in the Outputs menu. The SME selection does not send
commands or assert a FET gate; it forces any mapped Nano output LOW and
blocks Drive and behind-contactor charging until a future SME implementation
is added. A saved Nano pin mapping is retained while SME mode is selected.

The current installation uses the base device's COMH differential pair; COML
is not connected on the base board. The base COMH manual debug override used by
the successful stack-read procedure remains enabled. Contactors, isolated
inputs and current sensing are external hardware. This firmware does not
initialize S-box features from the original SP51 hardware.

## Cell balancing status

The firmware now has persistent balancing settings and a pack-wide selection
policy. On each complete stack scan it compares the highest and lowest cell,
selects the highest cell only when the spread exceeds the configured target
delta plus hysteresis, and inhibits balancing when the lowest cell is below
the configured minimum. Defaults are balancing OFF, 20 mV target delta, 10 mV
hysteresis, 3.8 V minimum cell, 10 s configured maximum pulse, 60 s base
cool-off and conservative level 2 (which caps pulses at 10 s and multiplies
cool-off by four).

The balancing submenu is item `10`. `set bdelta`, `set bhyst`, `set bmin`,
`set bon`, `set blevel` and `set bcool` configure those values. Level 0 uses
the configured pulse/cool-off values, level 1 caps each pulse at 30 s and
doubles the cool-off, and level 2 caps pulses at 10 s and quadruples the
cool-off. The BQ79616 timer is configured for each pulse; TI's die-temperature
pause freezes its timer, so the firmware cool-off and thermal review remain
necessary.
Balancing is also gated on a complete fresh scan, a Disarmed/Ready state, no
Run or charge request, and fresh near-zero current data.

**Hardware balancing remains inhibited.** The current resistor count suggests
two 50 Ω resistors in the cell bleed path, about 42 mA at 4.2 V, or 0.176 W
combined (about 0.088 W per resistor if they share equally). The actual power
rating is unreadable, so `BALANCING_HARDWARE_CONFIRMED` remains false in
`batterycomms.cpp`; the firmware sends no balance-register writes even if the
menu request is set ON. Confirm the fitted resistor wattage and circuit path
from a readable marking or schematic before removing that interlock. The 10 Ω
parts counted as sense filters are not included in this bleed-current estimate.
There is no validated resistor-temperature measurement, so timed cooling is
not a substitute for verifying thermal limits. This feature has not been
compiled or tested on hardware.

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

Each `batteryCommsScanStep()` call starts or services at most one UART
transaction. Reply timeouts, RX-quiet gaps and ADC-ready polling are timer
driven; the function returns to the main loop while waiting. It reads ADC
control/status, cell results and configured GPIO/TSREF results sequentially for
each address. A complete snapshot is marked valid only if all configured stack
devices and all active cell channels were read. GPIO/TSREF results have
separate validity flags, so GPIO problems are reported independently. The
requested scan period is a start-to-start target; a slow full-stack pass is
never overlapped with the next pass.

If a complete scan misses any configured cell-monitor device for three
consecutive passes, the controller invalidates the cell snapshot and reruns
the existing software stack startup sequence (WAKE, forward addressing/roles,
and the manual base-COMH override). It does not pulse hardware reset. The
controller allows three automatic restart attempts, with a five-second wait
when startup itself fails. During a Drive communication recovery, it does not
call the Disarm path or drop the mapped Drive contactor commands while the
physical RUN input remains active. USB and Nextion show `STACK FAULT` during
the recovery. If the stack recovers, a complete valid scan clears that
temporary flag. If recovery remains failed, Drive outputs stay commanded until
RUN goes inactive; that transition latches `Fault` and opens outputs. The
For communication-only lockouts, three consecutive complete, valid stack scans
automatically clear the saved lockout once Run and charge requests are
inactive. The historical read-fault events remain in the bounded fault log;
they do not keep the BMS in Fault. Other persistent fault types still require
`CLEARFAULT` after their cause is resolved, requests are inactive and fresh
stack data is available. A persistent failure outside Drive also latches Fault.
Automatic retries remain enabled while Fault is latched. This recovery behavior
has not yet been tested on hardware. While the stack is
recovering, new cell-voltage measurements are unavailable; the selected Drive
hold policy continues checking LEM current and RUN input, but cannot enforce
cell-voltage limits until a complete scan returns.

The fault flag is shown on USB and Nextion. There is no separate physical fault
output pin in the current pin map. A rotating CRC-protected Nano EEPROM fault
log retains up to 40 communication events and latched faults, including type,
state, boot count, uptime and drive-cycle number. A fault is tagged CURRENT
while its drive cycle is active and PREVIOUS after that cycle ends. Open the
configuration menu and select `9` / `faults` to view the history.
The drive-cycle counter advances when precharge for a Drive request begins;
older records are labeled `OLDER DRIVE CYCLE`.
The active BMS fault code is retained in the same journal and restored after a
Nano reboot. Communication-only codes clear automatically under the recovery
conditions above. Other codes remain latched until `clearfault` is issued with
Run/charge requests inactive and a complete fresh stack scan available.

### Cooperative loop scheduling

The main loop now gives CAN reception first service on every pass, then polls
the USB console, advances one non-blocking BQ scan step, and runs scheduled
work. State-machine supervision and the Outlander task are scheduled every
10 ms; cell-scan completion also triggers an immediate state update. SOC is
updated when a new LEM frame arrives or a complete cell scan finishes. Live
display frames are sent in chunks of at most 64 bytes, and console commands
wait until the current frame has drained so the two outputs do not interleave.
The SOC journal is serviced in a low-priority slot between full scans.

Stack initialization and an explicit stack-retry command still use the
synchronous setup path; BQ UART wait periods call the CAN receive service so
the LEM sensor is still polled during that initialization. EEPROM
configuration changes from the menu are also synchronous, and menu/help text
is printed directly. The periodic SOC write is infrequent but still a
synchronous EEPROM operation; it runs only between scans. This is cooperative
scheduling, not a hard real-time guarantee. Measure CAN timing and scan
duration on the hardware before relying on it for vehicle control.

### ADC conversion and temperature estimates

Cell ADC results use the BQ result registers and the conversion factor in the
source (`0.19073 mV/count`). GPIO and TSREF readings use their corresponding
source conversion factors. Raw GPIO millivolts are printed even when a GPIO is
not configured as an NTC.

The Monitor menu configures the cell scan target with `set p <ms>` and the
temperature/GPIO poll interval with `set temp <ms>` (500..10000 ms). GPIO data
is retained between its slower polls so a cell-only scan does not display
missing temperatures as read failures.

NTC estimates are provisional. Defaults assume a 22 kΩ pull-up, a 10 kΩ NTC at
25 °C, and a beta value of 3380 K; the firmware restricts estimates to the
configured divider-ratio range. These assumptions must be checked against the
actual BMW module resistor network and thermistors before treating the printed
temperatures as calibrated measurements.

## State machine and precharge

The state machine debounces both inputs for 30 ms. It requires all of the
following before automatically entering `Ready`:

- BQ stack communication initialized successfully.
- A fresh complete cell scan with valid readings.
- A fresh, valid LEM current frame.
- Main positive, main negative and precharge output pins assigned for Drive or
  behind-contactors charging. Direct-pack charging instead requires the mapped
  external charger relay, unless Outlander OBC CAN control is selected.
- The LEM sensor selected in configuration.

When Run and a charge request are active together, starting is inhibited. For
Outlander, charge request includes either the isolated 240 V input or fresh
`0x389` AC-mains detection. A Run request is blocked by under-voltage or the
discharge cut-off latch; a Drive request under either condition latches
`Fault`. Over-voltage blocks Charge but does not block Drive, so the battery
can discharge to recover. Charge can operate with under-voltage cells so the
pack can recover, but is blocked by invalid readings, over-voltage, the charge
target/hysteresis, or missing charger readiness. The
External Relay profile also needs its mapped relay output. Outlander charging
needs enabled CAN control and fresh `0x389` telemetry reporting AC mains present
without the OBC error flag.

The default charger path is **behind contactors**, which preserves the earlier
firmware behavior. Its precharge sequence is:

1. Command all outputs OFF.
2. Close main negative and energize the precharge path.
3. Wait at least the configured minimum time and until the absolute LEM current
   is at or below the configured threshold.
4. Close main positive, then open the precharge path.
5. Enter `Drive`, enable the external charger relay and enter `Charge`, or
   start Outlander CAN charge control after its mains/status check passes.

In **direct to pack** mode, the charger is assumed wired on the battery side of
the contactors. A Charge request leaves main positive, main negative and
precharge outputs OFF, then enables the external charger relay or sends an
Outlander CAN request. When changing from Drive to direct charging, the BMS
opens the contactors before enabling the charger. Behind-contactors mode runs
the precharge sequence when starting from Ready; a Drive-to-Charge transition
can keep the already closed contactors.

Defaults are 5,000 ms minimum precharge time, 1.0 A finish current and a
15,000 ms timeout. If the threshold is not reached before timeout, the state
machine latches `Fault` and commands every mapped output LOW. Loss of stack
communication during Drive starts bounded recovery while Drive output
commands are held; if the link stays failed, the controller waits for the
physical RUN request to go inactive before latching `Fault`. In Precharge and
Charge, stack communication loss, stale cell data, invalid/stale LEM data, or
a Drive under-voltage/discharge violation faults an active state. A fresh cell
over-voltage blocks Charge while in Ready; if it occurs during Charge or
charge-directed Precharge, it latches `Fault`. Over-voltage does not fault or
block Drive, allowing discharge recovery. Under-voltage remains a Charge
recovery condition and does not itself latch `Fault` while charging. Charge
stops when its request ends, Run becomes active, or the charge target latch is
reached.

Isolation-monitor settings are in the dedicated `8` / `isolation` menu.
Monitoring defaults OFF. The menu stores the source (`Bender` or `SME`), an
optional standalone Bender digital input pin, and the future fault-signal
active level. Neither Bender nor SME input is sampled in this build. Enabling
the option therefore keeps the state machine disarmed; if enabled while Drive,
Charge, or Precharge is active, the controller opens outputs and latches
`Fault`. A future validated monitor driver can report an unsafe condition via
`bmsStateMachineReportIsolationFault()`. This API does not read a pin itself.

With the **Outlander OBC** profile selected, the Nano and LEM share the 500
kbit/s CAN bus. While `Disarmed` or `Ready`, the firmware periodically sends
the SimpBMS `0x285` wake frame and listens for charger IDs `0x389` and `0x38A`.
Fresh `0x389` telemetry reporting AC mains present can request Charge without
the isolated 240 V input; that input remains an alternate charge request. The live
display shows AC-mains presence, charging/error/ready flags, HV and DC-bus
voltage, AC voltage/current, both reported DC-current values, four temperatures,
EVSE pilot percentage, CAN freshness and TX status. A Charge request waits for
a fresh `0x389` report with AC mains present and no charger error. In behind-
contactors mode it then precharges and closes the main contactors before
sending charge commands.

During `Charge`, the firmware periodically sends Outlander command `0x286` with
the configured pack voltage target (`chargeVoltageV × seriesCells`) and the
BMS's tapered current request. The command encodes voltage and current in 0.1 V
and 0.1 A units and caps requested current at 12 A, matching the supplied
SimpBMS implementation. Stopping Charge first queues a zero-current command
when the CAN transmit mailbox is available, then opens mapped outputs. `set
chargercan off` disables wake and command transmission while CAN reception and
telemetry remain active.

Choose the path in the Charging submenu with `set chargerpath behind` or
`set chargerpath direct`. The selected path is saved in EEPROM. The relay charge
path is available only when the charger profile is set to
`External Relay` and its output pin is mapped. The default charger profile is
`Outlander OBC`. Use `set chargercan on|off` to enable or disable its wake and
charge transmissions. Selecting Outlander forces the shared LEM/OBC CAN bus to
500 kbit/s; the default Outlander path has CAN enabled. Other listed charger
profiles remain unsupported by active CAN control. SOC is active, but remains
experimental and depends on the configured pack capacity, current polarity,
cell-voltage seed curve and LEM sensor configuration being correct.

`disarm` commands outputs OFF outside Drive; in Drive it is refused and the
physical RUN input controls the transition. A still-active Run, isolated charge
input or Outlander mains request keeps the controller inhibited until the
request releases. `clearfault` requires all debounced/detected requests to be
inactive and a complete fresh stack scan. Reboot and manual stack retry are
refused during Precharge, Drive and Charge. There is no contactor
auxiliary-contact feedback in this version.

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
| `2` / `charging` | Charge target, current settings, charger profile and direct/behind-contactors path |
| `3` / `current` | LEM, Off or SME-placeholder selection; CAN bitrate/ID and current sign |
| `4` / `soc` | Coulomb-count/voltage mode, SOC calibration and seed-curve anchors |
| `5` / `outputs` | Map outputs/inputs and select main-negative/precharge controller |
| `6` / `monitor` | Scan target and BQ stack retry |
| `7` / `state` | State status, precharge settings and input polarities |
| `8` / `isolation` | Bender/SME placeholder, optional input pin and fault polarity |
| `9` / `faults` | Persistent fault and stack communication history with drive-cycle tags |

All setting changes must start with `set`. Examples:

```text
help
2
set chargerpath behind
back
5
set pin mainpos 6
set pin mainneg 13
set pin precharge 11
set pin charger 10
set contactor mainneg fet
set contactor precharge fet
set pin run 2
set pin charge 3
back
8
set isolation off
set isolationtype bender
set pin isolation none
set isolationpol high
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
- `x` exits the menu and reinitializes BQ stack communication only outside an
  active Precharge, Drive or Charge state. During an active state it closes the
  menu without disarming or restarting the stack.
- `r` in the Monitor submenu retries BQ startup only outside an active state;
  automatic recovery is used while active.
- `disarm` commands mapped outputs OFF except in Drive, where the command is
  refused and the physical RUN input controls the state transition.
- `clearfault` in the State Control submenu clears a latched fault only after
  Run/charge requests are inactive and a complete fresh stack scan is available.
- `reboot` in the main menu restarts the Nano and reloads EEPROM settings.

Battery-limit commands are `set ov`, `set uv`, `set dv` and `set dhy`. Charging
commands include `set cv`, `set hys`, `set imax`, `set iend`, `set cap`,
`set series` and `set charger <type>`. The current menu provides
`set sensor lem|off|sme`, `set canrate`, `set canid` and `set invert`. SME
current sensing is a placeholder and is not decoded. The Outputs menu provides
`set contactor mainneg fet|sme` and `set contactor precharge fet|sme`; SME
contactor control is not implemented and blocks Drive/behind-contactor Charge.
The Isolation menu stores `set isolation on|off`, `set isolationtype
bender|sme`, `set pin isolation <pin|none>` and `set isolationpol high|low`.
These are configuration placeholders only; Bender and SME monitoring are not
implemented. Enabling monitoring inhibits BMS arming.
The SOC menu provides `set socmode`,
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
previous active record. The current settings format is version 9; earlier
version-1 through version-8 records are migrated so existing limits, charger
choices, output maps and input pins are retained. New SME controller selections
default to Nano low-side FET mode during migration. Isolation monitoring
migrates disabled with no input assigned and Bender selected as the default
placeholder. Version-1 through version-3 migrations set both isolated input
polarities to active LOW; versions 4 through 8 preserve their saved polarity
choices. Version-5 settings migrate to
behind-contactors charging to preserve the prior behavior. When an older record
selects Outlander, migration enables CAN control and sets the shared bus to
500 kbit/s.

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
`0x3C2`, the Outlander OBC profile with CAN wake/charge transmission enabled,
and the behind-contactors charge path. All contactor and relay output pins
start unassigned. Defaults may differ after a saved configuration has been
loaded.

## Build in Arduino IDE

1. Open the `SP51_basic_BMS` sketch folder, ensuring
   `SP51_basic_BMS.ino` remains in a folder with the same name.
2. Select **Arduino Nano R4** and the connected serial port.
3. Compile and upload from the Arduino IDE.
4. Open Serial Monitor at 115200 baud and send `help` to configure the sketch.

The shared CAN source includes `Arduino_CAN.h` and requires the installed Nano
R4 CAN class to provide `CAN.txReady()` for single-mailbox transmit scheduling.
The project now sends Outlander CAN frames, but the integrated state machine,
charger protocol, and hardware behavior have not yet been compiled or tested.

## Source map

| File | Responsibility |
| --- | --- |
| `SP51_basic_BMS.ino` | Startup order, cooperative scan scheduling, freshness checks and state updates |
| `batterycomms.cpp/.h` | BQ UART protocol, wake/address/role setup, manual COMH override and ADC reads |
| `bms_state_machine.cpp/.h` | Input debounce, Ready/Precharge/Drive/Charge transitions, interlocks and fault handling |
| `bms_outputs.cpp/.h` | Active-HIGH low-side FET output maps and safe pin changes |
| `charger_relay.cpp/.h` | Optional external charger relay adapter |
| `battery_can.cpp/.h` | Shared LEM/OBC CAN controller, LEM decode and transmit-ready writes |
| `outlander_charger.cpp/.h` | Outlander wake/charge frames, status decode and live telemetry |
| `soc_estimator.cpp/.h` | Signed Ah integration, voltage seed mode and SOC retention |
| `bms_limits.cpp/.h` | Cell validity, pack extrema, charge request values and hysteresis latches |
| `settings.cpp/.h` | Defaults, validation, EEPROM persistence and record migration |
| `serial_console.cpp/.h` | Configuration menus, command parsing and live display formatting |
| `fault_log.cpp/.h` | Persistent typed fault/event history and drive-cycle association |
| `history/` | Archived pre-monitor diagnostic sketch; not part of the active build |


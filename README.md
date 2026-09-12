## AsTechEvolves Fork Changes

This fork extends `lll-buffed` for CP2112-controlled Klipper / Moonraker toolchanger use, with a focus on multi-buffer routing, safer host-side movement commands, status capture, and control ia an I2C bridge like [CP2112](https://a.co/d/00Z4VKH1). 

In addition to the upstream firmware features, this fork adds the following::

Adds CP2112 USB-to-I²C control for operating the buffers from Klipper on a Raspberry Pi.
Adds multi-buffer/toolchanger support, with individually addressed buffers and example routing macros.
Adds a full Klipper-side command set for auto, off, hold, push, retract, bounded-distance moves, speed, and timeout settings.
Adds safer bounded movement, including motion confirmation, maximum-distance limits, and automatic return to normal buffer operation.
Adds readback of all three raw optical sensors.
Adds decoded buffer-fill states: low, normal-low, normal-mid, normal-high, over-full, and unknown.
Adds a fail-closed over-full safety check that can prevent additional feeding when the buffer is full or its sensor state is unknown.
Adds Moonraker state capture, allowing buffer status and sensor data to be stored in Klipper macro variables.
Adds a quiet buffer communication health check for startup or pre-print validation.
Raises the firmware’s default buffer speed from 30 to 45 mm/s.
Adds Raspberry Pi 5 setup instructions, CP2112 permissions/udev guidance, and complete example macros.
Documents the physical upgrades: stronger buffer springs and a redesigned output neck using a proper ECAS PTFE connector.

## Examples
Current versions of operational macros using the below listed features can be found at my [KlipperBackup](https://github.com/astechevolves/KlipperBackup)

The Klipper helper files and basic examples are located in ./klipper/extras/AsTechEvolves Toolchanger Version/

## Basic Configuration Changes

### PlatformIO

The default firmware build still enables UART and I2C support, with the default I2C address remaining `0x10`.

For multiple buffers, each buffer should be flashed with a unique I2C address, for example:

```ini
-D I2C_ADDR=0x10
-D I2C_ADDR=0x11
-D I2C_ADDR=0x12
-D I2C_ADDR=0x13
-D I2C_ADDR=0x14
```

The fork also keeps `BUFFER_ID` support for UART-addressed buffers.

### CP2112 / Klipper Helper Setup

The added CP2112 helper expects the Silicon Labs CP2112 USB HID-to-I2C bridge:

```text
USB ID: 10c4:ea90
```

The helper scripts are intended to run from the Klipper Python environment:

```bash
~/klippy-env/bin/python
```

The added setup notes include installing HID dependencies, installing `hidapi`, verifying that the CP2112 can be opened by the normal user, and applying udev rules if the device only works with `sudo`.

### Klipper Shell Commands

The fork adds example shell-command wiring for calling the CP2112 helper from Klipper macros. Typical commands include:

```text
lll_buffer_cp2112
lll_buffer_cp2112_verbose
lll_buffer_cp2112_check
```

The quiet check helper is intended for startup or pre-print checks. It should remain silent on success and report a loud `BUFFER COMMS ERROR` on failure.

### Buffer Macro Structure

The added macro examples use numbered buffer macro names, such as:

```text
BUFFER_AUTO_0
BUFFER_OFF_0
BUFFER_MOVE_0
BUFFER_STATUS_0
BUFFER_CAPTURE_0
BUFFER_APPLY_DEFAULTS_0
```

The active-tool routing examples then map the current active extruder to the matching buffer number and call the correct numbered macro.

Example mapping pattern:

```text
T0 -> Buffer0
T1 -> Buffer1
T2 -> Buffer2
T3 -> Buffer3
T4 -> Buffer4
```

### Buffer0 Defaults Example

The included Buffer0 macro example centralizes defaults in `_LLL_BUFFER_VARS_0`, including:

```text
addr: 0x10
default_speed: 60.0 mm/s
default_timeout_ms: 60000
default_emptying_timeout_ms: 2500
default_hold_timeout_ms: 10000
default_hold_timeout_enable: 1
default_multi_press_count: 2
max_speed: 80.0 mm/s
max_move_distance: 2000.0 mm
move_settle_ms: 250
```

These values are used by `BUFFER_APPLY_DEFAULTS_0` so firmware RAM-only settings can be reapplied after buffer power cycle, Klipper restart, or printer startup.

### New Status Data

The fork adds readback support for both raw and interpreted buffer state.

Raw sensor data:

```text
sensor_bits
optical1
optical2
optical3
```

Interpreted buffer state:

```text
low
normal-low
normal-mid
normal-high
over-full
unknown
```

The interpreted states are meant to make macro logic easier and safer than working only from raw optical sensor bits.

### Motion Safety Changes

The macro examples separate bounded movement from modal forced movement.

Preferred normal macro motion:

```text
BUFFER_MOVE_0 DISTANCE=<mm> SPEED=<mm/s> WAIT=1 AUTO=1
```

Manual/debug modal motion requires confirmation:

```text
BUFFER_PUSH_0 CONFIRM=1
BUFFER_PULL_0 CONFIRM=1
```

This keeps normal load/unload logic on bounded `MOVE_DIST` moves while still leaving forced push/pull available for testing.

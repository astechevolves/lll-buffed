
import argparse
import hid
import os
import struct
import sys
import time

#####################################################################
#                  LLL-Buffed CP2112 Buffer Helper
#####################################################################
# Purpose:
# - Control one or more lll-buffed buffer(s) over a CP2112 HID USB-to-SMBus bridge.
# - Expose the full current lll-buffed I2C toolbox:
#   - state commands: off, auto/regular, hold/manual
#   - modal debug motion: push/feed, pull/retract
#   - bounded firmware moves: move by distance
#   - settings: speed, timeout, emptying timeout, hold timeout,
#     hold timeout enable, and multi-press count
#   - status reads: filament prsent, timed out, mode, speed set, timeout values
#
# Basic examples:
#   BUFFER_CMD=status  ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=auto    ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=off     ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#
# Motion examples:
#   BUFFER_CMD=push ALLOW_MOTION=1 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=pull ALLOW_MOTION=1 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#
# Firmware-bounded distance moves:
#   BUFFER_CMD=move BUFFER_DISTANCE=10  ALLOW_MOTION=1 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=move BUFFER_DISTANCE=-10 ALLOW_MOTION=1 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#
# Settings examples:
#   BUFFER_CMD=speed BUFFER_SPEED=45 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=timeout BUFFER_TIMEOUT_MS=60000 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=hold-timeout BUFFER_HOLD_TIMEOUT_MS=10000 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_CMD=hold-timeout-enable BUFFER_HOLD_TIMEOUT_ENABLE=1 ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#
# Multi-buffer examples:
#   BUFFER_ADDR=0x10 BUFFER_CMD=status ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_ADDR=0x11 BUFFER_CMD=status ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#####################################################################

# CP2112 USB VID/PID.
DEFAULT_VID = 0x10C4
DEFAULT_PID = 0xEA90

# lll-buffed default 7-bit I2C address.
DEFAULT_ADDR_7BIT = 0x10

#####################################################################
#                  lll-buffed Virtual I2C Register Map
#####################################################################
# Register details match the lll-buffed README:
# - Little-endian values.
# - Float registers are IEEE754 32-bit.
# - Timeout/settings registers are uint32 or uint8.
# - Writes are staged byte-by-byte for 32-bit values because that is the
#   known-good CP2112 behavior from prior MOVE_DIST/SPEED testing.
#####################################################################

REG_COMMAND = 0x00
REG_MOVE_DIST = 0x01
REG_STATUS = 0x05
REG_MODE = 0x06
REG_MOTOR = 0x07
REG_SPEED = 0x08
REG_TIMEOUT = 0x0C
REG_EMPTYING_TIMEOUT = 0x10
REG_HOLD_TIMEOUT = 0x14
REG_HOLD_TIMEOUT_EN = 0x18
REG_MULTI_PRESS = 0x19

# CP2112 HID report IDs used for SMBus/I2C transactions.
DATA_WRITE_READ = 0x11
DATA_READ_FORCE_SEND = 0x12
DATA_READ_RESPONSE = 0x13
DATA_WRITE = 0x14
TRANSFER_STATUS_REQUEST = 0x15
TRANSFER_STATUS_RESPONSE = 0x16
CANCEL_TRANSFER = 0x17

# lll-buffed command register values.
# These commands change firmware mode or start modal forced motion.
BUFFER_CMDS = {
    "off": 0x00,
    "disable": 0x00,

    "regular": 0x01,
    "auto": 0x01,
    "normal": 0x01,
    "resume": 0x01,

    "hold": 0x02,
    "manual": 0x02,

    "push": 0x03,
    "feed": 0x03,

    "retract": 0x04,
    "pull": 0x04,
}

# Reported mode names. These are read from REG_MODE.
# NOTE: Command 0x02 is called hold/manual in commands, but reported modes
# can distinguish HOLD and MANUAL depending on firmware state.
MODE_NAMES = {
    0: "REGULAR",
    1: "CONTINUOUS",
    2: "MOVE_COMMAND",
    3: "HOLD",
    4: "MANUAL",
    5: "EMPTYING",
}

# Reported motor state names read from REG_MOTOR.
# The lll-buffed README lists Motor State as:
# 0=Push, 1=Retract, 2=Hold, 3=Off.
MOTOR_NAMES = {
    0: "PUSH",
    1: "RETRACT",
    2: "HOLD",
    3: "OFF",
}

# CP2112 transfer status names. These are bridge-level statuses, not buffer modes.
STATUS0_NAMES = {
    0x00: "IDLE",
    0x01: "BUSY",
    0x02: "COMPLETE",
    0x03: "ERROR",
}

STATUS1_NAMES = {
    0x00: "TIMEOUT_NACK",
    0x01: "TIMEOUT_BUS",
    0x02: "BUSY_DETAIL_OR_ARBITRATION_LOST",
    0x03: "READ_INCOMPLETE",
    0x04: "WRITE_INCOMPLETE",
    0x05: "SUCCESS",
}

DEBUG = False


#####################################################################
#                  Environment / CLI Parsing Helpers
#####################################################################

def parse_int_auto(value, default=None):
    # Accept decimal strings like "16" and hex strings like "0x10".
    if value is None or value == "":
        return default

    value = str(value).strip()

    if value.lower().startswith("0x"):
        return int(value, 16)

    return int(value, 10)


def env_bool(name, default=False):
    # Allow shell env vars such as ALLOW_MOTION=1 or BUFFER_DEBUG=true.
    value = os.environ.get(name)

    if value is None:
        return default

    return value.strip().lower() in ("1", "true", "yes", "y", "on")


def env_float(name, default=None):
    # Float parser used for speed and distance env vars.
    value = os.environ.get(name)

    if value is None or value == "":
        return default

    return float(value)


def env_int(name, default=None):
    # Integer parser used for timeout and count env vars.
    value = os.environ.get(name)

    if value is None or value == "":
        return default

    return parse_int_auto(value, default=default)


def dprint(message):
    # Debug prints are gated so normal Klipper console output stays readable.
    if DEBUG:
        print(message)


def rpt(data):
    # CP2112 HID reports are 64 bytes. Pad shorter reports.
    if len(data) > 64:
        raise ValueError(f"CP2112 report too long: {len(data)} bytes")

    return bytes(data + [0x00] * (64 - len(data)))


#####################################################################
#                  CP2112 / lll-buffed Bridge
#####################################################################

class BufferBridge:
    def __init__(self, addr_7bit=DEFAULT_ADDR_7BIT, vid=DEFAULT_VID, pid=DEFAULT_PID):
        self.addr_7bit = addr_7bit
        self.addr_cp2112 = addr_7bit << 1
        self.vid = vid
        self.pid = pid
        self.dev = None

    def open(self):
        # Pick a CP2112 by explicit path, serial, index, or default to index 0.
        # This keeps future multi-bridge setups possible without changing code.
        devices = hid.enumerate(self.vid, self.pid)

        if not devices:
            raise RuntimeError("No CP2112 found")

        wanted_serial = os.environ.get("CP2112_SERIAL")
        wanted_path = os.environ.get("CP2112_PATH")
        wanted_index = parse_int_auto(os.environ.get("CP2112_INDEX"), default=0)

        selected = None

        if wanted_path:
            wanted_path_bytes = wanted_path.encode()
            for device in devices:
                if device.get("path") == wanted_path_bytes:
                    selected = device
                    break

            if selected is None:
                raise RuntimeError(f"No CP2112 found with CP2112_PATH={wanted_path}")

        elif wanted_serial:
            for device in devices:
                if str(device.get("serial_number")) == wanted_serial:
                    selected = device
                    break

            if selected is None:
                raise RuntimeError(f"No CP2112 found with CP2112_SERIAL={wanted_serial}")

        else:
            if wanted_index < 0 or wanted_index >= len(devices):
                raise RuntimeError(
                    f"CP2112_INDEX={wanted_index} out of range; found {len(devices)} device(s)"
                )

            selected = devices[wanted_index]

        dprint(f"DEVICE: {selected}")

        self.dev = hid.device()
        self.dev.open_path(selected["path"])
        self.dev.set_nonblocking(False)

    def close(self):
        # Always close the HID handle so Klipper shell calls do not leave handles open.
        if self.dev is not None:
            self.dev.close()
            self.dev = None

    def drain_reports(self, timeout_s=0.05):
        # CP2112 can leave stale 0x13/0x16 reports queued.
        # Drain before starting a new transaction so old responses are not misread.
        self.dev.set_nonblocking(True)

        deadline = time.time() + timeout_s
        count = 0

        while time.time() < deadline:
            r = self.dev.read(64)

            if not r:
                break

            count += 1
            dprint(f"DRAIN {count} report=0x{r[0]:02x} raw={r}")

        self.dev.set_nonblocking(False)
        dprint(f"drained {count} report(s)")

        return count

    def cancel_transfer(self):
        # Cancel stale SMBus transfer state before each user-level command.
        # This proved helpful during CP2112 BUSY/ARBITRATION_LOST testing.
        dprint("CANCEL")
        self.dev.write(rpt([CANCEL_TRANSFER, 0x01]))
        time.sleep(0.10)
        self.drain_reports()

    def wait_transfer_complete(self, timeout_s=1.0):
        # Poll the CP2112 transfer status until the bridge reports complete.
        # This only confirms the USB-to-I2C transaction, not motor completion.
        deadline = time.time() + timeout_s
        last_status = None

        while time.time() < deadline:
            self.dev.write(rpt([TRANSFER_STATUS_REQUEST, 0x01]))
            r = self.dev.read(64, 100)

            if not r:
                continue

            dprint(f"STATUS report=0x{r[0]:02x} raw={r}")

            if r[0] != TRANSFER_STATUS_RESPONSE:
                continue

            status0 = r[1]
            status1 = r[2]
            retries = (r[3] << 8) | r[4]
            count = (r[5] << 8) | r[6]

            last_status = (status0, status1, retries, count, r)

            dprint(
                "decoded "
                f"status0=0x{status0:02x} {STATUS0_NAMES.get(status0, 'UNKNOWN')} "
                f"status1=0x{status1:02x} {STATUS1_NAMES.get(status1, 'UNKNOWN')} "
                f"retries={retries} count={count}"
            )

            if status0 == 0x01:
                # BUSY
                time.sleep(0.02)
                continue

            if status0 == 0x02:
                # COMPLETE
                return count

            if status0 == 0x03:
                raise RuntimeError(
                    "transfer error: "
                    f"status0=0x{status0:02x} {STATUS0_NAMES.get(status0, 'UNKNOWN')} "
                    f"status1=0x{status1:02x} {STATUS1_NAMES.get(status1, 'UNKNOWN')} "
                    f"retries={retries} count={count} raw={r}"
                )

            time.sleep(0.02)

        if last_status is not None:
            status0, status1, retries, count, raw = last_status
            raise RuntimeError(
                "transfer status timeout; last status was "
                f"status0=0x{status0:02x} {STATUS0_NAMES.get(status0, 'UNKNOWN')} "
                f"status1=0x{status1:02x} {STATUS1_NAMES.get(status1, 'UNKNOWN')} "
                f"retries={retries} count={count} raw={raw}"
            )

        raise RuntimeError("transfer status timeout; no transfer-status response received")

    def force_read_response(self, length):
        # Force CP2112 to emit the 0x13 data response after a write-read request.
        request = [
            DATA_READ_FORCE_SEND,
            (length >> 8) & 0xFF,
            length & 0xFF,
        ]

        dprint(f"FORCE_READ request={request}")
        self.dev.write(rpt(request))

    def read_forced_response(self, length, timeout_s=1.0):
        # Read the actual payload from the forced 0x13 response.
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            r = self.dev.read(64, 100)

            if not r:
                continue

            dprint(f"READ report=0x{r[0]:02x} raw={r}")

            if r[0] != DATA_READ_RESPONSE:
                continue

            # CP2112 data read response:
            # r[0] = 0x13 report ID
            # r[1] = status/unused for this use
            # r[2] = number of valid payload bytes
            # r[3:] = returned I2C data
            data_len = r[2]
            data = list(r[3:3 + data_len])

            if data_len < length:
                raise RuntimeError(f"short read expected={length} got={data_len} raw={r}")

            return data[:length]

        raise RuntimeError("read response timeout")

    def read_reg(self, reg, length=1):
        # Read one virtual register or a contiguous register value from lll-buffed.
        self.drain_reports()

        request = [
            DATA_WRITE_READ,
            self.addr_cp2112,
            (length >> 8) & 0xFF,
            length & 0xFF,
            0x01,
            reg & 0xFF,
        ]

        dprint(f"WRITE_READ request={request}")

        self.dev.write(rpt(request))

        count = self.wait_transfer_complete()

        if count < length:
            raise RuntimeError(f"read transfer complete but count={count}, expected={length}")

        self.force_read_response(length)
        return self.read_forced_response(length)

    def read_u8(self, reg):
        # Read an 8-bit register such as STATUS, MODE, MOTOR, or enable flags.
        return self.read_reg(reg, 1)[0]

    def read_u32(self, reg):
        # lll-buffed stores uint32 registers as 4-byte little-endian values.
        data = bytes(self.read_reg(reg, 4))
        return struct.unpack("<I", data)[0]

    def read_f32(self, reg):
        # lll-buffed stores float registers as 4-byte little-endian values.
        data = bytes(self.read_reg(reg, 4))
        return struct.unpack("<f", data)[0]

    def write_reg(self, reg, data):
        # CP2112 Data Write Request:
        # payload is [register] + data bytes.
        # For this setup, staged 2-byte writes have been the most reliable path.
        payload = [reg & 0xFF] + [b & 0xFF for b in data]

        if len(payload) > 61:
            raise RuntimeError(f"CP2112 write payload too long: {len(payload)} bytes")

        self.drain_reports()

        request = [
            DATA_WRITE,
            self.addr_cp2112,
            len(payload),
        ] + payload

        dprint(f"WRITE request={request}")

        self.dev.write(rpt(request))

        # For CP2112 write transfers, count is received/read bytes.
        # Successful writes normally report count=0.
        self.wait_transfer_complete()

        return len(payload)

    def write_bytes_staged(self, reg_base, payload):
        # Write a multi-byte lll-buffed value one byte at a time:
        # [reg_base + 0, byte0]
        # [reg_base + 1, byte1]
        # [reg_base + 2, byte2]
        # [reg_base + 3, byte3] -> firmware commits value.
        for offset, byte in enumerate(payload):
            self.write_reg(reg_base + offset, [byte & 0xFF])

        return payload

    def write_f32_staged(self, reg_base, value):
        # Stage a 32-bit float write for MOVE_DIST and SPEED.
        payload = list(struct.pack("<f", float(value)))
        dprint(f"STAGED_F32 reg=0x{reg_base:02x} value={value} bytes={payload}")

        return self.write_bytes_staged(reg_base, payload)

    def write_u32_staged(self, reg_base, value):
        # Stage a 32-bit unsigned integer write for timeout settings.
        value = int(value)

        if value < 0:
            raise RuntimeError("uint32 register value cannot be negative")

        payload = list(struct.pack("<I", value))
        dprint(f"STAGED_U32 reg=0x{reg_base:02x} value={value} bytes={payload}")

        return self.write_bytes_staged(reg_base, payload)

    def send_command(self, name):
        # Send a one-byte lll-buffed mode/motion command.
        command = BUFFER_CMDS[name]
        self.write_reg(REG_COMMAND, [command])
        print(f"sent command: {name} ({command})")

    def set_speed(self, speed):
        # Set firmware movement speed in mm/s.
        # This is not a Klipper live velocity override; it changes lll-buffed firmware speed.
        if speed <= 0:
            raise RuntimeError("speed must be greater than 0")

        payload = self.write_f32_staged(REG_SPEED, speed)
        readback = self.read_f32(REG_SPEED)

        print(f"set speed={speed:.2f} mm/s bytes={payload}")
        print(f"readback speed={readback:.2f} mm/s")

    def set_timeout_ms(self, timeout_ms):
        # Regular/continuous mode timeout.
        # This is the setting that prevents the buffer from feeding forever
        # when sensors do not show progress.
        timeout_ms = int(timeout_ms)

        if timeout_ms < 0:
            raise RuntimeError("timeout_ms cannot be negative")

        payload = self.write_u32_staged(REG_TIMEOUT, timeout_ms)
        readback = self.read_u32(REG_TIMEOUT)

        print(f"set timeout={timeout_ms} ms bytes={payload}")
        print(f"readback timeout={readback} ms")

    def set_emptying_timeout_ms(self, timeout_ms):
        # Emptying timeout used when no filament is detected and the firmware
        # is trying to finish pushing the last bit out.
        timeout_ms = int(timeout_ms)

        if timeout_ms < 0:
            raise RuntimeError("emptying_timeout_ms cannot be negative")

        payload = self.write_u32_staged(REG_EMPTYING_TIMEOUT, timeout_ms)
        readback = self.read_u32(REG_EMPTYING_TIMEOUT)

        print(f"set emptying_timeout={timeout_ms} ms bytes={payload}")
        print(f"readback emptying_timeout={readback} ms")

    def set_hold_timeout_ms(self, timeout_ms):
        # Hold power-save timeout.
        # When hold timeout is enabled, this controls how long HOLD keeps motor
        # output energized before power-save disables it.
        timeout_ms = int(timeout_ms)

        if timeout_ms < 0:
            raise RuntimeError("hold_timeout_ms cannot be negative")

        payload = self.write_u32_staged(REG_HOLD_TIMEOUT, timeout_ms)
        readback = self.read_u32(REG_HOLD_TIMEOUT)

        print(f"set hold_timeout={timeout_ms} ms bytes={payload}")
        print(f"readback hold_timeout={readback} ms")

    def set_hold_timeout_enable(self, enable):
        # Enable/disable hold power-save mode.
        enable = 1 if int(enable) else 0

        self.write_reg(REG_HOLD_TIMEOUT_EN, [enable])
        readback = self.read_u8(REG_HOLD_TIMEOUT_EN)

        print(f"set hold_timeout_enable={enable}")
        print(f"readback hold_timeout_enable={readback}")

    def set_multi_press_count(self, count):
        # Configure how many physical button presses enter continuous mode.
        count = int(count)

        if count < 0 or count > 255:
            raise RuntimeError("multi_press_count must be between 0 and 255")

        self.write_reg(REG_MULTI_PRESS, [count])
        readback = self.read_u8(REG_MULTI_PRESS)

        print(f"set multi_press_count={count}")
        print(f"readback multi_press_count={readback}")

    def move_distance(self, distance):
        # Trigger firmware-bounded MOVE_DIST.
        # Positive = push/feed toward the hotend.
        # Negative = pull/retract toward the spool.
        payload = self.write_f32_staged(REG_MOVE_DIST, distance)
        print(f"sent move distance={distance:.3f} mm bytes={payload}")

    def read_status(self):
        # Read both live state and configurable firmware settings.
        # This makes BUFFER_STATUS useful for confirming settings after writes.
        status = self.read_u8(REG_STATUS)
        mode = self.read_u8(REG_MODE)
        motor = self.read_u8(REG_MOTOR)
        speed = self.read_f32(REG_SPEED)
        timeout = self.read_u32(REG_TIMEOUT)
        emptying_timeout = self.read_u32(REG_EMPTYING_TIMEOUT)
        hold_timeout = self.read_u32(REG_HOLD_TIMEOUT)
        hold_timeout_en = self.read_u8(REG_HOLD_TIMEOUT_EN)
        multi_press = self.read_u8(REG_MULTI_PRESS)

        return {
            "status": status,
            "filament_present": status & 0x01,
            "timed_out": (status >> 1) & 0x01,
            "mode": mode,
            "mode_name": MODE_NAMES.get(mode, "UNKNOWN"),
            "motor": motor,
            "motor_name": MOTOR_NAMES.get(motor, "UNKNOWN"),
            "speed": speed,
            "timeout": timeout,
            "emptying_timeout": emptying_timeout,
            "hold_timeout": hold_timeout,
            "hold_timeout_en": hold_timeout_en,
            "multi_press": multi_press,
        }

    def print_status(self):
        # Console-friendly status output for Mainsail/Klipper shell_command logs.
        st = self.read_status()

        print("buffer read OK")
        print(f"addr=0x{self.addr_7bit:02x}")
        print(f"status=0x{st['status']:02x}")
        print(f"filament_present={st['filament_present']}")
        print(f"timed_out={st['timed_out']}")
        print(f"mode={st['mode']} ({st['mode_name']})")
        print(f"motor={st['motor']} ({st['motor_name']})")
        print(f"speed={st['speed']:.2f} mm/s")
        print(f"timeout={st['timeout']} ms")
        print(f"emptying_timeout={st['emptying_timeout']} ms")
        print(f"hold_timeout={st['hold_timeout']} ms")
        print(f"hold_timeout_en={st['hold_timeout_en']}")
        print(f"multi_press={st['multi_press']}")


#####################################################################
#                  CLI Definition
#####################################################################

def build_parser():
    parser = argparse.ArgumentParser(
        description="Control/read lll-buffed buffer through CP2112 HID bridge."
    )

    parser.add_argument(
        "action",
        nargs="?",
        default=os.environ.get("BUFFER_CMD", "status").lower(),
        help=(
            "Action: status, off, disable, auto, regular, normal, resume, "
            "hold, manual, push, feed, pull, retract, move, speed, set_speed, "
            "timeout, emptying-timeout, hold-timeout, hold-timeout-enable, multi-press"
        ),
    )

    parser.add_argument(
        "--addr",
        default=os.environ.get("BUFFER_ADDR", os.environ.get("BUFFER_I2C_ADDR", "0x10")),
        help="7-bit I2C address, for example 0x10. Env: BUFFER_ADDR",
    )

    parser.add_argument(
        "--distance",
        type=float,
        default=env_float("BUFFER_DISTANCE"),
        help="Distance in mm for action=move. Positive=push/feed, negative=pull/retract.",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=env_float("BUFFER_SPEED"),
        help="Speed in mm/s for action=speed/set_speed, or optional pre-set before move.",
    )

    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=env_int("BUFFER_TIMEOUT_MS"),
        help="Regular/continuous mode timeout in milliseconds. Env: BUFFER_TIMEOUT_MS",
    )

    parser.add_argument(
        "--emptying-timeout-ms",
        type=int,
        default=env_int("BUFFER_EMPTYING_TIMEOUT_MS"),
        help="Emptying timeout in milliseconds. Env: BUFFER_EMPTYING_TIMEOUT_MS",
    )

    parser.add_argument(
        "--hold-timeout-ms",
        type=int,
        default=env_int("BUFFER_HOLD_TIMEOUT_MS"),
        help="Hold power-save timeout in milliseconds. Env: BUFFER_HOLD_TIMEOUT_MS",
    )

    parser.add_argument(
        "--hold-timeout-enable",
        type=int,
        choices=[0, 1],
        default=env_int("BUFFER_HOLD_TIMEOUT_ENABLE"),
        help="Enable hold power-save mode: 0 or 1. Env: BUFFER_HOLD_TIMEOUT_ENABLE",
    )

    parser.add_argument(
        "--multi-press-count",
        type=int,
        default=env_int("BUFFER_MULTI_PRESS_COUNT"),
        help="Physical button multi-press count. Env: BUFFER_MULTI_PRESS_COUNT",
    )

    parser.add_argument(
        "--max-distance",
        type=float,
        default=env_float("BUFFER_MAX_DISTANCE", 300.0),
        help="Safety cap for absolute move distance in mm. Env: BUFFER_MAX_DISTANCE",
    )

    parser.add_argument(
        "--allow-motion",
        action="store_true",
        default=env_bool("ALLOW_MOTION", False),
        help="Required for modal push/pull and MOVE_DIST motion commands.",
    )

    parser.add_argument(
        "--read-after",
        action="store_true",
        default=env_bool("BUFFER_READ_AFTER", False),
        help="Read status after motion command. Default false for push/pull/move.",
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        default=env_bool("BUFFER_DEBUG", False),
        help="Print raw CP2112 reports.",
    )

    return parser


def normalize_action(action):
    # Accept several aliases so Klipper macro names can stay readable.
    action = action.lower().strip()

    aliases = {
        "set-speed": "speed",
        "set_speed": "speed",

        "moves": "move",
        "move_dist": "move",
        "move-distance": "move",

        "set-timeout": "timeout",
        "set_timeout": "timeout",

        "emptying_timeout": "emptying-timeout",
        "set-emptying-timeout": "emptying-timeout",
        "set_emptying_timeout": "emptying-timeout",

        "hold_timeout": "hold-timeout",
        "set-hold-timeout": "hold-timeout",
        "set_hold_timeout": "hold-timeout",

        "hold_timeout_enable": "hold-timeout-enable",
        "hold-timeout-en": "hold-timeout-enable",
        "hold_timeout_en": "hold-timeout-enable",
        "set-hold-timeout-enable": "hold-timeout-enable",
        "set_hold_timeout_enable": "hold-timeout-enable",
        "set_hold_timeout_en": "hold-timeout-enable",

        "multi_press": "multi-press",
        "multi-press-count": "multi-press",
        "multi_press_count": "multi-press",
        "set-multi-press-count": "multi-press",
        "set_multi_press_count": "multi-press",
    }

    return aliases.get(action, action)


#####################################################################
#                  Main Command Dispatcher
#####################################################################

def main():
    global DEBUG

    parser = build_parser()
    args = parser.parse_args()

    DEBUG = args.debug

    action = normalize_action(args.action)
    addr_7bit = parse_int_auto(args.addr, default=DEFAULT_ADDR_7BIT)

    if addr_7bit < 0x03 or addr_7bit > 0x77:
        raise RuntimeError(f"Invalid 7-bit I2C address: 0x{addr_7bit:02x}")

    bridge = BufferBridge(addr_7bit=addr_7bit)

    try:
        bridge.open()

        print("opened CP2112")
        print(f"target addr 7-bit=0x{addr_7bit:02x}, cp2112=0x{bridge.addr_cp2112:02x}")

        # Start every command from a clean CP2112 transfer state.
        bridge.cancel_transfer()

        if action == "status":
            bridge.print_status()
            return

        if action == "speed":
            if args.speed is None:
                raise RuntimeError("action=speed requires --speed or BUFFER_SPEED")

            bridge.set_speed(args.speed)
            return

        if action == "timeout":
            if args.timeout_ms is None:
                raise RuntimeError("action=timeout requires --timeout-ms or BUFFER_TIMEOUT_MS")

            bridge.set_timeout_ms(args.timeout_ms)
            return

        if action == "emptying-timeout":
            if args.emptying_timeout_ms is None:
                raise RuntimeError(
                    "action=emptying-timeout requires "
                    "--emptying-timeout-ms or BUFFER_EMPTYING_TIMEOUT_MS"
                )

            bridge.set_emptying_timeout_ms(args.emptying_timeout_ms)
            return

        if action == "hold-timeout":
            if args.hold_timeout_ms is None:
                raise RuntimeError(
                    "action=hold-timeout requires --hold-timeout-ms or BUFFER_HOLD_TIMEOUT_MS"
                )

            bridge.set_hold_timeout_ms(args.hold_timeout_ms)
            return

        if action == "hold-timeout-enable":
            if args.hold_timeout_enable is None:
                raise RuntimeError(
                    "action=hold-timeout-enable requires "
                    "--hold-timeout-enable or BUFFER_HOLD_TIMEOUT_ENABLE"
                )

            bridge.set_hold_timeout_enable(args.hold_timeout_enable)
            return

        if action == "multi-press":
            if args.multi_press_count is None:
                raise RuntimeError(
                    "action=multi-press requires --multi-press-count or BUFFER_MULTI_PRESS_COUNT"
                )

            bridge.set_multi_press_count(args.multi_press_count)
            return

        if action == "move":
            if args.distance is None:
                raise RuntimeError("action=move requires --distance or BUFFER_DISTANCE")

            if not args.allow_motion:
                raise RuntimeError("action=move requires --allow-motion or ALLOW_MOTION=1")

            if abs(args.distance) > args.max_distance:
                raise RuntimeError(
                    f"Refusing move distance {args.distance:.3f} mm; "
                    f"max is {args.max_distance:.3f} mm"
                )

            if args.speed is not None:
                bridge.set_speed(args.speed)

            bridge.move_distance(args.distance)

            # MOVE_DIST leaves the buffer outside native hall-sensor auto logic.
            # AUTO/REGULAR should be sent later only when intentionally returning
            # control to the firmware state machine.
            if args.read_after:
                bridge.print_status()

            return

        if action in BUFFER_CMDS:
            is_modal_motion = action in ("push", "feed", "retract", "pull")

            if is_modal_motion and not args.allow_motion:
                raise RuntimeError(f"BUFFER_CMD={action} requires ALLOW_MOTION=1")

            # If push/feed/pull/retract is given with BUFFER_DISTANCE, use the
            # firmware-bounded MOVE_DIST path instead of modal forced motion.
            # This is safer for macros than indefinite forced motion.
            if is_modal_motion and args.distance is not None:
                distance = abs(args.distance)

                if action in ("retract", "pull"):
                    distance = -distance

                if abs(distance) > args.max_distance:
                    raise RuntimeError(
                        f"Refusing move distance {distance:.3f} mm; "
                        f"max is {args.max_distance:.3f} mm"
                    )

                if args.speed is not None:
                    bridge.set_speed(args.speed)

                bridge.move_distance(distance)

                if args.read_after:
                    bridge.print_status()

                return

            bridge.send_command(action)

            # Push/feed and pull/retract are modal forced-motion commands.
            # Do not immediately read status afterward unless explicitly requested.
            # During forced motion, hall sensor logic is ignored until AUTO/REGULAR.
            if is_modal_motion and not args.read_after:
                return

            bridge.print_status()
            return

        valid = sorted(list(BUFFER_CMDS.keys()) + [
            "status",
            "move",
            "speed",
            "timeout",
            "emptying-timeout",
            "hold-timeout",
            "hold-timeout-enable",
            "multi-press",
        ])
        raise RuntimeError(f"Unknown action={action}. Valid actions: {', '.join(valid)}")

    finally:
        bridge.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
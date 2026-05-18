
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
# - Controls more than one buffer over a shared USB connection vis CP2112 HID bridge.
# - Provides ability to call buffer address at command line
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
# Multi-buffer examples:
#   BUFFER_ADDR=0x10 BUFFER_CMD=status ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#   BUFFER_ADDR=0x11 BUFFER_CMD=status ~/klippy-env/bin/python ~/lll_cp2112_buffer.py
#####################################################################

# CP2112 USB VID/PID.
DEFAULT_VID = 0x10C4
DEFAULT_PID = 0xEA90

# lll-buffed default 7-bit I2C address.
DEFAULT_ADDR_7BIT = 0x10 # Toolhead0's buffer

# lll-buffed I2C registers.
REG_COMMAND = 0x00
REG_MOVE_DIST = 0x01
REG_STATUS = 0x05
REG_MODE = 0x06
REG_MOTOR = 0x07
REG_SPEED = 0x08

# CP2112 HID report IDs.
DATA_WRITE_READ = 0x11
DATA_READ_FORCE_SEND = 0x12
DATA_READ_RESPONSE = 0x13
DATA_WRITE = 0x14
TRANSFER_STATUS_REQUEST = 0x15
TRANSFER_STATUS_RESPONSE = 0x16
CANCEL_TRANSFER = 0x17

# lll-buffed command register values. Different logical names that may be used for the same command are supported as aliases.
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

    "pull": 0x04,
    "retract": 0x04,
}

# These are intentionally separate from BUFFER_CMDS.
# The command number for "hold" is 0x02, but reported Motor::Hold is 3.
MODE_NAMES = {
    0: "REGULAR",
    1: "CONTINUOUS",
    2: "MOVE_COMMAND",
    3: "HOLD",
    4: "MANUAL",
}

MOTOR_NAMES = {
    0: "OFF",
    1: "PUSH",
    2: "RETRACT",
    3: "HOLD",
}

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


def parse_int_auto(value, default=None):
    if value is None or value == "":
        return default

    value = str(value).strip()

    if value.lower().startswith("0x"):
        return int(value, 16)

    return int(value, 10)


def env_bool(name, default=False):
    value = os.environ.get(name)

    if value is None:
        return default

    return value.strip().lower() in ("1", "true", "yes", "y", "on")


def env_float(name, default=None):
    value = os.environ.get(name)

    if value is None or value == "":
        return default

    return float(value)


def dprint(message):
    if DEBUG:
        print(message)


def rpt(data):
    # CP2112 HID reports are 64 bytes. Pad shorter reports.
    if len(data) > 64:
        raise ValueError(f"CP2112 report too long: {len(data)} bytes")

    return bytes(data + [0x00] * (64 - len(data)))


class BufferBridge:
    def __init__(self, addr_7bit=DEFAULT_ADDR_7BIT, vid=DEFAULT_VID, pid=DEFAULT_PID):
        self.addr_7bit = addr_7bit
        self.addr_cp2112 = addr_7bit << 1
        self.vid = vid
        self.pid = pid
        self.dev = None

    def open(self):
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
        if self.dev is not None:
            self.dev.close()
            self.dev = None

    def drain_reports(self, timeout_s=0.05):
        # CP2112 can leave stale 0x13/0x16 reports queued.
        # Drain before starting a new transaction.
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
        # Cancel stale SMBus transfer state.
        dprint("CANCEL")
        self.dev.write(rpt([CANCEL_TRANSFER, 0x01]))
        time.sleep(0.10)
        self.drain_reports()

    def wait_transfer_complete(self, timeout_s=1.0):
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
        # Force CP2112 to emit the 0x13 data response.
        request = [
            DATA_READ_FORCE_SEND,
            (length >> 8) & 0xFF,
            length & 0xFF,
        ]

        dprint(f"FORCE_READ request={request}")
        self.dev.write(rpt(request))

    def read_forced_response(self, length, timeout_s=1.0):
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
        return self.read_reg(reg, 1)[0]

    def read_f32(self, reg):
        # lll-buffed stores float registers as 4-byte little-endian values.
        data = bytes(self.read_reg(reg, 4))
        return struct.unpack("<f", data)[0]

    def write_reg(self, reg, data):
        # CP2112 Data Write Request:
        # payload is [register] + data bytes.
        # For this setup, 2-byte writes are reliable.
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

    def write_f32_staged(self, reg_base, value):
        # Firmware patch supports staged float writes:
        # [reg_base + 0, byte0]
        # [reg_base + 1, byte1]
        # [reg_base + 2, byte2]
        # [reg_base + 3, byte3] -> firmware commits value.
        payload = list(struct.pack("<f", float(value)))

        dprint(f"STAGED_F32 reg=0x{reg_base:02x} value={value} bytes={payload}")

        for offset, byte in enumerate(payload):
            self.write_reg(reg_base + offset, [byte])

        return payload

    def send_command(self, name):
        command = BUFFER_CMDS[name]
        self.write_reg(REG_COMMAND, [command])
        print(f"sent command: {name} ({command})")

    def set_speed(self, speed):
        if speed <= 0:
            raise RuntimeError("speed must be greater than 0")

        payload = self.write_f32_staged(REG_SPEED, speed)
        readback = self.read_f32(REG_SPEED)

        print(f"set speed={speed:.2f} mm/s bytes={payload}")
        print(f"readback speed={readback:.2f} mm/s")

    def move_distance(self, distance):
        payload = self.write_f32_staged(REG_MOVE_DIST, distance)
        print(f"sent move distance={distance:.3f} mm bytes={payload}")

    def read_status(self):
        status = self.read_u8(REG_STATUS)
        mode = self.read_u8(REG_MODE)
        motor = self.read_u8(REG_MOTOR)
        speed = self.read_f32(REG_SPEED)

        return {
            "status": status,
            "filament_present": status & 0x01,
            "timed_out": (status >> 1) & 0x01,
            "mode": mode,
            "mode_name": MODE_NAMES.get(mode, "UNKNOWN"),
            "motor": motor,
            "motor_name": MOTOR_NAMES.get(motor, "UNKNOWN"),
            "speed": speed,
        }

    def print_status(self):
        st = self.read_status()

        print("buffer read OK")
        print(f"addr=0x{self.addr_7bit:02x}")
        print(f"status=0x{st['status']:02x}")
        print(f"filament_present={st['filament_present']}")
        print(f"timed_out={st['timed_out']}")
        print(f"mode={st['mode']} ({st['mode_name']})")
        print(f"motor={st['motor']} ({st['motor_name']})")
        print(f"speed={st['speed']:.2f} mm/s")


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
            "hold, manual, push, feed, pull, retract, move, speed, set_speed"
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
    action = action.lower().strip()

    aliases = {
        "set-speed": "speed",
        "set_speed": "speed",
        "moves": "move",
        "move_dist": "move",
        "move-distance": "move",
    }

    return aliases.get(action, action)


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

        bridge.cancel_transfer()

        if action == "status":
            bridge.print_status()
            return

        if action == "speed":
            if args.speed is None:
                raise RuntimeError("action=speed requires --speed or BUFFER_SPEED")

            bridge.set_speed(args.speed)
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

        valid = sorted(list(BUFFER_CMDS.keys()) + ["status", "move", "speed"])
        raise RuntimeError(f"Unknown action={action}. Valid actions: {', '.join(valid)}")

    finally:
        bridge.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
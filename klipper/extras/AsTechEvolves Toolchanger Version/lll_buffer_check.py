#!/usr/bin/env python3
"""
LLL buffer CP2112 comms validator.

Default behavior:
- Print nothing on success.
- Print a loud error and exit non-zero on failure.

Use --verbose if you want success output while testing.
"""

import argparse
import subprocess
import sys
from pathlib import Path


HELPER = Path("/home/goofballtech/printer_data/config/helpers/lll_cp2112_buffer.py")


def fail(name: str, addr: str, message: str, output: str = "") -> int:
    """Print only failure details so normal checks stay invisible."""
    print(f"!! BUFFER COMMS ERROR: {name} addr={addr}: {message}", file=sys.stderr)

    if output:
        print(f"!! ---- {name} raw helper output ----", file=sys.stderr)
        for line in output.splitlines():
            print(f"!! {line}", file=sys.stderr)
        print("!! ---- end helper output ----", file=sys.stderr)

    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Check LLL buffer CP2112 comms health")
    parser.add_argument("--name", default="Buffer", help="Friendly buffer name for console output")
    parser.add_argument("--addr", required=True, help="7-bit I2C address, for example 0x10")
    parser.add_argument("--timeout", type=float, default=12.0, help="Helper timeout in seconds")
    parser.add_argument("--verbose", action="store_true", help="Print success output for manual testing")
    args = parser.parse_args()

    name = args.name
    addr = args.addr

    if not HELPER.is_file():
        return fail(name, addr, f"CP2112 helper not found: {HELPER}")

    cmd = [
        sys.executable,
        str(HELPER),
        "--addr",
        addr,
        "status",
    ]

    try:
        result = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            timeout=args.timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        output = ((exc.stdout or "") + "\n" + (exc.stderr or "")).strip()
        return fail(name, addr, f"helper timed out after {args.timeout:.1f}s", output)

    output = (result.stdout + "\n" + result.stderr).strip()

    if result.returncode != 0:
        return fail(name, addr, f"helper exited with code {result.returncode}", output)

    if "buffer read OK" not in output:
        return fail(name, addr, "missing 'buffer read OK'", output)

    status_line = ""
    timed_out_line = ""
    mode_line = ""
    motor_line = ""

    for line in output.splitlines():
        clean = line.strip()
        if clean.startswith("status=") and not status_line:
            status_line = clean
        elif clean.startswith("timed_out=") and not timed_out_line:
            timed_out_line = clean
        elif clean.startswith("mode=") and not mode_line:
            mode_line = clean
        elif clean.startswith("motor=") and not motor_line:
            motor_line = clean

    if status_line != "status=0x00":
        return fail(name, addr, f"unhappy status: {status_line or 'missing status line'}", output)

    if timed_out_line != "timed_out=0":
        return fail(name, addr, f"buffer reports timed_out: {timed_out_line or 'missing timed_out line'}", output)

    if args.verbose:
        print(
            f"BUFFER COMMS OK: {name} addr={addr} "
            f"{status_line} {timed_out_line} {mode_line} {motor_line}".strip()
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
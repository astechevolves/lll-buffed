import os
import subprocess
import tempfile
import time

from serial import Serial, SerialException
from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo

Import("env")

DFU_VID = 0x0483
DFU_PID = 0xDF11

CDC_VID = 0x0483
CDC_PID = 0xF072

# After the reboot, Windows lists the DFU device before it can be opened, so
# wait until dfu-util can actually talk to it. Some buffers have taken 30-35 s.
DFU_READY_TIMEOUT_S = 60.0
POLL_INTERVAL_S = 0.5
PROGRESS_INTERVAL_S = 5.0
# If the buffer is still on its serial port this long after a reboot_dfu,
# send it again.
RESEND_INTERVAL_S = 5.0

DFU_MANUAL_HINT = (
    "Put the buffer in DFU mode manually by pressing the R button while holding"
    " the B button on the main board, then run the upload again."
)


def dfu_util() -> str:
    # The same dfu-util PlatformIO uploads with, not whatever is on PATH.
    return env.subst("$UPLOADER").strip('"')


_last_diag = None


def dfu_listed() -> bool:
    try:
        out = subprocess.run(
            [dfu_util(), "-l"], capture_output=True, text=True, timeout=10
        ).stdout
    except subprocess.TimeoutExpired:
        return False
    return f"{DFU_VID:04x}:{DFU_PID:04x}" in out


def report(diag: str) -> None:
    # Print each distinct readiness result once, so a long wait stays readable.
    global _last_diag
    if diag != _last_diag:
        print(f"  [dfu check] {diag}")
        _last_diag = diag


def dfu_ready() -> bool:
    # A 4-byte read only succeeds once the device can be opened and claimed,
    # which is exactly what the upload needs.
    probe = os.path.join(tempfile.mkdtemp(), "probe.bin")
    try:
        result = subprocess.run(
            [
                dfu_util(),
                "-d",
                f"{DFU_VID:04x}:{DFU_PID:04x}",
                "-a",
                "0",
                "-s",
                "0x08000000:4",
                "-U",
                probe,
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0:
            return True
        lines = [
            ln.strip()
            for ln in (result.stdout + result.stderr).splitlines()
            if ln.strip()
            and not ln.startswith(("dfu-util ", "Copyright", "This program", "Please report"))
        ]
        error = lines[-1] if lines else "(no output)"
        listed = "listed" if dfu_listed() else "not listed"
        report(f"probe exit {result.returncode}, device {listed} by -l: {error}")
        return False
    except subprocess.TimeoutExpired:
        report("probe timed out after 10 s")
        return False
    finally:
        if os.path.exists(probe):
            os.remove(probe)
        os.rmdir(os.path.dirname(probe))


def send_reboot_dfu(device: str) -> None:
    # The buffer only reads USB serial data while the port is open with DTR
    # set. Writing and closing straight away left the command unread until the
    # next time something opened the port, so: open, let the buffer see DTR,
    # write, then hold the port open while the buffer reads it and reboots.
    try:
        with Serial(device, 115200, timeout=1, write_timeout=1) as ser:
            ser.dtr = True
            time.sleep(0.5)
            ser.write(b"\nreboot_dfu\n")
            ser.flush()
            time.sleep(1.0)
    except (SerialException, OSError) as e:
        # Usually just the port vanishing because the buffer rebooted.
        print(f"  serial port closed during reboot ({e.__class__.__name__})")


def wait_for_dfu_ready(timeout_s: float) -> bool:
    start = time.time()
    next_progress = start + PROGRESS_INTERVAL_S
    next_resend = start + RESEND_INTERVAL_S
    while time.time() - start < timeout_s:
        if dfu_ready():
            print(f"  DFU ready after {time.time() - start:.0f} s")
            return True
        now = time.time()
        if now >= next_resend:
            cdc_port = find_serial_port(CDC_VID, CDC_PID)
            if cdc_port is not None:
                print(f"  buffer still running on {cdc_port.device}, resending reboot_dfu")
                send_reboot_dfu(cdc_port.device)
            next_resend = time.time() + RESEND_INTERVAL_S
        if now >= next_progress:
            print(f"  still waiting... {now - start:.0f} s")
            next_progress += PROGRESS_INTERVAL_S
        time.sleep(POLL_INTERVAL_S)
    return False


def find_serial_port(vid: int, pid: int) -> ListPortInfo | None:
    for port in list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port
    return None


def before_upload(source, target, env):
    if dfu_ready():
        print("Found buffer in DFU mode, proceeding with upload")
        return

    cdc_port = find_serial_port(CDC_VID, CDC_PID)
    if cdc_port is None:
        # Not running firmware; it may still be coming up in DFU.
        print("Buffer not found as a serial port, waiting for DFU mode...")
        if wait_for_dfu_ready(DFU_READY_TIMEOUT_S):
            print("Buffer ready in DFU mode, proceeding with upload")
            return
        raise RuntimeError("Buffer not detected. " + DFU_MANUAL_HINT)

    print(f"Found buffer on {cdc_port.device}, rebooting into DFU mode...")
    send_reboot_dfu(cdc_port.device)

    print(f"Waiting up to {DFU_READY_TIMEOUT_S:.0f} s for DFU mode...")
    if not wait_for_dfu_ready(DFU_READY_TIMEOUT_S):
        raise RuntimeError("DFU device not ready. " + DFU_MANUAL_HINT)

    print("Buffer ready in DFU mode, proceeding with upload")


env.AddPreAction("upload", before_upload)

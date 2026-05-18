# CP2112 HID Recovery Reference

Recovery notes for getting a Silicon Labs CP2112 USB HID-to-I2C bridge working on a Raspberry Pi 5 running Klipper/Moonraker.

This was written for a Voron/Klipper setup where helper scripts run from the Klipper Python environment:

```bash
~/klippy-env/bin/python
```

Known device:

```text
Silicon Labs CP2112 HID I2C Bridge
USB ID: 10c4:ea90
```

Important distinction:

```text
/dev/i2c-1 is the Raspberry Pi GPIO I2C bus.
The CP2112 is a USB HID bridge and does not require /dev/i2c-1 to work.
```

---

## Confirmed Problem Pattern

The CP2112 may enumerate correctly but fail to open as the normal user.

Example:

```text
CP2112 devices found: 1
Traceback (most recent call last):
  File "<stdin>", line 4, in <module>
  File "hid.pyx", line 143, in hid.device.open
OSError: open failed
```

If the same open test works with `sudo`, the issue is permissions, not the CP2112 hardware.

On Raspberry Pi OS / Bookworm / Pi 5, fixing only `/dev/hidraw*` may not be enough. `hidapi` may also need permission to open the USB device node under `/dev/bus/usb/...`.

---

## 1. Install HID Dependencies

```bash
sudo apt update
sudo apt install -y libhidapi-hidraw0 libhidapi-dev

~/klippy-env/bin/python -m pip install hidapi
```

---

## 2. Create CP2112 Check Helper

Save this as:

```text
/home/<username>/printer_data/config/helpers/cp2112_hid_check.py
```

Create the file:

```bash
cat > /home/<username>/printer_data/config/helpers/cp2112_hid_check.py <<'PY'
#!/usr/bin/env python3
"""
CP2112 HID visibility/open test.

This checks the exact thing Klipper/Moonraker helper scripts need:
- Python hidapi module imports
- CP2112 enumerates
- CP2112 opens as the current user

Run with:
~/klippy-env/bin/python /home/<username>/printer_data/config/helpers/cp2112_hid_check.py
"""

import os
import sys

VID = 0x10C4
PID = 0xEA90

try:
    import hid
except ModuleNotFoundError:
    print("FAIL: Python module 'hid' is missing.")
    print("Fix: ~/klippy-env/bin/python -m pip install hidapi")
    sys.exit(1)

print(f"Running as UID={os.getuid()} GID={os.getgid()}")
print(f"Checking CP2112 VID:PID {VID:04x}:{PID:04x}")

devices = hid.enumerate(VID, PID)

print(f"CP2112 devices found: {len(devices)}")

for index, dev_info in enumerate(devices, start=1):
    print(f"\nDevice {index}:")
    print(f"  path: {dev_info.get('path')}")
    print(f"  manufacturer: {dev_info.get('manufacturer_string')}")
    print(f"  product: {dev_info.get('product_string')}")
    print(f"  serial: {dev_info.get('serial_number')}")
    print(f"  interface_number: {dev_info.get('interface_number')}")

if not devices:
    print("\nFAIL: CP2112 did not enumerate.")
    print("Check USB cable, USB port, and lsusb output.")
    sys.exit(2)

try:
    dev = hid.device()
    dev.open(VID, PID)
    print("\nPASS: CP2112 opened successfully as the current user.")
    dev.close()
except OSError as exc:
    print("\nFAIL: CP2112 enumerated but could not be opened.")
    print(f"Error: {exc}")
    print("\nLikely cause: USB/hidraw permissions.")
    print("If this works with sudo but not as normal user, apply the udev fix.")
    sys.exit(3)
PY

chmod +x /home/<username>/printer_data/config/helpers/cp2112_hid_check.py
```

Run it:

```bash
~/klippy-env/bin/python /home/<username>/printer_data/config/helpers/cp2112_hid_check.py
```

Expected good result:

```text
CP2112 devices found: 1
PASS: CP2112 opened successfully as the current user.
```

---

## 3. Create CP2112 Permission Fix Helper

Save this as:

```text
/home/<username>/printer_data/config/helpers/fix_cp2112_hid_permissions.sh
```

Create the file:

```bash
cat > /home/<username>/printer_data/config/helpers/fix_cp2112_hid_permissions.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-$HOME/klippy-env/bin/python}"
HELPER_DIR="$HOME/printer_data/config/helpers"
CHECK_SCRIPT="$HELPER_DIR/cp2112_hid_check.py"

echo "Installing HID dependencies..."
sudo apt update
sudo apt install -y libhidapi-hidraw0 libhidapi-dev

echo "Installing hidapi into Klipper Python env..."
"$PYTHON_BIN" -m pip install hidapi

echo "Writing CP2112 udev rules..."
sudo tee /etc/udev/rules.d/99-cp2112.rules >/dev/null <<'EOF'
# Silicon Labs CP2112 USB HID-to-I2C bridge.
# Needed because hidapi may open the USB bus node directly.
SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea90", MODE="0666", TAG+="uaccess"

# Also allow access if hidapi uses the hidraw backend.
KERNEL=="hidraw*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea90", MODE="0666", TAG+="uaccess"
EOF

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Applying permission fix to currently connected CP2112 USB node..."
CP2112_NODE="$(lsusb | awk 'tolower($0) ~ /10c4:ea90/ {gsub(":", "", $4); printf "/dev/bus/usb/%03d/%03d\n", $2, $4; exit}')"

if [[ -n "${CP2112_NODE}" && -e "${CP2112_NODE}" ]]; then
    echo "Found CP2112 USB node: ${CP2112_NODE}"
    sudo chmod 666 "${CP2112_NODE}"
else
    echo "WARNING: Could not find CP2112 USB node from lsusb."
fi

echo "Applying permission fix to matching hidraw nodes if present..."
for H in /sys/class/hidraw/hidraw*; do
    [[ -e "$H" ]] || continue

    NODE="/dev/$(basename "$H")"
    UEVENT="$(cat "$H/device/uevent" 2>/dev/null || true)"

    if echo "$UEVENT" | grep -qi "10C4.*EA90"; then
        echo "Found CP2112 hidraw node: ${NODE}"
        sudo chmod 666 "${NODE}"
    fi
done

echo
echo "Running CP2112 check as normal user..."
"$PYTHON_BIN" "$CHECK_SCRIPT"
SH

chmod +x /home/<username>/printer_data/config/helpers/fix_cp2112_hid_permissions.sh
```

Run it:

```bash
/home/<username>/printer_data/config/helpers/fix_cp2112_hid_permissions.sh
```

Expected result:

```text
PASS: CP2112 opened successfully as the current user.
```

---

## 4. Manual Verification Commands

USB sees the CP2112:

```bash
lsusb | grep -Ei '10c4|cp2112|silicon'
```

Expected:

```text
ID 10c4:ea90 Silicon Labs CP2112 HID I2C Bridge
```

Python can enumerate and open it:

```bash
~/klippy-env/bin/python /home/<username>/printer_data/config/helpers/cp2112_hid_check.py
```

If normal user fails but root works:

```bash
sudo ~/klippy-env/bin/python /home/<username>/printer_data/config/helpers/cp2112_hid_check.py
```

Then rerun:

```bash
/home/<username>/printer_data/config/helpers/fix_cp2112_hid_permissions.sh
```

---

## 5. Final Confirmed-Good State

These should both pass:

```bash
lsusb | grep -Ei '10c4|cp2112|silicon'

~/klippy-env/bin/python - <<'PY'
import hid

dev = hid.device()
dev.open(0x10C4, 0xEA90)
print("CP2112 opened successfully as normal user")
dev.close()
PY
```

Expected final output:

```text
CP2112 opened successfully as normal user
```

---

## 6. Then Check the Actual LLL Buffer Helper

Only after the CP2112 opens as the normal user:

```bash
~/klippy-env/bin/python /home/<username>/printer_data/config/helpers/lll_cp2112_buffer.py --help
```

Then run a status/read-only check first before any movement commands.

---


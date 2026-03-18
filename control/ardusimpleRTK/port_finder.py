#!/usr/bin/env python3
"""
Find which serial port your GNSS receiver (e.g., Ardusimple RTK3B with UM982) is on.

- Cross-platform (Linux/Raspberry Pi OS, Windows, macOS)
- Lists all ports with rich metadata
- Heuristically ranks most-likely GNSS ports
- Optionally probes top candidates briefly to detect NMEA sentences

Usage:
    python3 find_gps_port.py
"""

import os
import sys
import re
import time
import platform
from contextlib import closing

try:
    from serial.tools import list_ports
    import serial
except ImportError:
    print("This script requires pyserial. Install with: pip install pyserial")
    sys.exit(1)

IS_LINUX = (platform.system().lower() == "linux")
IS_MAC   = (platform.system().lower() == "darwin")
IS_WIN   = (platform.system().lower() == "windows")

# --- Configuration -----------------------------------------------------------

# Common GNSS/GPS identifiers seen in port descriptors
GNSS_KEYWORDS = [
    "gnss", "gps", "um982", "unicore", "ardu", "ardusimple", "rtk", "u-blox",
    "f9p", "uart", "nmea", "holybro", "simpleRTK3B"
]

# Known vendor IDs (VID) that often appear for GNSS boards/USB-serial bridges.
KNOWN_VIDS = {
    0x1546: "u-blox",         # Many u-blox devices (e.g., F9P; Ardusimple variants)
    0x10C4: "Silicon Labs",   # CP210x USB-UART bridges
    0x1A86: "QinHeng/CH340",  # CH340 USB-serial
    0x0403: "FTDI",           # FT232 USB-serial
    0x067B: "Prolific",       # PL2303 USB-serial
    0x2E3C: "Unicorecomm",    # Some Unicore devices
}

# Baudrates to try when probing for NMEA (expand for Linux/Pi)
PROBE_BAUDS = [921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600]

# How long to listen per baud when probing (seconds)
PROBE_TIME_PER_BAUD = 0.6

# Prefer stable symlinks on Linux (e.g., /dev/serial/by-id/usb-*)
PREFER_BY_ID = True if IS_LINUX else False


# --- Helpers ----------------------------------------------------------------

def _lower_join(parts):
    return " ".join([p for p in parts if p]).lower()

def _linux_preferred_name(dev_name: str) -> int:
    """
    Extra boost for Linux device names that are typically GNSS:
      /dev/serial/by-id/*  (stable, best)
      /dev/ttyACM*, /dev/ttyUSB*  (USB CDC/ACM & USB-serial bridges)
      /dev/ttyAMA0 (PL011) and /dev/ttyS0 (mini-UART) for GPIO UART
    Lower return value means *more* preferred in sorting (used internally).
    """
    if not IS_LINUX:
        return 999
    name = dev_name or ""
    if name.startswith("/dev/serial/by-id/"):
        return 0
    if "/ttyACM" in name:
        return 1
    if "/ttyUSB" in name:
        return 2
    if name.endswith("/ttyAMA0"):
        return 3
    if name.endswith("/ttyS0"):
        return 4
    return 10

def _resolve_by_id(p):
    """
    If /dev/serial/by-id symlink exists that points to p.device, return that path.
    This gives a stable name on Linux.
    """
    if not (IS_LINUX and PREFER_BY_ID):
        return None
    by_id_dir = "/dev/serial/by-id"
    if not os.path.isdir(by_id_dir):
        return None
    try:
        for entry in os.listdir(by_id_dir):
            full = os.path.join(by_id_dir, entry)
            if os.path.islink(full):
                try:
                    # Resolve link target
                    target = os.path.realpath(full)
                    # Same device node?
                    if os.path.realpath(p.device) == target:
                        return full
                except OSError:
                    continue
    except PermissionError:
        # No permission to read directory; ignore
        return None
    return None

def port_score(p):
    """
    Score a port based on how likely it is to be the GNSS receiver.
    Higher score = more likely.
    """
    score = 0

    text = _lower_join([
        p.device,
        p.description or "",
        getattr(p, "manufacturer", None) or "",
        getattr(p, "product", None) or "",
        getattr(p, "interface", None) or "",
        getattr(p, "serial_number", None) or "",
        str(getattr(p, "hwid", "") or "")
    ])

    # Keywords
    for kw in GNSS_KEYWORDS:
        if kw in text:
            score += 3

    # VID match
    if getattr(p, "vid", None) in KNOWN_VIDS:
        score += 2

    # Generic hints
    if "cdc" in text or "acm" in text:
        score += 1
    if any(tok in text for tok in ["ttyacm", "usbmodem", "usbserial", "ttyusb"]):
        score += 1

    # Linux device name preference
    score += max(0, 5 - _linux_preferred_name(p.device))

    return score

def looks_like_nmea(line: str) -> bool:
    """
    Very light check if a line resembles an NMEA sentence.
    """
    line = line.strip()
    if not line.startswith("$") and not line.startswith("!"):
        return False
    # Common talker IDs and sentences (GNRMC/GNGGA/.., sometimes P- sentences)
    return bool(re.match(r'^[!$][A-Z]{2,3}[A-Z]{3},', line))

def probe_for_nmea(port_name: str, bauds=PROBE_BAUDS, t_per_baud=PROBE_TIME_PER_BAUD):
    """
    Try opening a port at several baud rates briefly and read for NMEA.
    Returns (bool found, baud_used, sample_lines, err)
    """
    for b in bauds:
        try:
            with closing(serial.Serial(port=port_name, baudrate=b, timeout=0.12)) as s:
                start = time.time()
                sample_lines = []
                while time.time() - start < t_per_baud:
                    try:
                        raw = s.readline()
                    except serial.SerialException as e:
                        return False, None, [], e
                    if not raw:
                        continue
                    try:
                        line = raw.decode("ascii", errors="ignore").strip()
                    except Exception:
                        continue
                    if line:
                        sample_lines.append(line)
                        if looks_like_nmea(line):
                            return True, b, sample_lines[:10], None
        except (serial.SerialException, OSError) as e:
            # Permission errors on Linux are common if user is not in 'dialout'
            last_err = e
            continue
    return False, None, [], None

def describe_port(p):
    vidpid = f"{p.vid:04X}:{p.pid:04X}" if getattr(p, "vid", None) and getattr(p, "pid", None) else "-"
    by_id = _resolve_by_id(p)
    parts = [
        f"Port:          {p.device}",
        f"  Stable link:  {by_id or '-'}",
        f"  Description:  {p.description or '-'}",
        f"  Manufacturer: {getattr(p, 'manufacturer', None) or '-'}",
        f"  Product:      {getattr(p, 'product', None) or '-'}",
        f"  Interface:    {getattr(p, 'interface', None) or '-'}",
        f"  SerialNumber: {getattr(p, 'serial_number', None) or '-'}",
        f"  HWID:         {getattr(p, 'hwid', None) or '-'}",
        f"  VID:PID:      {vidpid}",
    ]
    return "\n".join(parts)


# --- Main -------------------------------------------------------------------

def main():
    # On Linux: hint for permissions
    if IS_LINUX:
        # If the user is not in 'dialout', opening ports may raise EACCES.
        # We just print a reminder here if probing fails later.
        pass

    # include_links=True → show symlinks (e.g., /dev/serial/by-id/…)
    ports = list(list_ports.comports(include_links=True))
    if not ports:
        print("No serial ports found.")
        if IS_LINUX:
            print("• On Raspberry Pi, typical device names: /dev/ttyACM*, /dev/ttyUSB*, /dev/ttyAMA0, /dev/ttyS0")
            print("• If you’re using the 40‑pin UART, ensure it’s enabled and the serial console is disabled.")
        return

    print("=== Serial Ports Detected ===")
    for p in ports:
        print(describe_port(p))
        print()

    # Score ports by likelihood
    scored = sorted(ports, key=lambda x: ( -port_score(x), _linux_preferred_name(x.device), x.device ))

    print("=== Heuristic Guess (most likely first) ===")
    for p in scored:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if getattr(p, "vid", None) and getattr(p, "pid", None) else "----:----"
        print(f"{p.device:>22}  score={port_score(p):2d}  desc='{p.description}'  VID:PID={vidpid}")
    print()

    # Take top candidates to probe for NMEA (limit to avoid long waits)
    candidates = [p for p in scored if port_score(p) > 0] or scored[:3]
    print("Probing likely ports for NMEA...")
    best_match = None
    first_error = None

    for p in candidates[:5]:
        found, baud, samples, err = probe_for_nmea(p.device)
        if err and first_error is None:
            first_error = err
        if found:
            best_match = (p, baud, samples)
            break

    if best_match:
        p, baud, samples = best_match
        stable = _resolve_by_id(p) or p.device
        print("\n✅ Confirmed GNSS/NMEA detected on:")
        print(describe_port(p))
        print(f"  Suggested baud (probe): {baud}")
        if samples:
            print("\nSample lines:")
            for ln in samples[:5]:
                print("   ", ln)
        print(f"\nUse this in your reader script:\n  PORT = '{stable}'  # baud {baud} (or your device's default)")
    else:
        print("\n⚠️  Could not confirm NMEA by probing.")
        if first_error and IS_LINUX:
            msg = str(first_error).lower()
            if "permission" in msg or "access" in msg or "eacces" in msg:
                print("\nPermission hint (Raspberry Pi OS):")
                print("  sudo usermod -a -G dialout $USER")
                print("  # Log out/in (or reboot) so group change takes effect.")
        print("\nBased on descriptors, most likely port is:")
        p = scored[0]
        print(describe_port(p))
        stable = _resolve_by_id(p) or p.device
        print(f"\nTry it first in your app:\n  PORT = '{stable}'")
        if IS_LINUX:
            print("\nTips (Linux/Raspberry Pi OS):")
            print("  • If using the 40‑pin UART: typical device is /dev/ttyAMA0 (PL011) or /dev/ttyS0 (mini‑UART).")
            print("  • If using USB: look for /dev/ttyACM* (CDC‑ACM) or /dev/ttyUSB* (USB‑serial bridges).")
            print("  • Prefer stable path if present: /dev/serial/by-id/usb-… → it won’t change across reboots.")

if __name__ == "__main__":
    main()
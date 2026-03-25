#!/usr/bin/env python3
"""
UM982 (Unicore) — 20 Hz NMEA + (optional) NTRIP + Terminal print (Raspberry Pi OS)

Features
- (Optional) Factory reset the UM982 (FRESET) for a clean start
- Configure MODE ROVER + MODE HEADING (dual-antenna heading)
- Enable GGA/RMC/HDT at 20 Hz on internal COM1 (USB-exposed port)
- Optional VTG at 10 Hz
- Host-side NTRIP client (e.g., FLEPOS VRS) that sends periodic GGA and streams RTCM to the receiver
- Non-blocking serial reader; prints ~20 Hz one-liners to terminal with UTC, fix, sats, lat/lon/alt, speed, course, heading

Dependencies:
  pip3 install pyserial
"""

import base64
import socket
import ssl
import serial
import time
import threading
import queue
import sys
import math
from contextlib import closing

# =========================
# User configuration
# =========================

# ---- Serial (Raspberry Pi) ----
# Use your confirmed device; prefer /dev/serial/by-id/... if available for stability across reboots.
PORT = '/dev/ttyAMA0'        # <— your Pi result
BAUD = 115200                # Keep in sync with COM1 config below
COM1_BAUD = 115200           # Module COM1 baudrate to set (match BAUD)

# ---- Setup behavior ----
RESET_ON_START = False        # Send FRESET at startup (clean slate)
SETUP_ENABLE_VTG = False     # Also enable VTG at 10 Hz (course/speed)? keep False for lighter bandwidth

# ---- NTRIP client (fill these if you want RTK position) ----
ENABLE_NTRIP = True
NTRIP_HOST = "flepos.vlaanderen.be"  # confirm your FLEPOS host
NTRIP_PORT = 2101
NTRIP_MOUNT = "FLEPOSVRS32GREC"      # exact case from sourcetable
NTRIP_USER = "username"
NTRIP_PASSWORD = "password"
NTRIP_USE_SSL = False
NTRIP_SEND_GGA_EVERY = 10.0          # seconds; VRS expects periodic GGA

# ---- Print options ----
PRINT_HZ = 20               # target print rate (Hz)
SHOW_INPUT_RATE = True      # print measured inbound NMEA line rate
USER_HEADING_OFFSET_DEG = 0.0   # apply if antenna bar is not perfectly aligned with vehicle axis

# =========================
# Internal shared state
# =========================

# For RTCM forwarding
rtcm_q = queue.Queue(16000)

# Latest GGA (raw) for VRS GGA uploads
_latest_gga_raw = None
_latest_gga_lock = threading.Lock()

def set_latest_gga(raw_line: str):
    global _latest_gga_raw
    with _latest_gga_lock:
        _latest_gga_raw = raw_line

def get_latest_gga():
    with _latest_gga_lock:
        return _latest_gga_raw

# State for terminal printing
state_lock = threading.Lock()
state = {
    "lat": None, "lon": None, "alt_m": None,
    "fix": None, "nsats": None, "hdop": None,
    "speed_mps": None, "course_deg": None,
    "heading_true": None,
    "utc_time": None,     # hhmmss.sss (from RMC/GGA)
    "utc_date": None,     # ddmmyy (from RMC)
}

# =========================
# Serial helpers & setup
# =========================

def open_port(port, baud, timeout=0):
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=timeout,
        write_timeout=2,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )

def write_cmd(ser, cmd, quiet=False):
    # Unicore expects CR+LF
    data = (cmd + "\r\n").encode("ascii", errors="ignore")
    ser.write(data)
    ser.flush()
    if not quiet:
        print(f"[setup] >> {cmd}")

def drain_for(ser, seconds=0.5):
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        n = ser.in_waiting if hasattr(ser, "in_waiting") else 0
        if n:
            _ = ser.read(n)

def setup_um982_to_10hz_com1_heading():
    print(f"[setup] Opening {PORT} @ {BAUD} ...")
    try:
        ser = open_port(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print(f"[setup] Could not open {PORT}: {e}")
        print("Hint: On Raspberry Pi, ensure your user is in the 'dialout' group and the device path is correct.")
        sys.exit(1)

    if RESET_ON_START:
        write_cmd(ser, "FRESET")
        drain_for(ser, 0.2)
        ser.close()
        print("[setup] Factory reset sent. Waiting for receiver to reboot ...")
        time.sleep(10.0)
        # Re-open after reboot
        for _ in range(12):
            try:
                ser = open_port(PORT, BAUD, timeout=0.2)
                print(f"[setup] Reconnected to {PORT}.")
                break
            except Exception:
                time.sleep(0.7)
        else:
            print("[setup] Failed to reconnect after reset.")
            sys.exit(1)
        drain_for(ser, 0.5)

    # Enable rover + heading mode, set COM1 baud, enable 10 Hz NMEA
    write_cmd(ser, "MODE ROVER")
    write_cmd(ser, "MODE HEADING")                 # Dual-antenna heading mode
    write_cmd(ser, f"CONFIG COM1 {COM1_BAUD}")     # Align module COM1 with host BAUD
    write_cmd(ser, f"CONFIG COM3 {COM1_BAUD}")     # Align COM3 for shield

    # Essential 10 Hz sentences
    write_cmd(ser, "GPGGA COM1 0.05")
    write_cmd(ser, "GPRMC COM1 0.05")
    write_cmd(ser, "GPHDT COM1 0.05")               # True heading at 10 Hz
    write_cmd(ser, "GPGGA COM3 0.05")
    write_cmd(ser, "GPRMC COM3 0.05")
    write_cmd(ser, "GPHDT COM3 0.05")
    if SETUP_ENABLE_VTG:
        write_cmd(ser, "GNVTG COM1 0.05")
        write_cmd(ser, "GNVTG COM3 0.05")

    write_cmd(ser, "SAVECONFIG")
    drain_for(ser, 0.5)
    ser.close()
    print("[setup] Configuration done and saved (10 Hz on COM1 with Heading mode).")

# =========================
# NMEA parsing utilities
# =========================

def nmea_checksum_ok(line: str) -> bool:
    if "*" not in line:
        return True
    try:
        data, cs = line[1:].split("*", 1)
        calc = 0
        for ch in data:
            calc ^= ord(ch)
        return int(cs[:2], 16) == calc
    except Exception:
        return False

def looks_like_nmea(s: str):
    return (s.startswith("$") or s.startswith("!")) and "," in s

def dm_to_deg(dm: str, hemi: str):
    if not dm or not hemi:
        return None
    try:
        i = dm.index(".")
        deg_len = i - 2
        deg = float(dm[:deg_len])
        minutes = float(dm[deg_len:])
        val = deg + minutes / 60.0
        if hemi in ("S", "W"):
            val = -val
        return val
    except Exception:
        return None

def parse_gga(parts):
    if len(parts) < 10:
        return {}
    return {
        "time_utc": parts[1] or None,
        "lat": dm_to_deg(parts[2], parts[3]),
        "lon": dm_to_deg(parts[4], parts[5]),
        "fix": {"0":"No Fix","1":"GPS Fix","2":"DGPS","4":"RTK Fixed","5":"RTK Float"}.get(parts[6], parts[6] or "Unknown"),
        "nsats": int(parts[7]) if parts[7].isdigit() else None,
        "hdop": float(parts[8]) if parts[8] else None,
        "alt_m": float(parts[9]) if parts[9] else None,
    }

def parse_rmc(parts):
    if len(parts) < 10:
        return {}
    sog = float(parts[7]) * 0.514444 if parts[7] else None  # knots -> m/s
    cog = float(parts[8]) if parts[8] else None
    return {
        "time_utc": parts[1] or None,
        "status": parts[2] or None,
        "lat": dm_to_deg(parts[3], parts[4]),
        "lon": dm_to_deg(parts[5], parts[6]),
        "speed_mps": sog,
        "course_deg": cog,
        "date_ddmmyy": parts[9] or None,
    }

def parse_vtg(parts):
    d = {}
    try: d["course_t"] = float(parts[1]) if parts[1] else None
    except: d["course_t"] = None
    try: d["speed_kn"] = float(parts[5]) if parts[5] else None
    except: d["speed_kn"] = None
    try: d["speed_kmh"] = float(parts[7]) if len(parts) > 7 and parts[7] else None
    except: d["speed_kmh"] = None
    return d

def parse_hdt(parts):
    # $..HDT,heading,T
    if len(parts) < 3: return None
    try: return float(parts[1]) if parts[1] else None
    except: return None

# =========================
# Serial reader thread (reads NMEA; writes RTCM when present)
# =========================

def reader_thread(stop_evt):
    try:
        ser = open_port(PORT, BAUD, timeout=0)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[reader] Open failed: {e}")
        return

    buf = bytearray()

    def write_rtcm_chunks():
        wrote = False
        # Avoid unlimited draining in one go
        for _ in range(500):
            try:
                chunk = rtcm_q.get_nowait()
            except queue.Empty:
                break
            try:
                ser.write(chunk)
                wrote = True
            except Exception:
                pass
        if wrote:
            ser.flush()

    try:
        while not stop_evt.is_set():
            # 1) forward RTCM
            write_rtcm_chunks()

            # 2) read input
            n = ser.in_waiting if hasattr(ser, "in_waiting") else 0
            if n:
                buf.extend(ser.read(n))
                while True:
                    idx = buf.find(b"\n")
                    if idx == -1:
                        break
                    line = buf[:idx+1]
                    del buf[:idx+1]
                    s = line.decode("ascii", errors="ignore").strip()
                    if not s:
                        continue

                    # Keep raw GGA for VRS
                    if s.startswith("$") and "GGA" in s and nmea_checksum_ok(s):
                        set_latest_gga(s)

                    # Parse NMEA
                    if looks_like_nmea(s) and nmea_checksum_ok(s):
                        parts = s.split(",")
                        head = parts[0]
                        if head.endswith("GGA"):
                            d = parse_gga(parts)
                            if d:
                                with state_lock:
                                    state["lat"] = d.get("lat", state["lat"])
                                    state["lon"] = d.get("lon", state["lon"])
                                    state["alt_m"] = d.get("alt_m", state["alt_m"])
                                    state["fix"] = d.get("fix", state["fix"])
                                    state["nsats"] = d.get("nsats", state["nsats"])
                                    state["hdop"] = d.get("hdop", state["hdop"])
                                    state["utc_time"] = d.get("time_utc", state["utc_time"])
                        elif head.endswith("RMC"):
                            d = parse_rmc(parts)
                            if d:
                                with state_lock:
                                    state["lat"] = d.get("lat", state["lat"])
                                    state["lon"] = d.get("lon", state["lon"])
                                    state["speed_mps"] = d.get("speed_mps", state["speed_mps"])
                                    state["course_deg"] = d.get("course_deg", state["course_deg"])
                                    state["utc_time"] = d.get("time_utc", state["utc_time"])
                                    state["utc_date"] = d.get("date_ddmmyy", state["utc_date"])
                        elif head.endswith("VTG"):
                            d = parse_vtg(parts)
                            spd_ms = None
                            if d.get("speed_kmh") is not None:
                                spd_ms = d["speed_kmh"] / 3.6
                            elif d.get("speed_kn") is not None:
                                spd_ms = d["speed_kn"] * 0.514444
                            with state_lock:
                                if spd_ms is not None:
                                    state["speed_mps"] = spd_ms
                                if d.get("course_t") is not None:
                                    state["course_deg"] = d["course_t"]
                        elif head.endswith("HDT"):
                            hdg = parse_hdt(parts)
                            if hdg is not None:
                                hdg = (hdg + USER_HEADING_OFFSET_DEG) % 360.0
                                with state_lock:
                                    state["heading_true"] = hdg
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[reader] Closed.")

# =========================
# NTRIP client thread (optional)
# =========================

def ntrip_client_thread(stop_evt):
    if not ENABLE_NTRIP:
        return

    auth = base64.b64encode(f"{NTRIP_USER}:{NTRIP_PASSWORD}".encode("ascii")).decode("ascii") if NTRIP_USER else None
    path = f"/{NTRIP_MOUNT.lstrip('/')}"  # exact, case-sensitive
    headers = [
        f"GET {path} HTTP/1.0",
        f"Host: {NTRIP_HOST}",
        "User-Agent: NTRIP pyclient/1.0",
        "Accept: */*",
        "Connection: close",
        "Ntrip-Version: Ntrip/2.0",
    ]
    if auth:
        headers.append(f"Authorization: Basic {auth}")
    request = ("\r\n".join(headers) + "\r\n\r\n").encode("ascii")

    while not stop_evt.is_set():
        sock = None
        try:
            raw = socket.create_connection((NTRIP_HOST, NTRIP_PORT), timeout=10)
            if NTRIP_USE_SSL:
                ctx = ssl.create_default_context()
                sock = ctx.wrap_socket(raw, server_hostname=NTRIP_HOST)
            else:
                sock = raw

            sock.sendall(request)

            # Read headers
            buff = b""
            while b"\r\n\r\n" not in buff:
                chunk = sock.recv(4096)
                if not chunk:
                    raise RuntimeError("NTRIP: connection closed during headers")
                buff += chunk
            head, rest = buff.split(b"\r\n\r\n", 1)
            head_text = head.decode("iso-8859-1", errors="ignore")
            first_line = head_text.splitlines()[0] if head_text else ""
            if "401" in first_line:
                print("[ntrip] 401 Unauthorized: check username/password.")
                sock.close(); sock = None
                time.sleep(3); continue
            if "404" in first_line:
                print("[ntrip] 404 Not Found: mountpoint incorrect or case mismatch.")
                sock.close(); sock = None
                time.sleep(5); continue
            if not ("200" in first_line or "ICY 200" in first_line):
                print("[ntrip] Bad response:\n", head_text)
                sock.close(); sock = None
                time.sleep(3); continue

            print(f"[ntrip] Connected to mount '{NTRIP_MOUNT}'; streaming RTCM...")
            if rest:
                rtcm_q.put(rest)

            sock.settimeout(2.0)
            prev_gga_send = time.monotonic()

            while not stop_evt.is_set():
                # periodic GGA for VRS
                if (time.monotonic() - prev_gga_send) >= NTRIP_SEND_GGA_EVERY:
                    gga = get_latest_gga()
                    if gga and gga.startswith("$") and "GGA" in gga:
                        try:
                            sock.sendall((gga + "\r\n").encode("ascii", errors="ignore"))
                        except Exception:
                            pass
                    prev_gga_send = time.monotonic()

                # stream RTCM chunks
                try:
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise RuntimeError("NTRIP: stream ended")
                    rtcm_q.put(chunk)
                except socket.timeout:
                    pass

        except Exception as e:
            print(f"[ntrip] Reconnecting in 3 s: {e}")
            try:
                if sock: sock.close()
            except Exception:
                pass
            time.sleep(3)

# =========================
# Terminal printer (10 Hz)
# =========================

def format_time_iso(utc_date_ddmmyy, utc_time_hhmmss):
    if not utc_time_hhmmss or len(utc_time_hhmmss) < 6:
        return "-"
    try:
        hh = int(utc_time_hhmmss[0:2]); mm = int(utc_time_hhmmss[2:4]); ssf = float(utc_time_hhmmss[4:])
        ss = int(ssf); ms = int(round((ssf - ss) * 1000.0))
        t = f"{hh:02d}:{mm:02d}:{ss:02d}.{ms:03d}Z"
    except Exception:
        t = f"{utc_time_hhmmss[0:2]}:{utc_time_hhmmss[2:4]}:{utc_time_hhmmss[4:6]}Z"
    if not utc_date_ddmmyy or len(utc_date_ddmmyy) != 6:
        return t
    try:
        dd = int(utc_date_ddmmyy[0:2]); mo = int(utc_date_ddmmyy[2:4]); yy = int(utc_date_ddmmyy[4:6])
        return f"20{yy:02d}-{mo:02d}-{dd:02d}T{t}"
    except Exception:
        return t

def run_terminal_printer(stop_evt):
    # deterministic print scheduler
    FRAME_DT = 1.0 / max(1, PRINT_HZ)
    next_t = time.perf_counter() + FRAME_DT

    # input line rate meter
    last_rate_t = time.perf_counter()
    lines_seen = 0

    # hook to count incoming lines: we’ll increment it in reader via queue? Simpler:
    # We estimate by comparing successive headings/time updates: instead keep moving avg using elapsed seconds.
    # For practicality, we keep a simple self-timer that uses a shared variable not present.
    # Here we’ll show “— lps” if not measured elsewhere.

    measured_lps = None  # not strictly needed; SHOW_INPUT_RATE will remain informational

    while not stop_evt.is_set():
        now = time.perf_counter()
        if now >= next_t:
            with state_lock:
                lat = state["lat"]; lon = state["lon"]; alt = state["alt_m"]
                fix = state["fix"]; ns = state["nsats"]; hdop = state["hdop"]
                spd = state["speed_mps"]; crs = state["course_deg"]
                hdg = state["heading_true"]
                ts = format_time_iso(state["utc_date"], state["utc_time"])

            def fmt(v, nd):
                return f"{v:.{nd}f}" if isinstance(v,(int,float)) and not math.isnan(v) else "-"

            # Compose one-liner
            line = (
                f"UTC={ts} | Fix={fix or '-'} | Sats={ns or '-'} | HDOP={fmt(hdop,1)} | "
                f"Lat={fmt(lat,7)} | Lon={fmt(lon,7)} | Alt={fmt(alt,2)} m | "
                f"Spd={fmt(spd,2)} m/s | Crs={fmt(crs,1)}° | HdgTrue={fmt(hdg,1)}°"
            )
            if SHOW_INPUT_RATE and measured_lps is not None:
                line += f" | InRate={measured_lps:.0f} lps"

            print(line)
            next_t += FRAME_DT
            if now > next_t + FRAME_DT:
                next_t = now + FRAME_DT

        # light sleep
        time.sleep(0.01)

# =========================
# Main
# =========================

def main():
    # 1) Configure UM982
    setup_um982_to_10hz_com1_heading()

    # 2) Start threads
    stop_evt = threading.Event()

    t_reader = threading.Thread(target=reader_thread, args=(stop_evt,), daemon=True)
    t_reader.start()

    t_ntrip = None
    if ENABLE_NTRIP:
        t_ntrip = threading.Thread(target=ntrip_client_thread, args=(stop_evt,), daemon=True)
        t_ntrip.start()

    # 3) Start terminal printer (main thread stays alive)
    try:
        run_terminal_printer(stop_evt)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        if t_reader: t_reader.join(timeout=1.0)
        if t_ntrip:  t_ntrip.join(timeout=1.0)
        print("[main] Stopped.")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
UM982 (Unicore) — 10/20 Hz NMEA + (optional) NTRIP + Dash viewer (Raspberry Pi OS)

Live Dash window:
  - Plots current and recent positions in UTM meters.
  - Press 'c' in the terminal to "calibrate": set current position as UTM (0,0),
    clear previous points, and re-center.
  - Color scale indicates fix type (No Fix, GPS Fix, DGPS, RTK Float, RTK Fixed).

Dependencies (use venv on Pi):
  pip install pyserial dash plotly utm
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

# --------- NEW: UTM + Dash/Plotly imports ----------
import utm
from dash import Dash, dcc, html, Output, Input
import plotly.graph_objects as go
# ---------------------------------------------------

# =========================
# User configuration
# =========================

# ---- Serial (Raspberry Pi) ----
PORT = '/dev/ttyAMA0'     # ← your Pi UART device
BAUD = 115200             # Keep in sync with COM1 config below
COM1_BAUD = 115200        # Module COM1 baudrate to set (match BAUD)

# ---- Setup behavior ----
RESET_ON_START = False    # Send FRESET at startup (clean slate)
SETUP_ENABLE_VTG = False  # Also enable VTG at 10 Hz? keep False for lighter bandwidth

# ---- NTRIP client (fill to get RTK) ----
ENABLE_NTRIP = True
NTRIP_HOST = "flepos.vlaanderen.be"
NTRIP_PORT = 2101
NTRIP_MOUNT = "FLEPOSVRS32GREC"      # case-sensitive
NTRIP_USER = "username"
NTRIP_PASSWORD = "password"
NTRIP_USE_SSL = False
NTRIP_SEND_GGA_EVERY = 10.0          # seconds

# ---- Viewer update rate ----
PRINT_HZ = 10                        # still used for timing concepts (not printing)
DASH_UPDATE_HZ = 5                  # UI refresh rate (Hz)

# Apply if antenna bar is not perfectly aligned with vehicle axis
USER_HEADING_OFFSET_DEG = 0.0

# =========================
# Internal shared state
# =========================

rtcm_q = queue.Queue(16000)  # For RTCM forwarding

# Latest GGA (raw) for VRS uploads
_latest_gga_raw = None
_latest_gga_lock = threading.Lock()
def set_latest_gga(raw_line: str):
    global _latest_gga_raw
    with _latest_gga_lock:
        _latest_gga_raw = raw_line
def get_latest_gga():
    with _latest_gga_lock:
        return _latest_gga_raw

# Current parsed state
state_lock = threading.Lock()
state = {
    "lat": None, "lon": None, "alt_m": None,
    "fix": None, "nsats": None, "hdop": None,
    "speed_mps": None, "course_deg": None,
    "heading_true": None,
    "utc_time": None,     # hhmmss.sss
    "utc_date": None,     # ddmmyy
}

# ------------- NEW: Origin & Trail buffers (thread-safe) -------------
origin_lock = threading.Lock()
origin = {
    "lat0": None, "lon0": None,
    "zone_number": None, "zone_letter": None,
    "e0": None, "n0": None,   # UTM easting/northing at calibration
    "set": False
}

trail_lock = threading.Lock()
# Each element: (x_rel_m, y_rel_m, fix_code, fix_label, ts_float)
trail = []  # cleared on each calibration
# --------------------------------------------------------------------

# Fix type mapping to numeric color scale
FIX_CODE = {
    "No Fix": 0,
    "GPS Fix": 1,
    "DGPS": 2,
    "RTK Float": 3,
    "RTK Fixed": 4,
    "Unknown": -1
}
# For colorbar labeling
FIX_TICKS = [0, 1, 2, 3, 4]
FIX_TICKTXT = ["No Fix", "GPS", "DGPS", "Float", "Fixed"]

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
    data = (cmd + "\r\n").encode("ascii", errors="ignore")  # Unicore expects CRLF
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
        print("Hint: On Raspberry Pi, ensure your user is in 'dialout' group and the device path is correct.")
        sys.exit(1)

    if RESET_ON_START:
        write_cmd(ser, "FRESET")
        drain_for(ser, 0.2)
        ser.close()
        print("[setup] Factory reset sent. Waiting for receiver to reboot ...")
        time.sleep(10.0)
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

    # Rover + Heading mode, set COM1/COM3 baud, enable 10 Hz NMEA
    write_cmd(ser, "MODE ROVER")
    write_cmd(ser, "MODE HEADING")
    write_cmd(ser, f"CONFIG COM1 {COM1_BAUD}")
    write_cmd(ser, f"CONFIG COM3 {COM1_BAUD}")

    # 0.05 s period = 20 Hz (UM982 may handle 20 Hz).
    # If you truly want 10 Hz, change to 0.1
    period = 0.05  # 20 Hz
    write_cmd(ser, f"GPGGA COM1 {period}")
    write_cmd(ser, f"GPRMC COM1 {period}")
    write_cmd(ser, f"GPHDT COM1 {period}")
    write_cmd(ser, f"GPGGA COM3 {period}")
    write_cmd(ser, f"GPRMC COM3 {period}")
    write_cmd(ser, f"GPHDT COM3 {period}")
    if SETUP_ENABLE_VTG:
        write_cmd(ser, "GNVTG COM1 0.1")
        write_cmd(ser, "GNVTG COM3 0.1")

    write_cmd(ser, "SAVECONFIG")
    drain_for(ser, 0.5)
    ser.close()
    print("[setup] Configuration saved (GGA/RMC/HDT @ ~20 Hz).")

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
            write_rtcm_chunks()

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
    path = f"/{NTRIP_MOUNT.lstrip('/')}"
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
            sock = ssl.create_default_context().wrap_socket(raw, server_hostname=NTRIP_HOST) if NTRIP_USE_SSL else raw
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
                print("[ntrip] 401 Unauthorized")
                sock.close(); sock = None; time.sleep(3); continue
            if "404" in first_line:
                print("[ntrip] 404 Not Found: check mountpoint")
                sock.close(); sock = None; time.sleep(5); continue
            if not ("200" in first_line or "ICY 200" in first_line):
                print("[ntrip] Bad response:\n", head_text)
                sock.close(); sock = None; time.sleep(3); continue

            print(f"[ntrip] Connected to mount '{NTRIP_MOUNT}'; streaming RTCM...")
            if rest: rtcm_q.put(rest)

            sock.settimeout(2.0)
            prev_gga_send = time.monotonic()

            while not stop_evt.is_set():
                # periodic GGA for VRS
                if (time.monotonic() - prev_gga_send) >= NTRIP_SEND_GGA_EVERY:
                    gga = get_latest_gga()
                    if gga and gga.startswith("$") and "GGA" in gga:
                        try: sock.sendall((gga + "\r\n").encode("ascii", errors="ignore"))
                        except Exception: pass
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
# Keyboard: 'c' to calibrate; 'e' to exit
# =========================

import termios, tty, select
def keyboard_thread(stop_evt):
    """
    Non-blocking TTY listener:
      'c' -> set origin to current lat/lon, clear trail
      'e' -> exit program
    """
    fd = sys.stdin.fileno()
    try:
        old_attrs = termios.tcgetattr(fd)
    except Exception:
        print("[key] Not a TTY; keyboard controls disabled.")
        return

    tty.setcbreak(fd)
    print("[key] Press 'c' to calibrate (set 0,0 & clear), 'e' to exit")

    try:
        while not stop_evt.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                c = ch.lower()
                if c == 'c':
                    with state_lock:
                        lat = state["lat"]; lon = state["lon"]
                    if lat is None or lon is None:
                        print("[key] Calibration ignored (no valid fix yet).")
                        continue

                    try:
                        e0, n0, zn, zl = utm.from_latlon(lat, lon)
                        with origin_lock:
                            origin.update(dict(lat0=lat, lon0=lon,
                                               e0=e0, n0=n0,
                                               zone_number=zn, zone_letter=zl,
                                               set=True))
                        with trail_lock:
                            trail.clear()
                        print(f"[key] Calibrated at lat={lat:.7f}, lon={lon:.7f} -> UTM zone {zn}{zl} (0,0 set). Cleared trail.")
                    except Exception as ex:
                        print(f"[key] UTM calibration error: {ex}")

                elif c == 'e':
                    print("[key] Exit requested.")
                    stop_evt.set()
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)

# =========================
# Dash app (live 2D UTM viewer)
# =========================

def build_empty_fig(msg="Press 'c' to calibrate (set current as 0,0)"):
    fig = go.Figure()
    fig.update_layout(
        title=msg,
        xaxis_title="X (m, UTM relative)",
        yaxis_title="Y (m, UTM relative)",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        yaxis=dict(),
        margin=dict(l=40, r=20, t=50, b=40),
        template="plotly_white",
        showlegend=False,
    )
    return fig

app = Dash(__name__)
app.title = "GNSS UTM Live Viewer"

app.layout = html.Div(
    style={"fontFamily":"Segoe UI, Roboto, Arial", "margin":"10px"},
    children=[
        html.H3("UM982 — UTM Live Viewer (press 'c' in terminal to calibrate)"),
        html.Div(id="readout", style={"marginBottom":"6px", "color":"#444"}),
        dcc.Graph(id="utm_plot", figure=build_empty_fig(), style={"height":"75vh"}),
        dcc.Interval(id="tick", interval=int(1000.0 / DASH_UPDATE_HZ), n_intervals=0),
    ]
)

@app.callback(
    Output("utm_plot", "figure"),
    Output("readout", "children"),
    Input("tick", "n_intervals"),
)
def refresh(_):
    # 1) Snapshot current lat/lon + fix
    with state_lock:
        lat = state["lat"]; lon = state["lon"]; fix = state["fix"] or "Unknown"

    # 2) If calibrated, convert to UTM and append to trail
    with origin_lock:
        ori = origin.copy()

    if lat is not None and lon is not None and ori["set"]:
        try:
            e, n, zn, zl = utm.from_latlon(lat, lon)
            # Enforce same zone as origin (ignore if not matching)
            if zn == ori["zone_number"] and zl == ori["zone_letter"]:
                x_rel = e - ori["e0"]
                y_rel = n - ori["n0"]
                code = FIX_CODE.get(fix, -1)
                with trail_lock:
                    # Keep everything since last calibration
                    trail.append((x_rel, y_rel, code, fix, time.time()))
        except Exception:
            pass

    # 3) Build figure
    with trail_lock:
        pts = list(trail)

    if not ori["set"]:
        fig = build_empty_fig()
        ro = "Waiting for calibration… press 'c' when a valid fix is available."
        return fig, ro

    # Separate arrays (if empty, show empty fig with calibration title)
    if not pts:
        lat0 = ori["lat0"]; lon0 = ori["lon0"]
        title = f"Calibrated at lat={lat0:.7f}, lon={lon0:.7f} | UTM zone {ori['zone_number']}{ori['zone_letter']}"
        fig = build_empty_fig(title)
        ro = "No points yet… awaiting GNSS updates."
        return fig, ro

    import numpy as np
    arr = np.array(pts, dtype=object)
    xs = arr[:,0].astype(float)
    ys = arr[:,1].astype(float)
    codes = arr[:,2].astype(float)
    labels = arr[:,3].astype(str)

    # Main trail scatter (small points)
    trail_scatter = go.Scattergl(
        x=xs, y=ys, mode="markers",
        marker=dict(
            size=7,
            color=codes,
            colorscale="Turbo",
            cmin=0, cmax=4,
            colorbar=dict(
                title="Fix",
                tickvals=FIX_TICKS,
                ticktext=FIX_TICKTXT,
            ),
            line=dict(width=0)
        ),
        text=labels,
        hovertemplate="(%{x:.3f}, %{y:.3f}) m<br>%{text}<extra></extra>",
        name="trail"
    )

    # Current point (bigger)
    curr_scatter = go.Scattergl(
        x=[xs[-1]], y=[ys[-1]], mode="markers",
        marker=dict(size=14, color="black", symbol="x"),
        hovertemplate="CURRENT<br>(%{x:.3f}, %{y:.3f}) m<extra></extra>",
        name="current"
    )

    lat0 = ori["lat0"]; lon0 = ori["lon0"]
    title = f"Calibrated at lat={lat0:.7f}, lon={lon0:.7f} → UTM zone {ori['zone_number']}{ori['zone_letter']} (0,0)"
    fig = go.Figure(data=[trail_scatter, curr_scatter])
    fig.update_layout(
        title=title,
        xaxis_title="X (m, UTM relative)",
        yaxis_title="Y (m, UTM relative)",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        yaxis=dict(),
        margin=dict(l=40, r=20, t=50, b=40),
        template="plotly_white",
        showlegend=False
    )

    # Auto-range with a small padding
    pad = 0.2  # 20 cm extra margin (nice for cm‑level testing)
    xmin = float(xs.min()) - pad
    xmax = float(xs.max()) + pad
    ymin = float(ys.min()) - pad
    ymax = float(ys.max()) + pad
    # If both min and max are near zero (single point), give small window
    if abs(xmax - xmin) < 0.5:  # 50 cm
        xmin, xmax = xs[-1] - 0.5, xs[-1] + 0.5
    if abs(ymax - ymin) < 0.5:
        ymin, ymax = ys[-1] - 0.5, ys[-1] + 0.5
    fig.update_xaxes(range=[xmin, xmax])
    fig.update_yaxes(range=[ymin, ymax])

    ro = f"Current: X={xs[-1]:.3f} m, Y={ys[-1]:.3f} m | Fix={labels[-1]}"
    return fig, ro

# =========================
# Main
# =========================

def main():
    # 1) Configure UM982 (20 Hz recommended)
    setup_um982_to_10hz_com1_heading()

    # 2) Start threads
    stop_evt = threading.Event()

    t_reader = threading.Thread(target=reader_thread, args=(stop_evt,), daemon=True)
    t_reader.start()

    t_ntrip = None
    if ENABLE_NTRIP:
        t_ntrip = threading.Thread(target=ntrip_client_thread, args=(stop_evt,), daemon=True)
        t_ntrip.start()

    t_key = threading.Thread(target=keyboard_thread, args=(stop_evt,), daemon=True)
    t_key.start()

    # 3) Run Dash app (blocks until Ctrl+C or 'e')
    try:
        app.run(host="0.0.0.0", port=8050, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        if t_reader: t_reader.join(timeout=1.0)
        if t_ntrip:  t_ntrip.join(timeout=1.0)
        if t_key:    t_key.join(timeout=1.0)
        print("[main] Stopped.")

if __name__ == "__main__":
    main()
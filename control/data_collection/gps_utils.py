# NMEA parsing utilities
import numpy as np

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
        "fix": {"0":0,"1":1,"2":2,"4":4,"5":3}.get(parts[6], parts[6] or -1),
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

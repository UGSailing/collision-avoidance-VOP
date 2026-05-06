"""
    GPS / NMEA utilities.
"""

import asyncio
import config

async def configure_um982(writer):
    """Configure the UM982 serial connection for the GPS."""
    print("Configuring UM982...")
    period = 1 / config.GPS_UPDATE_RATE_HZ if config.GPS_UPDATE_RATE_HZ > 0 else 0.05
    commands = [
        "MODE ROVER",
        "MODE HEADING",
        f"CONFIG COM1 {config.GPS_BAUD}",
        f"CONFIG COM3 {config.GPS_BAUD}",
        f"GPGGA COM1 {period}",
        f"GPRMC COM1 {period}",
        f"GPHDT COM1 {period}",
        f"GPGGA COM3 {period}",
        f"GPRMC COM3 {period}",
        f"GPHDT COM3 {period}",
        "SAVECONFIG"
    ]
    
    for cmd in commands:
        msg = (cmd + "\r\n").encode("ascii")
        writer.write(msg)
        await writer.drain()
        await asyncio.sleep(0.1) # small delay between commands

    print("UM982 Configuration sent.")

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

def process_nmea_line(line: str, latest_gps: dict) -> bool:
    """
    Parses a NMEA line and updates the state dictionary.
    Returns True if the GPS state was updated, False otherwise.
    """
    parts = line.split(",")
    head = parts[0]
    
    gps_updated = False

    if head.endswith("GGA"):
        d = parse_gga(parts)
        if d:
            if d.get("lat") is not None: latest_gps['latitude'] = d["lat"]
            if d.get("lon") is not None: latest_gps['longitude'] = d["lon"]
            gps_updated = True

    elif head.endswith("RMC"):
        d = parse_rmc(parts)
        if d:
            if d.get("lat") is not None: latest_gps['latitude'] = d["lat"]
            if d.get("lon") is not None: latest_gps['longitude'] = d["lon"]
            gps_updated = True

    elif head.endswith("HDT"):
        hdg = parse_hdt(parts)
        if hdg is not None:
            hdg = (hdg + config.USER_HEADING_OFFSET_DEG) % 360.0
            latest_gps['heading'] = hdg
            gps_updated = True

    return gps_updated

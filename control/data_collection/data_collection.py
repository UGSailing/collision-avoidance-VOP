"""
    DataCollector manages the input connections to the GPS and the obstacle Pi.
    It is started with it's run method.
"""

import asyncio
import serial_asyncio
import numpy as np
import pandas as pd
import config
from pathlib import Path
from . import gps_utils
from . import ntrip_client

class DataCollector:
    def __init__(self, run_dir: Path):
        self.run_dir = run_dir
        self.gps_id = 0
        self.camera_id = 0
        self.csv_lock = asyncio.Lock()
        
        # State shared between tasks
        self.latest_gps: dict[str, float | None] = {
            'latitude': None,
            'longitude': None,
            'heading': None,
        }
        self.latest_gga_raw = None # For NTRIP uplinks
        self.rtcm_queue = asyncio.Queue(maxsize=100) # RTCM chunks for the GPS

    def _parse_obstacle_line(self, line: str):
        """Parses obstacle payload into (angle_deg, distance_m) tuples."""
        pair_sep = str(config.OBSTACLE_PAIR_SEPARATOR)
        val_sep = str(config.OBSTACLE_VALUE_SEPARATOR)
        objects = []

        for raw_pair in line.split(pair_sep):
            pair = raw_pair.strip()
            if not pair:
                continue

            parts = [p.strip() for p in pair.split(val_sep)]
            if len(parts) != 2:
                continue

            try:
                angle = float(parts[0])
                distance = float(parts[1])
            except ValueError:
                continue

            if not np.isfinite(angle) or not np.isfinite(distance):
                continue

            objects.append((angle, distance))

        return objects

    async def _obstacle_listener(self):
        """Continuously listens for obstacle data from the TCP stream, parses it, and appends new obstacle points to points.csv."""
        while True:
            writer = None
            try:
                reader, writer = await asyncio.open_connection(
                    host=config.OBSTACLE_TCP_HOST,
                    port=config.OBSTACLE_TCP_PORT,
                )
                print(f"Obstacle TCP connection established to {config.OBSTACLE_TCP_HOST}:{config.OBSTACLE_TCP_PORT}.")
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"Warning: obstacle TCP unavailable ({e}). Retrying in {config.OBSTACLE_RECONNECT_DELAY_S} seconds...")
                await asyncio.sleep(config.OBSTACLE_RECONNECT_DELAY_S)
                continue

            try:
                while True:
                    try:
                        raw_line = await asyncio.wait_for(
                            reader.readline(),
                            timeout=config.OBSTACLE_TCP_READ_TIMEOUT_S,
                        )
                    except asyncio.TimeoutError:
                        # No packet in this interval; keep connection open.
                        continue

                    if not raw_line:
                        raise ConnectionError("obstacle TCP stream closed")

                    line = raw_line.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    objects = self._parse_obstacle_line(line)
                    if not objects:
                        if config.OBSTACLE_INPUT_DEBUG:
                            print(f"Obstacle input parse skipped: {line}")
                        continue

                    if None in self.latest_gps.values():
                        continue  # no GPS position yet, skip

                    lat0 = self.latest_gps['latitude']
                    lon0 = self.latest_gps['longitude']
                    heading = self.latest_gps['heading']
                    meters_per_degree_lon = config.METERS_PER_DEGREE_LAT * np.cos(np.radians(lat0)) # type: ignore
                    if abs(meters_per_degree_lon) < 1e-6:
                        continue

                    rows = []
                    for angle, distance in objects:
                        object_direction = heading + angle # type: ignore
                        d_north_m = distance * np.cos(np.radians(object_direction))
                        d_east_m = distance * np.sin(np.radians(object_direction))

                        # Convert local meter offsets to latitude/longitude deltas.
                        lat = lat0 + (d_north_m / config.METERS_PER_DEGREE_LAT) # type: ignore
                        lon = lon0 + (d_east_m / meters_per_degree_lon) # type: ignore
                        rows.append({
                            'id': self.camera_id,
                            'category': 'camera',
                            'latitude': lat,
                            'longitude': lon
                        })
                        self.camera_id += 1

                    if not rows:
                        continue

                    new_row = pd.DataFrame(rows)
                    async with self.csv_lock:
                        await asyncio.to_thread(
                            new_row.to_csv,
                            self.run_dir / 'points.csv',
                            mode='a', header=False, index=False
                        )
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"Obstacle TCP disconnected: {e}")
            finally:
                if writer is not None:
                    writer.close()
                    try:
                        await writer.wait_closed()
                    except Exception:
                        pass

            await asyncio.sleep(config.OBSTACLE_RECONNECT_DELAY_S)

    async def _gps_listener(self):
        """Continuously listens for GPS data from the serial port, parses it, updates latest_gps state, and appends new GPS points to points.csv."""
        while True:
            try:
                print(f"[setup] Opening {config.GPS_PORT} @ {config.GPS_BAUD} ...")
                reader, writer = await serial_asyncio.open_serial_connection(
                    url=config.GPS_PORT, baudrate=config.GPS_BAUD
                )
                print("GPS Serial connection established.")
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"[setup] Could not open {config.GPS_PORT}: {e}")
                print("Hint: On Raspberry Pi, ensure your user is in the 'dialout' group and the device path is correct.")
                await asyncio.sleep(config.OBSTACLE_RECONNECT_DELAY_S)
                continue

            write_task = None
            try:
                # Perform device setup.
                await gps_utils.configure_um982(writer)

                async def rtcm_writer_task():
                    """Writes RTCM data from queue to Serial"""
                    while True:
                        data = await self.rtcm_queue.get()
                        try:
                            writer.write(data)
                            await writer.drain()
                        except Exception as e:
                            print(f"Error writing RTCM to serial: {e}")

                # Start the RTCM writer background task.
                write_task = asyncio.create_task(rtcm_writer_task())

                # StreamReader.readline() is convenient but might buffer.
                while True:
                    try:
                        raw_line = await reader.readline()
                    except Exception:
                        raise ConnectionError("GPS serial stream closed")

                    line = raw_line.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    if not gps_utils.looks_like_nmea(line) or not gps_utils.nmea_checksum_ok(line):
                        continue

                    parts = line.split(",")
                    head = parts[0]

                    # Keep raw GGA for NTRIP VRS
                    if head.endswith("GGA"):
                        self.latest_gga_raw = line

                    # Parse and update state
                    gps_updated = gps_utils.process_nmea_line(line, self.latest_gps)

                    # Log to CSV if we have a valid update
                    if gps_updated and self.latest_gps['latitude'] is not None and self.latest_gps['heading'] is not None:
                        new_row = pd.DataFrame([{
                            'id': self.gps_id,
                            'category': 'gps',
                            'latitude': self.latest_gps['latitude'],
                            'longitude': self.latest_gps['longitude'],
                            'heading': self.latest_gps['heading'],
                        }])
                        async with self.csv_lock:
                            await asyncio.to_thread(
                                new_row.to_csv,
                                self.run_dir / 'points.csv',
                                mode='a', header=False, index=False
                            )
                        self.gps_id += 1

            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"GPS Loop Error: {e}")
                await asyncio.sleep(config.OBSTACLE_RECONNECT_DELAY_S)
            finally:
                if write_task is not None and not write_task.done():
                    write_task.cancel()
                    try:
                        await write_task
                    except asyncio.CancelledError:
                        pass

                writer.close()
                try:
                    await writer.wait_closed()
                except Exception:
                    pass

    async def run(self, stop_event: asyncio.Event):
        """Starts the data collection tasks in the class. This will run indefinitely until stop_event is set."""

        # We pass a lambda/function to get the latest GGA
        def get_gga():
            return self.latest_gga_raw

        tasks = [
            asyncio.create_task(self._obstacle_listener()),
            asyncio.create_task(self._gps_listener()),
            asyncio.create_task(ntrip_client.run_ntrip_client(self.rtcm_queue, get_gga)),
        ]
        
        try:
            await stop_event.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

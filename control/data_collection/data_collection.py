import asyncio
import can
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
        self._csv_lock = asyncio.Lock()
        
        # State shared between tasks
        self.latest_gps: dict[str, float | None] = {
            'latitude': None,
            'longitude': None,
            'heading': None,
        }
        self.latest_gga_raw = None # For NTRIP uplinks
        self.rtcm_queue = asyncio.Queue(maxsize=100) # RTCM chunks for the GPS

    async def _can_listener(self):
        bus = can.interface.Bus(
            channel=config.CAN_CHANNEL,
            bustype=config.CAN_BUSTYPE,
            bitrate=config.CAN_BITRATE,
        )
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
        try:
            async for msg in reader:
                # print(f"{msg.arbitration_id:X}: {msg.data}")  # temp
                if None in self.latest_gps.values():
                    continue  # no GPS fix yet, skip

                # TODO extract angle & distance from CAN msg
                angle = 0    # placeholder
                distance = 0 # placeholder

                object_direction = self.latest_gps['heading'] + angle # type: ignore
                lat = self.latest_gps['latitude'] + distance * np.cos(np.radians(object_direction))
                lon = self.latest_gps['longitude'] + distance * np.sin(np.radians(object_direction))

                new_row = pd.DataFrame([{
                    'id': self.camera_id,
                    'category': 'camera',
                    'latitude': lat,
                    'longitude': lon
                }])
                async with self._csv_lock:
                    await asyncio.to_thread(
                        new_row.to_csv,
                        self.run_dir / 'points.csv',
                        mode='a', header=False, index=False
                    )
                self.camera_id += 1
        finally:
            notifier.stop()
            bus.shutdown()

    async def _setup_um982(self, writer):
        """Sends initial configuration to UM982 via the serial writer."""
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

    async def _gps_listener(self):
        try:
            reader, writer = await serial_asyncio.open_serial_connection(
                url=config.GPS_PORT, baudrate=config.GPS_BAUD
            )
            print("GPS Serial connection established.")
        except Exception as e:
            print(f"Warning: GPS not connected. {e}")
            return

        # Perform device setup
        await self._setup_um982(writer)

        async def rtcm_writer_task():
            """Writes RTCM data from queue to Serial"""
            while True:
                data = await self.rtcm_queue.get()
                try:
                    writer.write(data)
                    await writer.drain()
                except Exception as e:
                    print(f"Error writing RTCM to serial: {e}")

        # Start the RTCM writer background task
        write_task = asyncio.create_task(rtcm_writer_task())

        # Buffer for reading lines
        # StreamReader.readline() is convenient but might buffer.
        
        try:
            while True:
                try:
                    raw_line = await reader.readline()
                except Exception:
                    break
                    
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
                parsed_data = {}
                gps_updated = False

                if head.endswith("GGA"):
                    d = gps_utils.parse_gga(parts)
                    if d:
                        if d.get("lat") is not None: self.latest_gps['latitude'] = d["lat"]
                        if d.get("lon") is not None: self.latest_gps['longitude'] = d["lon"]
                        gps_updated = True

                elif head.endswith("RMC"):
                    d = gps_utils.parse_rmc(parts)
                    if d:
                        if d.get("lat") is not None: self.latest_gps['latitude'] = d["lat"]
                        if d.get("lon") is not None: self.latest_gps['longitude'] = d["lon"]
                        gps_updated = True

                elif head.endswith("HDT"):
                    hdg = gps_utils.parse_hdt(parts)
                    if hdg is not None:
                        hdg = (hdg + config.USER_HEADING_OFFSET_DEG) % 360.0
                        self.latest_gps['heading'] = hdg
                        gps_updated = True

                # Log to CSV if we have a valid update
                if gps_updated and self.latest_gps['latitude'] is not None and self.latest_gps['heading'] is not None:
                    new_row = pd.DataFrame([{
                        'id': self.gps_id,
                        'category': 'gps',
                        'latitude': self.latest_gps['latitude'],
                        'longitude': self.latest_gps['longitude'],
                        'heading': self.latest_gps['heading'],
                    }])
                    async with self._csv_lock:
                         await asyncio.to_thread(
                            new_row.to_csv,
                            self.run_dir / 'points.csv',
                            mode='a', header=False, index=False
                        )
                    self.gps_id += 1

        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"GPS Loop Error: {e}")
        finally:
            if not write_task.done():
                write_task.cancel()
            try:
                await write_task
            except asyncio.CancelledError:
                pass
                
            writer.close()
            try:
                await writer.wait_closed()
            except:
                pass

    async def run(self, stop_event: asyncio.Event):
        # We pass a lambda/function to get the latest GGA
        def get_gga():
            return self.latest_gga_raw

        tasks = [
            asyncio.create_task(self._can_listener()),
            asyncio.create_task(self._gps_listener()),
            # asyncio.create_task(ntrip_client.run_ntrip_client(self.rtcm_queue, get_gga)),
        ]
        
        try:
            await stop_event.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

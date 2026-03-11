import asyncio
import can
import serial_asyncio
import pynmea2
import numpy as np
import pandas as pd
from pathlib import Path
import config


class DataCollector:
    def __init__(self, run_dir: Path):
        self.run_dir = run_dir
        self.gps_id = 0
        self.camera_id = 0

    async def _can_listener(self):
        bus = can.interface.Bus(
            channel=config.CAN_CHANNEL,
            bustype=config.CAN_BUSTYPE,
            bitrate=config.CAN_BITRATE,
        )
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(bus, [reader], loop=asyncio.get_event_loop())
        try:
            async for msg in reader:
                print(f"{msg.arbitration_id:X}: {msg.data}")  # temp
                # TODO extract angle & distance from CAN msg
                angle = msg.data[0] # placeholder
                distance = msg.data[1] # placeholder
                
                df = pd.read_csv(self.run_dir / 'points.csv')
                gps_points = df[df['category'] == 'gps']
                current_location = gps_points.loc[gps_points['id'].idxmax()]

                object_direction = current_location['heading'] + angle
                lat = current_location['latitude'] + distance * np.cos(np.radians(object_direction))
                lon = current_location['longitude'] + distance * np.sin(np.radians(object_direction))

                new_row = pd.DataFrame([{
                    'id': self.camera_id,
                    'category': 'camera',
                    'latitude': lat,
                    'longitude': lon
                }])
                new_row.to_csv(self.run_dir / 'points.csv', mode='a', header=False, index=False)
                self.camera_id += 1
        finally:
            notifier.stop()
            bus.shutdown()

    async def _gps_listener(self):
        # credit: https://github.com/Knio/pynmea2/blob/master/examples/read_serial.py
        try:
            stream, _ = await serial_asyncio.open_serial_connection(
                url=config.GPS_PORT, baudrate=config.GPS_BAUDRATE
            )
            print("GPS Serial connection established.")
        except Exception as e:
            print(f"Warning: GPS not connected. {e}")
            return

        while True:
            try:
                raw = await stream.readline()
                msg = pynmea2.parse(raw.decode("ascii", errors="replace"))
                new_row = pd.DataFrame([{
                    'id': self.gps_id,
                    'category': 'gps',
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'heading': msg.heading,
                }])
                new_row.to_csv(self.run_dir / 'points.csv', mode='a', header=False, index=False)
                self.gps_id += 1
            except pynmea2.ParseError:
                pass  # not every NMEA sentence has lat/lon/heading
            except Exception as e:
                print(f"GPS error: {e}")

    async def run(self, stop_event: asyncio.Event):
        tasks = [
            asyncio.create_task(self._can_listener()),
            asyncio.create_task(self._gps_listener()),
        ]
        await stop_event.wait()
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
import io
import serial
import pynmea2
import pandas as pd
from pathlib import Path
from path_execution import BoatCANInterface
from . import config


class DataCollector:
    def __init__(self, run_dir):
        self.can_bus = BoatCANInterface(channel=config.CAN_CHANNEL, bustype=config.CAN_BUSTYPE, bitrate=config.CAN_BITRATE)

        try:
            gps_serial = serial.Serial(config.GPS_PORT, baudrate=config.GPS_BAUDRATE, timeout=config.READ_TIMEOUT)
            self.gps_reader = io.TextIOWrapper(io.BufferedRWPair(gps_serial, gps_serial))  # type: ignore[arg-type]

            print("GPS Serial connection established.")
        except serial.SerialException as e:
            print(f"Warning: GPS not connected. {e}")
            self.gps_reader = None #zodat dit werkt zonder gps

        self.run_dir = run_dir
        self.gps_id = 0
    
    def update_data(self):
        """
        Reads hardware sensors and updates the state.
        Called continuously by the collection thread in main.py.
        """
        self._read_gps()
        self._read_can()

    def _read_gps(self):
        """
            Reads GPS data and updates the points.csv file with the latest position and heading.
        """
        # credit: https://github.com/Knio/pynmea2/blob/master/examples/read_serial.py
        
        if self.gps_reader is None:
            return
        
        try:
            line = self.gps_reader.readline()
            msg = pynmea2.parse(line)
        except serial.SerialException as e:
            print('GPS error: {}'.format(e))
            return
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            return
                
        try:
            new_row = pd.DataFrame([{
                'id': self.gps_id,
                'category': 'gps',
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'heading': msg.heading
            }])

            self.gps_id += 1

            new_row.to_csv(self.run_dir / 'points.csv', mode='a', header=False, index=False)
            
        except Exception as e:
            print(f"Error updating points.csv: {e}")

    def _read_can(self):
        # TODO read CAN and update CSV
        pass
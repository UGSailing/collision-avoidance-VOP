import serial
import time

while True:
    try:
        with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
            print("Connected to ESP32. Reading data...")
            while True:
                angle = 10.0
                thrust = 0.0
                msg = f"{angle:.2f},{thrust:.3f}\n"
                ser.write(msg.encode("ascii"))
                time.sleep(1)
                angle = 0.0
                thrust = 0.0
                msg = f"{angle:.2f},{thrust:.3f}\n"
                time.sleep(1)
                ser.write(msg.encode("ascii"))
    except serial.SerialException as e:
        print(f"Serial error: {e}. Retrying in 5 seconds...")
        time.sleep(5)
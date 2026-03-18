import can

class BoatCANInterface:
    def __init__(self, channel='can0', bustype='socketcan', bitrate=1000000):
        """
        Initializes the CAN bus connection.
        Defaulting to 'socketcan' and 'can0' which is standard for Linux/Raspberry Pi.
        Bitrate is set to 1000000 (1 Mbps) to match the ESP32 code.
        """
        try:
            self.bus = can.interface.Bus(
                channel=channel, 
                bustype=bustype, 
                bitrate=bitrate
            )
            print(f"CAN Bus initialized on {channel} at {bitrate} bps.")
        except (can.CanError, OSError) as e:
            print(f"Failed to initialize CAN bus: {e}")
            self.bus = None

    def read_angle_message(self, timeout=1.0):
        """
        Listens for the AS5600 angle message from the ESP32 (ID 0x01).
        Returns the raw angle (0-4095) or None if no message is received.
        """
        if self.bus is None:
            return None

        # Read a message from the bus
        msg = self.bus.recv(timeout)
        
        if msg is not None:
            # Check if it's the specific ID sending the angle (0x01)
            if msg.arbitration_id == 0x01 and msg.dlc == 2:
                # Reconstruct the 16-bit integer from the 2 bytes (High byte, Low byte)
                raw_angle = (msg.data[0] << 8) | msg.data[1]
                return raw_angle
                
        return None

    def send_rudder_command(self, target_raw_angle):
        """
        Sends a desired angle to the ESP32 over CAN (ID 0x02).
        target_raw_angle should be an integer between 0 and 4095.
        """
        if self.bus is None:
            print("Cannot send, CAN bus not initialized.")
            return False

        # Ensure the angle is within the 12-bit range of the AS5600 logic
        target_raw_angle = max(0, min(4095, int(target_raw_angle)))

        # Split the integer into 2 bytes (High byte, Low byte)
        data = [
            (target_raw_angle >> 8) & 0xFF,
            target_raw_angle & 0xFF
        ]

        # Create the CAN message (standard 11-bit ID)
        msg = can.Message(
            arbitration_id=0x02,
            data=data,
            is_extended_id=False
        )

        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"Message failed to send: {e}")
            return False
            
    def shutdown(self):
        """Cleanly close the CAN bus."""
        if self.bus is not None:
            self.bus.shutdown()


#Deze code en de communicatie is gebaseerd op 1 github commit in de ugs github, kan dus fout zijn, check later
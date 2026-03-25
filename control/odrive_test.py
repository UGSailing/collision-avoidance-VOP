import odrive
from odrive.enums import AxisState, ControlMode
from odrive.utils import request_state
import time
import config

def main():
    print("Searching for ODrive Pro over USB...")
    # This will block until an ODrive is found
    my_drive = odrive.find_any()
    print(f"Found ODrive! Serial number: {my_drive.serial_number}")

    # Clear any lingering errors from previous sessions or the UART controller
    my_drive.clear_errors()

    try:
        print("Setting control mode to VELOCITY_CONTROL...")
        # Tell the controller we want to control velocity (speed)
        my_drive.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        
        print(f"Setting input velocity to {config.TURNS_PER_SEC:.2f} turns/s ({config.TARGET_RPM} RPM)...")
        # Set the target speed
        my_drive.axis0.controller.input_vel = config.TURNS_PER_SEC
        
        print("Entering CLOSED_LOOP_CONTROL state. Motor should spin!")
        # Engage the motor
        my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        
        # Keep the script alive while the motor spins
        print("Press Ctrl+C to stop the script and the motor.")
        while True:
            # You can print out the estimated velocity here if you want to monitor it
            current_vel = my_drive.axis0.encoder.vel_estimate
            print(f"Current Velocity: {current_vel:.2f} turns/s", end='\r')
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Stopping motor...")
        
    finally:
        # Safety net: Always ensure the motor stops and disengages when exiting
        print("Disengaging motor...")
        my_drive.axis0.controller.input_vel = 0.0
        my_drive.axis0.requested_state = AxisState.IDLE
        print("Motor is IDLE and safe.")

if __name__ == "__main__":
    main()
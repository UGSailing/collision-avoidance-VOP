import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import sys

def check_errors(odrv):
    print("\n--- Current Errors ---")
    dump_errors(odrv)
    print("----------------------\n")

def main():
    print("Finding ODrive...")
    odrv0 = odrive.find_any()
    print(f"Found ODrive: {odrv0.serial_number}")
    print(f"Bus Voltage: {odrv0.vbus_voltage:.2f}V")

    if odrv0.vbus_voltage < 10.0:
        print("\n\033[91mCRITICAL WARNING: DC Bus Voltage is extremely low!\033[0m")
        print("Ensure the main power supply (battery or PSU) is connected and turned on.")
        print("USB power is NOT sufficient to spin the motor.\n")
    
    # Check for initial errors (compatible with ODrive v0.5 and v0.6+)
    active_errors = 0
    motor_errors = 0
    encoder_errors = 0
    
    # Try to fetch errors based on available attributes
    try:
        # v0.5 style
        if hasattr(odrv0.axis0, 'error'):
            active_errors = odrv0.axis0.error
            motor_errors = odrv0.axis0.motor.error
            encoder_errors = odrv0.axis0.encoder.error
        # v0.6 style (ODrive Pro/S1)
        elif hasattr(odrv0.axis0, 'active_errors'):
            active_errors = odrv0.axis0.active_errors
            motor_errors = odrv0.axis0.motor.active_errors if hasattr(odrv0.axis0.motor, 'active_errors') else 0
            encoder_errors = odrv0.axis0.encoder.active_errors if hasattr(odrv0.axis0.encoder, 'active_errors') else 0
            
            # Print specific disarm reason if available
            if hasattr(odrv0.axis0, 'disarm_reason') and odrv0.axis0.disarm_reason != 0:
                print(f"Last Disarm Reason: {odrv0.axis0.disarm_reason}")
            if hasattr(odrv0.axis0, 'procedure_result') and odrv0.axis0.procedure_result != 0:
                print(f"Last Procedure Result: {odrv0.axis0.procedure_result}")

    except Exception as e:
        print(f"Error checking status flags: {e}")

    if active_errors != 0 or motor_errors != 0 or encoder_errors != 0:
        print("!! Initial Errors Detected !!")
        check_errors(odrv0)
    else:
        print("No initial errors found.")

    # Check calibration status
    print(f"\nAxis0 State: {odrv0.axis0.current_state}")
    is_calibrated = False
    if hasattr(odrv0.axis0, 'is_calibrated'):
        is_calibrated = odrv0.axis0.is_calibrated
        print(f"Axis calibrated: {is_calibrated}")
    elif hasattr(odrv0.axis0.motor, 'is_calibrated'):
         is_calibrated = odrv0.axis0.motor.is_calibrated
         print(f"Motor calibrated: {odrv0.axis0.motor.is_calibrated}")
         print(f"Encoder calibrated: {odrv0.axis0.encoder.is_calibrated}")
    
    if not is_calibrated:
        print("\nWARNING: Motor or Encoder is not calibrated. The motor will not spin in CLOSED_LOOP_CONTROL without calibration.")
        print("If you have a pre-calibrated setup, ensure 'startup_motor_calibration' or 'startup_encoder_index_search' are working.")
        print("You may need to run odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE first.")

    confirm = input("\nDo you want to attempt clearing errors and setting CLOSED_LOOP_CONTROL? (y/n): ")
    if confirm.lower() != 'y':
        print("Exiting.")
        return

    print("Clearing errors...")
    odrv0.clear_errors()
    
    print("Setting Control Mode to VELOCITY_CONTROL...")
    odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv0.axis0.controller.input_vel = 2.0  # Slow test speed (2 turns/sec)
    
    print("Requesting CLOSED_LOOP_CONTROL...")
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    
    # Wait a moment for state change
    time.sleep(0.5)
    
    # Check if it stayed in closed loop
    if odrv0.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
        print("\nSUCCESS: State is CLOSED_LOOP_CONTROL")
        print("Motor should be spinning. Monitoring for 5 seconds...")
        for i in range(50):
            print(f"Vel: {odrv0.axis0.encoder.vel_estimate:.2f} turns/s | Curr: {odrv0.axis0.motor.current_control.Iq_measured:.2f} A", end='\r')
            time.sleep(0.1)
            # Check for sudden errors
            if odrv0.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
                print("\n\n!! Motor dropped out of CLOSED_LOOP_CONTROL !!")
                check_errors(odrv0)
                break
    else:
        print(f"\nFAILED: State is {odrv0.axis0.current_state} (Expected 8: CLOSED_LOOP_CONTROL)")
        check_errors(odrv0)

    print("\nStopping motor...")
    odrv0.axis0.controller.input_vel = 0
    odrv0.axis0.requested_state = AxisState.IDLE

if __name__ == "__main__":
    main()

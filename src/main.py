import odrive
import odrive.enums # Import enums for constants
# import serial # Arduino dependency - commented out
import time
import csv
import numpy as np
from datetime import datetime
import threading
import json
import queue # Using queue for thread-safe communication
import math

# --- Configuration ---
# ARDUINO_PORT = 'COM5'  # Arduino dependency - commented out
# ARDUINO_BAUD = 115200 # Arduino dependency - commented out
CSV_FILENAME_TEMPLATE = "wheel_test_data_ODRIVE_ONLY_{test_name}_{timestamp}.csv" # Updated template name
SAMPLE_RATE_HZ = 50  # How many samples per second to target for ODrive

# --- ODrive Parameter Configuration ---
MOTOR_POLE_PAIRS = 7
MOTOR_PHASE_RESISTANCE = 0.039
MOTOR_KV = 270
MOTOR_CURRENT_LIMIT = 20
MOTOR_TYPE = odrive.enums.MOTOR_TYPE_HIGH_CURRENT
ENCODER_MODE = odrive.enums.ENCODER_MODE_HALL
ENCODER_CPR = MOTOR_POLE_PAIRS * 6
POS_GAIN = 20.0
VEL_GAIN = 0.16
VEL_INTEGRATOR_GAIN = 0.32
VEL_LIMIT = 10
VBUS_VOLTAGE = 24 # CHECK YOUR ACTUAL POWER SUPPLY
VOLTAGE_LIMIT_MARGIN = 4
CURRENT_LIMIT_MARGIN = 5
# !! CRITICAL: Set this to the actual value of your connected brake resistor !!
BRAKE_RESISTANCE_OHMS = 0.5 # Ohms - CHANGE THIS TO MATCH YOUR HARDWARE
TRAJ_ACCEL_LIMIT = 1.0
TRAJ_DECEL_LIMIT = 1.0

# --- Global Variables for Threading ---
# arduino_data_queue = queue.Queue() # Arduino dependency - commented out
odrive_data_queue = queue.Queue()
collection_active = threading.Event()

# --- Helper Functions ---

def connect_to_odrive():
    """Connect to the ODrive."""
    print("Looking for ODrive...")
    try:
        odrv = odrive.find_any()
        print(f"Found ODrive! Serial: {odrv.serial_number}")
        if hasattr(odrv, 'axis0') and odrv.axis0 is not None:
             print("ODrive Axis 0 found.")
        else:
            print("Error: ODrive Axis 0 not found.")
            return None
        return odrv
    except Exception as e:
        print(f"Error connecting to ODrive: {e}")
        return None

# def connect_to_arduino(port, baud): # Arduino dependency - commented out
#     """Connect to the Arduino."""
#     try:
#         ser = serial.Serial(port, baud, timeout=1)
#         print(f"Attempting connection to Arduino on {port}...")
#         time.sleep(2)
#         if ser.is_open:
#             print(f"Connected to Arduino on {port}")
#             ser.flushInput()
#             return ser
#         else:
#             print(f"Failed to open serial port {port}")
#             return None
#     except serial.SerialException as e:
#         print(f"Failed to connect to Arduino: {e}")
#         return None

# --- ODrive Configuration Function (Unchanged) ---
def configure_odrive_parameters(odrv):
    """Sets key ODrive parameters based on constants defined above."""
    print("Configuring ODrive parameters...")
    axis = odrv.axis0
    try:
        # Motor Config
        print("Setting Motor Config...")
        axis.motor.config.pole_pairs = MOTOR_POLE_PAIRS
        axis.motor.config.motor_type = MOTOR_TYPE
        axis.motor.config.phase_resistance = MOTOR_PHASE_RESISTANCE
        axis.motor.config.torque_constant = 8.27 / MOTOR_KV
        axis.motor.config.current_lim = MOTOR_CURRENT_LIMIT
        axis.motor.config.current_lim_margin = CURRENT_LIMIT_MARGIN
        # Encoder Config
        print("Setting Encoder Config...")
        axis.encoder.config.mode = ENCODER_MODE
        axis.encoder.config.cpr = ENCODER_CPR
        axis.encoder.config.use_index = False
        # Controller Config
        print("Setting Controller Config...")
        axis.controller.config.pos_gain = POS_GAIN
        axis.controller.config.vel_gain = VEL_GAIN
        axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
        axis.controller.config.vel_limit = VEL_LIMIT
        axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.input_mode = odrive.enums.INPUT_MODE_PASSTHROUGH
        # System Config
        print("Setting System Config...")
        print(f"Setting brake resistance to: {BRAKE_RESISTANCE_OHMS} Ohms (Ensure this matches hardware!)")
        odrv.config.brake_resistance = BRAKE_RESISTANCE_OHMS
        print(f"Setting voltage limits based on VBUS_VOLTAGE = {VBUS_VOLTAGE}V")
        odrv.config.dc_bus_undervoltage_trip_level = 8.0
        odrv.config.dc_bus_overvoltage_trip_level = VBUS_VOLTAGE + VOLTAGE_LIMIT_MARGIN

        print("Parameter configuration complete.")
        return True
    except Exception as e:
        print(f"Error during ODrive parameter configuration: {e}")
        return False

# --- ODrive Calibration Function (Unchanged) ---
def run_odrive_calibration(odrv):
    """Runs necessary calibration sequences."""
    print("Starting ODrive calibration sequence...")
    axis = odrv.axis0
    try:
        # Motor Calibration
        print("Requesting Motor Calibration...")
        axis.requested_state = odrive.enums.AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(1)
        while axis.current_state == odrive.enums.AXIS_STATE_MOTOR_CALIBRATION: time.sleep(0.5); print("  Motor cal running...")
        if axis.motor.error != 0 or axis.error != 0:
            print(f"Error during motor calibration! Axis Error: {axis.error}, Motor Error: {axis.motor.error}"); odrive.dump_errors(odrv); return False
        if not axis.motor.config.phase_resistance or axis.motor.config.phase_resistance == 0 or \
           not axis.motor.config.phase_inductance or axis.motor.config.phase_inductance == 0:
             print("Warning: Motor R/L invalid after calibration.")
        else: print(f"  Motor calibration successful. R={axis.motor.config.phase_resistance:.4f}, L={axis.motor.config.phase_inductance:.6f}")
        # Encoder Calibration
        if ENCODER_MODE == odrive.enums.ENCODER_MODE_HALL:
            print("Requesting Hall Polarity Calibration...")
            axis.encoder.config.hall_polarity_calibrated = False
            axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
            time.sleep(1)
            while axis.current_state == odrive.enums.AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION: time.sleep(0.5); print("  Hall cal running...")
            if axis.encoder.error != 0 or axis.error != 0:
                print(f"Error during Hall polarity calibration! Axis Error: {axis.error}, Encoder Error: {axis.encoder.error}"); odrive.dump_errors(odrv); return False
            if not axis.encoder.config.hall_polarity_calibrated: print("Error: Hall polarity cal flag not set."); return False
            print("  Hall polarity calibration successful.")
        elif ENCODER_MODE in [odrive.enums.ENCODER_MODE_INCREMENTAL, odrive.enums.ENCODER_MODE_SPI_ABS_CUI]:
             print("Requesting Encoder Offset Calibration...")
             axis.encoder.config.pre_calibrated = False
             axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION
             time.sleep(1)
             while axis.current_state == odrive.enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION: time.sleep(0.5); print("  Encoder offset cal running...")
             if axis.encoder.error != 0 or axis.error != 0:
                 print(f"Error during encoder offset calibration! Axis Error: {axis.error}, Encoder Error: {axis.encoder.error}"); odrive.dump_errors(odrv); return False
             if not axis.encoder.config.pre_calibrated: print("Error: Encoder offset cal flag not set."); return False
             print("  Encoder offset calibration successful.")
        else: print(f"Encoder mode {ENCODER_MODE} needs no standard calibration here.")
        # Set Ready
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        print("Calibration sequence complete.")
        return True
    except Exception as e: print(f"Error during ODrive calibration: {e}"); return False

# --- Data Reader Threads ---

# def arduino_reader(ser): # Arduino dependency - commented out
#    pass # Keep function defined maybe, but do nothing

def odrive_reader(odrv):
    """Thread function to read data from ODrive and put it in a queue."""
    # print("ODrive reader thread started.") # Reduce noise
    sample_period = 1.0 / SAMPLE_RATE_HZ; next_sample_time = time.time()
    while collection_active.is_set():
        current_time = time.time()
        if current_time >= next_sample_time:
            try:
                if hasattr(odrv, 'axis0') and odrv.axis0:
                    data = {'sys_time': current_time, 'voltage': odrv.vbus_voltage,
                            'current_q': getattr(getattr(getattr(odrv.axis0, 'motor', {}), 'current_control', {}), 'Iq_measured', None),
                            'current_d': getattr(getattr(getattr(odrv.axis0, 'motor', {}), 'current_control', {}), 'Id_measured', None),
                            'velocity': getattr(getattr(odrv.axis0, 'encoder', {}), 'vel_estimate', None),
                            'position': getattr(getattr(odrv.axis0, 'encoder', {}), 'pos_estimate', None),
                            'temperature': getattr(getattr(getattr(odrv.axis0, 'motor', {}), 'fet_thermistor', {}), 'temperature', None),
                            'axis_error': getattr(odrv.axis0, 'error', 0), 'motor_error': getattr(getattr(odrv.axis0, 'motor', {}), 'error', 0),
                            'encoder_error': getattr(getattr(odrv.axis0, 'encoder', {}), 'error', 0)}
                    odrive_data_queue.put(data)
                else: time.sleep(0.1)
            except Exception as e: print(f"ODrive read error: {e}")
            next_sample_time += sample_period
            if next_sample_time < current_time: next_sample_time = current_time + sample_period
        else:
             sleep_time = next_sample_time - current_time - 0.001
             if sleep_time > 0: time.sleep(sleep_time)
    # print("ODrive reader thread finished.") # Reduce noise


# --- Data Processing (Modified for ODrive Only) ---
def merge_and_save_data(test_name):
    """Saves ODrive data ONLY to CSV."""
    print("Saving ODrive data...")

    # Drain ODrive queue into list
    odrive_data = []
    while not odrive_data_queue.empty():
        odrive_data.append(odrive_data_queue.get_nowait())

    print(f"Processing {len(odrive_data)} ODrive samples")
    if not odrive_data:
        print("No ODrive data collected to save.")
        return

    # Prepare CSV filename
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = CSV_FILENAME_TEMPLATE.format(test_name=test_name, timestamp=timestamp_str)

    # Define CSV headers - ODrive ONLY
    fieldnames = ['timestamp', 'voltage', 'current_q', 'current_d', 'velocity',
                  'position', 'temperature', 'axis_error', 'motor_error', 'encoder_error']

    # Sort data by timestamp just in case
    odrive_data.sort(key=lambda x: x['sys_time'])

    # Rename 'sys_time' to 'timestamp' for CSV header consistency
    for record in odrive_data:
        record['timestamp'] = record.pop('sys_time')

    # Write to CSV
    try:
        with open(filename, 'w', newline='') as csvfile:
            # Use only ODrive fieldnames
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(odrive_data) # Write the processed ODrive data
        print(f"ODrive data successfully saved to {filename}")
    except IOError as e:
        print(f"Error writing CSV {filename}: {e}")
    except Exception as e:
        print(f"Unexpected error writing CSV: {e}")


# --- Test Execution Function (Modified for ODrive Only) ---
def run_distance_test(odrv, test_name, target_velocity_rps, loaded_radius_cm, target_distance_m=0.80):
    """
    Runs a test using position control to travel a specific linear distance.
    (Modified to not require arduino_ser)
    """
    global collection_active

    if not odrv: # Check only for odrv
        print(f"Skipping test '{test_name}': ODrive not connected.")
        return False

    print(f"\n--- Starting Distance Test: {test_name} ---")
    print(f"Target Dist: {target_distance_m*100:.0f} cm, Loaded Radius: {loaded_radius_cm:.2f} cm, Target Vel: {target_velocity_rps:.2f} turns/sec")

    test_successful = False
    axis = odrv.axis0
    try:
        # Calculations (Unchanged)
        if loaded_radius_cm <= 0: print("Error: Loaded radius must be positive."); return False
        loaded_radius_m = loaded_radius_cm / 100.0; circumference_m = 2 * math.pi * loaded_radius_m
        if circumference_m <= 0: print("Error: Circumference calculation failed."); return False
        target_revolutions = target_distance_m / circumference_m; target_pos_turns = target_revolutions
        print(f"Calculated: Circumference={circumference_m:.3f} m, Target Revolutions={target_revolutions:.3f} turns")

        # ODrive Configuration (Unchanged logic)
        print("Configuring ODrive for position control...")
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE; time.sleep(0.1)
        if axis.error != 0: print(f"Axis error {axis.error} detected. Clearing."); odrive.dump_errors(odrv, True); time.sleep(0.1)
        if axis.error != 0: print("Failed to clear axis errors. Skipping."); return False
        axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = odrive.enums.INPUT_MODE_TRAP_TRAJ
        effective_vel_limit = min(target_velocity_rps, axis.controller.config.vel_limit)
        axis.trap_traj.config.vel_limit = effective_vel_limit
        axis.trap_traj.config.accel_limit = TRAJ_ACCEL_LIMIT
        axis.trap_traj.config.decel_limit = TRAJ_DECEL_LIMIT
        print(f"Trajectory Limits: Vel={effective_vel_limit:.2f}, Accel={TRAJ_ACCEL_LIMIT:.2f}, Decel={TRAJ_DECEL_LIMIT:.2f}")

        # Get Starting Position (Unchanged)
        start_pos_turns = axis.encoder.pos_estimate; print(f"Starting Position: {start_pos_turns:.3f} turns")
        final_target_pos_turns = start_pos_turns + target_pos_turns; print(f"Target Position: {final_target_pos_turns:.3f} turns")

        # Start Data Collection (ODrive ONLY)
        print("Starting ODrive data collection thread...")
        collection_active.set()
        # arduino_thread = threading.Thread(target=arduino_reader, args=(arduino_ser,), daemon=True) # Arduino dependency - commented out
        odrive_thread = threading.Thread(target=odrive_reader, args=(odrv,), daemon=True)
        # arduino_thread.start() # Arduino dependency - commented out
        odrive_thread.start()
        time.sleep(0.1)

        # Command the Move (Unchanged)
        print("Commanding position move...")
        axis.controller.input_pos = final_target_pos_turns
        axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL; time.sleep(0.1)
        if axis.current_state != odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"Error: Axis failed to enter closed loop. State: {axis.current_state}"); odrive.dump_errors(odrv); collection_active.clear(); return False

        # Wait for Move Completion (Unchanged logic)
        print("Waiting for trajectory completion...")
        start_wait_time = time.time()
        max_time_estimate = abs(target_pos_turns / effective_vel_limit) + (effective_vel_limit / TRAJ_ACCEL_LIMIT) + (effective_vel_limit / TRAJ_DECEL_LIMIT) + 5.0
        print(f"Estimated max move time: {max_time_estimate:.2f}s"); wait_timeout = max(10.0, max_time_estimate)
        move_complete = False
        while time.time() - start_wait_time < wait_timeout:
            if axis.controller.trajectory_done: move_complete = True; print("Trajectory done flag detected."); break
            if axis.error != 0: print(f"Axis error {axis.error} during move! Stopping."); odrive.dump_errors(odrv); break
            time.sleep(0.1)
        if not move_complete: print("Warning: Move did not complete within timeout or error occurred.")
        if axis.error != 0 and not move_complete: odrive.dump_errors(odrv) # Dump errors if move failed due to error

        # Stop Data Collection (Unchanged logic)
        print("Stopping data collection..."); collection_active.clear(); time.sleep(0.5); print("Data collection complete!")

        # Stop Motor (Unchanged logic)
        print("Setting ODrive to IDLE state..."); axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        axis.controller.input_pos = axis.encoder.pos_estimate; time.sleep(0.5); print("Motor stopped.")

        # Process and Save Data (Calls modified function)
        merge_and_save_data(test_name)
        test_successful = True

    except Exception as e:
        print(f"Error during distance test '{test_name}': {e}")
        try:
            if odrv and hasattr(odrv, 'axis0'): odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
        except Exception as cleanup_e: print(f"Error during emergency cleanup: {cleanup_e}")
    finally:
         collection_active.clear()
         try:
             if odrv and hasattr(odrv, 'axis0') and odrv.axis0.current_state != odrive.enums.AXIS_STATE_IDLE:
                  print("Ensuring ODrive is idle post-test."); odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
         except Exception as final_cleanup_e: print(f"Error during final ODrive cleanup: {final_cleanup_e}")

    return test_successful


# --- Main Execution Logic (Modified for ODrive Only) ---
def main():
    """Main function to connect, configure, calibrate, run ODrive tests, and cleanup."""
    odrv = None
    # arduino_ser = None # Arduino dependency - commented out
    try:
        # Connect to ODrive ONLY
        odrv = connect_to_odrive()
        # arduino_ser = connect_to_arduino(ARDUINO_PORT, ARDUINO_BAUD) # Arduino dependency - commented out

        if not odrv: # Check only odrv
            print("Failed to connect to ODrive device. Exiting.")
            return

        # Configure ODrive Parameters (Unchanged)
        if not configure_odrive_parameters(odrv): print("ODrive parameter configuration failed. Exiting."); return

        # Run ODrive Calibration (Unchanged)
        print("\n*** WARNING: ODrive calibration will spin the motor briefly. ***"); print("*** Ensure the wheel is free to rotate without obstruction. ***")
        input("Press Enter to start calibration...")
        if not run_odrive_calibration(odrv): print("ODrive calibration failed. Exiting."); odrive.dump_errors(odrv); return
        print("Calibration successful.")

        # Save Configuration (Unchanged)
        print("Saving ODrive configuration...");
        try: odrv.save_configuration(); print("Configuration saved.")
        except Exception as e: print(f"Error saving configuration: {e}")

        # Run Interactive Distance Tests (Modified to call updated function)
        print("\n--- Starting Interactive Distance Test Sequence (ODrive Only) ---")
        while True:
            print("\nEnter test parameters (or type 'quit' for test name to exit):")
            test_name = input("Enter a unique name for this test run: ")
            if test_name.lower() == 'quit': break
            try:
                target_velocity_str = input(f"Enter target velocity during move (turns/sec, e.g., 1.0, max {VEL_LIMIT}): ")
                target_velocity_rps = float(target_velocity_str)
                if target_velocity_rps <= 0 or target_velocity_rps > VEL_LIMIT: print(f"Invalid velocity. Must be > 0 and <= {VEL_LIMIT}."); continue
                loaded_radius_str = input("Enter current LOADED wheel radius (cm): ")
                loaded_radius_cm = float(loaded_radius_str)
                if loaded_radius_cm <= 0: print("Invalid radius. Must be positive."); continue
                print("\n*** Ensure the correct vertical load is applied to the wheel! ***")
                input("Press Enter when ready to start the test...")
                # Call the test function WITHOUT arduino_ser
                run_distance_test(odrv, test_name, target_velocity_rps, loaded_radius_cm)
            except ValueError: print("Invalid input. Please enter numbers.")
            except Exception as e: print(f"An error occurred setting up test: {e}")
        print("\n--- Test Sequence Finished ---")

    except KeyboardInterrupt: print("\nProgram interrupted by user"); collection_active.clear()
    except Exception as e: print(f"An critical error occurred in the main program: {e}"); collection_active.clear()
    finally:
        # Cleanup (ODrive ONLY)
        print("Cleaning up resources..."); collection_active.clear(); time.sleep(0.5)
        if odrv and hasattr(odrv, 'axis0'):
            try: print("Setting ODrive to IDLE state."); odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
            except Exception as e: print(f"Error idling ODrive: {e}")
        # if arduino_ser and arduino_ser.is_open: # Arduino dependency - commented out
        #     try: print("Closing Arduino connection."); arduino_ser.close()
        #     except Exception as e: print(f"Error closing serial: {e}")
        print("Program finished.")

if __name__ == "__main__":
    main()

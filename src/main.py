import odrive
import odrive.enums # Import enums for constants
import serial
import time
import csv
import numpy as np
from datetime import datetime
import threading
import json
import queue # Using queue for thread-safe communication

# --- Configuration ---
ARDUINO_PORT = 'COM5'  # CHANGE TO YOUR Arduino port (e.g., '/dev/ttyACM0' on Linux)
ARDUINO_BAUD = 115200
CSV_FILENAME_TEMPLATE = "wheel_test_data_{test_name}_{timestamp}.csv" # Template for CSV filenames
SAMPLE_RATE_HZ = 50  # How many samples per second to target
DEFAULT_COLLECTION_TIME_SEC = 10 # Default duration for data collection if not specified per test

# --- ODrive Parameter Configuration ---
# !! IMPORTANT: Verify these parameters match your specific hardware !!
MOTOR_POLE_PAIRS = 7 # Confirmed for D5065 (14 pole motor)
MOTOR_PHASE_RESISTANCE = 0.039 # From datasheet (39 mOhm)
MOTOR_KV = 270 # From datasheet
MOTOR_CURRENT_LIMIT = 20 # Amps - SAFE STARTING LIMIT, ADJUST AS NEEDED (Max rating 65A)
MOTOR_TYPE = odrive.enums.MOTOR_TYPE_HIGH_CURRENT # Or MOTOR_TYPE_GIMBAL

# Assuming HALL SENSORS - If using an encoder, change mode and CPR!
ENCODER_MODE = odrive.enums.ENCODER_MODE_HALL
ENCODER_CPR = MOTOR_POLE_PAIRS * 6 # CPR for Hall sensors = 6 * pole_pairs

# Controller Gains - !! LIKELY NEED TUNING !!
# Start low and increase carefully. Refer to ODrive tuning guide.
POS_GAIN = 20.0
VEL_GAIN = 0.16 # Example: 0.16
VEL_INTEGRATOR_GAIN = 0.32 # Example: 0.32

# Limits
VEL_LIMIT = 10 # turns/sec (Make sure this is > max test speed)
VBUS_VOLTAGE = 24 # Your typical bus voltage - CHECK YOUR ACTUAL POWER SUPPLY
VOLTAGE_LIMIT_MARGIN = 4 # Volts - Margin for overvoltage trip
CURRENT_LIMIT_MARGIN = 5 # Amps - Margin for current limit setting

# --- ODrive v3.6 Specific ---
# !! CRITICAL: Set this to the actual value of your connected brake resistor !!
# !! Common values are 0.5, 2.0. Using 0 means NO resistor, which is WRONG for v3.6 !!
BRAKE_RESISTANCE_OHMS = 0.5 # Ohms - CHANGE THIS TO MATCH YOUR HARDWARE

# --- Global Variables for Threading ---
arduino_data_queue = queue.Queue()
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

def connect_to_arduino(port, baud):
    """Connect to the Arduino."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Attempting connection to Arduino on {port}...")
        time.sleep(2)
        if ser.is_open:
            print(f"Connected to Arduino on {port}")
            ser.flushInput()
            return ser
        else:
            print(f"Failed to open serial port {port}")
            return None
    except serial.SerialException as e:
        print(f"Failed to connect to Arduino: {e}")
        return None

# --- ODrive Configuration Function ---

def configure_odrive_parameters(odrv):
    """Sets key ODrive parameters based on constants defined above."""
    print("Configuring ODrive parameters...")
    axis = odrv.axis0 # Assuming axis0

    try:
        # === Motor Configuration ===
        print("Setting Motor Config...")
        axis.motor.config.pole_pairs = MOTOR_POLE_PAIRS
        axis.motor.config.motor_type = MOTOR_TYPE
        axis.motor.config.phase_resistance = MOTOR_PHASE_RESISTANCE
        # axis.motor.config.phase_inductance = ... # Let calibration find this
        axis.motor.config.torque_constant = 8.27 / MOTOR_KV # ODrive calculates this during calibration anyway
        axis.motor.config.current_lim = MOTOR_CURRENT_LIMIT
        axis.motor.config.current_lim_margin = CURRENT_LIMIT_MARGIN

        # === Encoder Configuration ===
        print("Setting Encoder Config...")
        # !! VERIFY ENCODER_MODE AND ENCODER_CPR BASED ON YOUR HARDWARE !!
        axis.encoder.config.mode = ENCODER_MODE
        axis.encoder.config.cpr = ENCODER_CPR
        axis.encoder.config.use_index = False # Assuming no index pulse for Hall

        # === Controller Configuration ===
        print("Setting Controller Config...")
        axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL # Default to velocity for tests
        axis.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP # Use ramp for smoother control
        axis.controller.config.vel_limit = VEL_LIMIT
        # Set gains - !! LIKELY NEED TUNING !!
        axis.controller.config.pos_gain = POS_GAIN
        axis.controller.config.vel_gain = VEL_GAIN
        axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN

        # === System Configuration ===
        print("Setting System Config...")
        # !! CRITICAL for Odrive v3.6: Set correct brake resistance !!
        print(f"Setting brake resistance to: {BRAKE_RESISTANCE_OHMS} Ohms (Ensure this matches hardware!)")
        odrv.config.brake_resistance = BRAKE_RESISTANCE_OHMS
        # Set voltage limits based on your ACTUAL power supply voltage
        print(f"Setting voltage limits based on VBUS_VOLTAGE = {VBUS_VOLTAGE}V")
        odrv.config.dc_bus_undervoltage_trip_level = 8.0 # Default, adjust if needed for lower voltage supplies
        odrv.config.dc_bus_overvoltage_trip_level = VBUS_VOLTAGE + VOLTAGE_LIMIT_MARGIN
        # Optional: Set DC current limits if needed
        # odrv.config.dc_max_positive_current = ...
        # odrv.config.dc_max_negative_current = ... # Limit regen current, related to brake resistor power

        print("Parameter configuration complete.")
        return True

    except Exception as e:
        print(f"Error during ODrive parameter configuration: {e}")
        return False

# --- ODrive Calibration Function (Unchanged logic, but relies on correct config) ---
def run_odrive_calibration(odrv):
    """Runs necessary calibration sequences."""
    print("Starting ODrive calibration sequence...")
    axis = odrv.axis0

    try:
        # === Motor Calibration ===
        print("Requesting Motor Calibration (AXIS_STATE_MOTOR_CALIBRATION)...")
        axis.requested_state = odrive.enums.AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(1) # Give time to start
        while axis.current_state == odrive.enums.AXIS_STATE_MOTOR_CALIBRATION:
            time.sleep(0.5)
            print("  Motor calibration running...")
        # Check for motor errors AFTER calibration state finishes
        if axis.motor.error != 0 or axis.error != 0: # Check motor and axis errors
            print(f"Error during motor calibration! Axis Error: {axis.error}, Motor Error: {axis.motor.error}")
            odrive.dump_errors(odrv) # Dump errors for diagnosis
            return False
        # Check if calibration actually populated values (can sometimes fail silently)
        if not axis.motor.config.phase_resistance or axis.motor.config.phase_resistance == 0 or \
           not axis.motor.config.phase_inductance or axis.motor.config.phase_inductance == 0:
             print("Warning: Motor resistance/inductance might not be valid after calibration. Check values.")
             # Optionally return False here if strict check needed
        else:
             print(f"  Motor calibration successful. R={axis.motor.config.phase_resistance:.4f}, L={axis.motor.config.phase_inductance:.6f}")


        # === Encoder Calibration ===
        if ENCODER_MODE == odrive.enums.ENCODER_MODE_HALL:
            print("Requesting Hall Polarity Calibration (AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION)...")
            axis.encoder.config.hall_polarity_calibrated = False # Reset flag
            axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
            time.sleep(1)
            while axis.current_state == odrive.enums.AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION:
                time.sleep(0.5)
                print("  Hall polarity calibration running...")
            if axis.encoder.error != 0 or axis.error != 0:
                print(f"Error during Hall polarity calibration! Axis Error: {axis.error}, Encoder Error: {axis.encoder.error}")
                odrive.dump_errors(odrv)
                return False
            if not axis.encoder.config.hall_polarity_calibrated:
                 print("Error: Hall polarity calibration failed to set calibrated flag.")
                 return False
            print("  Hall polarity calibration successful.")

        elif ENCODER_MODE == odrive.enums.ENCODER_MODE_INCREMENTAL or ENCODER_MODE == odrive.enums.ENCODER_MODE_SPI_ABS_CUI: # Add other encoder types needing offset cal
             print("Requesting Encoder Offset Calibration (AXIS_STATE_ENCODER_OFFSET_CALIBRATION)...")
             axis.encoder.config.pre_calibrated = False # Ensure it runs calibration
             axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION
             time.sleep(1)
             while axis.current_state == odrive.enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
                 time.sleep(0.5)
                 print("  Encoder offset calibration running...")
             if axis.encoder.error != 0 or axis.error != 0:
                 print(f"Error during encoder offset calibration! Axis Error: {axis.error}, Encoder Error: {axis.encoder.error}")
                 odrive.dump_errors(odrv)
                 return False
             if not axis.encoder.config.pre_calibrated:
                 print("Error: Encoder offset calibration failed to set pre_calibrated flag.")
                 return False
             print("  Encoder offset calibration successful.")
        else:
            print(f"Encoder mode {ENCODER_MODE} does not require standard calibration sequence here.")


        # === Set Axis Ready ===
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        print("Calibration sequence complete.")
        return True

    except Exception as e:
        print(f"Error during ODrive calibration: {e}")
        return False


# --- Data Reader Threads (Unchanged) ---
def arduino_reader(ser):
    """Thread function to read data from Arduino and put it in a queue."""
    # print("Arduino reader thread started.") # Reduce noise
    while collection_active.is_set():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    data = json.loads(line)
                    data['sys_time'] = time.time()
                    arduino_data_queue.put(data)
            except json.JSONDecodeError as e:
                print(f"Arduino JSON decode error: {e} - Line: '{line}'")
            except Exception as e:
                print(f"Error reading from Arduino: {e}")
        time.sleep(0.001)
    # print("Arduino reader thread finished.") # Reduce noise


def odrive_reader(odrv):
    """Thread function to read data from ODrive and put it in a queue."""
    # print("ODrive reader thread started.") # Reduce noise
    sample_period = 1.0 / SAMPLE_RATE_HZ
    next_sample_time = time.time()

    while collection_active.is_set():
        current_time = time.time()
        if current_time >= next_sample_time:
            try:
                if hasattr(odrv, 'axis0') and odrv.axis0:
                    data = {
                        'sys_time': current_time,
                        'voltage': odrv.vbus_voltage,
                        'current_q': getattr(getattr(getattr(odrv.axis0, 'motor', {}), 'current_control', {}), 'Iq_measured', None),
                        'current_d': getattr(getattr(getattr(odrv.axis0, 'motor', {}), 'current_control', {}), 'Id_measured', None),
                        'velocity': getattr(getattr(odrv.axis0, 'encoder', {}), 'vel_estimate', None),
                        'position': getattr(getattr(odrv.axis0, 'encoder', {}), 'pos_estimate', None),
                        'temperature': getattr(getattr(getattr(odrv.axis0, 'motor', {}), 'fet_thermistor', {}), 'temperature', None),
                        'axis_error': getattr(odrv.axis0, 'error', 0),
                        'motor_error': getattr(getattr(odrv.axis0, 'motor', {}), 'error', 0),
                        'encoder_error': getattr(getattr(odrv.axis0, 'encoder', {}), 'error', 0),
                    }
                    odrive_data_queue.put(data)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"Error reading from ODrive: {e}")

            next_sample_time += sample_period
            if next_sample_time < current_time:
                 next_sample_time = current_time + sample_period
        else:
             sleep_time = next_sample_time - current_time - 0.001
             if sleep_time > 0:
                 time.sleep(sleep_time)
    # print("ODrive reader thread finished.") # Reduce noise


# --- Data Processing (Unchanged) ---
def merge_and_save_data(test_name):
    """Merge data from queues based on timestamps and save to CSV."""
    print("Merging and saving data...")
    arduino_data = []
    odrive_data = []
    while not arduino_data_queue.empty():
        arduino_data.append(arduino_data_queue.get_nowait())
    while not odrive_data_queue.empty():
        odrive_data.append(odrive_data_queue.get_nowait())

    print(f"Processing {len(arduino_data)} Arduino samples and {len(odrive_data)} ODrive samples")
    if not arduino_data and not odrive_data:
        print("No data collected to save.")
        return

    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = CSV_FILENAME_TEMPLATE.format(test_name=test_name, timestamp=timestamp_str)
    fieldnames = ['timestamp', 'arduino_ts', 'accel_x', 'accel_y', 'accel_z',
                  'voltage', 'current_q', 'current_d', 'velocity', 'position',
                  'temperature', 'axis_error', 'motor_error', 'encoder_error']

    merged_data = []
    all_data = sorted(
        [{**d, 'source': 'arduino'} for d in arduino_data] +
        [{**d, 'source': 'odrive'} for d in odrive_data],
        key=lambda x: x['sys_time']
    )

    last_arduino_vals = {'arduino_ts': None, 'accel_x': None, 'accel_y': None, 'accel_z': None}
    last_odrive_vals = {'voltage': None, 'current_q': None, 'current_d': None, 'velocity': None,
                        'position': None, 'temperature': None, 'axis_error': None, 'motor_error': None, 'encoder_error': None}

    for record in all_data:
        if record['source'] == 'arduino':
            last_arduino_vals = {'arduino_ts': record.get('ts'), 'accel_x': record.get('ax'),
                                 'accel_y': record.get('ay'), 'accel_z': record.get('az')}
        elif record['source'] == 'odrive':
            last_odrive_vals = {'voltage': record.get('voltage'), 'current_q': record.get('current_q'),
                                'current_d': record.get('current_d'), 'velocity': record.get('velocity'),
                                'position': record.get('position'), 'temperature': record.get('temperature'),
                                'axis_error': record.get('axis_error'), 'motor_error': record.get('motor_error'),
                                'encoder_error': record.get('encoder_error')}
        merged_row = {'timestamp': record['sys_time'], **last_arduino_vals, **last_odrive_vals}
        merged_data.append(merged_row)

    unique_merged_data = list({d['timestamp']: d for d in merged_data}.values())
    unique_merged_data.sort(key=lambda x: x['timestamp'])

    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(unique_merged_data)
        print(f"Data successfully saved to {filename}")
    except IOError as e:
        print(f"Error writing to CSV file {filename}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred during CSV writing: {e}")


# --- Test Execution (Unchanged logic) ---
def run_test_sequence(odrv, arduino_ser, test_name, velocity_setpoint, duration=DEFAULT_COLLECTION_TIME_SEC):
    """Configures ODrive, runs data collection, and saves results for one test."""
    global collection_active

    if not odrv or not arduino_ser:
        print(f"Skipping test '{test_name}': ODrive or Arduino not connected.")
        return False # Indicate failure

    print(f"\n--- Starting Test: {test_name} ---")
    print(f"Setting velocity to {velocity_setpoint} turns/sec for {duration} seconds.")
    test_successful = False
    try:
        # --- ODrive Configuration for Test ---
        axis = odrv.axis0
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        time.sleep(0.1)

        if axis.error != 0:
             print(f"Axis error {axis.error} detected before starting test. Clearing errors.")
             odrive.dump_errors(odrv, True)
             time.sleep(0.1)
             if axis.error != 0:
                  print("Failed to clear axis errors. Skipping test.")
                  return False

        axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP
        axis.controller.config.vel_ramp_rate = 1
        axis.controller.input_vel = velocity_setpoint

        axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        print("ODrive axis engaging...")
        time.sleep(0.5)

        if axis.current_state != odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"Error: Axis failed to enter closed loop control. State: {axis.current_state}")
            odrive.dump_errors(odrv)
            # Attempt to idle the axis
            try:
                axis.requested_state = odrive.enums.AXIS_STATE_IDLE
            except Exception: pass # Ignore errors during emergency stop
            return False

        print("Axis engaged. Waiting for ramp...")
        ramp_time = 0
        if axis.controller.config.vel_ramp_rate > 0:
             ramp_time = abs(velocity_setpoint / axis.controller.config.vel_ramp_rate)
        time.sleep(ramp_time + 0.5)
        print("Ramp complete. Starting data collection.")

        # --- Start Data Collection ---
        collection_active.set()
        arduino_thread = threading.Thread(target=arduino_reader, args=(arduino_ser,), daemon=True)
        odrive_thread = threading.Thread(target=odrive_reader, args=(odrv,), daemon=True)
        arduino_thread.start()
        odrive_thread.start()
        time.sleep(duration)

        # --- Stop Data Collection ---
        print("Stopping data collection...")
        collection_active.clear()
        time.sleep(0.5)
        print("Data collection complete!")

        # --- Stop Motor ---
        print("Stopping motor...")
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        axis.controller.input_vel = 0
        time.sleep(0.5)
        print("Motor stopped.")

        # --- Process and Save Data ---
        merge_and_save_data(test_name)
        test_successful = True

    except Exception as e:
        print(f"Error during test '{test_name}': {e}")
        try:
            if odrv and hasattr(odrv, 'axis0'):
                odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
                odrv.axis0.controller.input_vel = 0
        except Exception as cleanup_e:
            print(f"Error during cleanup: {cleanup_e}")
    finally:
         collection_active.clear()
         try:
             if odrv and hasattr(odrv, 'axis0') and odrv.axis0.current_state != odrive.enums.AXIS_STATE_IDLE:
                  print("Ensuring ODrive is idle post-test.")
                  odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
         except Exception as final_cleanup_e:
              print(f"Error during final ODrive cleanup: {final_cleanup_e}")

    return test_successful


# --- Main Execution Logic ---

def main():
    """Main function to connect, configure, calibrate, run tests, and cleanup."""
    odrv = None
    arduino_ser = None
    try:
        # Connect to devices
        odrv = connect_to_odrive()
        arduino_ser = connect_to_arduino(ARDUINO_PORT, ARDUINO_BAUD)

        if not odrv or not arduino_ser:
            print("Failed to connect to one or both devices. Exiting.")
            return

        # --- Configure ODrive Parameters ---
        if not configure_odrive_parameters(odrv):
             print("ODrive parameter configuration failed. Exiting.")
             return

        # --- Run ODrive Calibration ---
        print("\n*** WARNING: ODrive calibration will spin the motor briefly. ***")
        print("*** Ensure the wheel is free to rotate without obstruction. ***")
        input("Press Enter to start calibration...")
        if not run_odrive_calibration(odrv):
             print("ODrive calibration failed. Exiting.")
             odrive.dump_errors(odrv)
             return
        print("Calibration successful.")

        # --- Save Configuration ---
        print("Saving ODrive configuration...")
        try:
            odrv.save_configuration()
            print("Configuration saved.")
            # Optional: Reboot for some settings to take effect?
            # print("Rebooting ODrive...")
            # odrv.reboot()
            # time.sleep(5) # Wait for reboot
            # odrv = connect_to_odrive() # Reconnect
            # if not odrv: raise Exception("Failed to reconnect after reboot.")

        except Exception as e:
            print(f"Error saving configuration: {e}")


        # --- Define and Run Test Sequences ---
        print("\n--- Starting Test Sequences ---")

        # T5: Baseline (0N Load)
        print("\n*** Set Load to 0N for Baseline Tests ***")
        input("Press Enter when ready...")
        run_test_sequence(odrv, arduino_ser, "TreadA_0N_LowSpeed", velocity_setpoint=1.0)
        run_test_sequence(odrv, arduino_ser, "TreadA_0N_MedSpeed", velocity_setpoint=2.0)
        run_test_sequence(odrv, arduino_ser, "TreadA_0N_HighSpeed", velocity_setpoint=3.0)

        # T6: Load 1 (40N Load)
        print("\n*** Please set load to 40N before proceeding! ***")
        input("Press Enter to continue...")
        run_test_sequence(odrv, arduino_ser, "TreadA_40N_LowSpeed", velocity_setpoint=1.0)
        run_test_sequence(odrv, arduino_ser, "TreadA_40N_MedSpeed", velocity_setpoint=2.0)
        run_test_sequence(odrv, arduino_ser, "TreadA_40N_HighSpeed", velocity_setpoint=3.0)

        # T7: Load 2 (80N Load)
        print("\n*** Please set load to 80N before proceeding! ***")
        input("Press Enter to continue...")
        run_test_sequence(odrv, arduino_ser, "TreadA_80N_LowSpeed", velocity_setpoint=1.0)
        run_test_sequence(odrv, arduino_ser, "TreadA_80N_MedSpeed", velocity_setpoint=2.0)
        run_test_sequence(odrv, arduino_ser, "TreadA_80N_HighSpeed", velocity_setpoint=3.0)

        # Add more calls for Tread B etc. here

        print("\n--- All Planned Tests Complete ---")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        collection_active.clear()
    except Exception as e:
        print(f"An critical error occurred in the main program: {e}")
        collection_active.clear()
    finally:
        # --- Cleanup ---
        print("Cleaning up resources...")
        collection_active.clear()
        time.sleep(0.5)

        if odrv and hasattr(odrv, 'axis0'):
            try:
                print("Setting ODrive to IDLE state.")
                odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
            except Exception as odrv_cleanup_e:
                 print(f"Error setting ODrive to idle during cleanup: {odrv_cleanup_e}")

        if arduino_ser and arduino_ser.is_open:
            try:
                print("Closing Arduino connection.")
                arduino_ser.close()
            except Exception as ser_cleanup_e:
                print(f"Error closing serial port during cleanup: {ser_cleanup_e}")

        print("Program finished.")


if __name__ == "__main__":
    main()

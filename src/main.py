import odrive
import odrive.enums # Import enums for constants
from odrive.utils import * # Import utils for functions like dump_errors

import serial # Arduino dependency 
import time
import csv
import numpy as np
from datetime import datetime
import threading
import json
import queue # Using queue for thread-safe communication
import math
import os

# --- Configuration ---
ARDUINO_PORT = 'COM7'  # Arduino dependency - adjust as needed
ARDUINO_BAUD = 115200 # Arduino dependency
CSV_FILENAME_TEMPLATE = "data/wheel_test_{test_name}_{timestamp}.csv"
SAMPLE_RATE_HZ = 50

# --- Global Variables for Threading ---
odrive_data_queue = queue.Queue()
arduino_data_queue = queue.Queue()
collection_active = threading.Event()

# Global variables for Arduino serial connection
arduino_ser = None

# --- Helper Functions ---
def connect_to_odrive():
    """Connect to the ODrive."""
    print("Looking for ODrive...")
    try:
        odrv = odrive.find_any(timeout=15)
        print(f"Found ODrive! Serial: {odrv.serial_number}")
        # Check for AXIS 1
        if hasattr(odrv, 'axis1') and odrv.axis1 is not None:
             print("ODrive Axis 1 found.")
        else:
            print("Error: ODrive Axis 1 not found.")
            return None
        # Verify communication
        print(f"Initial VBUS voltage: {odrv.vbus_voltage:.2f}V")
        return odrv
    except Exception as e:
        print(f"Error connecting to ODrive: {e}")
        return None

def clear_all_odrive_errors(odrv):
    """Clear all errors on all axes (axis0 and axis1 if present)."""
    print("Clearing all ODrive errors...")
    try:
        # Use the dump_errors function to report errors first
        dump_errors(odrv)
        
        # Reset error states by setting error to 0
        if hasattr(odrv, 'axis0'):
            try:
                odrv.axis0.error = 0
                odrv.axis0.motor.error = 0
                odrv.axis0.encoder.error = 0
                odrv.axis0.controller.error = 0
            except:
                pass
                
        if hasattr(odrv, 'axis1'):
            try:
                odrv.axis1.error = 0
                odrv.axis1.motor.error = 0
                odrv.axis1.encoder.error = 0
                odrv.axis1.controller.error = 0
            except:
                pass
                
        time.sleep(0.5)
        print("All ODrive errors cleared.")
    except Exception as e:
        print(f"Error clearing ODrive errors: {e}")

# --- ODrive Calibration Function (MODIFIED - Individual Steps) ---
def run_odrive_calibration(odrv):
    """
    Runs the full calibration sequence using the ODrive state machine.
    """
    print("Starting ODrive Full Calibration Sequence...")
    axis = odrv.axis1
    try:
        # Request full calibration sequence
        axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("  Full calibration sequence running... (Motor and encoder will spin)")
        time.sleep(15)  # Give time for calibration to complete
        # Check results
        if axis.error != 0:
            print("Error: Axis error detected after full calibration sequence.")
            dump_errors(odrv)
            axis.requested_state = odrive.enums.AXIS_STATE_IDLE
            return False
        if not axis.motor.is_calibrated or not axis.encoder.is_ready:
            print("Error: Motor or encoder not calibrated/ready after full calibration.")
            print(f"Motor Calibrated: {axis.motor.is_calibrated}, Encoder Ready: {axis.encoder.is_ready}")
            dump_errors(odrv)
            axis.requested_state = odrive.enums.AXIS_STATE_IDLE
            return False
        print("Full calibration sequence complete and successful.")
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        return True
    except Exception as e:
        print(f"Exception during ODrive full calibration: {e}")
        try: dump_errors(odrv)
        except: pass
        try: axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        except: pass
        return False

# --- Data Reader Threads (Unchanged) ---
def odrive_reader(odrv):
    """Thread function to read data from ODrive and put it in a queue."""
    sample_period = 1.0 / SAMPLE_RATE_HZ; next_sample_time = time.time()
    while collection_active.is_set():
        current_time = time.time()
        if current_time >= next_sample_time:
            try:
                local_odrv_ref = odrv
                if hasattr(local_odrv_ref, 'axis1') and local_odrv_ref.axis1:
                    data = {'sys_time': current_time, 'voltage': local_odrv_ref.vbus_voltage,
                            'current_q': getattr(getattr(getattr(local_odrv_ref.axis1, 'motor', {}), 'current_control', {}), 'Iq_measured', None),
                            'current_d': getattr(getattr(getattr(local_odrv_ref.axis1, 'motor', {}), 'current_control', {}), 'Id_measured', None),
                            'velocity': getattr(getattr(local_odrv_ref.axis1, 'encoder', {}), 'vel_estimate', None),
                            'position': getattr(getattr(local_odrv_ref.axis1, 'encoder', {}), 'pos_estimate', None),
                            'temperature': getattr(getattr(getattr(local_odrv_ref.axis1, 'motor', {}), 'fet_thermistor', {}), 'temperature', None),
                            'axis_error': getattr(local_odrv_ref.axis1, 'error', 0),
                            'motor_error': getattr(getattr(local_odrv_ref.axis1, 'motor', {}), 'error', 0),
                            'encoder_error': getattr(getattr(local_odrv_ref.axis1, 'encoder', {}), 'error', 0)}
                    odrive_data_queue.put(data)
                else: time.sleep(0.1)
            except Exception as e:
                print(f"ODrive read error (connection might be lost): {e}")
                time.sleep(0.5)
            next_sample_time += sample_period
            if next_sample_time < current_time: next_sample_time = current_time + sample_period
        else:
             sleep_time = next_sample_time - current_time - 0.001
             if sleep_time > 0: time.sleep(sleep_time)

def arduino_reader(arduino_port, arduino_baud):
    """Thread function to read data from Arduino and put it in a queue."""
    global arduino_ser
    
    # Always close the existing connection first to avoid resource conflicts
    if arduino_ser is not None:
        try:
            arduino_ser.close()
            print("Closed existing Arduino connection")
        except:
            pass
        arduino_ser = None
    
    try:
        # Create a fresh connection for each test
        print(f"Opening Arduino serial port {arduino_port} at {arduino_baud} baud...")
        arduino_ser = serial.Serial(arduino_port, arduino_baud, timeout=0.1)  # Reduced timeout for faster response
        print(f"Arduino serial port opened successfully")
        arduino_ser.flushInput()
        
        # Try reading some initial data to verify connection
        print("Checking Arduino data...")
        initial_check_time = time.time()
        initial_data_found = False
        
        # Try for up to 1 second to get initial data
        while time.time() - initial_check_time < 1.0 and not initial_data_found:
            if arduino_ser.in_waiting > 0:
                try:
                    test_data = arduino_ser.readline().decode('utf-8').strip()
                    if test_data:
                        print(f"Arduino connection verified with data: {test_data[:30]}...")
                        initial_data_found = True
                except:
                    pass
            time.sleep(0.01)
            
        if not initial_data_found:
            print("WARNING: No initial Arduino data detected!")
        
        # Read loop
        data_count = 0
        last_report_time = time.time()
        empty_read_count = 0
        
        while collection_active.is_set():
            try:
                # Check if there's data waiting
                if arduino_ser.in_waiting > 0:
                    # Read a line from the Arduino
                    raw_data = arduino_ser.readline()
                    
                    # Try to decode it
                    if raw_data:
                        data = raw_data.decode('utf-8').strip()
                        if data:
                            # Debug: print first few samples
                            if data_count < 5:
                                print(f"Arduino data sample {data_count+1}: {data[:50]}...")
                            
                            # Add to queue
                            arduino_data_queue.put(data)
                            data_count += 1
                            empty_read_count = 0
                            
                            # Periodically report count
                            current_time = time.time()
                            if current_time - last_report_time > 5.0:
                                print(f"Arduino data collected: {data_count} samples")
                                last_report_time = current_time
                else:
                    # Keep track of consecutive empty reads
                    empty_read_count += 1
                    if empty_read_count >= 100 and empty_read_count % 500 == 0:
                        print(f"Warning: {empty_read_count} consecutive empty reads from Arduino")
                    time.sleep(0.01)  # Short sleep when no data
                        
            except UnicodeDecodeError:
                print(f"Arduino decode error - received binary data: {raw_data}")
            except Exception as e:
                print(f"Arduino read error: {e}")
                time.sleep(0.01)  # Reduced sleep time to avoid missing data
        
        print(f"Arduino data collection complete: {data_count} total samples")
        
    except serial.SerialException as e:
        print(f"ERROR: Could not open Arduino serial port {arduino_port}: {e}")
        print("Check Arduino connection and COM port settings")
    except Exception as e:
        print(f"Unexpected error in Arduino reader: {e}")

# --- Data Processing ---
def merge_and_save_data(test_name):
    """
    Saves both ODrive and Arduino data to a single CSV file.
    Data from both sources will be merged based on timestamps.
    """
    print("Saving test data...")
    
    # Get ODrive data from queue
    odrive_data = []
    while not odrive_data_queue.empty():
        odrive_data.append(odrive_data_queue.get_nowait())
    print(f"Processing {len(odrive_data)} ODrive samples")
    
    # Get Arduino data from queue
    arduino_data = []
    while not arduino_data_queue.empty():
        try:
            json_str = arduino_data_queue.get_nowait()
            try:
                data = json.loads(json_str)
                # Add the source identifier
                data['source'] = 'arduino'
                arduino_data.append(data)
            except json.JSONDecodeError as e:
                print(f"Warning: Failed to parse Arduino JSON data: {e}")
        except Exception as e:
            print(f"Warning: Error retrieving Arduino data: {e}")
    print(f"Processing {len(arduino_data)} Arduino samples")
    
    if not odrive_data and not arduino_data:
        print("No data collected.")
        return
    
    # Ensure data directory exists
    if not os.path.exists("data"):
        os.makedirs("data")
    
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = CSV_FILENAME_TEMPLATE.format(test_name=test_name, timestamp=timestamp_str)
    
    # Define all possible fields for the combined CSV
    odrive_fields = ['timestamp', 'voltage', 'current_q', 'current_d', 'velocity', 'position',
                     'temperature', 'axis_error', 'motor_error', 'encoder_error']
    
    arduino_fields = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'roll', 'pitch', 'yaw', 'source']
    
    # Build a merged dataset
    merged_data = []
    
    # Add ODrive data to merged set with source identifier
    for item in odrive_data:
        record = {field: '' for field in odrive_fields + arduino_fields}  # Initialize with empty values
        for field in odrive_fields:
            if field in item:
                record[field] = item[field]
        record['source'] = 'odrive'
        merged_data.append(record)
    
    # Add Arduino data to merged set
    for item in arduino_data:
        record = {field: '' for field in odrive_fields + arduino_fields}  # Initialize with empty values
        for field in arduino_fields:
            if field in item:
                record[field] = item[field]
        # Convert Arduino timestamp to match format if needed
        if 'ts' in item:
            record['timestamp'] = item['ts'] / 1000.0  # Convert ms to seconds
        merged_data.append(record)
    
    # Sort the merged data by timestamp
    merged_data.sort(key=lambda x: float(x['timestamp']) if x['timestamp'] != '' else float('inf'))
    
    # Write to CSV
    all_fields = odrive_fields + [f for f in arduino_fields if f not in odrive_fields]
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=all_fields)
            writer.writeheader()
            for row in merged_data:
                writer.writerow(row)
        print(f"Data successfully saved to {filename}")
    except Exception as e:
        print(f"Error writing CSV file: {e}")

# --- Test Execution Function (Modified) ---
def run_distance_test(odrv, test_name, target_velocity_rps, radius_cm, target_distance_m=0.80):
    """
    Runs a test using position control to travel a specific linear distance.
    Uses ODrive's built-in trajectory planning for proper acceleration/deceleration.
    
    Args:
        odrv: ODrive object
        test_name: Name for the test (used in data file)
        target_velocity_rps: Target velocity in revolutions per second
        radius_cm: Wheel radius in cm
        target_distance_m: Target distance to travel in meters (default 0.80m)
    
    Returns:
        True if test completed successfully, False otherwise
    """
    # Convert radius from cm to meters for calculations
    radius_m = radius_cm / 100.0
    
    # Calculate circumference (in meters per revolution)
    circumference_m = 2 * math.pi * radius_m
    
    # Calculate target revolutions based on wheel radius and distance
    target_revolutions = target_distance_m / circumference_m
    
    # Access axis directly
    if not hasattr(odrv, 'axis1'):
        print("Error: ODrive Axis 1 not found.")
        return False
    axis = odrv.axis1
    
    # Validate calibration
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        print("Error: Motor/encoder not calibrated!")
        print(f"Motor Calibrated: {axis.motor.is_calibrated}, Encoder Ready: {axis.encoder.is_ready}")
        return False
        
    # Start position first...
    start_pos_turns = axis.encoder.pos_estimate
    target_end_pos = start_pos_turns + target_revolutions  # Add the revolutions (distance)
    
    # Make sure existing errors are cleared
    if axis.error != 0:
        print(f"Clearing Axis 1 error state: {axis.error}")
        axis.error = 0
        
    # Print test parameters
    print(f"\n--- Starting Distance Test: {test_name} ---")
    print(f"Target Dist: {target_distance_m*100:.0f} cm, Radius: {radius_cm:.2f} cm, Target Vel: {target_velocity_rps:.2f} turns/sec")
    print(f"Calculated: Circumference={circumference_m:.3f} m, Target Revolutions={target_revolutions:.3f} turns")
    
    # Approximate time estimate (rough, not accounting for acceleration/deceleration)
    estimated_time = target_revolutions / target_velocity_rps
    print(f"Estimated move time: {estimated_time:.2f} seconds (approximate)")
    
    # Configure for position control with trajectory
    print("Configuring ODrive for position control with trajectory planning...")
    
    # Keep track of current state (for comparison)
    print("\n--- Current ODrive Configuration ---")
    print(f"Control Mode: {axis.controller.config.control_mode}")
    print(f"Input Mode: {axis.controller.config.input_mode}")
    print(f"Starting Position: {start_pos_turns:.3f} turns")
    print(f"Target Position: {target_end_pos:.3f} turns")
    
    # Configure controller for proper POSITION control with trajectory planning
    axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = odrive.enums.INPUT_MODE_TRAP_TRAJ  # Critical for proper trajectory
    print(f"Set control mode to POSITION_CONTROL with TRAP_TRAJ input mode")
    
    # Get configuration parameters (without verbose comparison)
    existing_vel_limit = axis.trap_traj.config.vel_limit
    existing_accel_limit = axis.trap_traj.config.accel_limit
    existing_decel_limit = axis.trap_traj.config.decel_limit
    
    # Only update velocity limit if the existing one is too low for our target
    if existing_vel_limit < target_velocity_rps:
        print(f"Setting velocity limit to match target: {target_velocity_rps:.2f} turns/sec")
        axis.trap_traj.config.vel_limit = target_velocity_rps
    else:
        print(f"Using velocity limit: {existing_vel_limit:.2f} turns/sec")
        
    # Use whichever is smaller: the target velocity or the configured limit
    effective_velocity = min(target_velocity_rps, axis.trap_traj.config.vel_limit)
    
    # Re-estimate move time based on the effective velocity and acceleration limits
    estimated_move_time = target_revolutions / effective_velocity + effective_velocity / existing_accel_limit
    print(f"Estimated move time: {estimated_move_time:.2f} seconds (using configured limits)")
    
    # Global synchronization lock
    global movement_lock
    if not hasattr(run_distance_test, 'movement_lock'):
        run_distance_test.movement_lock = threading.Lock()
    
    # Make sure collection is not active from previous tests
    collection_active.clear()
    
    # Ensure any existing threads are stopped
    time.sleep(0.2)
    
    # Acquire lock before starting the movement sequence
    with run_distance_test.movement_lock:
        # Start Data Collection
        print("Starting ODrive data collection thread...")
        collection_active.set()
        odrive_thread = threading.Thread(target=odrive_reader, args=(odrv,), daemon=True)
        odrive_thread.start()
        time.sleep(0.1)
        
        # Start Arduino data collection thread
        print("Starting Arduino data collection thread...")
        arduino_thread = threading.Thread(target=arduino_reader, args=(ARDUINO_PORT, ARDUINO_BAUD), daemon=True)
        arduino_thread.start()
        time.sleep(0.1)
        
        # Wait for both threads to start
        time.sleep(0.5)
        
        # First enable closed loop control
        print("\n--- Initiating Movement ---")
        axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        print("Enabling closed loop control...")
        time.sleep(0.2)
        
        if axis.current_state != odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"Error: Failed to enter closed loop control. Current state: {axis.current_state}")
            dump_errors(odrv)
            collection_active.clear()
            return False
        
        # THEN set position command to start movement
        print(f"Commanding position move to {target_end_pos:.3f} turns...")
        axis.controller.input_pos = target_end_pos
        
        # Wait for move to complete
        print("Waiting for trajectory completion...")
        start_time = time.time()
        move_complete = False
        
        # Maximum wait time is estimated time plus 50% margin
        timeout = estimated_move_time * 1.5
        
        # Monitor the movement
        while time.time() - start_time < timeout:
            current_pos = axis.encoder.pos_estimate
            current_velocity = axis.encoder.vel_estimate
            position_error = abs(current_pos - target_end_pos)
            distance_traveled = abs(current_pos - start_pos_turns)
            elapsed = time.time() - start_time
            
            # Check for errors during move
            if axis.error != 0:
                print(f"\nAxis error {axis.error} during move! Stopping.") 
                dump_errors(odrv)
                break
                
            # Check if we've reached target position (within tolerance)
            if position_error < 0.05 and abs(current_velocity) < 0.1:  # Within 0.05 turns and nearly stopped
                print(f"\nReached target position at {current_pos:.3f} turns")
                move_complete = True
                break
                
            # Print status every 0.5 seconds, overwriting previous line
            if elapsed % 0.5 < 0.1:
                progress = (distance_traveled / target_revolutions) * 100
                remaining = max(0, estimated_move_time - elapsed)
                print(f"\rTime: {elapsed:.1f}/{estimated_move_time:.1f}s Pos: {current_pos:.3f} Vel: {current_velocity:.2f} Dist: {distance_traveled:.3f}/{target_revolutions:.3f} Progress: {progress:.1f}%", end="")
                
            time.sleep(0.1)
            
        print("")  # New line after progress reporting
        
        # Final position report
        final_pos = axis.encoder.pos_estimate
        final_distance = abs(final_pos - start_pos_turns)
        print(f"\nMove complete. Final position: {final_pos:.3f} turns")
        print(f"Distance traveled: {final_distance:.3f} turns of {target_revolutions:.3f} target")
        
        # Check for tolerance
        distance_error_pct = abs(final_distance - target_revolutions) / target_revolutions * 100
        if distance_error_pct <= 10.0:  # Within 10% of target
            print(f"Successfully reached target position (within 10% tolerance)")
        else:
            print(f"Warning: Final position error of {distance_error_pct:.1f}% exceeds 10% tolerance")
    
    # Always stop data collection and return motor to IDLE
    print("Stopping data collection...")
    collection_active.clear()
    time.sleep(0.2)  # Make sure data collection thread completes
    print("Data collection complete!")
    
    print("Setting ODrive to IDLE state...")
    try:
        axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        print("Motor stopped.")
    except Exception as e:
        print(f"Error setting ODrive to IDLE: {e}")
    
    # Save the data using the helper function
    merge_and_save_data(test_name)
    
    return True

# --- Main Execution Logic (Using Step-by-Step Calibration) ---
def main():
    """Main function to connect, configure, calibrate, run ODrive tests, and cleanup."""
    global arduino_ser
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        print("Failed to connect to ODrive. Exiting.")
        return
    
    # Try to connect to Arduino early
    print("Initializing Arduino connection...")
    try:
        arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        print(f"Arduino connected on {ARDUINO_PORT}")
        # Flush any initial data
        arduino_ser.flushInput()
    except Exception as e:
        print(f"Warning: Could not initialize Arduino: {e}")
        print("Will try again during test")
    
    # Clear any existing errors
    clear_all_odrive_errors(odrv)
    
    # Run Calibration
    print("\n*** WARNING: ODrive calibration will spin the motor. ***")
    print("*** Ensure the wheel is free to rotate without obstruction. ***")
    input("Press Enter to start calibration...")
    
    calibration_success = run_odrive_calibration(odrv)
    if calibration_success:
        print("Calibration successful.")
    else:
        print("Calibration failed. Please check ODrive connection and try again.")
        return
    
    # Interactive Test Sequence
    print("\n--- Starting Interactive Distance Test Sequence (ODrive Only) ---")
    
    while True:
        # Verify ODrive is still responsive
        print("\nVerifying ODrive connection before next test...")
        if not hasattr(odrv, 'axis1') or not odrv.axis1.motor.is_calibrated:
            print("ODrive connection lost or motor needs recalibration.")
            return
        else:
            print("Connection OK and Axis 1 calibrated.")
            
        # Get test parameters from user
        print("\nEnter test parameters (or type 'quit' for test name to exit):")
        test_name = input("Enter a unique name for this test run: ")
        if test_name.lower() == 'quit':
            break
            
        target_velocity = input("Enter target velocity during move (turns/sec, e.g., 1.0): ")
        try:
            target_velocity = float(target_velocity)
        except ValueError:
            print("Invalid velocity. Using default of 1.0 turns/sec")
            target_velocity = 1.0
            
        wheel_radius = input("Enter wheel radius (cm): ")
        try:
            wheel_radius = float(wheel_radius)
            if wheel_radius <= 0:
                raise ValueError("Radius must be positive")
        except ValueError:
            print("Invalid radius. Using default of 5.0 cm")
            wheel_radius = 5.0
            
        # Display ready message and wait for final confirmation
        print("\n*** Ensure the correct vertical load is applied to the wheel! ***")
        input("Press Enter when ready to start the test...")
        
        # Run the test
        run_distance_test(odrv, test_name, target_velocity, wheel_radius)
    
    # Test sequence complete
    print("\n--- Test Sequence Finished ---")
    print("Cleaning up resources...")
    
    # Clean shutdown
    try:
        # Put ODrive in idle state
        print("Setting ODrive Axis 1 to IDLE state.")
        odrv.axis1.requested_state = odrive.enums.AXIS_STATE_IDLE
        
        # Close Arduino serial port if open
        if arduino_ser is not None:
            print("Closing Arduino serial connection.")
            arduino_ser.close()
            arduino_ser = None
            
    except Exception as e:
        print(f"Error during cleanup: {e}")
    
    print("Program finished.")

if __name__ == "__main__":
    main()

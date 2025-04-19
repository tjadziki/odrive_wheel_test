"""
Arduino debug script with raw serial access.

This script focuses exclusively on reading data from the Arduino
with minimal overhead to troubleshoot connection issues.
"""

import serial
import time
import json
import sys

def debug_arduino(port='COM7', baud=115200, duration=30):
    """
    Debug Arduino serial communication using the lowest level access.
    
    Args:
        port: Serial port name (e.g., 'COM7')
        baud: Baud rate (e.g., 115200)
        duration: Test duration in seconds
    """
    print(f"Debugging Arduino on {port} at {baud} baud for {duration} seconds...")
    
    # Close all possible prior connections to this port
    print("Attempting to close any existing connections...")
    try:
        test_ser = serial.Serial(port, baud)
        test_ser.close()
        print("Closed existing connection")
    except:
        print("No existing connection found")
    
    # Wait a moment to ensure port is fully released
    time.sleep(1.0)
    
    try:
        # Open the serial port with aggressive timeouts
        print(f"Opening {port} with raw settings...")
        ser = serial.Serial(
            port=port, 
            baudrate=baud, 
            timeout=0.1,           # Short timeout for frequent checking
            write_timeout=0.5,     # Don't hang on writes
            inter_byte_timeout=None,
            exclusive=True         # Ensure exclusive access
        )
        
        print(f"Port opened successfully: {ser}")
        print(f"Settings: {ser.get_settings()}")
        
        # Flush buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Read data for specified duration
        print(f"\nReading RAW data for {duration} seconds...")
        print("-" * 60)
        
        data_count = 0
        bytes_count = 0
        json_count = 0
        errors = 0
        
        start_time = time.time()
        last_status_time = start_time
        
        while time.time() - start_time < duration:
            try:
                # Check bytes waiting
                waiting = ser.in_waiting
                if waiting > 0:
                    # Read raw data
                    raw_data = ser.read(waiting)
                    bytes_count += len(raw_data)
                    
                    # Try to decode
                    try:
                        # Try to find complete lines
                        text = raw_data.decode('utf-8')
                        lines = text.strip().split('\n')
                        for line in lines:
                            if line.strip():
                                data_count += 1
                                print(f"[{data_count}] RAW: {line[:100]}")
                                
                                # Try to parse JSON
                                try:
                                    data = json.loads(line)
                                    json_count += 1
                                    print(f"  ✓ Valid JSON: {json_count} total")
                                    
                                    # Check for expected fields
                                    if all(k in data for k in ['ax', 'ay', 'az']):
                                        print(f"  ✓ IMU data: ax={data['ax']:.3f}, ay={data['ay']:.3f}, az={data['az']:.3f}")
                                    else:
                                        print(f"  ✗ Missing IMU fields: {[k for k in ['ax','ay','az'] if k not in data]}")
                                except json.JSONDecodeError:
                                    print(f"  ✗ Not valid JSON: {line[:30]}...")
                    except UnicodeDecodeError:
                        print(f"Binary data: {raw_data}")
                
                # Status update every 3 seconds
                current_time = time.time()
                if current_time - last_status_time >= 3:
                    elapsed = current_time - start_time
                    print(f"\n--- STATUS after {elapsed:.1f}s ---")
                    print(f"Total bytes: {bytes_count}, Lines: {data_count}, Valid JSON: {json_count}, Errors: {errors}")
                    print("-" * 30)
                    last_status_time = current_time
                    
                # Small delay between reads
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("\nUser interrupted")
                break
            except Exception as e:
                print(f"ERROR: {type(e).__name__}: {e}")
                errors += 1
                time.sleep(0.1)  # Longer delay after error
        
        # Close the port
        ser.close()
        print("Serial port closed")
        
        # Final report
        elapsed = time.time() - start_time
        print(f"\n=== FINAL REPORT after {elapsed:.1f}s ===")
        print(f"Total bytes: {bytes_count}")
        print(f"Total lines: {data_count}")
        print(f"Valid JSON: {json_count}")
        print(f"Errors: {errors}")
        
        if json_count == 0:
            print("\nTROUBLESHOOTING:")
            print("1. Verify Arduino has accelerometer sketch uploaded and running")
            print("2. Ensure no other program (Arduino IDE Serial Monitor) is using the port")
            print("3. Try power cycling the Arduino")
            print("4. Check physical connections")
            print(f"5. Try a different COM port (currently using {port})")
        
    except serial.SerialException as e:
        print(f"SERIAL ERROR: {e}")
        print("\nPossible solutions:")
        print("1. Check if port is correct (use Device Manager)")
        print("2. Close Arduino IDE Serial Monitor if open")
        print("3. Unplug and reconnect Arduino")
        print("4. Try running as administrator")
    except Exception as e:
        print(f"GENERAL ERROR: {type(e).__name__}: {e}")

if __name__ == "__main__":
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    
    # Run with default settings
    debug_arduino(port=port)

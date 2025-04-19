"""
Simple Arduino serial test script.

This script tests communication with the Arduino by opening the serial port
and reading data for a specified period. It helps diagnose problems with
Arduino connectivity in the main test program.
"""

import serial
import time
import json
import sys

def test_arduino(port='COM7', baud=115200, duration=20):
    """
    Test Arduino serial communication.
    
    Args:
        port: Serial port name (e.g., 'COM7')
        baud: Baud rate (e.g., 115200)
        duration: Test duration in seconds
    """
    print(f"Testing Arduino on {port} at {baud} baud for {duration} seconds...")
    
    try:
        # Open the serial port
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Port opened successfully")
        
        # Wait for Arduino to reset
        print("Waiting for Arduino to initialize...")
        time.sleep(2)
        
        # Flush any initial data
        ser.flushInput()
        
        # Read data for specified duration
        print(f"\nReading data for {duration} seconds...")
        print("-" * 60)
        
        valid_json_count = 0
        invalid_data_count = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            try:
                # Read a line
                data = ser.readline()
                
                if data:
                    # Try to decode as UTF-8
                    data_str = data.decode('utf-8').strip()
                    
                    if data_str:
                        print(f"DATA: {data_str[:80]}...")
                        
                        # Try to parse as JSON
                        try:
                            json_data = json.loads(data_str)
                            valid_json_count += 1
                            
                            # Check required IMU fields
                            if all(field in json_data for field in ['ax', 'ay', 'az', 'roll', 'pitch', 'yaw']):
                                print(f"✓ VALID JSON with IMU data: ax={json_data['ax']:.2f}, ay={json_data['ay']:.2f}, az={json_data['az']:.2f}")
                                print(f"  Orientation: roll={json_data['roll']:.2f}, pitch={json_data['pitch']:.2f}, yaw={json_data['yaw']:.2f}")
                            else:
                                missing = [f for f in ['ax', 'ay', 'az', 'roll', 'pitch', 'yaw'] if f not in json_data]
                                print(f"✗ JSON missing fields: {missing}")
                        except json.JSONDecodeError:
                            print(f"✗ NOT VALID JSON")
                            invalid_data_count += 1
                    
            except UnicodeDecodeError:
                print(f"✗ BINARY DATA: {data}")
                invalid_data_count += 1
            except Exception as e:
                print(f"ERROR: {e}")
            
            # Small delay
            time.sleep(0.1)
        
        # Close the port
        ser.close()
        
        # Print summary
        print("-" * 60)
        print(f"Test complete. Summary:")
        print(f"- Valid JSON messages: {valid_json_count}")
        print(f"- Invalid data received: {invalid_data_count}")
        
        if valid_json_count == 0:
            print("\nTROUBLESHOOTING TIPS:")
            print("1. Verify Arduino has the correct sketch uploaded")
            print("2. Check if Arduino Serial Monitor is open (close it!)")
            print("3. Make sure the Arduino has power (LED should be on)")
            print("4. Try unplugging and reconnecting the Arduino")
            print("5. Upload the sketch again in the Arduino IDE")
        
    except serial.SerialException as e:
        print(f"ERROR: Could not open port {port}: {e}")
        print("\nTROUBLESHOOTING TIPS:")
        print(f"1. Make sure Arduino is connected to {port}")
        print("2. Check if another program is using the port")
        print("3. Try a different USB port")
        print("4. Verify port in Device Manager (Windows)")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    
    # Run the test
    test_arduino(port=port)

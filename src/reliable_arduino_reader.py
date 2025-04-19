"""
Reliable Arduino IMU data reader with reconnect functionality.

This script implements best practices for Arduino serial communication:
1. Proper port opening with multiple retry attempts
2. Non-blocking reads with appropriate timeouts
3. Clear error handling and recovery
4. Thread-safe implementation
"""

import serial
import time
import json
import threading
import queue
import os
from datetime import datetime

# Configuration
ARDUINO_PORT = "COM7"  # Default, can be changed via command line
ARDUINO_BAUD = 115200
MAX_CONNECT_ATTEMPTS = 5
RECONNECT_DELAY = 1.0  # seconds
READ_TIMEOUT = 0.1  # seconds

# Global variables
data_queue = queue.Queue()
running = threading.Event()
sample_count = 0
last_error_time = 0

def create_arduino_connection(port, baud, attempt=1):
    """Create a serial connection to Arduino with retry logic."""
    try:
        print(f"Connecting to Arduino on {port} (attempt {attempt}/{MAX_CONNECT_ATTEMPTS})...")
        
        # Close any existing connections first
        try:
            test_ser = serial.Serial(port)
            test_ser.close()
            print("Closed existing connection")
            time.sleep(0.5)  # Wait for port to release
        except:
            pass
        
        # Create new connection
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=READ_TIMEOUT,
            write_timeout=1.0,
            exclusive=True  # Request exclusive access
        )
        
        # Reset buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print(f"Connected to Arduino: {ser}")
        return ser
        
    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
        if attempt < MAX_CONNECT_ATTEMPTS:
            print(f"Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
            return create_arduino_connection(port, baud, attempt + 1)
        else:
            print("Maximum connection attempts reached. Could not connect to Arduino.")
            return None

def arduino_reader_thread(port, baud):
    """Thread function to continuously read from Arduino."""
    global sample_count, last_error_time
    
    ser = create_arduino_connection(port, baud)
    if not ser:
        print("Failed to create Arduino connection. Thread exiting.")
        return
    
    reconnect_count = 0
    last_data_time = time.time()
    data_timeout = 5.0  # seconds before assuming connection is dead
    
    print("Arduino reader thread started")
    
    while running.is_set():
        try:
            # Check if there's data
            if ser.in_waiting > 0:
                # Read data
                line = ser.readline()
                last_data_time = time.time()
                
                # Try to decode and parse
                try:
                    data_str = line.decode('utf-8').strip()
                    if data_str:
                        # Show first few samples
                        if sample_count < 5:
                            print(f"Sample {sample_count+1}: {data_str[:60]}...")
                        
                        # Try to parse JSON
                        try:
                            # Parse JSON and add timestamp
                            data = json.loads(data_str)
                            data['received_time'] = time.time()
                            data_queue.put(data)
                            sample_count += 1
                            
                            # Log status periodically
                            if sample_count % 100 == 0:
                                print(f"Received {sample_count} total samples")
                        except json.JSONDecodeError:
                            print(f"Invalid JSON: {data_str[:30]}...")
                except UnicodeDecodeError:
                    print(f"Binary data received: {line[:20]}")
            else:
                # Check for connection timeout
                if time.time() - last_data_time > data_timeout:
                    print(f"No data received for {data_timeout} seconds, attempting reconnect...")
                    ser.close()
                    ser = create_arduino_connection(port, baud)
                    if not ser:
                        print("Reconnection failed. Thread exiting.")
                        return
                    reconnect_count += 1
                    last_data_time = time.time()
                    print(f"Reconnected (attempt {reconnect_count})")
                
                # Short sleep when no data
                time.sleep(0.01)
                
        except serial.SerialException as e:
            # Only log errors periodically to avoid flooding console
            current_time = time.time()
            if current_time - last_error_time > 5.0:
                print(f"Serial error: {e}")
                last_error_time = current_time
            
            # Try to reconnect
            try:
                ser.close()
            except:
                pass
            
            time.sleep(RECONNECT_DELAY)
            ser = create_arduino_connection(port, baud)
            if not ser:
                print("Reconnection failed after error. Thread exiting.")
                return
        
        except Exception as e:
            # Handle other exceptions
            print(f"Unexpected error: {type(e).__name__}: {e}")
            time.sleep(0.1)
    
    # Clean up when thread exits
    print("Arduino reader thread stopping")
    try:
        ser.close()
        print("Serial port closed")
    except:
        pass

def main():
    """Main function to start reading and save data."""
    global running, sample_count
    
    # Ensure data directory exists
    if not os.path.exists("data"):
        os.makedirs("data")
    
    # Start running
    running.set()
    
    # Create and start reader thread
    reader = threading.Thread(
        target=arduino_reader_thread,
        args=(ARDUINO_PORT, ARDUINO_BAUD)
    )
    reader.daemon = True
    reader.start()
    
    print(f"Started Arduino reader on {ARDUINO_PORT} at {ARDUINO_BAUD} baud")
    print("Press Ctrl+C to stop")
    
    # Prepare for data collection
    collect_duration = 10  # seconds
    start_time = time.time()
    all_data = []
    
    try:
        # Collect data for specified duration
        print(f"Collecting data for {collect_duration} seconds...")
        while time.time() - start_time < collect_duration:
            # Get data from queue
            try:
                data = data_queue.get(timeout=0.5)
                all_data.append(data)
            except queue.Empty:
                pass
        
        # Stop collection
        running.clear()
        reader.join(timeout=2.0)
        
        # Report results
        print(f"Collection complete. Received {sample_count} samples.")
        
        # Save data if we got any
        if all_data:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"data/arduino_test_{timestamp}.json"
            
            # Save as JSON
            with open(filename, 'w') as f:
                json.dump(all_data, f, indent=2)
            
            print(f"Saved {len(all_data)} samples to {filename}")
        else:
            print("No data collected")
        
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        # Ensure thread stops
        running.clear()
        if reader.is_alive():
            reader.join(timeout=2.0)
            
        print("Test complete")

if __name__ == "__main__":
    import sys
    
    # Override port if specified
    if len(sys.argv) > 1:
        ARDUINO_PORT = sys.argv[1]
    
    main()

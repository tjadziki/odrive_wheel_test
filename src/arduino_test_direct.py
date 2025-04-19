"""
Direct Arduino test with raw byte-level diagnostics.
"""

import serial
import time
import binascii
import sys

def test_arduino_direct(port='COM7'):
    """
    Test Arduino communication at the lowest level possible.
    """
    print(f"Testing Arduino on {port} with direct access method")
    print("-" * 60)
    
    # Try to close port first if it's open
    try:
        ser = serial.Serial(port)
        ser.close()
        print(f"Closed existing port connection")
        time.sleep(1)  # Wait for port to release
    except:
        pass
    
    try:
        # Open with minimal configuration
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"Port opened: {ser}")
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("Starting continuous read test - Press Ctrl+C to stop")
        print("\nRAW DATA:")
        print("-" * 60)
        
        # Read continuously for 30 seconds
        start_time = time.time()
        bytes_received = 0
        lines_received = 0
        
        while time.time() - start_time < 30:
            # Get any available bytes
            available = ser.in_waiting
            if available > 0:
                # Read raw bytes
                data = ser.read(available)
                bytes_received += len(data)
                
                # Print both hex and decoded attempt
                hex_data = binascii.hexlify(data).decode('ascii')
                print(f"HEX: {hex_data[:60]}")
                
                try:
                    # Try to decode as text
                    text_data = data.decode('utf-8')
                    lines = text_data.strip().split('\n')
                    for line in lines:
                        if line.strip():
                            lines_received += 1
                            print(f"TEXT [{lines_received}]: {line[:80]}")
                except UnicodeDecodeError:
                    print(f"Cannot decode as utf-8")
                
                # Status update
                elapsed = time.time() - start_time
                print(f"STATUS: {bytes_received} bytes, {lines_received} lines after {elapsed:.1f}s")
                print("-" * 30)
            
            # Small delay
            time.sleep(0.1)
        
        # Close when done
        ser.close()
        print(f"Test complete - port closed")
        
    except KeyboardInterrupt:
        print("\nUser interrupted test")
        try:
            ser.close()
        except:
            pass
    except Exception as e:
        print(f"ERROR: {type(e).__name__}: {e}")
        
        # Connection troubleshooting tips
        print("\nTROUBLESHOOTING STEPS:")
        print("1. Is the Arduino powered? Check if the power LED is on")
        print("2. Try a different USB port")
        print("3. In Device Manager, check what COM port the Arduino is actually using")
        print("4. Ensure the Arduino sketch is sending data at 115200 baud")
        print("5. Try completely unplugging the Arduino, waiting 10 seconds, then reconnecting")
        print("6. Upload the accelerationData.ino sketch again using Arduino IDE")
        print("7. Try a simple test sketch that just prints 'hello' every second")

if __name__ == "__main__":
    # Get port from command line args or use default
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    test_arduino_direct(port)

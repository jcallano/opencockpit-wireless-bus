import serial
import time
import sys

def capture_serial(port='COM3', baudrate=115200, duration=10):
    try:
        print(f"Opening {port} at {baudrate}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        
        # Reset Sequence (DTR/RTS toggle for ESP32)
        print("Resetting board...")
        ser.dtr = False
        ser.rts = False
        time.sleep(0.1)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
        ser.dtr = False
        ser.rts = False
        
        print(f"Capturing for {duration} seconds...")
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                # Print hex for SLIP analysis and ascii for text
                try:
                    # different output for better visibility
                    for byte in data:
                        if 0x20 <= byte <= 0x7E or byte == 0x0A or byte == 0x0D:
                            sys.stdout.write(chr(byte))
                        else:
                            sys.stdout.write(f"[{byte:02X}]")
                    sys.stdout.flush()
                except Exception as e:
                    print(f"Error decoding: {e}")
            time.sleep(0.01)
            
        ser.close()
        print("\nCapture complete.")
        
    except Exception as e:
        print(f"\nError: {e}")

if __name__ == "__main__":
    capture_serial()

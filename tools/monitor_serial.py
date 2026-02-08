import serial
import time
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python monitor_serial.py <COM_PORT>")
        return

    port = sys.argv[1]
    print(f"Opening {port} at 115200...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        start = time.time()
        while time.time() - start < 10:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[SERIAL] {line}")
            time.sleep(0.01)
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

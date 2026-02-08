import serial
import time
import sys

def dump_raw(port, baud=115200, duration=5):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.dtr = True
        ser.rts = True
        print(f"Dumping raw data from {port} for {duration}s...")
        start = time.time()
        while time.time() - start < duration:
            data = ser.read(1024)
            if data:
                print(data.hex(' '))
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    dump_raw(port)

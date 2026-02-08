import serial
import sys
import time

port = "COM19"
try:
    print(f"Opening {port}...")
    s = serial.Serial(port, 115200, timeout=1)
    print("Opened!")
    time.sleep(1)
    s.close()
    print("Closed.")
except Exception as e:
    print(f"FAIL: {e}")

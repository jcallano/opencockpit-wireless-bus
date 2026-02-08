import serial
import time
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description='Direct Serial Tester for Ursa Minor (Node D Mode)')
    parser.add_argument('port', help='COM Port of Node D (e.g. COM6)')
    args = parser.parse_args()
    
    try:
        ser = serial.Serial(args.port, 115200, timeout=1)
        print(f"Opened {args.port}. Waiting for boot...")
        time.sleep(2)
        
        while True:
            # Check for input from device
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"DEVICE: {line}")
            
            print("\n=== DIRECT COMMANDS ===")
            print("1. Fire Test (Toggles IDs 0x03-0x06)")
            print("2. LCD Test")
            print("3. Vib Test")
            print("4. Manual Command (e.g. 'LED 4 255')")
            print("Q. Quit")
            
            cmd = input("Select: ").upper().strip()
            
            if cmd == 'Q':
                break
            elif cmd == '1':
                print("Testing LEDs...")
                for i in range(3, 7):
                    cmd_str = f"LED {i} 255\n"
                    print(f"Sending: {cmd_str.strip()}")
                    ser.write(cmd_str.encode())
                    time.sleep(0.5)
                    cmd_str = f"LED {i} 0\n"
                    ser.write(cmd_str.encode())
            elif cmd == '2':
                print("Testing LCD...")
                cmds = ["LCD L 8.8", "LCD R 8.8", "LCD L 1.2", "LCD R 3.4"]
                for c in cmds:
                    print(f"Sending: {c}")
                    ser.write((c + "\n").encode())
                    time.sleep(0.5)
            elif cmd == '3':
                print("Testing Vibration...")
                ser.write(b"VIB 14 200\n")
                time.sleep(0.5)
                ser.write(b"VIB 14 0\n")
            elif cmd == '4':
                manual = input("Enter Command: ").strip()
                if manual:
                    ser.write((manual + "\n").encode())
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()

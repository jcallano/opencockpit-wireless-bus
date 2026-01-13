import serial
import threading
import time
import sys

"""
MiniFCU Hardware Emulator
-------------------------
This script emulates the behavior of the physical MiniFCU hardware.
It allows the official Windows software to connect to this script
via a Virtual COM Port pair (e.g., COM16 <-> COM17).

Protocol Findings:
- Baud: 9600
- Handshake: PC sends 'C,', Device MUST reply '901;956;959;'
- KeepAlive: PC sends '6,', Device replies with status.
"""

# --- CONFIGURATION ---
# This should be one end of a Virtual Serial Pair (e.g., com0com)
# The Windows Software should be connected to the OTHER end.
PORT_VIRTUAL = 'COM17' 
BAUD_RATE    = 9600

# --- FIRMWARE SIGNATURES ---
# The magic string required for the Windows Soft to accept the connection
HANDSHAKE_RESPONSE = b"901;956;959;"
# Default response to keep-alive polling
STATUS_RESPONSE    = b"99;95;952;962;972;982;"

def listener_thread(ser):
    """
    Background thread that listens for commands from the Windows Software.
    It handles the Handshake and Keep-Alive automatically.
    """
    buffer = ""
    while True:
        try:
            if ser.in_waiting:
                # Read char by char to detect the comma terminator
                char = ser.read().decode('ascii', errors='ignore')
                buffer += char
                
                # PC commands always end with a comma ','
                if char == ',':
                    timestamp = time.strftime("%H:%M:%S")
                    cmd_received = buffer.strip()
                    
                    # --- PROTOCOL LOGIC ---
                    
                    # 1. HANDSHAKE (The critical step)
                    if cmd_received == "C,":
                        print(f"\n[{timestamp}] ⚡ HANDSHAKE RECEIVED (C,) -> Sending Firmware ID...")
                        ser.write(HANDSHAKE_RESPONSE)
                    
                    # 2. KEEP ALIVE
                    elif cmd_received == "6,":
                        # Respond silently to maintain connection
                        ser.write(STATUS_RESPONSE)
                        
                    # 3. DATA UPDATE (Screen data)
                    else:
                        # Print what the sim wants to show on the screen
                        print(f"\n[PC -> SCREEN] {timestamp}: {cmd_received}")
                        print("Manual Command > ", end="", flush=True)
                    
                    buffer = "" # Clear buffer
                    
        except Exception as e:
            print(f"Listener Error: {e}")
            break

def main():
    print("--- MINIFCU HARDWARE EMULATOR ---")
    print(f"Connecting to {PORT_VIRTUAL} @ {BAUD_RATE}...")
    
    try:
        ser = serial.Serial(PORT_VIRTUAL, BAUD_RATE, timeout=0.1)
        # DTR/RTS must be true to mimic CH340 behavior
        ser.dtr = True
        ser.rts = True
    except Exception as e:
        print(f"❌ Connection Error: {e}")
        print("Ensure you are using a Virtual COM Port pair.")
        return

    # Start the listening thread
    thread = threading.Thread(target=listener_thread, args=(ser,), daemon=True)
    thread.start()

    print("✅ EMULATOR RUNNING.")
    print("   Open the official Windows Software now (connect to the paired COM port).")
    print("   Waiting for 'C,' handshake...")
    print("\n--- MANUAL INPUT SIMULATION ---")
    print("   Type a command to simulate a knob turn or button push.")
    print("   Examples:")
    print("   - SPD Knob: 14,100;")
    print("   - AP1 Btn:  51;")
    print("   - LOC Btn:  54;")

    try:
        while True:
            # User can type commands to simulate hardware interaction
            cmd = input("Manual Command > ")
            
            if cmd.lower() in ['exit', 'quit']: break
            if not cmd: continue

            # Hardware always terminates with a semicolon ';'
            if not cmd.endswith(';'): cmd += ';'
            
            # Send to Windows Software
            ser.write(cmd.encode('ascii'))

    except KeyboardInterrupt:
        print("\nStopping...")
        ser.close()

if __name__ == "__main__":
    main()

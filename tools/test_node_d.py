import sys
import time
import struct
import threading
import argparse
import os

# Allow importing from software/bridge
current_dir = os.path.dirname(os.path.abspath(__file__))
bridge_dir = os.path.join(current_dir, '..', 'software', 'bridge')
sys.path.append(bridge_dir)
from slip_serial import SlipSerial

# Protocol Constants
FRAME_HEADER = 0xAA
MSG_HID_INPUT = 0x20
MSG_MCDU_DISPLAY = 0x40
MSG_HID_OUTPUT = 0x30 # New raw output type
NODE_D_THROTTLE = 0x03
DEV_URSA_MINOR = 0x06

def calc_crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def build_message(msg_type, dst_node, payload):
    # Header: [AA, TYPE, SRC, DST]
    header = bytes([FRAME_HEADER, msg_type, 0x00, dst_node])
    msg = header + payload
    crc = calc_crc8(msg)
    return msg + bytes([crc])

def send_led_command(slip, zone_id, value):
    # Construct Report 0x02 payload
    # Format: [0x02, 0x10, 0xB9, 0x00, 0x00, 0x03, 0x49, ID, VAL]
    # Padded to 64 bytes (although driver might accept less, firmware sends 64)
    report = bytearray(64)
    prefix = [0x02, 0x10, 0xB9, 0x00, 0x00, 0x03, 0x49, zone_id, value]
    report[0:len(prefix)] = prefix
    
    # MSG_HID_OUTPUT payload: [DEV_ID, REP_ID, LEN, DATA...]
    # But wait, firmware logic for HID_OUTPUT:
    # if (payload->device_id == DEV_URSA_MINOR)
    #    memcpy(op.data, payload->report_data, payload->report_length);
    
    # We need to structure the payload for the Coordinator to forward to Node D logic
    # Actually Node D firmware expects:
    # MSG_HID_OUTPUT -> payload is HIDOutputPayload struct
    # struct HIDOutputPayload { uint8_t device_id; uint8_t report_id; uint8_t report_length; uint8_t report_data[]; }
    
    hid_payload = bytearray()
    hid_payload.append(DEV_URSA_MINOR) # Device ID
    hid_payload.append(0x02)           # Report ID (Not strictly used by fw logic but good for metadata)
    hid_payload.append(64)             # Length
    hid_payload.extend(report)         # Data
    
    msg = build_message(MSG_HID_OUTPUT, NODE_D_THROTTLE, hid_payload)
    slip.send(msg)

def reader_thread(slip):
    print("Listening for inputs...")
    last_print = 0
    for frame in slip.frames():
        if len(frame) < 5: continue
        if calc_crc8(frame[:-1]) != frame[-1]: continue
            
        header = frame[:4]
        payload = frame[4:-1]
        msg_type = header[1]
        src_node = header[2]
        
        if src_node == NODE_D_THROTTLE and msg_type == MSG_HID_INPUT:
             # Basic rate limiting for visual sanity
             if time.time() - last_print > 0.1:
                 # Payload: DEV_ID, REP_ID, LEN, DATA...
                 # Data starts at index 3
                 if len(payload) > 3:
                     raw = payload[3:]
                     # Axis X/Y/Z/Slider are roughly bytes 0-7? 
                     # Let's just print hex
                     print(f"\rInput: {raw[:16].hex()}...", end="")
                     last_print = time.time()

def main():
    parser = argparse.ArgumentParser(description='Test Node D (Ursa Minor) LEDs/LCD')
    parser.add_argument('port', help='Coordinator COM Port')
    args = parser.parse_args()
    
    try:
        with SlipSerial(args.port) as slip:
            t = threading.Thread(target=reader_thread, args=(slip,), daemon=True)
            t.start()
            
            print("\n=== Ursa Minor Tester ===")
            print("1. Toggle Backlights (ID 0x00)")
            print("2. Toggle Marker/LCD Backlight (ID 0x02)")
            print("3. Scan LED IDs (0x00 - 0x10)")
            print("4. Send LCD Test ('L 1.2 / R 3.4')")
            print("Q. Quit")
            
            while True:
                cmd = input("\nCommand: ").upper().strip()
                if cmd == 'Q':
                    break
                elif cmd == '1':
                    print("Sending Backlight 100%...")
                    send_led_command(slip, 0x00, 255)
                    time.sleep(1)
                    print("Sending Backlight 0%...")
                    send_led_command(slip, 0x00, 0)
                elif cmd == '2':
                    print("Sending Marker 100%...")
                    send_led_command(slip, 0x02, 255)
                    time.sleep(1)
                    print("Sending Marker 0%...")
                    send_led_command(slip, 0x02, 0)
                elif cmd == '3':
                    print("Scanning IDs 0-16...")
                    for i in range(17):
                        print(f"Lighting ID 0x{i:02X}...")
                        send_led_command(slip, i, 255)
                        time.sleep(0.5)
                        send_led_command(slip, i, 0) # Off
                elif cmd == '4':
                     val_l = "L 1.2"
                     val_r = "R 3.4"
                     print(f"Sending LCD: {val_l} | {val_r}")
                     
                     # Send Left
                     text = val_l.encode('ascii')
                     payload = struct.pack('BBBBB', 0x01, 0, 0, 0x02, len(text)) + text
                     msg = build_message(MSG_MCDU_DISPLAY, NODE_D_THROTTLE, payload)
                     slip.send(msg)
                     time.sleep(0.1)
                     # Send Right
                     text = val_r.encode('ascii')
                     payload = struct.pack('BBBBB', 0x01, 0, 0, 0x02, len(text)) + text
                     msg = build_message(MSG_MCDU_DISPLAY, NODE_D_THROTTLE, payload)
                     slip.send(msg)
                elif cmd == '5':
                    print("Testing Fire/Fault LEDs...")
                    ids = [0x03, 0x04, 0x05, 0x06]
                    names = ["ENG 1 FAULT", "ENG 1 FIRE", "ENG 2 FAULT", "ENG 2 FIRE"]
                    for i, name in zip(ids, names):
                        print(f"  {name} ON")
                        send_led_command(slip, i, 255)
                        time.sleep(0.5)
                        print(f"  {name} OFF")
                        send_led_command(slip, i, 0)
                elif cmd == '6':
                    print("Testing Vibration...")
                    print("  Motor 1 (Left)")
                    send_led_command(slip, 0x0E, 200)
                    time.sleep(0.5)
                    send_led_command(slip, 0x0E, 0)
                    print("  Motor 2 (Right)")
                    send_led_command(slip, 0x0F, 200)
                    time.sleep(0.5)
                    send_led_command(slip, 0x0F, 0)
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

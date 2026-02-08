import serial
import time
import threading
import struct
import sys
import argparse

# ==============================================================================
# CONSTANTS & PROTOCOL
# ==============================================================================
FRAME_HEADER = 0xAA
NODE_COORDINATOR = 0x00
NODE_D_THROTTLE = 0x03
DEV_URSA_MINOR = 0x06

# Message Types
MSG_HEARTBEAT     = 0x10
MSG_HID_INPUT     = 0x20
MSG_HID_OUTPUT    = 0x21
MSG_MCDU_DISPLAY  = 0x40

# SLIP constants
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

# Axis mapping (from node_d.ino)
AXIS_DEFS = [
    (13, "LEFT THROTTLE"),
    (15, "RIGHT THROTTLE"),
    (19, "SPOILER"),
    (21, "FLAPS")
]

BTN_NAMES = {
    1: "ENG 1 MASTER ON",   2: "ENG 1 MASTER OFF",
    3: "ENG 2 MASTER ON",   4: "ENG 2 MASTER OFF",
    5: "ENG 1 FIRE",        6: "ENG 2 FIRE",
    # ... add more if needed
}

# ==============================================================================
# CRC8 IMPLEMENTATION
# ==============================================================================
def crc8(data):
    crc = 0x00
    for byte in data:
        inbyte = byte
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc

# ==============================================================================
# SLIP IMPLEMENTATION
# ==============================================================================
def slip_encode(data):
    encoded = bytearray([SLIP_END])
    for byte in data:
        if byte == SLIP_END:
            encoded.extend([SLIP_ESC, SLIP_ESC_END])
        elif byte == SLIP_ESC:
            encoded.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            encoded.append(byte)
    encoded.append(SLIP_END)
    return bytes(encoded)

class SlipDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.escaped = False

    def decode_byte(self, byte):
        if self.escaped:
            self.escaped = False
            if byte == SLIP_ESC_END:
                self.buffer.append(SLIP_END)
            elif byte == SLIP_ESC_ESC:
                self.buffer.append(SLIP_ESC)
            else:
                self.buffer.append(byte)
        else:
            if byte == SLIP_END:
                if len(self.buffer) > 0:
                    frame = bytes(self.buffer)
                    self.buffer.clear()
                    return frame
            elif byte == SLIP_ESC:
                self.escaped = True
            else:
                self.buffer.append(byte)
        return None

# ==============================================================================
# MESSAGE BUILDERS
# ==============================================================================
def build_message(msg_type, dst, payload):
    # Header: AA <type> <src> <dst>
    header = struct.pack('<BBBB', FRAME_HEADER, msg_type, NODE_COORDINATOR, dst)
    msg = header + payload
    crc = crc8(msg)
    return msg + bytes([crc])

def build_backlight_msg(val):
    # Report 0x02, Type 0x10 (Backlight), ID 0x00 (Throttle) + Others
    # We will send 4 separate updates to cover all zones like "BL <val>" command
    msgs = []
    
    # Zones: (Type, ID)
    zones = [(0x10, 0x00), (0x10, 0x02), (0x01, 0x00), (0x01, 0x02)]
    
    for z_type, z_id in zones:
        # Construct 64-byte HID Output Report
        # 02 <type> B9 00 00 03 49 <id> <val> ...
        report = bytearray(64)
        report[0:9] = struct.pack('<BBBBBBBBB', 0x02, z_type, 0xB9, 0x00, 0x00, 0x03, 0x49, z_id, val)
        
        # Payload: dev_id, rpt_id, rpt_len, data...
        # struct HIDOutputPayload { u8 dev; u8 rpt; u8 len; u8 data[64]; }
        # Packed manually:
        payload = struct.pack('<BBB', DEV_URSA_MINOR, 0x02, 64) + report
        
        msgs.append(build_message(MSG_HID_OUTPUT, NODE_D_THROTTLE, payload))
        
    return msgs

def build_lcd_msg(side, val_float):
    # CMD=1 (Text)
    # Payload: cmd(1), row, col, color, len, text(24)
    text = f"{side} {val_float:.1f}".encode('ascii')
    payload = struct.pack('<BBBBB', 0x01, 0, 0, 0, len(text)) + text + b'\x00'*(24-len(text))
    return build_message(MSG_MCDU_DISPLAY, NODE_D_THROTTLE, payload)

# ==============================================================================
# MAIN LOGIC
# ==============================================================================
class TestApp:
    def __init__(self, port):
        print(f"Opening {port}...")
        self.serial = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5)
        # self.serial.dtr = True # Removed to avoid hanging on some CDC
        # self.serial.rts = True
        print("Port opened successfully.")
        self.decoder = SlipDecoder()
        self.running = True

    def reader(self):
        print("Reader thread started.")
        while self.running:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                print(f"RX {len(data)}B", end=" ", flush=True) # Debug raw RX
                for b in data:
                    frame = self.decoder.decode_byte(b)
                    if frame:
                        self.handle_frame(frame)
            time.sleep(0.001)

    def handle_frame(self, frame):
        if len(frame) < 5: return
        if crc8(frame[:-1]) != frame[-1]:
            print("CRC FAIL")
            return
            
        header = frame[:4]
        hdr_mark, msg_type, src, dst = struct.unpack('<BBBB', header)
        payload = frame[4:-1]

        if msg_type == MSG_HID_INPUT:
            self.parse_hid_input(payload)
        elif msg_type == MSG_HEARTBEAT:
            # print("HB", end=" ", flush=True)
            pass
        elif msg_type == 0xE1: # MSG_TEST_RSP
            print("PING", end=" ", flush=True)
        elif msg_type == 0x02: # MSG_DISCOVERY_RSP
            print(f"DISC(Node{src})", end=" ", flush=True)

    # ... parse_hid_input ...

    def run(self):
        t = threading.Thread(target=self.reader)
        t.daemon = True
        t.start()
        
        print("Starting output loop (Ctrl+C to stop)...")
        print("RX ONLY MODE. Waiting for Pings/Inputs...")
        
        try:
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            if self.serial.is_open:
                self.serial.close()
                print("Port closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    args = parser.parse_args()
    
    app = TestApp(args.port)
    app.run()

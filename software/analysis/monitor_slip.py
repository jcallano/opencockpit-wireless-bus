
import serial
import time
import struct
import argparse
import sys

# SLIP special characters
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

# Frame Header
FRAME_HEADER = 0xAA

# Message Types
MSG_TYPES = {
    0x01: "DISCOVERY_REQ",
    0x02: "DISCOVERY_RSP",
    0x03: "REGISTER_REQ",
    0x04: "REGISTER_ACK",
    0x10: "HEARTBEAT",
    0x11: "HEARTBEAT_ACK",
    0x20: "HID_INPUT",
    0x21: "HID_OUTPUT",
    0x25: "HID_PACKED_SIDESTICK",
    0x26: "HID_PACKED_QUADRANT",
    0x30: "SERIAL_DATA",
    0x40: "MCDU_DISPLAY",
    0x41: "MCDU_INPUT",
    0xE0: "TEST_REQ",
    0xE1: "TEST_RSP",
    0xF0: "ERROR",
    0xFF: "RESET"
}

NODE_NAMES = {
    0x00: "COORDINATOR",
    0x01: "NODE_B_JOYSTICK",
    0x02: "NODE_C_QUADRANT",
    0xFF: "BROADCAST"
}

def crc8(data):
    crc = 0x00
    for byte in data:
        extract = byte
        for _ in range(8):
            sum = (crc ^ extract) & 0x01
            crc >>= 1
            if sum:
                crc ^= 0x8C
            extract >>= 1
    return crc

class SlipDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.escaped = False

    def decode_byte(self, byte):
        if byte == SLIP_END:
            if len(self.buffer) > 0:
                packet = self.buffer[:]
                self.buffer = bytearray()
                self.escaped = False
                return packet
        elif byte == SLIP_ESC:
            self.escaped = True
        elif self.escaped:
            if byte == SLIP_ESC_END:
                self.buffer.append(SLIP_END)
            elif byte == SLIP_ESC_ESC:
                self.buffer.append(SLIP_ESC)
            else:
                # Protocol violation, but append anyway
                self.buffer.append(byte)
            self.escaped = False
        else:
            self.buffer.append(byte)
        return None

def parse_packet(packet):
    if len(packet) < 5: # Header(4) + CRC(1)
        return "Invalid Length"
    
    # Check Header
    if packet[0] != FRAME_HEADER:
        return f"Invalid Header: 0x{packet[0]:02X}"

    # Verify CRC
    payload_len = len(packet) - 1
    received_crc = packet[-1]
    calculated_crc = crc8(packet[:-1])
    
    crc_status = "OK" if received_crc == calculated_crc else f"FAIL (Exp 0x{calculated_crc:02X})"

    # Parse Header
    try:
        msg_type = packet[1]
        src_node = packet[2]
        dst_node = packet[3]
        payload = packet[4:-1]

        type_str = MSG_TYPES.get(msg_type, f"UNKNOWN(0x{msg_type:02X})")
        src_str = NODE_NAMES.get(src_node, f"NODE_{src_node:02X}")
        dst_str = NODE_NAMES.get(dst_node, f"NODE_{dst_node:02X}")

        timestamp = time.strftime("%H:%M:%S")
        
        details = ""
        # Payload decoding
        if msg_type == 0x02: # Discovery Response
            if len(payload) >= 8:
                node_type = payload[0]
                caps = payload[1]
                dev_cnt = payload[2]
                mac = ":".join(f"{b:02X}" for b in payload[3:9])
                details = f"Type={node_type} Caps=0x{caps:02X} Devs={dev_cnt} MAC={mac}"
        elif msg_type == 0x10 or msg_type == 0x11: # Heartbeat
            if len(payload) >= 4:
                ts = struct.unpack("<I", payload[:4])[0]
                details = f"TS={ts}ms"
        elif msg_type == 0x20: # HID Input
            if len(payload) >= 3:
                dev_id = payload[0]
                report_id = payload[1]
                length = payload[2]
                data_hex = " ".join(f"{b:02X}" for b in payload[3:])
                details = f"Dev={dev_id} Rpt={report_id} Len={length} Data=[{data_hex}]"
        elif msg_type == 0x25: # Packed Sidestick
            if len(payload) >= 8:
                # Unpack 64-bit integer (Little Endian)
                val = struct.unpack('<Q', payload[:8])[0]
                
                axis_x      = (val >> 0)  & 0xFFF
                axis_y      = (val >> 12) & 0xFFF
                axis_z      = (val >> 24) & 0x3FF
                axis_slider = (val >> 34) & 0x3FF
                buttons     = (val >> 44) & 0xFFFF
                hat         = (val >> 60) & 0xF
                
                hat_str = str(hat) if hat != 15 else "Center"
                details = (f"X:{axis_x:<4} Y:{axis_y:<4} Z:{axis_z:<4} S:{axis_slider:<4} "
                           f"Btn:0x{buttons:04X} Hat:{hat_str}")
    except Exception as e:
        return f"ACCESS ERROR: {e} | Raw: {packet.hex()}"
    
    return f"[{timestamp}] {src_str} -> {dst_str} : {type_str} | Len={len(payload)} CRC={crc_status} | {details}"

def main():
    parser = argparse.ArgumentParser(description='Monitor SLIP traffic on COM port')
    parser.add_argument('port', help='COM port (e.g. COM22)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        ser.dtr = True # Enable DTR (Required for some CDC implementations)
        ser.rts = True 
        print(f"Opened {args.port} at {args.baud} baud. Waiting for data...")
    except Exception as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    decoder = SlipDecoder()
    last_print = time.time()
    byte_count = 0

    try:
        while True:
            chunk = ser.read(1024)
            if chunk:
                byte_count += len(chunk)
                for byte in chunk:
                    packet = decoder.decode_byte(byte)
                    if packet:
                        print(parse_packet(packet))
            
            # Print status every 2 seconds if active but no packets
            if time.time() - last_print > 2.0:
                if byte_count > 0:
                   print(f"[STATUS] Received {byte_count} bytes total so far...")
                   last_print = time.time()

    except KeyboardInterrupt:

        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

"""
URSA MINOR 32 - Interactive LCD Controller
Based on hardware_validator.py logic.

Usage:
    python scripts/tests/verify_lcd.py

Commands:
    L <value>   - Set Left display (e.g. L 1.5)
    R <value>   - Set Right display (e.g. R 2.0)
    Q           - Quit
"""

import hid
import time
import sys
import os

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VID = 0x4098
PID = 0xB920

# ---------------------------------------------------------------------------
# 7-Segment LCD Encoding (programmatic, no lookup table)
# ---------------------------------------------------------------------------

#  aaa
# f   b
#  ggg
# e   c
#  ddd
SEVEN_SEG = {
    0: (1,1,1,1,1,1,0),  # a b c d e f g
    1: (0,1,1,0,0,0,0),
    2: (1,1,0,1,1,0,1),
    3: (1,1,1,1,0,0,1),
    4: (0,1,1,0,0,1,1),
    5: (1,0,1,1,0,1,1),
    6: (1,0,1,1,1,1,1),
    7: (1,1,1,0,0,0,0),
    8: (1,1,1,1,1,1,1),
    9: (1,1,1,1,0,1,1),
}

# Slot -> segment mapping: slot0=f, slot1=e, slot2=d, slot3=c, slot4=b, slot5=a
SLOT_SEG_INDEX = [5, 4, 3, 2, 1, 0]  # f, e, d, c, b, a

# Side bit patterns (bit 0 of each slot)
SIDE_BITS = {
    'L': [1, 1, 1, 0, 0, 0],
    'R': [1, 1, 0, 1, 1, 1],
}

BLANK_SEG = (0, 0, 0, 0, 0, 0, 0)  # all segments off = blank digit

def encode_lcd(side, integer, fractional):
    """
    Encode LCD value to protocol bytes.
    """
    tens = integer // 10
    ones = integer % 10

    frac_segs = SEVEN_SEG[fractional]  # (a, b, c, d, e, f, g)
    ones_segs = SEVEN_SEG[ones]
    tens_segs = SEVEN_SEG[tens] if tens > 0 else BLANK_SEG

    side_bit = 1 if side == 'R' else 0
    b29 = (frac_segs[6] << 3) | (ones_segs[6] << 2) | (tens_segs[6] << 1) | side_bit

    bit0 = SIDE_BITS[side]
    slots = []
    for i in range(6):
        seg_idx = SLOT_SEG_INDEX[i]
        frac_bit = frac_segs[seg_idx]
        ones_bit = ones_segs[seg_idx]
        tens_bit = tens_segs[seg_idx]
        slot_val = (frac_bit << 3) | (ones_bit << 2) | (tens_bit << 1) | bit0[i]
        slots.append(slot_val)

    return b29, slots

class LCDController:
    def __init__(self):
        self.device = None
        self.counter = 0

    def connect(self):
        try:
            self.device = hid.device()
            self.device.open(VID, PID)
            print(f"Connected to URSA MINOR 32 (VID={VID:04X}, PID={PID:04X})")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def close(self):
        if self.device:
            self.device.close()
            self.device = None

    def send_value(self, side, integer, fractional):
        if not self.device:
            return

        b29, slots = encode_lcd(side, integer, fractional)

        # DATA packet
        pkt = [0] * 64
        pkt[0] = 0xF0
        pkt[2] = self.counter & 0xFF
        pkt[3] = 0x38
        pkt[4], pkt[5] = 0x01, 0xB9
        pkt[8], pkt[9] = 0x02, 0x01
        pkt[17] = 0x24
        pkt[25] = 0x04
        pkt[29] = b29
        for i, s in enumerate(slots):
            pkt[33 + i * 4] = s
        pkt[57], pkt[58] = 0x01, 0xB9
        
        try:
            self.device.write(pkt)
            self.counter = (self.counter + 1) & 0xFF
            time.sleep(0.005)

            # COMMIT packet
            commit = [0] * 64
            commit[0] = 0xF0
            commit[2] = self.counter & 0xFF
            commit[3] = 0x0E
            commit[5], commit[6] = 0x03, 0x01
            self.device.write(commit)
            self.counter = (self.counter + 1) & 0xFF
            time.sleep(0.005)
            
            print(f" -> Sent: {side} {integer}.{fractional}")
            
        except Exception as e:
            print(f"Error sending data: {e}")

def main():
    print("URSA MINOR 32 - LCD Control Utility")
    
    controller = LCDController()
    if not controller.connect():
        return
    
    print("\nEnter command (e.g. 'L 1.5' or 'Q' to quit):")
    
    try:
        while True:
            cmd = input("> ").strip().upper()
            
            if not cmd:
                continue
                
            if cmd == 'Q':
                break
                
            parts = cmd.split()
            if len(parts) == 2 and parts[0] in ['L', 'R']:
                try:
                    val = float(parts[1])
                    integer = int(val)
                    fractional = int(round((val - integer) * 10))
                    
                    if fractional >= 10:
                        integer += 1
                        fractional = 0
                        
                    controller.send_value(parts[0], integer, fractional)
                    
                except ValueError:
                    print("Invalid number format")
            else:
                print("Invalid command. Use 'L <value>' or 'R <value>'")
                
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        print("\nDisconnected.")

if __name__ == "__main__":
    main()

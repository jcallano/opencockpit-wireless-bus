"""
URSA MINOR 32 - Hardware Validator

Interactive tool to validate all hardware: 42 buttons, 4 axes, and LCD display.
LCD cycles through L 20.0 -> L 0.0 -> R 0.0 -> R 20.0 continuously.
Flicker-free display using ANSI cursor positioning.

Usage:
    conda activate ursa
    python scripts/tests/hardware_validator.py

Controls:
    Q / ESC  - Quit
    SPACE    - Pause/Resume LCD cycling
    R        - Reset all counters
    +/-      - Adjust LCD cycle speed
"""

import hid
import struct
import time
import sys
import os
import msvcrt

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VID = 0x4098
PID = 0xB920
WIDTH = 78

BUTTON_NAMES = {
    1: "ENG 1 MASTER ON",       2: "ENG 1 MASTER OFF",
    3: "ENG 2 MASTER ON",       4: "ENG 2 MASTER OFF",
    5: "ENG 1 FIRE BUTTON",     6: "ENG 2 FIRE BUTTON",
    7: "ENG MODE CRANK",        8: "ENG MODE NORM",
    9: "ENG MODE IGN/START",   10: "L THROT AUTO DISC",
   11: "R THROT AUTO DISC",    12: "L THROT TO/GA",
   13: "L THROT FLEX/MCT",     14: "L THROT CL",
   15: "L THROT IDLE",         16: "L THROT IDLE REV",
   17: "L THROT FULL REV",     18: "R THROT TO/GA",
   19: "R THROT FLEX/MCT",     20: "R THROT CL",
   21: "R THROT IDLE",         22: "R THROT IDLE REV",
   23: "R THROT FULL REV",     24: "ENG MODE PUSH BTN",
   25: "TRIM RESET",           26: "RUDDER TRIM L",
   27: "RUDDER TRIM NEUTRAL",  28: "RUDDER TRIM R",
   29: "PARKING BRAKE OFF",    30: "PARKING BRAKE ON",
   31: "FLAPS 4",              32: "FLAPS 3",
   33: "FLAPS 2",              34: "FLAPS 1",
   35: "FLAPS 0",              36: "SPOILER FULL",
   37: "SPOILER ONE HALF",     38: "SPOILER RET",
   39: "SPOILER ARM",          40: "L THROT REV MODE ON",
   41: "R THROT REV MODE ON",  42: "BUTTON 42",
}

AXIS_INFO = [
    ('Rx',     'LEFT THROTTLE'),
    ('Ry',     'RIGHT THROTTLE'),
    ('Slider', 'SPOILER'),
    ('Dial',   'FLAPS'),
]

# Axis byte offsets within Report 0x01 (after report ID)
# Order: X, Y, Z, Rx, Ry, Rz, Slider, Dial  (each 16-bit LE)
AXIS_OFFSETS = {'Rx': 13, 'Ry': 15, 'Slider': 19, 'Dial': 21}

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

    Args:
        side: 'L' or 'R'
        integer: 0-20 (tens digit blanked when < 10)
        fractional: tenths digit 0-9

    Returns:
        (b29, slots) tuple ready for the DATA packet

    Slot bit layout:
        bit 3 = fractional digit segment
        bit 2 = ones digit segment
        bit 1 = tens digit segment (0 when blank)
        bit 0 = side identifier
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


# ---------------------------------------------------------------------------
# LCD Sequence: L 20.0, L 19.9, ..., L 0.0, R 0.0, R 0.1, ..., R 20.0
# ---------------------------------------------------------------------------

def build_lcd_sequence():
    """Build the 402-value LCD cycle sequence (0.0 to 20.0 per side)."""
    seq = []
    # L 20.0 down to L 0.0 (201 values)
    for val in range(200, -1, -1):
        seq.append(('L', val // 10, val % 10))
    # R 0.0 up to R 20.0 (201 values)
    for val in range(0, 201):
        seq.append(('R', val // 10, val % 10))
    return seq


LCD_SEQUENCE = build_lcd_sequence()

# ---------------------------------------------------------------------------
# Report parsing
# ---------------------------------------------------------------------------

def parse_report01(data):
    """Parse Report 0x01: buttons + axes."""
    buttons = set()
    for byte_idx in range(6):
        byte_val = data[byte_idx + 1]
        for bit_idx in range(8):
            btn = byte_idx * 8 + bit_idx + 1
            if btn > 42:
                break
            if byte_val & (1 << bit_idx):
                buttons.add(btn)

    axes = {}
    for code, offset in AXIS_OFFSETS.items():
        if offset + 1 < len(data):
            axes[code] = struct.unpack('<H', bytes(data[offset:offset + 2]))[0]

    return buttons, axes


# ---------------------------------------------------------------------------
# Display rendering
# ---------------------------------------------------------------------------

def bar(value, width=20, max_val=65535):
    """Render a bar ##########..........  for a 0-max_val value."""
    filled = int(value / max_val * width) if max_val else 0
    return '#' * filled + '.' * (width - filled)


def render_frame(lcd_idx, lcd_paused, lcd_speed, axes_state, axes_min, axes_max,
                 buttons_ever, buttons_now, frame_count):
    """Build the full screen as a list of strings (one per line)."""
    lines = []
    ln = lines.append
    sep = '=' * WIDTH

    ln(sep)
    ln('  URSA MINOR 32 - HARDWARE VALIDATOR'.ljust(WIDTH))
    ln(sep)
    ln('')

    # --- LCD status ---
    side, integer, frac = LCD_SEQUENCE[lcd_idx]
    lcd_text = f"{side}{integer:3d}.{frac}"
    total = len(LCD_SEQUENCE)
    pct = int(lcd_idx / max(total - 1, 1) * 30)
    progress = '#' * pct + '-' * (30 - pct)
    pause_tag = '  PAUSED' if lcd_paused else ''
    ln(f"  LCD:   {lcd_text}  [{progress}] {lcd_idx + 1}/{total}"
       f"  ({lcd_speed:.1f}s){pause_tag}")
    ln('')

    # --- Axes ---
    ln(f"  AXES:{' ' * 35}Min   Value     Max  Range")
    ln('  ' + '-' * (WIDTH - 4))
    for code, label in AXIS_INFO:
        val = axes_state.get(code, 0)
        mn = axes_min.get(code, val)
        mx = axes_max.get(code, val)
        rng = (mx - mn) / 65535 * 100 if 65535 else 0
        b = bar(val)
        ln(f"  {label:15s} [{code:6s}] {b} {mn:5d} {val:7d} {mx:7d}  {rng:3.0f}%")
    ln('')

    # --- Buttons ---
    validated = len(buttons_ever)
    pressed_list = sorted(buttons_now)
    pressed_str = ', '.join(str(b) for b in pressed_list) if pressed_list else 'none'
    ln(f"  BUTTONS: {validated}/42 validated    Pressed: [{pressed_str}]")
    ln('  ' + '-' * (WIDTH - 4))

    # 2-column layout, 21 rows
    half = 21
    for row in range(half):
        parts = []
        for col in range(2):
            btn = row + 1 + col * half
            if btn > 42:
                parts.append(' ' * 36)
                continue
            name = BUTTON_NAMES.get(btn, f"BUTTON {btn}")
            if btn in buttons_now:
                marker = '>>>'
            elif btn in buttons_ever:
                marker = '[*]'
            else:
                marker = '[ ]'
            parts.append(f"  {marker} {btn:02d} {name:25s}")
        ln(''.join(parts))
    ln('')

    # --- Footer ---
    ln(f"  [Q] Quit  [SPACE] Pause LCD  [R] Reset  [+/-] Speed"
       f"    frame {frame_count}")
    ln(sep)

    return lines


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    # --- Connect device ---
    print("Connecting to URSA MINOR 32 ...")
    device = hid.device()
    try:
        device.open(VID, PID)
    except Exception as e:
        print(f"ERROR: Cannot open device (VID={VID:04X} PID={PID:04X}): {e}")
        return 1
    device.set_nonblocking(1)

    lcd_counter = 0

    def lcd_send(side, ones, frac):
        """Send one LCD value to the device."""
        nonlocal lcd_counter
        b29, slots = encode_lcd(side, ones, frac)

        pkt = [0] * 64
        pkt[0] = 0xF0
        pkt[2] = lcd_counter & 0xFF
        pkt[3] = 0x38
        pkt[4], pkt[5] = 0x01, 0xB9
        pkt[8], pkt[9] = 0x02, 0x01
        pkt[17] = 0x24
        pkt[25] = 0x04
        pkt[29] = b29
        for i, s in enumerate(slots):
            pkt[33 + i * 4] = s
        pkt[57], pkt[58] = 0x01, 0xB9
        device.write(pkt)
        lcd_counter = (lcd_counter + 1) & 0xFF
        time.sleep(0.005)

        commit = [0] * 64
        commit[0] = 0xF0
        commit[2] = lcd_counter & 0xFF
        commit[3] = 0x0E
        commit[5], commit[6] = 0x03, 0x01
        device.write(commit)
        lcd_counter = (lcd_counter + 1) & 0xFF
        time.sleep(0.005)

    # --- State ---
    lcd_idx = 0
    lcd_paused = False
    lcd_speed = 0.5       # seconds per step
    lcd_last_step = time.time()

    axes_state = {}
    axes_min = {}
    axes_max = {}
    buttons_ever = set()
    buttons_now = set()
    frame_count = 0

    # --- Prepare terminal ---
    os.system('cls')
    sys.stdout.write('\033[?25l')   # hide cursor
    sys.stdout.flush()

    # Send initial LCD value
    s, o, f = LCD_SEQUENCE[lcd_idx]
    lcd_send(s, o, f)

    try:
        while True:
            now = time.time()

            # --- Keyboard input (non-blocking) ---
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):
                    msvcrt.getch()  # consume extended key
                    continue
                c = ch.lower()
                if c in (b'q', b'\x1b'):       # Q or ESC
                    return 0
                elif c == b' ':
                    lcd_paused = not lcd_paused
                elif c == b'r':
                    axes_min.clear()
                    axes_max.clear()
                    buttons_ever.clear()
                    lcd_idx = 0
                    lcd_last_step = now
                    s, o, f = LCD_SEQUENCE[lcd_idx]
                    lcd_send(s, o, f)
                elif c == b'+' or c == b'=':
                    lcd_speed = min(lcd_speed + 0.1, 5.0)
                elif c == b'-':
                    lcd_speed = max(lcd_speed - 0.1, 0.1)

            # --- Read HID report ---
            data = device.read(64)
            if data and len(data) > 0 and data[0] == 0x01:
                buttons_now, axes_state = parse_report01(data)
                buttons_ever |= buttons_now
                for code in axes_state:
                    v = axes_state[code]
                    if code not in axes_min:
                        axes_min[code] = v
                        axes_max[code] = v
                    else:
                        if v < axes_min[code]:
                            axes_min[code] = v
                        if v > axes_max[code]:
                            axes_max[code] = v

            # --- LCD step ---
            if not lcd_paused and (now - lcd_last_step) >= lcd_speed:
                lcd_idx = (lcd_idx + 1) % len(LCD_SEQUENCE)
                s, o, f = LCD_SEQUENCE[lcd_idx]
                lcd_send(s, o, f)
                lcd_last_step = now

            # --- Render ---
            frame_count += 1
            lines = render_frame(lcd_idx, lcd_paused, lcd_speed,
                                 axes_state, axes_min, axes_max,
                                 buttons_ever, buttons_now, frame_count)

            # Move cursor to top-left, overwrite each line padded to WIDTH
            buf = ['\033[H']
            for line in lines:
                buf.append(line[:WIDTH].ljust(WIDTH))
                buf.append('\n')
            sys.stdout.write(''.join(buf))
            sys.stdout.flush()

            time.sleep(0.033)  # ~30 FPS

    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\033[?25h')  # restore cursor
        sys.stdout.flush()
        device.close()
        print("\nDevice closed.")

    return 0


if __name__ == '__main__':
    sys.exit(main())

# URSA MINOR 32 - HID Protocol Reference

Complete protocol specification for the **WINCTRL URSA MINOR 32 Throttle Metal / 32 PAC Metal** (Winwing).

**Last Updated:** 2026-02-05
**Firmware:** PID 0xB920 (updated 2026-01-31)

---

## Table of Contents

- [Device Information](#device-information)
- [Report 0x01 - Joystick](#report-0x01---joystick)
- [Report 0x02 - Auxiliary Controls](#report-0x02---auxiliary-controls)
- [Report 0xF0 - LCD Display](#report-0xf0---lcd-display)
- [Tools and Scripts](#tools-and-scripts)
- [Setup](#setup)

---

## Device Information

| Parameter | Value |
|-----------|-------|
| Manufacturer | Winwing |
| Product | WINCTRL URSA MINOR 32 Throttle Metal / 32 PAC Metal |
| VID | 0x4098 |
| PID | 0xB920 |
| Old PID (pre-update) | 0xBB13 |
| USB Class | HID |
| USB Speed | Full-Speed (12 Mbps) |
| Polling Interval | 1 ms |
| Driver | hidusb.sys (Windows standard) |

### HID Reports

| Report ID | Direction | Size | Purpose |
|-----------|-----------|------|---------|
| 0x01 | IN | 64 bytes | Joystick (42 buttons + 8 axes) |
| 0x02 | OUT | 9+ bytes | Auxiliary (backlights, LEDs, motors) |
| 0xF0 | OUT | 64 bytes | LCD display control |

### Communication Library

Use **hidapi** (Python `hid` package). No driver replacement needed.

```python
import hid

VID = 0x4098
PID = 0xB920

device = hid.device()
device.open(VID, PID)
```

---

## Report 0x01 - Joystick

**Direction:** IN (device to host)
**Size:** 64 bytes
**Type:** Standard HID Joystick

### Byte Layout

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0 | Report ID | uint8 | Always 0x01 |
| 1-6 | Buttons | 42 bits | Bit-packed, LSB first |
| 7-8 | X Axis | uint16 LE | Unused (constant) |
| 9-10 | Y Axis | uint16 LE | Unused (constant) |
| 11-12 | Z Axis | uint16 LE | Unused (constant) |
| 13-14 | Rx Axis | uint16 LE | LEFT THROTTLE (0-65535) |
| 15-16 | Ry Axis | uint16 LE | RIGHT THROTTLE (0-65535) |
| 17-18 | Rz Axis | uint16 LE | Unused (constant) |
| 19-20 | Slider | uint16 LE | SPOILER (0-65535) |
| 21-22 | Dial | uint16 LE | FLAPS (0-65535) |
| 23-63 | Reserved | - | Zeros |

### Active Axes

| Axis | Physical Control | Range |
|------|------------------|-------|
| Rx (bytes 13-14) | LEFT THROTTLE | 0-65535 |
| Ry (bytes 15-16) | RIGHT THROTTLE | 0-65535 |
| Slider (bytes 19-20) | SPOILER | 0-65535 |
| Dial (bytes 21-22) | FLAPS | 0-65535 |

### Button Mapping (42 buttons)

Buttons are packed in bytes 1-6, LSB first. Button pressed = 1.

| # | Control | # | Control |
|---|---------|---|---------|
| 1 | ENG 1 MASTER ON | 22 | RIGHT THROTTLE IDLE REVERSE |
| 2 | ENG 1 MASTER OFF | 23 | RIGHT THROTTLE FULL REVERSE |
| 3 | ENG 2 MASTER ON | 24 | ENG MODE PUSH BUTTON |
| 4 | ENG 2 MASTER OFF | 25 | TRIM RESET |
| 5 | ENG 1 FIRE BUTTON | 26 | RUDDER TRIM L |
| 6 | ENG 2 FIRE BUTTON | 27 | RUDDER TRIM NEUTRAL |
| 7 | ENG MODE CRANK | 28 | RUDDER TRIM R |
| 8 | ENG MODE NORM | 29 | PARKING BRAKE OFF |
| 9 | ENG MODE IGN/START | 30 | PARKING BRAKE ON |
| 10 | LEFT THROTTLE AUTO TRUST DISC | 31 | FLAPS 4 |
| 11 | RIGHT THROTTLE AUTO TRUST DISC | 32 | FLAPS 3 |
| 12 | LEFT THROTTLE TO/GA | 33 | FLAPS 2 |
| 13 | LEFT THROTTLE FLEX/MCT | 34 | FLAPS 1 |
| 14 | LEFT THROTTLE CL | 35 | FLAPS 0 |
| 15 | LEFT THROTTLE IDLE | 36 | SPOILER FULL |
| 16 | LEFT THROTTLE IDLE REVERSE | 37 | SPOILER ONE HALF |
| 17 | LEFT THROTTLE FULL REVERSE | 38 | SPOILER RET |
| 18 | RIGHT THROTTLE TO/GA | 39 | SPOILER ARM |
| 19 | RIGHT THROTTLE FLEX/MCT | 40 | LEFT THROTTLE REVERSE MODE ON |
| 20 | RIGHT THROTTLE CL | 41 | RIGHT THROTTLE REVERSE MODE ON |
| 21 | RIGHT THROTTLE IDLE | 42 | (not mapped to physical control) |

### Read Example

```python
import struct

data = device.read(64)
if data and data[0] == 0x01:
    # Buttons (bytes 1-6)
    buttons = []
    for byte_idx in range(6):
        for bit_idx in range(8):
            if data[1 + byte_idx] & (1 << bit_idx):
                btn = byte_idx * 8 + bit_idx + 1
                if btn <= 42:
                    buttons.append(btn)

    # Axes
    rx = struct.unpack('<H', bytes(data[13:15]))[0]      # LEFT THROTTLE
    ry = struct.unpack('<H', bytes(data[15:17]))[0]      # RIGHT THROTTLE
    slider = struct.unpack('<H', bytes(data[19:21]))[0]  # SPOILER
    dial = struct.unpack('<H', bytes(data[21:23]))[0]    # FLAPS
```

---

## Report 0x02 - Auxiliary Controls

**Direction:** OUT (host to device)
**Purpose:** Backlights, LEDs, vibration motors

### Packet Structure

```
Byte 0:    0x02           (Report ID)
Byte 1:    cmd_type       (0x01 or 0x10)
Bytes 2-6: 0xB9 0x00 0x00 0x03 0x49  (fixed header)
Byte 7:    control_id     (identifies the control)
Byte 8:    value          (0-255)
```

### Control Map (10 controls)

#### Backlights (4 zones, variable brightness 0-255)

| Zone | Name | cmd_type | control_id | Description |
|------|------|----------|------------|-------------|
| 1 | Throttle Backlight | 0x10 | 0x00 | Throttle lever handles |
| 2 | Marker Backlight | 0x10 | 0x02 | ENG FIRE/FAULT buttons area |
| 3 | 32-pack Backlight | 0x01 | 0x00 | Flaps/Spoiler module |
| 4 | LCD Backlight | 0x01 | 0x02 | LCD display backlight |

#### LEDs (4 LEDs, binary ON/OFF)

| LED | Name | cmd_type | control_id | Values |
|-----|------|----------|------------|--------|
| 1 | ENG 1 FAULT | 0x10 | 0x03 | 0=OFF, 255=ON |
| 2 | ENG 1 FIRE | 0x10 | 0x04 | 0=OFF, 255=ON |
| 3 | ENG 2 FAULT | 0x10 | 0x05 | 0=OFF, 255=ON |
| 4 | ENG 2 FIRE | 0x10 | 0x06 | 0=OFF, 255=ON |

#### Vibration Motors (2 motors, variable intensity 0-255)

| Motor | Name | cmd_type | control_id |
|-------|------|----------|------------|
| 1 | Motor 1 | 0x10 | 0x0E |
| 2 | Motor 2 | 0x10 | 0x0F |

### Send Example

```python
def send_report02(device, cmd_type, control_id, value):
    buf = [0x02, cmd_type, 0xB9, 0x00, 0x00, 0x03, 0x49, control_id, value]
    buf += [0x00] * (64 - len(buf))
    device.write(buf)

# Throttle backlight at 50%
send_report02(device, 0x10, 0x00, 128)

# ENG 1 FIRE LED on
send_report02(device, 0x10, 0x04, 255)

# Motor 1 vibrate at full
send_report02(device, 0x10, 0x0E, 255)
```

---

## Report 0xF0 - LCD Display

**Direction:** OUT (host to device)
**Purpose:** Control the 7-segment LCD trim display
**Format:** `[L/R] [integer].[fractional]` (e.g. "L 0.5", "R 1.3")

### Encoding Type: Packed 7-Segment Data

The LCD protocol sends 7-segment display data packed across 6 slots plus a mode byte. This is NOT BCD, and NOT a high-level command system. Each slot carries one bit of segment data for the fractional digit, combined with integer and side information.

### Packet Protocol

Each LCD update requires 2 packets: **DATA + COMMIT**.

#### DATA Packet (type 0x38)

64 bytes total. Most bytes are zero.

| Byte | Value | Description |
|------|-------|-------------|
| 0 | 0xF0 | Report ID |
| 2 | counter | Incrementing packet counter (0-255) |
| 3 | 0x38 | Packet type: DATA |
| 4-5 | 0x01, 0xB9 | Constant |
| 8-9 | 0x02, 0x01 | Constant |
| 17 | 0x24 | Constant |
| 25 | 0x04 | Constant |
| 29 | b29 | **Side + segment g** (see below) |
| 33 | slot[0] | **Segment f** of fractional digit |
| 37 | slot[1] | **Segment e** of fractional digit |
| 41 | slot[2] | **Segment d** of fractional digit |
| 45 | slot[3] | **Segment c** of fractional digit |
| 49 | slot[4] | **Segment b** of fractional digit |
| 53 | slot[5] | **Segment a** of fractional digit |
| 57-58 | 0x01, 0xB9 | Constant |

#### COMMIT Packet (type 0x0E)

64 bytes total. Triggers the display update.

| Byte | Value | Description |
|------|-------|-------------|
| 0 | 0xF0 | Report ID |
| 2 | counter | Incrementing packet counter |
| 3 | 0x0E | Packet type: COMMIT |
| 5-6 | 0x03, 0x01 | Constant |

### Slot Encoding (4 bits per slot)

Each slot value is a 4-bit field using only bits 0, 2, and 3 (bit 1 is always 0).

| Bit | Purpose |
|-----|---------|
| Bit 3 | 7-segment data for the **fractional** digit (one of: a, b, c, d, e, f) |
| Bit 2 | 7-segment data for the **integer** (ones) digit (same segment as bit 3) |
| Bit 1 | Always 0 (unused) |
| Bit 0 | Side identifier (fixed pattern per side) |

**Bit 0 patterns:**

| Side | slot[0] | slot[1] | slot[2] | slot[3] | slot[4] | slot[5] |
|------|---------|---------|---------|---------|---------|---------|
| L | 1 | 1 | 1 | 0 | 0 | 0 |
| R | 1 | 1 | 0 | 1 | 1 | 1 |

**8 possible slot values:** 0, 1, 4, 5, 8, 9, 12, 13

### Segment-to-Slot Mapping

The 6 slots carry one segment each of the fractional digit (in bit 3):

```
Slot 0 -> segment f        aaa
Slot 1 -> segment e       f   b
Slot 2 -> segment d        ggg
Slot 3 -> segment c       e   c
Slot 4 -> segment b        ddd
Slot 5 -> segment a
```

Segment **g** (middle bar) is carried in **b29 bit 3**.

### b29 Encoding

b29 carries segment **g** for both digits plus the side identifier:

```
b29 = (frac_g << 3) | (ones_g << 2) | (side == 'R' ? 1 : 0)
```

| Bit | Purpose |
|-----|---------|
| Bit 3 | Segment g of the **fractional** digit |
| Bit 2 | Segment g of the **integer** (ones) digit |
| Bit 0 | Side: 0 = L, 1 = R |

Common values (integers 0 and 1 have g=0):

| b29 | Side | Frac g | Ones g | Example |
|-----|------|--------|--------|---------|
| 0 | L | 0 | 0 | L 0.0, L 0.1, L 1.7 |
| 4 | L | 0 | 1 | L 2.0, L 3.1 |
| 8 | L | 1 | 0 | L 0.5, L 1.8 |
| 12 | L | 1 | 1 | L 2.9, L 5.3 |
| 1 | R | 0 | 0 | R 0.7, R 1.0 |
| 5 | R | 0 | 1 | R 2.0, R 4.1 |
| 9 | R | 1 | 0 | R 0.9, R 1.5 |
| 13 | R | 1 | 1 | R 3.8, R 6.2 |

### 7-Segment Digit Table

Standard 7-segment encoding for digits 0-9:

| Digit | a | b | c | d | e | f | g | Mode |
|-------|---|---|---|---|---|---|---|------|
| 0 | 1 | 1 | 1 | 1 | 1 | 1 | 0 | 0 |
| 1 | 0 | 1 | 1 | 0 | 0 | 0 | 0 | 0 |
| 2 | 1 | 1 | 0 | 1 | 1 | 0 | 1 | 1 |
| 3 | 1 | 1 | 1 | 1 | 0 | 0 | 1 | 1 |
| 4 | 0 | 1 | 1 | 0 | 0 | 1 | 1 | 1 |
| 5 | 1 | 0 | 1 | 1 | 0 | 1 | 1 | 1 |
| 6 | 1 | 0 | 1 | 1 | 1 | 1 | 1 | 1 |
| 7 | 1 | 1 | 1 | 0 | 0 | 0 | 0 | 0 |
| 8 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| 9 | 1 | 1 | 1 | 1 | 0 | 1 | 1 | 1 |

### Algorithm to Calculate Slot Values

To display `{side} {integer}.{fractional}` (both digits support 0-9):

```
1. Look up segments a-g for the fractional digit (table above)
2. Look up segments a-g for the integer (ones) digit (same table)
3. b29 = (frac_g << 3) | (ones_g << 2) | (side == 'R' ? 1 : 0)
4. bit0_pattern:
     L = [1, 1, 1, 0, 0, 0]
     R = [1, 1, 0, 1, 1, 1]
5. For each slot i (0-5):
     seg_order = [f, e, d, c, b, a]   (segment index for each slot)
     bit3 = frac_seg[seg_order[i]] << 3   (fractional digit segment)
     bit2 = ones_seg[seg_order[i]] << 2   (integer digit segment)
     bit0 = bit0_pattern[i]
     slot[i] = bit3 | bit2 | bit0
```

### Worked Example: Display "R 0.5"

Fractional digit 5: a=1, b=0, c=1, d=1, e=0, f=1, g=1
Integer digit 0:    a=1, b=1, c=1, d=1, e=1, f=1, g=0

```
b29 = (frac_g=1 << 3) | (ones_g=0 << 2) | (R=1) = 8|0|1 = 9
bit0 pattern (R): [1, 1, 0, 1, 1, 1]

slot[0]: frac_f=1, ones_f=1 -> (8|4|1) = 13
slot[1]: frac_e=0, ones_e=1 -> (0|4|1) = 5
slot[2]: frac_d=1, ones_d=1 -> (8|4|0) = 12
slot[3]: frac_c=1, ones_c=1 -> (8|4|1) = 13
slot[4]: frac_b=0, ones_b=1 -> (0|4|1) = 5
slot[5]: frac_a=1, ones_a=1 -> (8|4|1) = 13

Result: b29=9, slots=[13, 5, 12, 13, 5, 13]
```

### Worked Example: Display "L 1.3"

Fractional digit 3: a=1, b=1, c=1, d=1, e=0, f=0, g=1
Integer digit 1:    a=0, b=1, c=1, d=0, e=0, f=0, g=0

```
b29 = (frac_g=1 << 3) | (ones_g=0 << 2) | (L=0) = 8|0|0 = 8
bit0 pattern (L): [1, 1, 1, 0, 0, 0]

slot[0]: frac_f=0, ones_f=0 -> (0|0|1) = 1
slot[1]: frac_e=0, ones_e=0 -> (0|0|1) = 1
slot[2]: frac_d=1, ones_d=0 -> (8|0|1) = 9
slot[3]: frac_c=1, ones_c=1 -> (8|4|0) = 12
slot[4]: frac_b=1, ones_b=1 -> (8|4|0) = 12
slot[5]: frac_a=1, ones_a=0 -> (8|0|0) = 8

Result: b29=8, slots=[1, 1, 9, 12, 12, 8]
```

### LCD Send Example

```python
# 7-segment encoding table: digit -> (a, b, c, d, e, f, g)
SEVEN_SEG = {
    0: (1,1,1,1,1,1,0), 1: (0,1,1,0,0,0,0), 2: (1,1,0,1,1,0,1),
    3: (1,1,1,1,0,0,1), 4: (0,1,1,0,0,1,1), 5: (1,0,1,1,0,1,1),
    6: (1,0,1,1,1,1,1), 7: (1,1,1,0,0,0,0), 8: (1,1,1,1,1,1,1),
    9: (1,1,1,1,0,1,1),
}

SLOT_SEG_INDEX = [5, 4, 3, 2, 1, 0]  # slot -> segment: f, e, d, c, b, a
SIDE_BITS = {'L': [1, 1, 1, 0, 0, 0], 'R': [1, 1, 0, 1, 1, 1]}

def calc_lcd_params(side, integer, fractional):
    """Calculate b29 and slots for any digit combination (0-9)."""
    frac_segs = SEVEN_SEG[fractional]  # (a, b, c, d, e, f, g)
    ones_segs = SEVEN_SEG[integer]
    side_bit = 1 if side == 'R' else 0
    b29 = (frac_segs[6] << 3) | (ones_segs[6] << 2) | side_bit

    bit0 = SIDE_BITS[side]
    slots = []
    for i in range(6):
        seg = SLOT_SEG_INDEX[i]
        slots.append((frac_segs[seg] << 3) | (ones_segs[seg] << 2) | bit0[i])

    return b29, slots

def send_lcd(device, side, integer, fractional, counter=0):
    b29, slots = calc_lcd_params(side, integer, fractional)

    # DATA packet
    data = [0] * 64
    data[0] = 0xF0
    data[2] = counter & 0xFF
    data[3] = 0x38
    data[4], data[5] = 0x01, 0xB9
    data[8], data[9] = 0x02, 0x01
    data[17] = 0x24
    data[25] = 0x04
    data[29] = b29
    for i, pos in enumerate([33, 37, 41, 45, 49, 53]):
        data[pos] = slots[i]
    data[57], data[58] = 0x01, 0xB9
    device.write(data)

    # COMMIT packet
    commit = [0] * 64
    commit[0] = 0xF0
    commit[2] = (counter + 1) & 0xFF
    commit[3] = 0x0E
    commit[5], commit[6] = 0x03, 0x01
    device.write(commit)

# Usage (integer and fractional both support 0-9):
# send_lcd(device, 'L', 0, 5)  # Display "L 0.5"
# send_lcd(device, 'R', 1, 0)  # Display "R 1.0"
# send_lcd(device, 'L', 2, 3)  # Display "L 2.3"
```

### Known Limitations

- Formula verified against all 20 captured values (integers 0 and 1). Integers 2-9 are calculated from the same generalized formula and **verified on hardware**.
- Decimal point display is implicit (always shown between integer and fractional).
- The display shows one side at a time; to update both L and R, send two DATA+COMMIT sequences.

---

## Tools and Scripts


### LCD Controller

```powershell
python scripts/lcd_analysis/lcd_controller.py L 0.5
python scripts/lcd_analysis/lcd_controller.py R 1.0
python scripts/lcd_analysis/lcd_controller.py --list
python scripts/lcd_analysis/lcd_controller.py --interactive
```


### USB Capture

```powershell
"C:\Program Files\Wireshark\tshark.exe" -i 13 -a duration:30 -w captures/output.pcapng
```

---

## Setup

### Environment

```powershell
conda activate ursa
pip install hidapi pyusb pyshark pandas matplotlib
```

### Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| hidapi | 0.15.0 | HID communication |
| pyusb | 1.3.1 | Alternative USB (optional) |
| pyshark | 0.6 | PCAP analysis |
| pandas | 3.0.0 | Data processing |
| matplotlib | 3.10.8 | Visualization |

---

## Revision History

| Date | Change |
|------|--------|
| 2026-01-31 | Initial reverse engineering. Reports 0x01, 0x02 fully mapped. |
| 2026-02-04 | LCD protocol (Report 0xF0) decoded. Lookup table with 20 values. |
| 2026-02-05 | **LCD encoding fully decoded as packed 7-segment data.** Slots can be calculated for any digit 0-9 without USB captures. |
| 2026-02-05 | **Bit 2 reinterpreted:** not a binary 0/1 flag but the 7-segment bit of the integer digit. b29 bit 2 carries ones_g. Formula generalized to support integers 0-9. Hardware validator added. |

# TCA Sidestick (VID 0x044F, PID 0x0405) — HID Input Report

## Summary
The sidestick sends an interrupt IN HID input report containing:
- X axis (roll)
- Y axis (pitch)
- Z twist (yaw/rotation)
- Slider (throttle on base / mini axis)
- Hat switch
- Two button bytes

All multi-byte fields are **little-endian**.

## Structure (validated field layout)
Offset | Size | Name       | Type      | Notes / Physical control
------ | ---- | ---------- | --------- | -------------------------
0      | 1    | reportId   | uint8     | Report ID (often 0x01)
1      | 2    | x_axis     | uint16 LE | Stick X (roll)
3      | 2    | y_axis     | uint16 LE | Stick Y (pitch)
5      | 1    | z_twist    | uint8     | Stick twist (yaw)
6      | 1    | slider     | uint8     | Base slider axis
7      | 1    | hat_sw     | uint8     | Hat switch state
8      | 1    | buttons_a  | uint8     | Buttons byte A (bits)
9      | 1    | buttons_b  | uint8     | Buttons byte B (bits)

Total (struct): 10 bytes

## Hat switch values
Value | Meaning
----- | -------
0x0   | North
0x1   | North-East
0x2   | East
0x3   | South-East
0x4   | South
0x5   | South-West
0x6   | West
0x7   | North-West
0xF   | Center / released (common HID convention)

## Button bytes
`buttons_a` and `buttons_b` are bitfields:
- bit0 = LSB
- bit7 = MSB

Bit-level mapping depends on the exact physical labeling and is documented by toggling one button at a time.
See `docs/methodology.md` for the mapping workflow.


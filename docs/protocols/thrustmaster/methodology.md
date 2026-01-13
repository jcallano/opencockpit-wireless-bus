# Methodology: How to Map Bits to Physical Controls

## Goal
Identify exactly which bit corresponds to each physical switch/button on the device.

## Procedure (recommended)
1. Flash the firmware and open Serial Monitor @ 115200
2. Ensure the device is detected and you see:
   - HID IF / EP / MPS
3. For each physical control:
   - Start from neutral
   - Toggle/press ONLY that control
   - Note which byte changed (e.g., btn_byte_01) and which bit flipped
4. Record results in a table:
   - Device (stick/quadrant)
   - Byte name
   - Bit number
   - Physical label
   - State meaning (ON/OFF or momentary)

## Tip: binary output
The firmware prints button bytes in binary:
- Leftmost is bit7
- Rightmost is bit0

Example:
`B01:00100000` means bit5 is set.

## Deliverable format
When contributing, provide:
- VID/PID
- reportId
- report length
- Byte/bit mapping table
- Any notes on latching vs momentary behavior

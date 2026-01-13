# WinWing MCDU Protocol Documentation

## Device Information
* **Vendor ID:** `0x4098`
* **Product ID:** `0xBB36` (WinWing MCDU)

## Display Geometry
The physical screen is mapped as a character grid:
* **Rows:** 14
* **Columns:** 24
* **Total Characters:** 336

### Data Structure (Per Character)
Each character position uses **3 Bytes**:
1. **Color Low Byte**
2. **Color High Byte**
3. **ASCII Character Code**

Total Visual Data: $14 \times 24 \times 3 = 1008$ bytes.

## USB Transfer Protocol
The device uses raw HID Interrupt transfers (64 bytes).

### 1. Initialization
A specific sequence of commands starting with `0xF0` must be sent at startup to configure margins and line stride. See `INIT_SEQUENCE` in source code.

### 2. The "Buffer Overflow" Issue
The MCDU microcontroller has a slow input buffer. 
* **Incorrect Behavior:** Bursting 1024 bytes at full USB speed causes the buffer to overflow. The chip clears the buffer to recover, resulting in **flickering** and **misalignment** (screen tearing).
* **Correct Behavior (Pacing):** The host must wait approximately **2ms** between each 64-byte USB packet.

### 3. Wrap-Around Logic
The hardware buffer size is exactly **1024 bytes** (1008 visual + 16 padding).
* If you write exactly 1024 bytes, the internal cursor automatically wraps back to `(0,0)`.
* **DO NOT** send the `Reset Cursor` command (`F0...`) between frames. Doing so interrupts the writing process and causes visual glitches.

### 4. Packet Structure
Data is sent in chunks of 63 bytes, prefixed by `0xF2`.

| Byte 0 | Byte 1..63 |
|--------|------------|
| `0xF2` | Pixel Data |

## Color Map (Approximate)
| Color  | Hex Value |
|--------|-----------|
| Green  | `0x0084`  |
| White  | `0x0042`  |
| Cyan   | `0x0063`  |
| Amber  | `0x0021`  |
| Magenta| `0x00A5`  |
| Red    | `0x00C6`  |
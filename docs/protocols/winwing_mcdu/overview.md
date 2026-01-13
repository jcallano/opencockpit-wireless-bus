# WinWing MCDU Reverse Engineered Driver

**An open-source, reverse-engineered driver for the WinWing Airbus MCDU hardware.**

This project provides the necessary protocol documentation and proof-of-concept code to interface with the WinWing MCDU without using the official "SimApp Pro" software. It is designed to facilitate the creation of wireless bridges (using ESP32) or custom integrations for flight simulators.

## 📚 Acknowledgements & Sources

This project stands on the shoulders of giants. Special thanks to the initial reverse engineering efforts by the community:

* **XSchenFly (WinWing MCDU for X-Plane):** * Original repository: [github.com/schenlap/winwing_mcdu](https://github.com/schenlap/winwing_mcdu)
    * This repository provided the foundational initialization sequences (`0xF0...`) and the memory mapping logic used for the Toliss Airbus on Linux.

* **Packet Analysis:**
    * Timing strategies were derived from analyzing raw USB traffic dumps (`.pcap`) from the official Windows driver, isolating the specific timing constraints required by the hardware's microcontroller.

## ⚙️ Technical Findings: The "Paced Streaming" Strategy

One of the most critical discoveries of this project is the handling of USB transmission timing.

### 1. Official Behavior: The "Ping-Pong" Protocol
The official SimApp Pro software utilizes a safe, bidirectional handshake protocol (Request-Response).
1.  **Host** sends a 64-byte data chunk.
2.  **Device** processes it and sends an acknowledgement (ACK/Pong).
3.  **Host** waits for the ACK before sending the next chunk.

While safe, this method introduces latency due to USB polling intervals and software overhead.

### 2. Our Implementation: "Paced Streaming" (Fire-and-Forget)
Through stress testing, we discovered that the device's microcontroller does not *require* the handshake to process data. It simply needs time to clear its input buffer.

We implemented a **Unidirectional Paced Streaming** strategy:
* We ignore the return "Pong" signal.
* Instead, we introduce a precise **micro-delay (~2ms)** between every 64-byte packet sent by the host.
* This matches the hardware's internal "digestion" speed, preventing buffer overflows and screen corruption (tearing/flickering) without the overhead of waiting for a response.

## 📝 Protocol Specification

### Display Geometry
* **Resolution:** 14 Lines × 24 Columns.
* **Character Depth:** 3 Bytes (Color Low, Color High, Char).
* **Visual Data:** 1008 Bytes.

### Memory Layout & Wrap-Around
The hardware uses an internal ring buffer of exactly **1024 Bytes**.
* **Bytes 0-1007:** Visual Data.
* **Bytes 1008-1023:** Padding (Ignored/Control).

**Critical Finding:** When exactly 1024 bytes are written to the device, the internal write cursor automatically wraps back to position `(0,0)`. 
* **Do NOT** send "Reset Cursor" commands between frames. Doing so interrupts the hardware's write cycle and causes flickering.
* Only sync the cursor once at startup.

### USB Packet Structure
Data is sent via USB HID Interrupt Transfers (Endpoint 1).
* **Header:** `0xF2`
* **Payload:** 63 Bytes of pixel data.

## 📂 Repository Structure

| Path | Description |
|------|-------------|
| `src/python/mcdu-driver.py` | **Golden Master** Python driver with 2ms pacing. |
| `src/esp32/esp32s3mcdudriverexample.ino` | ESP32-S3 USB Host implementation (see below). |
| `docs/protocol.md` | Detailed protocol notes and initialization sequences. |
| `captures/` | Wireshark dumps used for timing analysis. |

---

## 🔧 ESP32-S3 Implementation

### `src/esp32/esp32s3mcdudriverexample.ino`

**Purpose:** ESP32-S3 USB Host driver that interfaces directly with the WinWing MCDU hardware, enabling wireless cockpit integration.

**Hardware Requirements:**
- WeAct ESP32-S3 Dev Board (or compatible with USB Host support)
- USB cable to connect MCDU
- Pin configuration for VBUS control (see code for details)

**Key Features:**

1. **USB Host Driver**
   - Auto-detects WinWing MCDU (VID: 0x4098, PID: 0xBB36)
   - Claims HID interface and discovers endpoints
   - Handles both IN (buttons) and OUT (display) endpoints

2. **Display Buffer Management**
   - 1024-byte ring buffer matching hardware layout
   - 14 rows x 24 columns x 3 bytes per character
   - Color palette with WinWing-specific color codes

3. **Button Input Handling**
   - Reads 12 bytes of button state (96 buttons)
   - Edge detection for press events
   - Callback-based button ID reporting

4. **Initialization Sequence**
   - 17-packet initialization sequence (captured from official driver)
   - Brightness configuration for keys and display

**Color Codes (WinWing Proprietary):**
```cpp
#define COLOR_BLACK   0x0000
#define COLOR_AMBER   0x0021
#define COLOR_WHITE   0x0042
#define COLOR_CYAN    0x0063
#define COLOR_GREEN   0x0084
#define COLOR_MAGENTA 0x00A5
#define COLOR_RED     0x00C6
#define COLOR_YELLOW  0x00E7
```

**State Machine:**
```
STATE_WAITING → STATE_CONNECTED → STATE_CONFIGURING → STATE_RUNNING
```

**API Functions:**
- `clearBuffer()` — Fill display with spaces
- `drawChar(row, col, char, color)` — Write single character
- `flushDisplayBuffer()` — Send buffer to hardware (with pacing)
- Button events reported to serial as "BOTON PULSADO: ID X"

**Usage:**
1. Flash to ESP32-S3 with USB Host capability
2. Connect MCDU via USB (may require powered hub or VBUS mod)
3. Monitor serial output at 115200 baud
4. Button presses display on screen and serial

## ⚠️ Disclaimer
This is an unofficial project and is not affiliated with WinWing. Use this driver at your own risk. Reverse engineering hardware may void your warranty.
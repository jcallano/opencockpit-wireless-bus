# ESP32-S3 USB Host for WinWing Ursa Minor 32 — Implementation Guide

Complete reference for reproducing and integrating the standalone USB Host firmware
into the OpenCockpit Wireless Avionics Bus network.

**Hardware:** ESP32-S3 (WeAct Studio) + WinWing Ursa Minor 32 Throttle
**Firmware:** `firmware/node_d.ino` (~1000 lines, single file, zero external dependencies)
**Verified:** 2026-02-07

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Build Environment](#3-build-environment)
4. [Critical Lessons Learned](#4-critical-lessons-learned)
5. [Code Structure (15 Sections)](#5-code-structure)
6. [USB Host Lifecycle](#6-usb-host-lifecycle)
7. [HID Report Descriptor Parsing](#7-hid-report-descriptor-parsing)
8. [Output Method (Interrupt OUT)](#8-output-method)
9. [Report 0x02 — Auxiliary Controls](#9-report-0x02--auxiliary-controls)
10. [Report 0xF0 — LCD Display](#10-report-0xf0--lcd-display)
11. [Report 0x01 — Input (Buttons + Axes)](#11-report-0x01--input)
12. [Synchronization Model](#12-synchronization-model)
13. [Integration into Wireless Avionics Network](#13-integration-into-wireless-avionics-network)
14. [Serial Command Reference](#14-serial-command-reference)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│  ESP32-S3                                           │
│                                                     │
│  Core 0 (pri 20)          Core 1 (pri 1)            │
│  ┌──────────────┐         ┌──────────────────────┐  │
│  │  usb_task()   │         │  loop()              │  │
│  │  - lib events │         │  - Serial commands   │  │
│  │  - client evt │         │  - Setup sequence    │  │
│  │  - callbacks  │         │  - send_report02()   │  │
│  └──────────────┘         │  - send_lcd()        │  │
│                           └──────────────────────┘  │
│                                                     │
│  Shared Resources (mutex/semaphore protected):      │
│  - g_xfer_ctrl  (control endpoint 0)                │
│  - g_xfer_out   (interrupt OUT EP 0x02)             │
│  - g_xfer_in    (interrupt IN EP 0x81)              │
└───────────────┬─────────────────────────────────────┘
                │ USB Full-Speed (12 Mbps)
                ▼
┌─────────────────────────────────────────────────────┐
│  WinWing Ursa Minor 32 Throttle                     │
│  VID=0x4098  PID=0xB920                             │
│  Interface 0, Class 3 (HID)                         │
│  EP 0x81 IN  (MPS 64) — buttons/axes               │
│  EP 0x02 OUT (MPS 64) — LEDs/backlight/LCD          │
└─────────────────────────────────────────────────────┘
```

**Design Principles:**
- Single .ino file, no external headers (no `hardware_config.h`, `protocol.h`, etc.)
- USB task on Core 0 handles all driver events at high priority
- All output is synchronous (waits for transfer completion before returning)
- Report sizes discovered at runtime from HID report descriptor
- Output via Interrupt OUT endpoint (EP 0x02) — the only method this device responds to

---

## 2. Hardware Requirements

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3 with USB-OTG peripheral |
| Board | WeAct Studio ESP32-S3 (or any S3 with USB-A host) |
| UART0 TX | GPIO 43 (default) |
| UART0 RX | GPIO 44 (default) |
| USB Host Power | Board-specific (set `PIN_USB_PWR` or -1 to disable) |
| Status LED | Board-specific (set `PIN_LED_STATUS` or -1 to disable) |

**Important:** The USB-OTG port on the ESP32-S3 is used for the USB Host connection
to the throttle. Serial communication uses UART0 (separate pins), NOT USB CDC.

---

## 3. Build Environment

### arduino-cli.env
```bash
export ARDUINO_BOARD_manager=esp32:esp32
export ARDUINO_BOARD_fqbn=esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default
export ARDUINO_LIBS_DIR=../../common/libs
```

### FQBN Options (CRITICAL)

| Option | Value | Meaning |
|--------|-------|---------|
| `USBMode` | `default` | USB-OTG (TinyUSB) — required for USB Host |
| `CDCOnBoot` | `default` | Disabled — prevents USB being claimed as device |

**Do NOT use:** `USBMode=OTG`, `CDCOnBoot=0`, `CDCOnBoot=cdc`.
These are invalid values that cause build errors. The valid values come from
`arduino-cli board details --fqbn esp32:esp32:esp32s3`.

### Compile & Upload

The sketch must be in a folder matching the .ino filename:

```bash
ACLI="path/to/arduino-cli.exe"
FQBN="esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default"
SKETCH="path/to/firmware"

# Create matching folder (arduino-cli requirement)
mkdir -p "${SKETCH}/node_d"
cp "${SKETCH}/node_d.ino" "${SKETCH}/node_d/"

# Compile
$ACLI compile --fqbn "$FQBN" "${SKETCH}/node_d"

# Upload (close serial monitor first!)
$ACLI upload --fqbn "$FQBN" --port COM6 "${SKETCH}/node_d"

# Cleanup
rm -rf "${SKETCH}/node_d"
```

---

## 4. Critical Lessons Learned

### 4.1 Control Transfers: `usb_host_transfer_submit_control()`

**ESP-IDF v5.x / Arduino ESP32 v3.x** requires:
```c
// CORRECT — for control transfers (endpoint 0)
usb_host_transfer_submit_control(g_client, g_xfer_ctrl);

// WRONG — returns ESP_ERR_INVALID_ARG (258)
usb_host_transfer_submit(g_xfer_ctrl);
```

`usb_host_transfer_submit()` is only for interrupt/bulk/isochronous endpoints.
Control transfers MUST use the `_control` variant, which also requires the client handle.

### 4.2 Report Sizes Must Match HID Descriptor

The device **silently ignores** packets with incorrect sizes. The Windows HID driver
always sends the exact size defined in the HID report descriptor. You must do the same.

| Report | HID Descriptor Says | Actual Wire Size |
|--------|-------------------|-----------------|
| 0x02 (Aux) | REPORT_SIZE=8, REPORT_COUNT=13 | **14 bytes** (104 bits + 1 ID byte) |
| 0xF0 (LCD) | REPORT_SIZE=8, REPORT_COUNT=63 | **64 bytes** (504 bits + 1 ID byte) |

Sending 64 bytes for Report 0x02 completes without error but the device ignores it.
**Always parse the HID report descriptor to discover actual sizes.**

### 4.3 Do NOT Send Set_Idle / Set_Protocol

The Python scripts (which work correctly on Windows) do NOT send these HID class
requests. The WinWing device may not expect them. Omitting them is safe and matches
the behavior of the Windows HID driver for this specific device.

### 4.4 Semaphores: Create Once

The original code had a bug creating `g_ctrl_sem` twice (lines 773 and 791),
leaking the first semaphore. All semaphores must be created exactly once in `setup()`.

### 4.5 Transfer Buffer Allocation

Allocate buffers large enough for the largest possible report + setup packet overhead:
```c
usb_host_transfer_alloc(64,   0, &g_xfer_in);    // IN reports are 64 bytes max
usb_host_transfer_alloc(1024, 0, &g_xfer_out);   // OUT reports (size from descriptor)
usb_host_transfer_alloc(1024, 0, &g_xfer_ctrl);  // setup packet (8) + report data
```

---

## 5. Code Structure

The firmware is organized in 15 sequential sections:

| Section | Lines | Purpose |
|---------|-------|---------|
| 1 | Includes + Pin Defines | `Arduino.h`, `usb/usb_host.h`, GPIO config |
| 2 | Device Constants + Globals | VID/PID, transfer handles, semaphores, state |
| 3 | Button Names (42) | String table for input report logging |
| 4 | 7-Segment LCD Tables | `SEVEN_SEG[10][7]`, slot mappings, side bits |
| 5 | `send_ctrl_sync()` | Mutex-protected synchronous control transfer (used for GET_DESCRIPTOR) |
| 6 | `send_report02()` | Backlight/LED/motor helper (Report 0x02) via Interrupt OUT |
| 8 | LCD encode + send | `encode_lcd()`, `build_lcd_commit()`, `send_lcd()` |
| 9 | Input report parser | Delta-based button/axis change logging |
| 10 | Endpoint discovery | Config descriptor scan + HID descriptor extraction |
| 11 | USB event callback | Device connect/disconnect handling |
| 12 | USB Host task | FreeRTOS task on Core 0, priority 20 |
| 13 | Device setup sequence | Fetch HID descriptor, init backlights |
| 14 | Serial command processor | `LED`, `BL`, `VIB`, `LCD`, `STATUS`, `HELP` |
| 15 | `setup()` + `loop()` | Arduino entry points |

---

## 6. USB Host Lifecycle

```
setup()
  ├── Create semaphores (g_ctrl_mux, g_ctrl_sem, g_out_sem)
  ├── usb_host_install()
  ├── usb_host_client_register()  →  callback = usb_event_cb
  ├── Allocate transfers (IN, OUT, CTRL)
  └── Start usb_task on Core 0

usb_task (Core 0, runs forever)
  └── usb_host_lib_handle_events() + usb_host_client_handle_events()

usb_event_cb (NEW_DEV)
  ├── usb_host_device_open()
  ├── Check VID/PID (0x4098/0xB920)
  ├── discover_endpoints()  →  find EP 0x81 IN, EP 0x02 OUT, HID desc len
  ├── usb_host_interface_claim()
  ├── Start IN transfers (submit g_xfer_in)
  └── Set g_connected = true

loop() (Core 1, detects g_connected && !g_setup_done)
  └── device_setup_sequence()
        ├── fetch_report_descriptor()  →  parse OUTPUT report sizes
        ├── Init backlights (4 zones, val=255)
        └── Set g_setup_done = true

usb_event_cb (DEV_GONE)
  ├── usb_host_interface_release()
  ├── usb_host_device_close()
  └── Reset all state flags
```

**Why setup sequence runs from loop(), not from the callback:**
The USB event callback runs in the context of `usb_host_client_handle_events()`.
Control transfers submitted from the callback would deadlock because the USB task
is blocked inside the same function. The setup sequence uses control transfers
(fetch descriptor, SET_REPORT for backlights), so it must run from a different context.

---

## 7. HID Report Descriptor Parsing

### How It Works

1. During `discover_endpoints()`, the HID descriptor (type 0x21) is found in the
   config descriptor. Its `wDescriptorLength` field tells us how long the HID
   Report Descriptor is.

2. During `device_setup_sequence()`, `fetch_report_descriptor()` sends a
   GET_DESCRIPTOR control transfer:
   ```
   bmRequestType = 0x81  (Device→Host, Standard, Interface)
   bRequest      = 0x06  (GET_DESCRIPTOR)
   wValue        = 0x2200 (HID Report descriptor, index 0)
   wIndex        = interface number
   wLength       = descriptor length from step 1
   ```

3. The returned descriptor is parsed to find OUTPUT items. For each OUTPUT item,
   the current REPORT_ID, REPORT_SIZE, and REPORT_COUNT are used to calculate
   the total bits. The final byte size is `(bits + 7) / 8 + 1` (the +1 is the
   Report ID byte, which is NOT counted in the descriptor's bit fields).

### Ursa Minor HID Report Descriptor (decoded)

```
Report ID 0x01 (INPUT — Joystick):
  42 buttons (REPORT_SIZE=1, REPORT_COUNT=42)
  6 padding bits (REPORT_SIZE=1, REPORT_COUNT=6)
  8 axes × 16-bit each (X, Y, Z, Rx, Ry, Rz, Slider, Dial)
  7 vendor-specific × 16-bit (REPORT_COUNT=7, constant)
  Total: 504 bits = 63 bytes + 1 ID = 64 bytes

Report ID 0x02 (OUTPUT — Auxiliary):
  13 bytes (REPORT_SIZE=8, REPORT_COUNT=13)
  Total: 104 bits = 13 bytes + 1 ID = 14 bytes

Report ID 0xF0 (OUTPUT — LCD):
  63 bytes (REPORT_SIZE=8, REPORT_COUNT=63)
  Total: 504 bits = 63 bytes + 1 ID = 64 bytes
```

---

## 8. Output Method

### Interrupt OUT (EP 0x02)

All output is sent via the Interrupt OUT endpoint (0x02) using `usb_host_transfer_submit()`.
This matches how the Windows HID driver sends output reports via `WriteFile()` /
Python `hidapi.device.write()`.

```c
esp_err_t send_out_sync(const uint8_t *data, size_t len) {
    xSemaphoreTake(g_out_sem, 0);           // clear
    memcpy(g_xfer_out->data_buffer, data, len);
    g_xfer_out->bEndpointAddress = g_ep_out; // 0x02
    g_xfer_out->num_bytes = len;
    usb_host_transfer_submit(g_xfer_out);    // interrupt endpoint
    xSemaphoreTake(g_out_sem, timeout);      // wait for out_cb
}
```

**Important:** SET_REPORT control transfers (HID class request 0x09 on endpoint 0)
do NOT work for this device. The device accepts the transfer without error, but
silently ignores Report 0x02 sent via SET_REPORT. All output must go through the
Interrupt OUT endpoint.

---

## 9. Report 0x02 — Auxiliary Controls

### Wire Format (14 bytes total)

```
Byte  0: 0x02           Report ID
Byte  1: cmd_type        0x10 or 0x01
Byte  2: 0xB9           Fixed header
Byte  3: 0x00           Fixed
Byte  4: 0x00           Fixed
Byte  5: 0x03           Fixed
Byte  6: 0x49           Fixed
Byte  7: control_id     Identifies the control
Byte  8: value           0-255
Bytes 9-13: 0x00         Padding (to fill 14-byte report)
```

### Control Map

| Category | cmd_type | control_id | Name | Values |
|----------|----------|------------|------|--------|
| Backlight | 0x10 | 0x00 | Throttle Backlight | 0-255 |
| Backlight | 0x10 | 0x02 | Marker Backlight | 0-255 |
| Backlight | 0x01 | 0x00 | Flaps/Spoiler Backlight | 0-255 |
| Backlight | 0x01 | 0x02 | LCD Backlight | 0-255 |
| LED | 0x10 | 0x03 | ENG 1 FAULT | 0=OFF, 255=ON |
| LED | 0x10 | 0x04 | ENG 1 FIRE | 0=OFF, 255=ON |
| LED | 0x10 | 0x05 | ENG 2 FAULT | 0=OFF, 255=ON |
| LED | 0x10 | 0x06 | ENG 2 FIRE | 0=OFF, 255=ON |
| Motor | 0x10 | 0x0E | Motor 1 (Left) | 0-255 |
| Motor | 0x10 | 0x0F | Motor 2 (Right) | 0-255 |

### API

```c
esp_err_t send_report02(uint8_t type, uint8_t id, uint8_t val);
// Example: send_report02(0x10, 0x00, 128);  // Throttle backlight 50%
```

---

## 10. Report 0xF0 — LCD Display

### Protocol: DATA + COMMIT (2 packets)

Each LCD update requires two 64-byte packets sent sequentially.

#### DATA Packet (type 0x38)

| Byte | Value | Description |
|------|-------|-------------|
| 0 | 0xF0 | Report ID |
| 2 | counter++ | Incrementing counter |
| 3 | 0x38 | DATA type |
| 4-5 | 0x01, 0xB9 | Constant |
| 8-9 | 0x02, 0x01 | Constant |
| 17 | 0x24 | Constant |
| 25 | 0x04 | Constant |
| 29 | b29 | Side + segment G encoding |
| 33,37,41,45,49,53 | slots[0-5] | Segment data per slot |
| 57-58 | 0x01, 0xB9 | Constant |

#### COMMIT Packet (type 0x0E)

| Byte | Value | Description |
|------|-------|-------------|
| 0 | 0xF0 | Report ID |
| 2 | counter++ | Incrementing counter |
| 3 | 0x0E | COMMIT type |
| 5-6 | 0x03, 0x01 | Constant |

### 7-Segment Encoding

The display has 3 digit positions: **tens**, **ones**, **fractional** (range 0.0-99.9).

```
  aaa         Segment table:
 f   b        Digit  a b c d e f g
  ggg         0      1 1 1 1 1 1 0
 e   c        1      0 1 1 0 0 0 0
  ddd         2      1 1 0 1 1 0 1
              3      1 1 1 1 0 0 1
              4      0 1 1 0 0 1 1
              5      1 0 1 1 0 1 1
              6      1 0 1 1 1 1 1
              7      1 1 1 0 0 0 0
              8      1 1 1 1 1 1 1
              9      1 1 1 1 0 1 1
```

#### b29 Encoding (4 bits)

```
b29 = (frac_g << 3) | (ones_g << 2) | (tens_g << 1) | side_bit
```

- Bit 3: Segment G of fractional digit
- Bit 2: Segment G of ones digit
- Bit 1: Segment G of tens digit (0 when tens < 1, i.e. value < 10)
- Bit 0: Side (0=L, 1=R)

#### Slot Encoding (4 bits per slot, 6 slots)

```
slot[i] = (frac_seg << 3) | (ones_seg << 2) | (tens_seg << 1) | side_pattern[i]
```

Slot-to-segment mapping: slot0=f, slot1=e, slot2=d, slot3=c, slot4=b, slot5=a

Side patterns (bit 0):
- L: `[1, 1, 1, 0, 0, 0]`
- R: `[1, 1, 0, 1, 1, 1]`

Tens digit is blanked (all segments 0) when value < 10.

### API

```c
esp_err_t send_lcd(char side, uint8_t integer, uint8_t fractional);
// Example: send_lcd('L', 12, 5);  // Display "L 12.5"
// Example: send_lcd('R', 3, 8);   // Display "R 3.8"
```

---

## 11. Report 0x01 — Input

### Wire Format (64 bytes, received via IN endpoint 0x81)

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0 | Report ID | uint8 | Always 0x01 |
| 1-6 | Buttons | 42 bits | Bit-packed, LSB first |
| 7-12 | X/Y/Z axes | uint16 LE x3 | Unused (constant) |
| 13-14 | Rx | uint16 LE | LEFT THROTTLE (0-65535) |
| 15-16 | Ry | uint16 LE | RIGHT THROTTLE (0-65535) |
| 17-18 | Rz | uint16 LE | Unused (constant) |
| 19-20 | Slider | uint16 LE | SPOILER (0-65535) |
| 21-22 | Dial | uint16 LE | FLAPS (0-65535) |

### Delta Logging

The input parser only logs changes (XOR with previous report):
- Buttons: logs `BTN XX [NAME]: PRESS/REL`
- Axes: logs `AXIS_NAME: value` only when changed

First report is silently captured as baseline (no spam on connect).

---

## 12. Synchronization Model

```
┌──────────────────────────────────────────┐
│  Control Transfers (endpoint 0)          │
│                                          │
│  g_ctrl_mux (Mutex)                      │
│    └── Ensures single control xfer       │
│        at a time (from any task/context) │
│                                          │
│  g_ctrl_sem (Binary Semaphore)           │
│    └── Signaled by ctrl_cb() on          │
│        transfer completion               │
│    └── Waited by send_ctrl_sync()        │
│                                          │
│  Flow:                                   │
│    Take mutex → clear sem → submit →     │
│    wait sem → release mutex              │
└──────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│  Interrupt OUT Transfers (endpoint 0x02) │
│                                          │
│  g_out_sem (Binary Semaphore)            │
│    └── Signaled by out_cb() on           │
│        transfer completion               │
│    └── Waited by send_out_sync()         │
│                                          │
│  Flow:                                   │
│    Clear sem → submit → wait sem         │
│    (no mutex needed — single caller)     │
└──────────────────────────────────────────┘
```

**Important:** `send_ctrl_sync()` and `send_out_sync()` are both blocking.
They must NOT be called from the USB task (Core 0) or from USB callbacks,
as this would deadlock. They are safe to call from `loop()` (Core 1).

---

## 13. Integration into Wireless Avionics Network

### Step 1: Add ESP-NOW Back (Selectively)

Re-add WiFi/ESP-NOW initialization **after** the USB Host is working.
Key considerations:

```c
// In setup(), AFTER usb_host_install():
WiFi.mode(WIFI_STA);
esp_now_init();
esp_now_register_recv_cb(espnow_recv_cb);
```

**Do NOT** call `setupHardware()` or `enableUsbHostPower()` from external headers.
These functions may reconfigure GPIOs or clocks that conflict with the USB Host.

### Step 2: Bridge Input Reports to ESP-NOW

Replace the serial logging in `process_input_report()` with ESP-NOW forwarding:

```c
static void process_input_report(const uint8_t *data, size_t len) {
    // Delta logging (keep for debug)
    // ...existing code...

    // Forward to coordinator via ESP-NOW
    if (g_have_coordinator) {
        HIDInputPayload payload = {};
        payload.device_id = DEV_URSA_MINOR;
        payload.report_id = data[0];
        payload.report_length = len;
        memcpy(payload.report_data, data, len);

        uint8_t buffer[MAX_ESPNOW_PAYLOAD];
        size_t msg_len = buildMessage(buffer, MSG_HID_INPUT,
                                      g_node_id, NODE_COORDINATOR,
                                      &payload, 3 + len);
        esp_now_send(g_coordinator_mac, buffer, msg_len);
    }
}
```

### Step 3: Receive Output Commands via ESP-NOW

Add a protocol handler that calls the proven output functions:

```c
static void handle_hid_output(const uint8_t *payload, int len) {
    const HIDOutputPayload *out = (const HIDOutputPayload *)payload;
    if (out->device_id != DEV_URSA_MINOR) return;

    if (out->report_id == REPORT_ID_AUX && out->report_length >= 9) {
        // Direct Report 0x02 — extract type, id, val
        send_report02(out->report_data[1],   // type
                      out->report_data[7],   // id
                      out->report_data[8]);  // val
    }
    else if (out->report_id == REPORT_ID_LCD) {
        // LCD command — send raw packet via OUT endpoint
        send_out_sync(out->report_data, g_reportF0_size);
    }
}
```

### Step 4: Higher-Level LCD Commands

For the coordinator to send LCD values without encoding them:

```c
// Define a simple LCD command in your protocol
struct LCDCommand {
    uint8_t side;        // 'L' or 'R'
    uint8_t integer;     // 0-99
    uint8_t fractional;  // 0-9
};

// Node D handler:
void handle_lcd_command(const LCDCommand *cmd) {
    send_lcd(cmd->side, cmd->integer, cmd->fractional);
}
```

### Step 5: Task Architecture (Recommended)

```
Core 0 (pri 20): usb_task        — USB Host driver events
Core 1 (pri 5):  protocol_task   — ESP-NOW receive queue processing
Core 1 (pri 1):  loop()          — Serial debug + setup sequence
```

**Do NOT add an `out_task` with a queue.** The synchronous `send_report02()` and
`send_lcd()` functions are proven reliable. Queue-based async output was the source
of bugs in the original code (race conditions, missing completion waits).

### Best Practices for Integration

1. **Keep USB Host standalone first.** Test all output commands via serial before
   adding ESP-NOW. If something breaks, you know it's the network layer.

2. **Use the exact report sizes.** Always use `g_report02_size` (14 bytes) and
   `g_reportF0_size` (64 bytes). Never hardcode 64 for all reports.

3. **Parse HID descriptor at runtime.** Different firmware versions of the same
   device may have different report sizes. The descriptor parser handles this.

4. **Don't send Set_Idle/Set_Protocol.** The WinWing device doesn't need them
   and they may interfere with operation.

5. **Synchronous output only.** Don't fire-and-forget. Always wait for the
   transfer completion semaphore before sending the next packet.

6. **Allocate transfer buffers generously.** 1024 bytes for OUT and CTRL buffers
   costs only 2KB RAM but prevents overflow if report sizes change.

7. **Control transfers from Core 1 only.** Never call `send_ctrl_sync()` from
   USB callbacks or the USB task — it will deadlock.

---

## 14. Serial Command Reference

| Command | Args | Example | Description |
|---------|------|---------|-------------|
| `LED` | `<id> <val> [type]` | `LED 0x03 255` | Set LED (default type=0x10) |
| `BL` | `<val>` | `BL 128` | All 4 backlight zones |
| `VIB` | `<id> <val>` | `VIB 0x0E 128` | Vibration motor |
| `VIBTEST` | — | `VIBTEST` | Sweep both motors |
| `LCD` | `<L\|R> <val>` | `LCD L 12.5` | 7-segment (0.0-99.9) |
| `STATUS` | — | `STATUS` | Connection info |
| `HELP` | — | `HELP` | Print command list |

Values support hex (`0x0E`) and decimal (`14`) notation.

---

## 15. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `CTRL submit err=258` | Using `usb_host_transfer_submit()` for EP 0 | Use `usb_host_transfer_submit_control(g_client, xfer)` |
| Transfer OK but device ignores | Wrong report size (e.g. 64 instead of 14) | Parse HID descriptor, use `g_report02_size` |
| `CTRL timeout` | Deadlock: called from USB task context | Move control transfers to Core 1 (`loop()`) |
| `USB Host install FAILED` | CDCOnBoot enabled, USB claimed as device | Set `CDCOnBoot=default` in FQBN |
| No device detected | Wrong USBMode | Set `USBMode=default` (USB-OTG) in FQBN |
| `Build error: invalid value 'OTG'` | Wrong FQBN option string | Use `USBMode=default` not `USBMode=OTG` |
| Backlights don't change via SET_REPORT | Device ignores SET_REPORT for Report 0x02 | Use Interrupt OUT endpoint only (this is the default) |
| LCD not updating | Timing: need 5ms between DATA and COMMIT | `delay(5)` between the two packets |
| Double button presses on connect | First report delta against zeros | First report stored silently as baseline |

---

## Appendix: HID Report Descriptor (Raw Hex)

```
05 01 09 04 A1 01 85 01 05 09 19 01 29 2A 25 01
35 00 45 01 75 01 95 2A 81 02 75 01 95 06 81 01
05 01 09 30 15 00 27 FF FF 00 00 35 00 47 FF FF
00 00 75 10 95 01 81 02 05 01 09 31 15 00 27 FF
FF 00 00 35 00 47 FF FF 00 00 75 10 95 01 81 02
05 01 09 32 15 00 27 FF FF 00 00 35 00 47 FF FF
00 00 75 10 95 01 81 02 05 01 09 33 15 00 27 FF
FF 00 00 35 00 47 FF FF 00 00 75 10 95 01 81 02
05 01 09 34 15 00 27 FF FF 00 00 35 00 47 FF FF
00 00 75 10 95 01 81 02 05 01 09 35 15 00 27 FF
FF 00 00 35 00 47 FF FF 00 00 75 10 95 01 81 02
05 01 09 36 15 00 27 FF FF 00 00 35 00 47 FF FF
00 00 75 10 95 01 81 02 05 01 09 37 15 00 27 FF
FF 00 00 35 00 47 FF FF 00 00 75 10 95 01 81 02
05 01 15 00 27 FF FF 00 00 35 00 47 FF FF 00 00
09 D0 09 D1 09 D2 09 D3 09 D4 09 D5 09 D6 75 10
95 07 81 01 85 02 06 FF 00 09 01 15 00 26 FF 00
35 00 46 FF 00 75 08 95 0D 81 02 09 02 91 02 85
F0 06 FF 00 09 03 95 3F 81 02 09 04 91 02 C0
```

# OpenCockpit Wireless Avionics Bus
## Design Specification Document

**Version:** 1.1
**Date:** January 2026  
**Target Platform:** MSFS 2020/2024 + Fenix A320  
**Hardware:** ESP32-S3 | ESP-NOW Protocol

---

## Table of Contents

1. [Introduction and Problem Statement](#1-introduction-and-problem-statement)
2. [System Overview](#2-system-overview)
3. [Hardware Architecture](#3-hardware-architecture)
4. [Network Architecture](#4-network-architecture)
5. [ESP-NOW Protocol Specification](#5-esp-now-protocol-specification)
6. [Node Specifications](#6-node-specifications)
7. [Peripheral Integration Protocols](#7-peripheral-integration-protocols)
8. [Message Protocol Specification](#8-message-protocol-specification)
9. [Auto-Configuration System](#9-auto-configuration-system)
10. [Coordinator USB Interface](#10-coordinator-usb-interface)
11. [Software Architecture](#11-software-architecture)
12. [Performance Requirements](#12-performance-requirements)
13. [Development Environment](#13-development-environment)
14. [Testing Strategy](#14-testing-strategy)
15. [Appendix: Reference Documentation](#appendix-reference-documentation)

---

## 1. Introduction and Problem Statement

### 1.1 Background

In DIY home cockpit build scenarios, cable management and integration of different vendor peripherals becomes increasingly complex once more than 3 devices are added to the setup. Traditional USB-based solutions create a web of cables that is difficult to manage, maintain, and troubleshoot.

### 1.2 Problem Statement

Current challenges in home cockpit builds include:

- Complex cable routing between multiple USB peripherals and the host PC
- Limited USB ports requiring hubs that can introduce latency and power issues
- Vendor-specific software creating conflicts and resource contention
- Difficulty in physical placement due to cable length constraints
- Maintenance complexity when troubleshooting connection issues
- Lack of standardized communication between different vendor hardware

### 1.3 Project Objective

Validate an architecture that maintains a **low-latency, connectionless wireless bus network** for interconnecting different cockpit peripherals to flight simulator software running on a PC.

### 1.4 Technology Selection Rationale

After risk-benefit analysis between BLE Mesh, Zigbee, ESP-NOW, and Matter protocols:

| Protocol | Latency | Overhead | Complexity | Selection |
|----------|---------|----------|------------|-----------|
| BLE Mesh | ~50-100ms | High | High | ❌ |
| Zigbee | ~15-30ms | Medium | High | ❌ |
| Matter | ~20-50ms | High | Very High | ❌ |
| **ESP-NOW** | **~3ms** | **Low** | **Low** | ✅ |

**ESP-NOW selected based on:**
- Low protocol overhead (no connection establishment required)
- Demonstrated low latency (~3ms RTT achievable with optimization)
- Connectionless nature enabling quick recovery from packet loss
- Native support in ESP32 ecosystem with mature libraries
- Peer-to-peer communication without access point requirement

---

## 2. System Overview

### 2.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              HOST PC                                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────────────┐  │
│  │   MSFS      │  │   Fenix     │  │      Bridge Software            │  │
│  │  2020/2024  │◄─┤    A320     │◄─┤  (SLIP Protocol Handler)        │  │
│  └─────────────┘  └─────────────┘  └───────────────┬─────────────────┘  │
└────────────────────────────────────────────────────┼────────────────────┘
                                                     │ USB CDC Serial
                                                     │ (SLIP encoded)
                                                     ▼
                           ┌─────────────────────────────────────────┐
                           │           NODE A - COORDINATOR          │
                           │              (ESP32-S3)                 │
                           │                                         │
                           │  ┌───────────────────────────────────┐  │
                           │  │         USB CDC Serial            │  │
                           │  │      (SLIP Protocol I/O)          │  │
                           │  └───────────────┬───────────────────┘  │
                           │                  │                      │
                           │          ┌───────┴───────┐              │
                           │          │   Message     │              │
                           │          │   Router      │              │
                           │          └───────┬───────┘              │
                           │                  │                      │
                           │          ┌───────┴───────┐              │
                           │          │  ESP-NOW      │              │
                           │          │  Coordinator  │              │
                           │          └───────┬───────┘              │
                           └──────────────────┼──────────────────────┘
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    │                         │                         │
                    ▼                         │                         ▼
   ┌────────────────────────────┐             │        ┌────────────────────────────┐
   │      NODE B - PERIPHERAL   │             │        │      NODE C - PERIPHERAL   │
   │         (ESP32-S3)         │             │        │         (ESP32-S3)         │
   │                            │             │        │                            │
   │  ┌──────────────────────┐  │             │        │  ┌──────────────────────┐  │
   │  │    USB Host Mode     │  │             │        │  │    USB Host Mode     │  │
   │  └──────────┬───────────┘  │             │        │  └──────────┬───────────┘  │
   │             │              │◄────────────┴───────►│             │              │
   │      ┌──────┴──────┐       │       ESP-NOW        │      ┌──────┴──────┐       │
   │      │             │       │                      │      │             │       │
   │      ▼             ▼       │                      │      ▼             ▼       │
   │ ┌─────────┐  ┌─────────┐   │                      │ ┌─────────┐  ┌─────────┐   │
   │ │Joystick │  │ WinWing │   │                      │ │Quadrant │  │ MiniFCU │   │
   │ │TCA Side │  │  MCDU   │   │                      │ │TCA Capt │  │ +EFIS   │   │
   │ └─────────┘  └─────────┘   │                      │ └─────────┘  └─────────┘   │
   └────────────────────────────┘                      └────────────────────────────┘
```

### 2.2 Communication Flow

```
PC Software                Node A                    Peripheral Nodes
    │                        │                              │
    │   SLIP Frame           │                              │
    │  (Request/Command)     │                              │
    ├───────────────────────►│                              │
    │                        │  ESP-NOW Message             │
    │                        │  (Routed by Node ID)         │
    │                        ├─────────────────────────────►│
    │                        │                              │
    │                        │  ESP-NOW Response            │
    │                        │◄─────────────────────────────┤
    │   SLIP Frame           │                              │
    │  (Response/Event)      │                              │
    │◄───────────────────────┤                              │
    │                        │                              │
```

**Key Advantages of SLIP-based Architecture:**
- Single USB interface simplifies firmware and driver requirements
- Bidirectional communication without HID descriptor complexity
- PC software handles protocol translation (more flexible than firmware)
- Easier debugging - SLIP frames can be logged and analyzed
- No composite USB device enumeration issues

### 2.3 Design Goals

| # | Goal | Priority |
|---|------|----------|
| 1 | Star topology with single coordinator, multiple peripheral nodes | Critical |
| 2 | ESP32-S3 microcontroller, Arduino environment, ESP Core 3.3.5 | Critical |
| 3 | Native libraries only (no deprecated third-party dependencies) | Critical |
| 4 | SLIP-based serial protocol for PC communication | Critical |
| 5 | Auto-configuration messaging on node connection | High |
| 6 | Stable RTT ≤ 3ms using optimized ESP-NOW parameters | High |
| 7 | Bidirectional message routing between PC and peripheral nodes | High |
| 8 | Hot-plug support for peripheral nodes | Medium |
| 9 | Graceful degradation on node disconnection | Medium |

### 2.4 Scope - Hardware for Validation

| Node | Peripherals | Interface |
|------|-------------|-----------|
| Node A | PC Connection | USB CDC Serial (SLIP) |
| Node B | Thrustmaster TCA Sidestick + WinWing MCDU | USB Host |
| Node C | Thrustmaster TCA Quadrant + MiniCockpit MiniFCU + MiniEFIS | USB Host + Serial |

---

## 3. Hardware Architecture

### 3.1 Microcontroller Specification

| Parameter | Specification |
|-----------|---------------|
| MCU | ESP32-S3 (Dual Core Xtensa LX7, 240MHz) |
| WiFi | 802.11 b/g/n (2.4GHz) - Forced to 11g/n for low latency |
| USB | Native USB OTG supporting Host and Device modes |
| SRAM | 512KB |
| PSRAM | 8MB (board dependent) |
| Flash | 4MB-16MB (board dependent) |
| Development Board | WeAct ESP32-S3 Dev Board Rev A (or compatible) |

### 3.2 Node Hardware Configuration

#### 3.2.1 Node A - Coordinator

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-S3 DevKit                      │
│                                                         │
│  USB-C ────► USB Device Mode                            │
│              └── CDC Serial (SLIP Protocol)             │
│                  ├── Receives requests from PC          │
│                  ├── Routes to appropriate node         │
│                  └── Forwards responses to PC           │
│                                                         │
│  GPIO/Internal ────► ESP-NOW Radio                      │
│                      └── Coordinator Role               │
│                                                         │
│  Power: USB Bus Powered (500mA)                         │
└─────────────────────────────────────────────────────────┘
```

**SLIP Protocol Rationale:**
- Simpler than composite USB HID - no complex descriptor emulation
- Standard framing with escape sequences for reliable binary data
- PC software handles translation to simulator APIs (more flexible)
- Easy to debug with serial monitors

#### 3.2.2 Node B - Joystick + MCDU

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-S3 DevKit                      │
│                                                         │
│  USB-A Host ────► USB Hub (if needed)                   │
│                   ├── Thrustmaster TCA Sidestick        │
│                   │   VID: 0x044F  PID: 0x0405          │
│                   └── WinWing MCDU                      │
│                       VID: 0x4098  PID: 0xBB3A          │
│                                                         │
│  GPIO/Internal ────► ESP-NOW Radio                      │
│                      └── Peripheral Role                │
│                                                         │
│  Power: External 5V 2A (USB Host requires power)        │
└─────────────────────────────────────────────────────────┘
```

#### 3.2.3 Node C - Quadrant + FCU

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-S3 DevKit                      │
│                                                         │
│  USB-A Host ────► Thrustmaster TCA Quadrant             │
│                   VID: 0x044F  PID: 0x0407              │
│                                                         │
│  UART (GPIO) ────► USB-Serial CH340                     │
│                    └── MiniCockpit MiniFCU + MiniEFIS   │
│                        Baud: 9600, 8N1, DTR/RTS=HIGH    │
│                                                         │
│  GPIO/Internal ────► ESP-NOW Radio                      │
│                      └── Peripheral Role                │
│                                                         │
│  Power: External 5V 2A                                  │
└─────────────────────────────────────────────────────────┘
```

### 3.3 Bill of Materials (per node)

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32-S3 DevKit (WeAct or similar) | 1 | Must have native USB |
| USB-A Female Breakout | 1-2 | For USB Host nodes |
| USB Hub | 0-1 | If node requires multiple USB devices |
| 5V 2A Power Supply | 1 | For USB Host nodes |
| Enclosure | 1 | Optional |

---

## 4. Network Architecture

### 4.1 Topology

**Star Topology** with Node A as central coordinator:

```
                    ┌──────────┐
         ┌──────────│  Node A  │──────────┐
         │          │Coordinator│          │
         │          └──────────┘          │
         │                                │
         ▼                                ▼
    ┌──────────┐                    ┌──────────┐
    │  Node B  │                    │  Node C  │
    │Peripheral│                    │Peripheral│
    └──────────┘                    └──────────┘
```

### 4.2 Addressing Scheme

| Node | MAC Address | Role | Description |
|------|-------------|------|-------------|
| Node A | Coordinator MAC | MASTER | Stores peer table of all nodes |
| Node B | Factory MAC | SLAVE | Registered with coordinator on boot |
| Node C | Factory MAC | SLAVE | Registered with coordinator on boot |
| Broadcast | FF:FF:FF:FF:FF:FF | - | Used for discovery only |

### 4.3 Channel Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| WiFi Channel | 1 (fixed) | Prevents scanning delays |
| Protocol | 802.11g/n only | Forces OFDM (6Mbps+), avoids 802.11b (1Mbps) |
| Bandwidth | 20MHz | Standard ESP-NOW requirement |

---

## 5. ESP-NOW Protocol Specification

### 5.1 Optimized Configuration

Based on documented latency testing (reference: ESPNOW-Latency-test repository):

```cpp
// Critical initialization sequence for ~3ms RTT
void initEspNow() {
    // 1. Force Dual Mode to keep radio alive
    WiFi.mode(WIFI_AP_STA);
    
    // 2. CRITICAL: Ban 802.11b to prevent 1Mbps negotiation
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_protocol(WIFI_IF_AP,  WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    // 3. Force Channel 1 (prevent scanning)
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // 4. Disable Power Save - NO SLEEP
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // 5. Initialize ESP-NOW
    esp_now_init();
    
    // 6. Register callbacks
    esp_now_register_send_cb(onSendCallback);
    esp_now_register_recv_cb(onRecvCallback);
}
```

### 5.2 Latency Optimization Summary

| Stage | Configuration | Result |
|-------|---------------|--------|
| Baseline | Default settings | ~8.2ms RTT |
| Stage 1 | Disable power management | ~8.0ms RTT |
| Stage 2 | Force 802.11g/n protocol | ~3.2ms RTT |
| Stage 3 | Lock channel + 240MHz CPU | **~3.0ms RTT** |

### 5.3 Transmission Parameters

| Parameter | Value |
|-----------|-------|
| Max Payload Size | 250 bytes |
| Encryption | Disabled (for minimum latency) |
| Retries | ESP-NOW default (hardware managed) |
| Data Rate | 6Mbps minimum (OFDM) |

---

## 6. Node Specifications

### 6.1 Node A - Coordinator

#### 6.1.1 Responsibilities

| Function | Description |
|----------|-------------|
| USB CDC Serial | Single serial port with SLIP framing for PC communication |
| SLIP Decode | Decode incoming SLIP frames from PC into messages |
| SLIP Encode | Encode outgoing messages to PC as SLIP frames |
| Message Router | Route messages to/from appropriate peripheral nodes |
| ESP-NOW Master | Manage peer table, coordinate wireless communication |
| Auto-Config | Handle node registration and capability discovery |

#### 6.1.2 State Machine

```
┌─────────────────────────────────────────────────────────────┐
│                    COORDINATOR STATE MACHINE                │
└─────────────────────────────────────────────────────────────┘

    ┌──────────┐
    │   INIT   │
    └────┬─────┘
         │ USB enumerated + ESP-NOW init
         ▼
    ┌──────────┐         Timeout
    │DISCOVERY │◄────────────────────────────────┐
    └────┬─────┘                                 │
         │ Nodes registered                      │
         ▼                                       │
    ┌──────────┐         Node lost               │
    │  ACTIVE  │─────────────────────────────────┤
    └────┬─────┘                                 │
         │                                       │
         ▼                                       │
    ┌──────────┐                                 │
    │ ROUTING  │ ◄───────────────────────────────┘
    └──────────┘   New node discovered
```

#### 6.1.3 Core 0 / Core 1 Task Distribution

| Core | Task | Priority |
|------|------|----------|
| Core 0 | ESP-NOW TX/RX | Highest |
| Core 0 | Peer management | High |
| Core 1 | USB CDC Serial I/O | High |
| Core 1 | SLIP encode/decode | High |
| Core 1 | Message routing | Medium |

#### 6.1.4 Hardware Reference (ESP32-S3-USB-OTG)

Node A uses the ESP32-S3-USB-OTG board and runs in USB device mode only.

| Signal | GPIO | Notes |
|--------|------|-------|
| USB_SEL | GPIO18 | LOW = USB_DEV (PC connection). Kept LOW for Node A. |
| USB D- | GPIO19 | Connected to USB differential D-. |
| USB D+ | GPIO20 | Connected to USB differential D+. |
| LIMIT_EN | GPIO17 | USB_HOST current limit enable (unused for Node A). |
| DEV_VBUS_EN | GPIO12 | USB_HOST VBUS enable (unused for Node A). |
| BOOST_EN | GPIO13 | Battery boost enable (unused for Node A). |
| LCD_RST | GPIO8 | LCD reset (active low). |
| LCD_EN | GPIO5 | LCD enable (active low). |
| LCD_DC | GPIO4 | LCD data/command select. |
| LCD_SCLK | GPIO6 | LCD SPI clock. |
| LCD_MOSI | GPIO7 | LCD SPI MOSI. |
| LCD_BL | GPIO9 | LCD backlight. |

#### 6.1.5 LCD Network Metrics

The LCD renders coordinator status once per second. Metrics are derived from ESP-NOW traffic
plus an internal test ping (MSG_TEST) sent by Node A to an active node.

| Metric | Description |
|--------|-------------|
| Nodes | Connected nodes in the peer table. |
| SLIP RX/TX | SLIP frames per second between PC and Node A. |
| ESP RX/TX | ESP-NOW messages per second. |
| RTT avg | Average round-trip time for internal MSG_TEST pings (ms). |
| Jitter max | Maximum delta between consecutive RTT samples (ms). |
| Chan use | Estimated channel utilization = (ESP RX bytes + ESP TX bytes) * 8 / 6 Mbps. |

### 6.2 Node B - Joystick + MCDU Peripheral

#### 6.2.1 Responsibilities

| Function | Description |
|----------|-------------|
| USB Host | Enumerate and poll Thrustmaster + WinWing devices |
| HID Parser | Parse HID reports from connected devices |
| ESP-NOW Slave | Send input events, receive output commands |
| MCDU Display | Forward display data to MCDU via HID output |

#### 6.2.2 USB Host Configuration

```cpp
// USB Host device detection
const usb_device_info_t supported_devices[] = {
    { 0x044F, 0x0405, "Thrustmaster TCA Sidestick" },
    { 0x4098, 0xBB3A, "WinWing MCDU" },
    { 0x4098, 0xBB3A, "WinWing MCDU Observer" },  // Alternate PID
};
```

### 6.3 Node C - Quadrant + FCU Peripheral

#### 6.3.1 Responsibilities

| Function | Description |
|----------|-------------|
| USB Host | Enumerate and poll Thrustmaster Quadrant |
| Serial Master | Communicate with MiniFCU via 9600 baud serial |
| HID Parser | Parse quadrant HID reports |
| FCU Protocol | Handle MiniFCU ASCII protocol |
| ESP-NOW Slave | Bidirectional communication with coordinator |

#### 6.3.2 Serial Configuration for MiniFCU

```cpp
// MiniFCU Serial Parameters
#define MINIFCU_BAUD     9600
#define MINIFCU_CONFIG   SERIAL_8N1

// CRITICAL: DTR and RTS must be HIGH for power
void initMiniFCU(HardwareSerial& serial) {
    serial.begin(MINIFCU_BAUD, MINIFCU_CONFIG, RX_PIN, TX_PIN);
    // Set DTR/RTS high via GPIO or USB-Serial adapter
    pinMode(DTR_PIN, OUTPUT);
    pinMode(RTS_PIN, OUTPUT);
    digitalWrite(DTR_PIN, HIGH);
    digitalWrite(RTS_PIN, HIGH);
}
```

---

## 7. Peripheral Integration Protocols

### 7.1 Thrustmaster TCA HID Protocol

Reference: `thrustmaster-tca-hid-esp32s3` repository

#### 7.1.1 Sidestick (PID: 0x0405)

| Byte | Description | Range |
|------|-------------|-------|
| 0-1 | X Axis (Roll) | 0x0000 - 0xFFFF |
| 2-3 | Y Axis (Pitch) | 0x0000 - 0xFFFF |
| 4 | Buttons byte 1 | Bit-mapped |
| 5 | Buttons byte 2 | Bit-mapped |
| 6 | Hat Switch | 0-8 (POV directions + center) |

#### 7.1.2 Quadrant (PID: 0x0407)

| Byte(s) | Control | Notes |
|---------|---------|-------|
| 0-1 | Throttle 1 Axis | With detent flags |
| 2-3 | Throttle 2 Axis | With detent flags |
| 4-5 | Flaps Axis | Discrete detents (0/1/2/3/FULL) |
| 6-7 | Speedbrake Axis | With retract latch flag |
| 8+ | Button states | Engine masters, A/THR, FIRE, etc. |

**Throttle Detent Detection:**
- IDLE: ~0x0000 - 0x0100
- CL (Climb): ~0x4000
- FLX/MCT: ~0x6000
- TOGA: ~0x8000 - 0xFFFF
- Reverse zone flag: Separate bit indicator

### 7.2 WinWing MCDU Protocol

Reference: Community reverse engineering (schenlap/winwing_mcdu)

#### 7.2.1 Device Information

| Parameter | Value |
|-----------|-------|
| VID | 0x4098 |
| PID | 0xBB3A (Captain), 0xBB3A (Observer variant) |
| Display | 24 x 14 alphanumeric color LCD |
| Interface | USB HID |

#### 7.2.2 Display Protocol

```
Display Write Command Structure:
┌──────┬──────┬──────┬──────┬──────┬─────────────────────┐
│ 0xF0 │ 0x00 │ LEN  │ CMD  │ DATA │ ... (up to 64 bytes)│
└──────┴──────┴──────┴──────┴──────┴─────────────────────┘

Key Commands:
- Display text: Variable length, position + color + chars
- Set brightness: 0xF0 followed by brightness value
- Set LEDs: LED mask + state
```

#### 7.2.3 Button Mapping (Input Report)

| Button Index | Function |
|--------------|----------|
| 0-5 | LSK 1-6 Left |
| 6-11 | LSK 1-6 Right |
| 12-17 | Function row (DIR, PROG, PERF, etc.) |
| 18-25 | Page controls (BRT, DIM, FPLN, etc.) |
| 26-31 | Arrow keys + OVFY |
| 32-41 | Number pad 0-9 |
| 42-67 | Letter keys A-Z |
| 68-71 | Special keys (CLR, SP, /, +/-) |

### 7.3 MiniCockpit MiniFCU + MiniEFIS Protocol

Reference: `OpenCockpit_MiniFCU_Comm_Protocol` repository

#### 7.3.1 Physical Layer

| Parameter | Value |
|-----------|-------|
| Interface | USB Serial (CH340) |
| Baud Rate | 9600 bps |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| **Power Requirement** | DTR and RTS must be HIGH |

#### 7.3.2 Protocol Overview

| Direction | Delimiter | Example |
|-----------|-----------|---------|
| PC → Hardware | Comma `,` | `A32000,` |
| Hardware → PC | Semicolon `;` | `17;` |

#### 7.3.3 Startup Sequence

```
1. Open serial port
2. Assert DTR=HIGH, RTS=HIGH
3. Send handshake: C,
4. Send initialization burst:
   Q400,K100,-99,+10,n49000,b100,[6000,]-6000,
   Z9900,X-9900,I,Y,W,O,{1,(3248,}2200,=1100,$745,%0,
5. Repaint all displays and LEDs
```

#### 7.3.4 Input Events (Hardware → PC)

**FCU Encoders:**

| Event | Code | Description |
|-------|------|-------------|
| Speed CW | `13;` | Speed encoder clockwise |
| Speed CCW | `14;` | Speed encoder counter-clockwise |
| Speed Push | `11;` | Speed managed mode |
| Speed Pull | `12;` | Speed selected mode |
| Heading CW | `3;` | Heading encoder clockwise |
| Heading CCW | `4;` | Heading encoder counter-clockwise |
| Heading Push | `1;` | Heading managed mode |
| Heading Pull | `2;` | Heading selected mode |
| Altitude CW | `17;` | Altitude encoder clockwise |
| Altitude CCW | `18;` | Altitude encoder counter-clockwise |
| Altitude Push | `15;` | Altitude managed mode |
| Altitude Pull | `16;` | Altitude selected mode |
| VS/FPA CW | `21;` | VS/FPA encoder clockwise |
| VS/FPA CCW | `22;` | VS/FPA encoder counter-clockwise |
| VS/FPA Push | `19;` | Level change |
| VS/FPA Pull | `20;` | Open VS/FPA window |

**FCU Buttons:**

| Event | Code |
|-------|------|
| AP1 | `50;` |
| AP2 | `51;` |
| A/THR | `52;` |
| LOC | `53;` |
| EXPED | `54;` |
| APPR | `55;` |
| SPD/MACH | `56;` |
| HDG/TRK | `57;` |
| METRIC | `58;` |

**EFIS Buttons:**

| Event | Code |
|-------|------|
| FD | `62;` |
| LS | `63;` |
| CSTR | `64;` |
| WPT | `65;` |
| VOR.D | `66;` |
| NDB | `67;` |
| ARPT | `68;` |

#### 7.3.5 Output Commands (PC → Hardware)

**Displays:**

| Display | Command | Example |
|---------|---------|---------|
| Speed | `Sxxx,` | `S280,` |
| Heading | `Hxxx,` | `H090,` |
| Altitude | `Axxxxx,` | `A35000,` |
| VS/FPA | `F####,` | `F-700,` |
| QNH/BARO | `#xxxx,` | `#1013,` |
| Backlight | `Bxxxx,` | `B1000,` |

**LEDs (Uppercase = ON, Lowercase = OFF):**

| LED | ON | OFF |
|-----|-----|-----|
| AP1 | `P` | `p` |
| AP2 | `U` | `u` |
| A/THR | `T` | `t` |
| LOC | `L` | `l` |
| EXPED | `E` | `e` |
| APPR | `R` | `r` |

**EFIS LEDs (Numeric pairs):**

| LED | ON | OFF |
|-----|-----|-----|
| FD | `51` | `50` |
| LS | `41` | `40` |
| CSTR | `31` | `30` |
| WPT | `21` | `20` |
| VOR.D | `11` | `10` |
| NDB | `01` | `00` |
| ARPT | `!1` | `!0` |

---

## 8. Message Protocol Specification

### 8.1 ESP-NOW Message Frame Structure

```
┌────────────────────────────────────────────────────────────────────┐
│                    ESP-NOW MESSAGE FRAME                           │
├────────┬────────┬────────┬────────┬────────────────────┬──────────┤
│ Header │Msg Type│Src Node│Dst Node│      Payload       │ Checksum │
│ (1B)   │ (1B)   │ (1B)   │ (1B)   │     (0-246B)       │  (1B)    │
├────────┼────────┼────────┼────────┼────────────────────┼──────────┤
│  0xAA  │  Type  │  Src   │  Dst   │       Data         │   CRC8   │
└────────┴────────┴────────┴────────┴────────────────────┴──────────┘
```

### 8.2 Message Types

| Type ID | Name | Direction | Description |
|---------|------|-----------|-------------|
| 0x01 | DISCOVERY_REQ | Coord → Broadcast | Node discovery request |
| 0x02 | DISCOVERY_RSP | Node → Coord | Node capability announcement |
| 0x03 | REGISTER_REQ | Coord → Node | Registration acknowledgment |
| 0x04 | REGISTER_ACK | Node → Coord | Registration confirmed |
| 0x10 | HEARTBEAT | Bidirectional | Keep-alive ping |
| 0x11 | HEARTBEAT_ACK | Bidirectional | Keep-alive response |
| 0x20 | HID_INPUT | Node → Coord | HID input report (joystick, buttons) |
| 0x21 | HID_OUTPUT | Coord → Node | HID output report (LEDs, display) |
| 0x30 | SERIAL_DATA | Bidirectional | Serial passthrough data |
| 0x40 | MCDU_DISPLAY | Coord → Node | MCDU display update |
| 0x41 | MCDU_INPUT | Node → Coord | MCDU button press |
| 0xF0 | ERROR | Bidirectional | Error notification |
| 0xFF | RESET | Coord → Node | Reset command |

### 8.3 Node Identifiers

| Node ID | Description |
|---------|-------------|
| 0x00 | Coordinator (Node A) |
| 0x01 | Node B (Joystick + MCDU) |
| 0x02 | Node C (Quadrant + FCU) |
| 0xFF | Broadcast |

### 8.4 Message Payload Structures

#### 8.4.1 Discovery Response (0x02)

```cpp
struct DiscoveryResponse {
    uint8_t node_type;          // 0x01=HID, 0x02=Serial, 0x03=Mixed
    uint8_t capabilities;       // Bitmap: HID_IN, HID_OUT, SERIAL, etc.
    uint8_t device_count;       // Number of connected USB devices
    uint8_t mac_address[6];     // Node MAC address
    char    firmware_version[8]; // e.g., "1.0.0"
    char    node_name[16];      // e.g., "NODE_B_STICK"
};
```

#### 8.4.2 HID Input (0x20)

```cpp
struct HIDInputMessage {
    uint8_t device_id;          // Which device on this node
    uint8_t report_id;          // HID report ID
    uint8_t report_length;      // Length of HID report
    uint8_t report_data[64];    // Raw HID input report
};
```

#### 8.4.3 HID Output (0x21)

```cpp
struct HIDOutputMessage {
    uint8_t device_id;          // Target device
    uint8_t report_id;          // HID report ID
    uint8_t report_length;      // Length of HID report
    uint8_t report_data[64];    // Raw HID output report
};
```

#### 8.4.4 Serial Data (0x30)

```cpp
struct SerialDataMessage {
    uint8_t port_id;            // Which serial port (for future expansion)
    uint8_t data_length;        // Length of serial data
    uint8_t data[200];          // Serial data payload
};
```

#### 8.4.5 MCDU Display (0x40)

```cpp
struct MCDUDisplayMessage {
    uint8_t command_type;       // 0x01=Text, 0x02=LED, 0x03=Brightness
    uint8_t row;                // Display row (0-13)
    uint8_t column;             // Display column (0-23)
    uint8_t color;              // Text color
    uint8_t length;             // Text length
    uint8_t text[24];           // Text data
};
```

### 8.5 Flow Control

| Mechanism | Description |
|-----------|-------------|
| Implicit ACK | ESP-NOW hardware acknowledgment |
| Sequence Numbers | Optional for ordered delivery |
| Retry | Hardware-managed retransmission |
| Timeout | 100ms for critical messages |

---

## 9. Auto-Configuration System

### 9.1 Discovery Sequence

```
┌──────────────┐                              ┌──────────────┐
│ COORDINATOR  │                              │   PERIPHERAL │
│   (Node A)   │                              │   (Node B/C) │
└──────┬───────┘                              └──────┬───────┘
       │                                             │
       │  ══════════ POWER ON ══════════             │
       │                                             │
       │◄───────────────────────────────────────────►│
       │         ESP-NOW Initialization              │
       │                                             │
       │ DISCOVERY_REQ (broadcast)                   │
       │────────────────────────────────────────────►│
       │                                             │
       │                     DISCOVERY_RSP           │
       │◄────────────────────────────────────────────│
       │           (capabilities, MAC, name)         │
       │                                             │
       │ REGISTER_REQ                                │
       │────────────────────────────────────────────►│
       │    (assigned node_id, config params)        │
       │                                             │
       │                     REGISTER_ACK            │
       │◄────────────────────────────────────────────│
       │                                             │
       │ ═══════════ OPERATIONAL ═══════════         │
       │                                             │
       │◄───────────────────────────────────────────►│
       │         Normal message exchange             │
```

### 9.2 Capability Bitmap

```cpp
enum NodeCapabilities : uint8_t {
    CAP_HID_INPUT      = 0x01,  // Can send HID input reports
    CAP_HID_OUTPUT     = 0x02,  // Can receive HID output reports
    CAP_SERIAL         = 0x04,  // Has serial port capability
    CAP_DISPLAY        = 0x08,  // Has display output (MCDU)
    CAP_BIDIRECTIONAL  = 0x10,  // Requires bidirectional comm
    CAP_LOW_LATENCY    = 0x20,  // Requires low latency path
};

// Node B capabilities: CAP_HID_INPUT | CAP_HID_OUTPUT | CAP_DISPLAY | CAP_BIDIRECTIONAL
// Node C capabilities: CAP_HID_INPUT | CAP_SERIAL | CAP_BIDIRECTIONAL
```

### 9.3 Heartbeat and Health Monitoring

| Parameter | Value |
|-----------|-------|
| Heartbeat Interval | 1000ms |
| Heartbeat Timeout | 3000ms (3 missed = node lost) |
| Reconnection Backoff | 500ms, 1000ms, 2000ms, 5000ms |

---

## 10. Coordinator USB Interface

### 10.1 USB CDC Serial Configuration

Node A exposes a single USB CDC Serial port for all communication with the PC.

```
USB Device (Node A)
└── Configuration 1
    └── Interface 0: CDC ACM (Serial)
        ├── Endpoint 1 IN (Bulk, 64B) - Data to PC
        ├── Endpoint 1 OUT (Bulk, 64B) - Data from PC
        └── Endpoint 2 IN (Interrupt) - CDC notifications
```

| Parameter | Value |
|-----------|-------|
| Baud Rate | 115200 (configurable) |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| Buffer Size | RX 4096 bytes (firmware), TX default (core) |

**Operational notes (Node A):**
- SLIP must use the native USB CDC port; debug logs go to UART0 (pins 44/43) to keep CDC clean.
- Avoid blocking writes on CDC: only transmit a SLIP frame when the full encoded frame fits in the TX buffer.
- When running inside a VM, do not reset Node A while testing; CDC can detach and change the host port.

### 10.2 SLIP Protocol Specification

**SLIP (Serial Line Internet Protocol)** provides simple, reliable framing for binary data over serial.

#### 10.2.1 SLIP Special Characters

| Name | Value | Description |
|------|-------|-------------|
| END | 0xC0 | Frame delimiter |
| ESC | 0xDB | Escape character |
| ESC_END | 0xDC | Escaped END (0xC0 in data) |
| ESC_ESC | 0xDD | Escaped ESC (0xDB in data) |

#### 10.2.2 SLIP Encoding Rules

```
To send a packet:
1. Send END character (0xC0) to flush any line noise
2. For each byte in packet:
   - If byte == 0xC0: send ESC (0xDB) then ESC_END (0xDC)
   - If byte == 0xDB: send ESC (0xDB) then ESC_ESC (0xDD)
   - Otherwise: send byte as-is
3. Send END character (0xC0) to terminate frame
```

#### 10.2.3 SLIP Frame Structure

```
┌──────┬─────────────────────────────────────────────────────┬──────┐
│ END  │              SLIP-Encoded Payload                   │ END  │
│ 0xC0 │    (ESP-NOW message with escape sequences)          │ 0xC0 │
└──────┴─────────────────────────────────────────────────────┴──────┘
```

#### 10.2.4 Example SLIP Encoding

```
Original message:  [0xAA, 0x20, 0x01, 0x02, 0xC0, 0x10, 0xDB, 0x55]
                                           ^^^^        ^^^^
                                        (needs ESC) (needs ESC)

SLIP encoded:      [0xC0, 0xAA, 0x20, 0x01, 0x02, 0xDB, 0xDC, 0x10, 0xDB, 0xDD, 0x55, 0xC0]
                    ^^^^                          ^^^^^^^^^^        ^^^^^^^^^^        ^^^^
                   START                         ESC + ESC_END     ESC + ESC_ESC       END
```

### 10.3 Message Routing

Node A routes SLIP-decoded messages based on the destination node ID in the message header:

```
PC Software                    Node A                         Peripheral
    │                            │                               │
    │  SLIP Frame                │                               │
    │  [END][Msg to Node B][END] │                               │
    ├───────────────────────────►│                               │
    │                            │ Decode SLIP                   │
    │                            │ Extract: Dst=0x01 (Node B)    │
    │                            │                               │
    │                            │ ESP-NOW to Node B             │
    │                            ├──────────────────────────────►│
    │                            │                               │
    │                            │ ESP-NOW response              │
    │                            │◄──────────────────────────────┤
    │                            │                               │
    │                            │ Encode SLIP                   │
    │  SLIP Frame                │                               │
    │  [END][Response][END]      │                               │
    │◄───────────────────────────┤                               │
```

### 10.4 Implementation Reference

```cpp
// SLIP constants
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

// SLIP encoder
void slip_encode(const uint8_t* data, size_t len, Stream& output) {
    output.write(SLIP_END);  // Start frame
    for (size_t i = 0; i < len; i++) {
        switch (data[i]) {
            case SLIP_END:
                output.write(SLIP_ESC);
                output.write(SLIP_ESC_END);
                break;
            case SLIP_ESC:
                output.write(SLIP_ESC);
                output.write(SLIP_ESC_ESC);
                break;
            default:
                output.write(data[i]);
        }
    }
    output.write(SLIP_END);  // End frame
}

// SLIP decoder (state machine)
enum SlipState { NORMAL, ESCAPED };

bool slip_decode_byte(uint8_t byte, uint8_t* buffer, size_t* index, SlipState* state) {
    switch (*state) {
        case NORMAL:
            switch (byte) {
                case SLIP_END:
                    if (*index > 0) {
                        return true;  // Frame complete
                    }
                    break;  // Empty frame or leading END
                case SLIP_ESC:
                    *state = ESCAPED;
                    break;
                default:
                    buffer[(*index)++] = byte;
            }
            break;
        case ESCAPED:
            *state = NORMAL;
            switch (byte) {
                case SLIP_ESC_END:
                    buffer[(*index)++] = SLIP_END;
                    break;
                case SLIP_ESC_ESC:
                    buffer[(*index)++] = SLIP_ESC;
                    break;
                default:
                    // Protocol error - invalid escape
                    buffer[(*index)++] = byte;
            }
            break;
    }
    return false;
}
```

---

## 11. Software Architecture

### 11.1 Development Environment

| Component | Specification |
|-----------|---------------|
| IDE | Arduino IDE 2.x or PlatformIO |
| Framework | Arduino |
| ESP32 Core | v3.3.5 (ESP-IDF 5.5 based) |
| Target | ESP32-S3 |

### 11.2 Library Dependencies

**Native/Built-in Libraries Only:**

| Library | Purpose |
|---------|---------|
| WiFi.h | WiFi initialization for ESP-NOW |
| esp_now.h | ESP-NOW protocol |
| esp_wifi.h | Low-level WiFi configuration |
| USB.h | USB Device mode |
| USBCDC.h | CDC serial class (SLIP communication) |
| usb/usb_host.h | USB Host mode (peripheral nodes) |

### 11.3 Code Organization

```
project/
├── common/
│   ├── protocol.h           # Message structures and types
│   ├── espnow_config.h      # ESP-NOW optimization settings
│   ├── slip.h               # SLIP encode/decode functions
│   └── crc8.h               # Checksum implementation
├── node_a_coordinator/
│   ├── main.cpp
│   ├── slip_serial.cpp      # USB CDC with SLIP framing
│   ├── espnow_master.cpp    # Coordinator ESP-NOW logic
│   ├── routing.cpp          # Message routing (SLIP ↔ ESP-NOW)
│   └── auto_config.cpp      # Node discovery and registration
├── node_b_joystick/
│   ├── main.cpp
│   ├── usb_host.cpp         # USB Host for peripherals
│   ├── hid_parser.cpp       # Thrustmaster + MCDU HID parsing
│   └── espnow_slave.cpp     # Peripheral ESP-NOW logic
├── node_c_quadrant/
│   ├── main.cpp
│   ├── usb_host.cpp         # USB Host for Quadrant
│   ├── serial_minifcu.cpp   # MiniFCU serial protocol
│   └── espnow_slave.cpp     # Peripheral ESP-NOW logic
└── pc_bridge_software/
    ├── main.py              # Bridge application entry point
    ├── slip_serial.py       # SLIP protocol over serial
    ├── device_handlers/
    │   ├── joystick.py      # Joystick → vJoy/simulator
    │   ├── mcdu.py          # MCDU display/buttons handler
    │   └── fcu.py           # MiniFCU protocol handler
    └── simulator_bridge.py  # SimConnect/Fenix API interface
```

### 11.4 FreeRTOS Task Structure

#### Node A (Coordinator)

| Task | Core | Priority | Stack | Description |
|------|------|----------|-------|-------------|
| espnow_tx_task | 0 | 24 | 4096 | ESP-NOW transmission |
| espnow_rx_task | 0 | 24 | 4096 | ESP-NOW reception |
| slip_rx_task | 1 | 22 | 4096 | SLIP frame reception from USB CDC |
| slip_tx_task | 1 | 22 | 4096 | SLIP frame transmission to USB CDC |
| routing_task | 1 | 18 | 2048 | Message routing (SLIP ↔ ESP-NOW) |
| discovery_task | 1 | 10 | 2048 | Node discovery |

#### Node B/C (Peripheral)

| Task | Core | Priority | Stack | Description |
|------|------|----------|-------|-------------|
| espnow_task | 0 | 24 | 4096 | ESP-NOW communication |
| usb_host_task | 1 | 20 | 4096 | USB device polling |
| hid_process_task | 1 | 18 | 2048 | HID report processing |
| serial_task* | 1 | 16 | 2048 | MiniFCU serial (Node C only) |

### 11.5 PC Bridge Software Architecture

The PC Bridge Software handles SLIP communication and interfaces with the flight simulator.

```
┌─────────────────────────────────────────────────────────────────────┐
│                     PC BRIDGE SOFTWARE                               │
├─────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │   SLIP      │  │  Message    │  │  Device     │  │ Simulator  │  │
│  │   Serial    │◄─┤  Router     │◄─┤  Handlers   │◄─┤  Bridge    │  │
│  │   I/O       │  │             │  │             │  │            │  │
│  └──────┬──────┘  └─────────────┘  └─────────────┘  └──────┬─────┘  │
│         │                                                   │        │
│    USB CDC                                             SimConnect    │
│    Serial                                              / Fenix API   │
└─────────┼───────────────────────────────────────────────────┼────────┘
          │                                                   │
          ▼                                                   ▼
     ┌──────────┐                                    ┌─────────────────┐
     │  Node A  │                                    │  MSFS 2020/2024 │
     │(ESP32-S3)│                                    │  + Fenix A320   │
     └──────────┘                                    └─────────────────┘
```

#### 11.5.1 Device Handler Responsibilities

| Handler | Input Processing | Output Processing |
|---------|------------------|-------------------|
| Joystick | HID reports → vJoy axis/button updates | N/A (input only) |
| MCDU | Button presses → Fenix MCDU API | Display data → MCDU output messages |
| FCU | Encoder/button events → Fenix FCU API | Display/LED state → FCU output commands |
| Quadrant | HID reports → throttle/flaps/spoiler | N/A (input only) |

#### 11.5.2 Simulator Bridge Interface

```python
# Example: Fenix A320 Integration
class FenixBridge:
    def handle_fcu_event(self, event_code):
        """Route FCU event to Fenix"""
        if event_code == 0x11:  # Speed push
            self.fenix.fcu_speed_managed()
        elif event_code == 0x0D:  # Speed CW
            self.fenix.fcu_speed_increase()
        # ...

    def poll_simulator_state(self):
        """Get display values from Fenix"""
        return {
            'speed': self.fenix.get_fcu_speed(),
            'heading': self.fenix.get_fcu_heading(),
            'altitude': self.fenix.get_fcu_altitude(),
            'ap1_led': self.fenix.get_ap1_engaged(),
            # ...
        }
```

---

## 12. Performance Requirements

### 12.1 Latency Requirements

| Path | Requirement | Target |
|------|-------------|--------|
| Button press → Simulator | < 10ms | 5ms |
| Encoder rotation → Simulator | < 10ms | 5ms |
| Simulator → LED update | < 20ms | 10ms |
| Simulator → Display update | < 50ms | 30ms |
| ESP-NOW Round Trip | < 5ms | 3ms |

### 12.2 Throughput Requirements

| Data Type | Rate | Notes |
|-----------|------|-------|
| Joystick HID reports | 125 Hz | 8ms polling |
| FCU encoder events | Event-driven | Burst up to 50 events/sec |
| MCDU display updates | 10 Hz | Full screen refresh |
| Serial passthrough | 9600 baud | ~960 bytes/sec |

### 12.3 Reliability Requirements

| Metric | Requirement |
|--------|-------------|
| Packet delivery rate | > 99.9% |
| Node reconnection time | < 5 seconds |
| Maximum message latency | < 50ms (P99) |
| MTBF | > 1000 hours |

---

## 13. Development Environment

### 13.1 IDE Configuration

**Arduino IDE 2.x:**

1. Add ESP32 board URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Install ESP32 board package version 3.3.5
3. Select board: "ESP32-S3 Dev Module"
4. Configure:
   - USB Mode: "USB-OTG (TinyUSB)"
   - USB CDC On Boot: "Enabled"
   - CPU Frequency: "240MHz"
   - Flash Size: "4MB" (or as appropriate)
   - Partition Scheme: "Default 4MB with spiffs"

### 13.2 PlatformIO Configuration

```ini
[env:esp32s3]
platform = espressif32@6.9.0
board = esp32-s3-devkitc-1
framework = arduino
board_build.arduino.upstream_packages = no

build_flags = 
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DCORE_DEBUG_LEVEL=0

monitor_speed = 115200
upload_speed = 921600
```

### 13.3 Debug Configuration

| Feature | Configuration |
|---------|---------------|
| Serial Debug | USB CDC secondary port |
| ESP-NOW Debug | Packet counters, RTT measurement |
| USB Debug | Enumeration logging |
| Performance | FreeRTOS task stats |

---

## 14. Testing Strategy

### 14.1 Unit Tests

| Module | Test Cases |
|--------|------------|
| Protocol | Message serialization/deserialization |
| CRC | Checksum calculation accuracy |
| HID Parser | Thrustmaster report parsing |
| Serial Parser | MiniFCU command parsing |

### 14.2 Integration Tests

| Test | Description | Pass Criteria |
|------|-------------|---------------|
| Discovery | Node discovery and registration | All nodes register within 5s |
| Latency | RTT measurement under load | P99 < 5ms |
| Throughput | Sustained message rate | 500 msg/s without drops |
| Reconnection | Node power cycle recovery | Reconnect within 5s |
| USB Enum | Coordinator USB enumeration | All interfaces functional |

### 14.3 System Tests

| Test | Description |
|------|-------------|
| Flight Test | Complete flight with all peripherals |
| Endurance | 8-hour continuous operation |
| Interference | Operation with WiFi networks nearby |
| Hot Plug | Dynamic peripheral connection/disconnection |

### 14.4 Validation Checklist

- [ ] Joystick axes and buttons recognized by MSFS
- [ ] Throttle detents function correctly
- [ ] FCU encoders update Fenix FCU
- [ ] FCU displays show correct values
- [ ] MCDU screen mirrors simulator
- [ ] MCDU buttons register input
- [ ] All LEDs respond to aircraft state
- [ ] No perceptible input lag during flight

---

## Appendix: Reference Documentation

### A.1 Project Repositories

| Repository | Description | URL |
|------------|-------------|-----|
| ESP-NOW Latency Testing | Optimized ESP-NOW configuration | https://github.com/jcallano/ESPNOW-Latency-test-for-turnaround-comm-betwen-esp32S3-and-esp32C6 |
| Thrustmaster TCA HID | USB HID protocol documentation | https://github.com/jcallano/thrustmaster-tca-hid-esp32s3 |
| MiniFCU Protocol | Serial protocol documentation | https://github.com/jcallano/OpenCockpit_MiniFCU_Comm_Protocol |
| WinWing MCDU Protocol | MCDU USB protocol documentation | https://github.com/jcallano/winwing-mcdu-comm-protocol |

### A.2 External References

| Resource | Description |
|----------|-------------|
| ESP-NOW Programming Guide | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/network/esp_now.html |
| ESP32-S3 Technical Reference | https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf |
| USB HID Specification | https://www.usb.org/hid |
| Arduino ESP32 Core | https://github.com/espressif/arduino-esp32 |

### A.3 Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | January 2026 | Initial specification |
| 1.1 | January 2026 | Replaced composite HID device with SLIP-based serial protocol for simplified architecture |

---

**Document End**

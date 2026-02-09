# OpenCockpit Wireless Avionics Bus

A low-latency wireless communication system for flight simulator cockpit peripherals using ESP32 and ESP-NOW protocol.

## Overview

This project creates a wireless bridge between various flight simulator hardware peripherals and PC simulator software, eliminating cable clutter while maintaining the low latency required for responsive cockpit controls.

![Star Network Topology](docs/images/ilustration.png)
*ESP32S3 on the nodes if USB HOST capability are required.

```
┌─────────────────┐     USB CDC      ┌─────────────────┐
│   PC Software   │◄────(SLIP)──────►│   Coordinator   │
│  (Bridge App)   │                  │   (ESP32-S3)    │
└────────┬────────┘                  └────────┬────────┘
         │                                    │
    SimConnect                           ESP-NOW
    Fenix API                           (~3ms RTT)
         │                                    │
         ▼                     ┌──────────────┼──────────────┐
┌─────────────────┐            │              │              │
│  MSFS 2020/24   │            ▼              ▼              ▼
│  + Fenix A320   │      ┌──────────┐  ┌──────────┐  ┌──────────┐
└─────────────────┘      │ MiniFCU  │  │Thrustmaster│ │  MCDU    │
                         │  Node    │  │   Node    │  │  Node    │
                         └──────────┘  └──────────┘  └──────────┘
```

## Features

- **Low Latency**: ~3ms round-trip wireless communication
- **Multiple Peripherals**: Support for various cockpit hardware
- **SLIP Protocol**: Reliable serial framing for PC communication
- **Modular Design**: Easy to add new peripheral types
- **Reverse-Engineered Protocols**: Complete documentation of proprietary hardware

## Supported Hardware

| Peripheral | Type | Status |
|------------|------|--------|
| Minicockpit MiniFCU + MiniEFIS | FCU Panel | Documented |
| Thrustmaster TCA Sidestick | Joystick | Documented |
| Thrustmaster TCA Quadrant | Throttle | Documented |
| WinWing MCDU | CDU Display | Documented |

### Board Compatibility
- **Coordinator (Node A):** [Espressif ESP32-S3-USB-OTG](https://docs.espressif.com/projects/esp-iot-solution/en/latest/hw-reference/ESP32-S3-USB-OTG.html) (Recommended for LCD/Power control)
- **Peripherals (Node B/C):** [WeAct Studio ESP32-S3](https://github.com/WeActStudio/WeActStudio.ESP32C3-CoreBoard) (8MB Flash/8MB PSRAM) or generic equivalent.
> See [Hardware Compatibility Report](docs/hardware/hardware_compatibility_report.md) for details.

## Project Structure

```
opencockpit-wireless-bus/
├── docs/                    # Documentation
│   ├── design/              # System architecture specs
│   └── protocols/           # Peripheral protocol documentation
│       ├── minifcu/         # MiniFCU serial protocol
│       ├── thrustmaster/    # TCA HID reports
│       ├── winwing_mcdu/    # MCDU USB protocol
│       └── espnow/          # ESP-NOW optimization
├── firmware/                # Microcontroller code
│   ├── common/              # Shared libraries
│   ├── coordinator/         # Node A (PC connection)
│   ├── peripherals/         # Peripheral nodes
│   └── tools/               # Development utilities
├── software/                # PC applications
│   ├── bridge/              # Main bridge software
│   ├── emulators/           # Hardware emulators
│   └── tools/               # Utilities & Test Scripts
├── hardware/                # Hardware documentation
└── analysis/                # Protocol captures
```

## Quick Start

### Prerequisites

- ESP32-S3 development boards (WeAct or similar)
- Arduino IDE 2.x or PlatformIO
- ESP32 Arduino Core 3.3.5+
- Python 3.x (for PC software)

### Building Firmware

This project supports multiple board types via compile-time flags.

1.  **Install dependencies**: Arduino IDE + ESP32 Core 3.3.5+.
2.  **Select Board & Compile**:
    *   **Coordinator:** Use definition `BOARD_ESPRESSIF_USB_OTG`.
    *   **Peripherals:** Use definition `BOARD_WEACT_STUDIO_S3`.
3.  **Flash**:
    *   `arduino-cli` environment files (`arduino-cli.env`) are provided in each firmware directory with the correct flags pre-configured.
    *   See [LCD Configuration](docs/hardware/lcd_config.md) if using a display on the Coordinator.

### Running PC Software

```bash
cd software
pip install -r requirements.txt
python bridge/main.py
```

## Documentation

- [System Specification](docs/design/system_specification.md) - Complete architecture design
- [MiniFCU Protocol](docs/protocols/minifcu/protocol.md) - FCU serial protocol
- [Thrustmaster HID](docs/protocols/thrustmaster/) - Sidestick and Quadrant reports
- [WinWing MCDU](docs/protocols/winwing_mcdu/protocol.md) - MCDU USB protocol
- [WinWing Ursa Minor + Pac 32](docs/protocols/ursa_minor/protocol.md) - Throttle & LCD protocol
- [ESP-NOW Optimization](docs/protocols/espnow/latency_optimization.md) - Latency tuning
- [**Project Status**](docs/project_status.md) - Current executive summary

## Hardware Requirements

### Coordinator Node (Node A)
- ESP32-S3 with USB Device support
- USB-C connection to PC

### Peripheral Nodes
- ESP32-S3 with USB Host support
- External 5V 2A power supply
- USB-A female connector(s)

## Communication Protocol

### SLIP Framing (PC ↔ Coordinator)
```
┌──────┬─────────────────────────┬──────┐
│ 0xC0 │   Escaped Payload       │ 0xC0 │
└──────┴─────────────────────────┴──────┘
```

### ESP-NOW Messages (Coordinator ↔ Peripherals)
```
┌────────┬────────┬────────┬────────┬──────────┬──────────┐
│ Header │MsgType │  Src   │  Dst   │ Payload  │ Checksum │
│  0xAA  │  1B    │  1B    │  1B    │  0-245B  │   CRC8   │
└────────┴────────┴────────┴────────┴──────────┴──────────┘
```

## Performance

| Metric | Target | Achieved |
|--------|--------|----------|
| ESP-NOW RTT | <5ms | ~3ms |
| Input Latency | <10ms | ~5ms |
| Display Update | <50ms | ~30ms |

## Contributing

This is a personal project for home cockpit integration. Protocol documentation and code improvements are welcome.

## Disclaimer

This project is the result of reverse engineering work. It is NOT affiliated with any hardware manufacturer. Use at your own risk.

## License

MIT License - See [LICENSE](LICENSE) for details.

## Author

@jcallano

## Acknowledgments

* **Antigravity Framework** - *AI Agentic Assistance*

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.1 | Jan 2026 | SLIP-based architecture, unified repository |
| 1.0 | Jan 2026 | Initial multi-repo implementation |

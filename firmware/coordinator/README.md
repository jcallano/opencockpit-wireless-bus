# Node A - Coordinator Firmware

## Overview
This firmware runs on the **ESP32-S3-USB-OTG** board. It acts as the bridge between the PC (via USB CDC/SLIP) and the wireless peripheral nodes (via ESP-NOW).

## Hardware Configuration
- **Board**: ESP32-S3-USB-OTG
- **Interface**: USB Native (CDC)
- **Display**: ST7789 240x240 LCD (Built-in)
- **Backlight**: GPIO 9 (Must be explicitly enabled)

## Build Instructions

### Dependencies
- **Adafruit GFX Library**
- **Adafruit ST7735 and ST7789 Library**
- **Adafruit BusIO**

### CRITICAL: Adafruit BusIO Patch (ESP32 Core 3.x)
If compiling with **ESP32 Arduino Core v3.0.0+**, the `Adafruit_BusIO` library currently has a compatibility issue regarding `BitOrder`.

**Fix:**
Edit `Adafruit_SPIDevice.h` in your library folder (e.g., `Documents/Arduino/libraries/Adafruit_BusIO/Adafruit_SPIDevice.h`).
Add the following code block near the top of the file (before the `BusIOBitOrder` logic):

```cpp
// PATCH: ESP32 Core v3 Fix
#ifndef BitOrder
  typedef uint8_t BitOrder;
#endif
```

Without this patch, compilation will fail with `BusIOBitOrder has not been declared`.

### Build Flags
The following flags are required in `arduino-cli.env` or IDE keys:
- `-DBOARD_ESPRESSIF_USB_OTG`
- `-DARDUINO_USB_MODE=1` (Native USB)
- `-DARDUINO_USB_CDC_ON_BOOT=1` (CDC on Boot)
- `-DESP32` (Ensure macro is defined)

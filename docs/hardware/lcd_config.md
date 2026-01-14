# LCD Configuration & Initialization Guide

This document details the LCD configuration for the OpenCockpit Wireless Bus project, primarily for the Coordinator Node (Espressif USB OTG) and potentially for Peripheral Nodes (WeAct Studio) if equipped.

## 1. Hardware Specifications

*   **Display Controller:** ST7789
*   **Resolution:** 240x240 pixels
*   **Interface:** SPI (4-wire)
*   **Color Depth:** 16-bit RGB565

## 2. Pin Assignments

These definitions should be included in `firmware/common/hardware_config.h`.

### Coordinator (Espressif ESP32-S3 USB OTG)
Based on board schematic:
*   **LCD_RST (Reset):** GPIO 8
*   **LCD_EN (CS):** GPIO 5
*   **LCD_DC (Data/Cmd):** GPIO 4
*   **LCD_SCLK (Clock):** GPIO 6
*   **LCD_MOSI (Data):** GPIO 7
*   **LCD_BL (Backlight):** GPIO 9

### Peripheral Nodes (WeAct Studio ESP32-S3)
*Note: WeAct boards do not have an onboard LCD. If connecting an external ST7789 module, use the following recommended (but remappable) standard SPI pins or custom wiring:*
*   **LCD_SCLK:** GPIO 12 (Example)
*   **LCD_MOSI:** GPIO 11 (Example)
*   **LCD_RST:** GPIO 10 (Example)
*   **LCD_DC:** GPIO 9 (Example)
*   **LCD_CS:** GPIO 46 (Example)
*   **LCD_BL:** GPIO 48 (Example)
*   *(User must confirm actual wiring for peripherals)*

## 3. Libraries

The current project uses **Native SPI** logic to drive the screen to minimize dependencies and latency. No external display library (like Adafruit_GFX or TFT_eSPI) is currently required for the base firmware.

*   `#include <SPI.h>` (Standard Arduino Library)

## 4. Initialization Logic

The LCD initialization sequence must include the following ST7789 commands. This logic is currently embedded in `coordinator.ino` but should be abstracted.

**Standard Sequence:**
1.  **Hardware Reset:** Toggle LCD_RST low for 100ms, then high.
2.  **Software Reset (0x01):** Wait 150ms.
3.  **Sleep Out (0x11):** Wait 500ms.
4.  **Color Mode (0x3A):** Set to 0x05 (16-bit).
5.  **Memory Data Access (0x36):** Set orientation (e.g., 0x00 for default).
6.  **Inversion (0x21):** Enable inversion (typically needed for IPS screens).
7.  **Display On (0x29):** Enable output.
8.  **Backlight:** Set LCD_BL high.

## 5. Modular Implementation Plan

To enable this in `hardware_config.h`:

```cpp
#if defined(BOARD_ESPRESSIF_USB_OTG)
    #define HAS_LCD
    #define LCD_DRIVER_ST7789
    #define PIN_LCD_RST  8
    #define PIN_LCD_CS   5
    #define PIN_LCD_DC   4
    #define PIN_LCD_CLK  6
    #define PIN_LCD_MOSI 7
    #define PIN_LCD_BL   9
#elif defined(BOARD_WEACT_STUDIO_S3)
    // Uncomment to enable LCD on WeAct if connected
    // #define HAS_LCD
    // #define PIN_LCD_...
#endif
```

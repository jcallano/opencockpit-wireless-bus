# Hardware Compatibility Analysis & Solution Plan

## 1. Problem Identification
The current codebase assumes a uniform hardware platform (Espressif ESP32-S3 USB OTG Development Board) for all nodes. However, your actual deployment uses mixed hardware:
*   **Coordinator (Node A):** Espressif ESP32-S3 USB OTG (8MB PSRAM, No Ext Flash).
*   **Peripheral Nodes (B & C):** WeAct Studio ESP32-S3 (8MB PSRAM, 8MB OPI Flash).

These boards have fundamentally different layouts, pinouts, and power management circuits.

## 2. Hardware Comparison

### Coordinator: Espressif ESP32-S3 USB OTG
*   **Special Circuitry:** Has complex onboard switching logic for USB.
    *   **USB_SEL (GPIO 18):** Switches between Host/Device circuits.
    *   **BOOST_EN (GPIO 13):** Activates 5V boost converter (essential for powering USB devices in Host mode).
    *   **LIMIT_EN (GPIO 17):** Controls current limiting.
    *   **DEV_VBUS_EN (GPIO 12):** Controls power to the "Device" port.
*   **Peripherals:** Onboard ST7789 LCD, Buttons (up/down/menu), LEDs.
*   **Flash:** Quad SPI (Standard).

### Peripheral Code: WeAct Studio ESP32-S3
*   **Circuitry:** Minimalist "Core" board.
    *   **No USB Switching Logic:** USB D+/D- are connected directly to the USB-C port (or pins).
    *   **No Onboard 5V Boost:** You must provide clean 5V to your attached peripherals (Joystick/MCDU) externally or via the 5V pin.
*   **Flash/PSRAM:** Uses **Octal SPI (OPI)**.
    *   **Critical Constraint:** OPI mode consumes **GPIO 33, 34, 35, 36, 37** (and others).
    *   **Conflict:** The default Coordinator code uses GPIO 33-38 for SD Card. If code was shared, this would crash the WeAct board.
*   **User I/O:** Typically only:
    *   **LED:** GPIO 48 (RGB) or sometimes simple LED.
    *   **Button:** GPIO 0 (Boot).

## 3. Findings in Current Code
The file `firmware/peripherals/node_b/node_b.ino` currently contains this:

```cpp
// USB Host power/routing (ESP32-S3-USB-OTG board)
#define USB_SEL_PIN         18  // HIGH = USB_HOST
#define DEV_VBUS_EN_PIN     12  // High enables DEV_VBUS supply
#define VBUS_EN_OTG_PIN     17  // High enables current limit/OTG power
```

**Why this is a problem for WeAct Nodes:**
1.  **Phantom Pins:** GPIO 12, 17, 18 on the WeAct board are standard GPIOs. Toggling them does nothing helpful for USB Host power and might interfere if you connected something else there.
2.  **Missing Power:** The WeAct board doesn't turn on USB power via software. You must ensure your physical wiring provides 5V to the attached Joystick/MCDU keypads.
3.  **Flash Mode:** Compiling for specific Flash modes (QIO vs OPI) is handled by build flags, but pin definitions need to change in C++.

## 4. Implementation Plan

To solve this, we will introduce a modular hardware configuration system.

### Step 1: Create `firmware/common/hardware_config.h`
This file will use `#ifdef` macros to define pins based on the selected board.

```cpp
#if defined(BOARD_ESPRESSIF_USB_OTG)
    #define PIN_USB_SEL     18
    #define PIN_POWER_EN    12
    // ...
#elif defined(BOARD_WEACT_STUDIO_S3)
    // WeAct specific definitions
    #define PIN_LED_STATUS  48
    // No USB SEL or Boost pins needed
#endif
```

### Step 4: LCD Configuration (Ref: `docs/hardware/lcd_config.md`)
We will verify `HAS_LCD` definition to conditionally compile display logic.

*   **Coordinator:** `#define HAS_LCD` (Active by default).
*   **Peripherals:** Commented out by default unless external screen added.

### Step 2: Update `arduino-cli.env` build flags
We will inject the board definition via compile-time flags.

**Coordinator (node A):**
`EXTRA_FLAGS="-DBOARD_ESPRESSIF_USB_OTG -DARDUINO_USB_MODE=1"`

**Peripherals (node B/C):**
`EXTRA_FLAGS="-DBOARD_WEACT_STUDIO_S3 -DBOARD_HAS_PSRAM -DARDUINO_USB_MODE=1"`

### Step 3: Refactor Firmware
Update `.ino` files to remove hardcoded pin values and replace them with the macros from `hardware_config.h`.

## 5. Next Actions for User
*   **Confirm Wiring:** For Node B/C (WeAct), ensure you are physically supplying 5V to your USB Devices. The ESP32 cannot "switch" this power on for you on this specific board.
*   **Approve Refactor:** Shall I proceed to create the `hardware_config.h` and refactor the code?

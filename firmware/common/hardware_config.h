#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

/**
 * OpenCockpit Wireless Avionics Bus
 * Hardware Configuration Header
 * 
 * Select board type via build flags:
 * -DBOARD_ESPRESSIF_USB_OTG  (Coordinator)
 * -DBOARD_WEACT_STUDIO_S3    (Node B/C)
 */

#if defined(BOARD_ESPRESSIF_USB_OTG)
    // ------------------------------------------------------------------------
    // Espressif ESP32-S3-USB-OTG Development Board
    // ------------------------------------------------------------------------
    #define HAS_LCD
    #define HAS_USB_POWER_CONTROL
    
    // USB Control Pins
    #define PIN_USB_SEL         18  // HIGH=Host, LOW=Device
    #define PIN_DEV_VBUS_EN     12  // High enables Device VBUS
    #define PIN_BOOST_EN        13  // High enables 5V Boost (Host VBUS)
    #define PIN_LIMIT_EN        17  // High enables current limiter
    
    // LCD Pins (ST7789)
    #define PIN_LCD_RST         8
    #define PIN_LCD_CS          5
    #define PIN_LCD_EN          5 // Alias for CS
    #define PIN_LCD_DC          4
    #define PIN_LCD_CLK         6
    #define PIN_LCD_MOSI        7
    #define PIN_LCD_BL          9
    
    // User Interface
    #define PIN_BTN_OK          0
    #define PIN_BTN_UP          10
    #define PIN_BTN_DW          11
    #define PIN_BTN_MENU        14
    #define PIN_LED_GREEN       15
    #define PIN_LED_YELLOW      16

#elif defined(BOARD_WEACT_STUDIO_S3)
    // ------------------------------------------------------------------------
    // WeAct Studio ESP32-S3 Core Board (8MB Flash/8MB PSRAM)
    // ------------------------------------------------------------------------
    // No onboard LCD by default
    // No USB power control (VBUS is hardwired or user-supplied)
    
    // User Interface
    #define PIN_LED_STATUS      48  // RGB LED or Single LED depending on revision
    #define PIN_BTN_BOOT        0   // Boot button
    
    // UART0 (Default pins usually 43/44 or 4/5 depending on variant, 
    // but often we just use USB Serial JTAG for logs)
    // If needed:
    // #define UART0_TX_PIN ...
    // #define UART0_RX_PIN ...

#else
    #error "No board type defined! Use -DBOARD_ESPRESSIF_USB_OTG or -DBOARD_WEACT_STUDIO_S3"
#endif

// Power management helper macro
inline void setupHardware() {
    #if defined(HAS_USB_POWER_CONTROL)
        // Initialize USB OTG Power Circuitry
        pinMode(PIN_USB_SEL, OUTPUT);
        digitalWrite(PIN_USB_SEL, LOW); // Default to Device mode (will change if Host)
        
        pinMode(PIN_DEV_VBUS_EN, OUTPUT);
        digitalWrite(PIN_DEV_VBUS_EN, HIGH); // Enable Device VBUS by default
        
        pinMode(PIN_BOOST_EN, OUTPUT);
        digitalWrite(PIN_BOOST_EN, LOW); // Boost off by default
        
        pinMode(PIN_LIMIT_EN, OUTPUT);
        digitalWrite(PIN_LIMIT_EN, HIGH); // Enable current limit
    #endif
    
    #ifdef PIN_LED_STATUS
        pinMode(PIN_LED_STATUS, OUTPUT);
        digitalWrite(PIN_LED_STATUS, 0); // Off
    #endif
}

inline void enableUsbHostPower(bool enable) {
    #if defined(HAS_USB_POWER_CONTROL)
        if (enable) {
            digitalWrite(PIN_USB_SEL, HIGH);      // Switch Mux to Host
            digitalWrite(PIN_DEV_VBUS_EN, LOW);   // Disable Device VBUS
            digitalWrite(PIN_BOOST_EN, HIGH);     // Enable 5V Boost
        } else {
            digitalWrite(PIN_BOOST_EN, LOW);      // Disable 5V Boost
            digitalWrite(PIN_USB_SEL, LOW);       // Switch Mux to Device
            digitalWrite(PIN_DEV_VBUS_EN, HIGH);  // Enable Device VBUS
        }
    #endif
    // On WeAct, power is hardwired/external, so software control is N/A
}

#endif // HARDWARE_CONFIG_H

/**
 * WinWing MCDU HID Emulation
 * Test firmware to validate SimAppPro compatibility
 *
 * VID: 0x4098 (WinWing)
 * PID: 0xBB36 (MCDU)
 */

#ifndef MCDU_HID_H
#define MCDU_HID_H

#include <Arduino.h>

// WinWing MCDU identifiers
#define MCDU_VID 0x4098
#define MCDU_PID 0xBB36

// HID Report sizes
#define MCDU_OUTPUT_REPORT_SIZE 64  // PC -> MCDU (display data)
#define MCDU_INPUT_REPORT_SIZE  24  // MCDU -> PC (button states)

// Display constants
#define MCDU_ROWS 14
#define MCDU_COLS 24
#define MCDU_BYTES_PER_CHAR 3
#define MCDU_DISPLAY_BUFFER_SIZE 1024

// Callback types
typedef void (*MCDUOutputCallback)(const uint8_t* data, uint16_t len);

class MCDUDevice {
public:
    MCDUDevice();

    // Initialize the MCDU HID device
    bool begin();

    // Send button state to PC
    bool sendButtonReport(const uint8_t* buttons, uint8_t len);

    // Set callback for output reports from PC
    void setOutputCallback(MCDUOutputCallback callback);

    // Check if device is connected to host
    bool isConnected();

    // Process pending data (call from loop)
    void process();

private:
    MCDUOutputCallback _output_callback;
    bool _connected;
    uint8_t _last_buttons[MCDU_INPUT_REPORT_SIZE];
};

extern MCDUDevice mcduDevice;

#endif // MCDU_HID_H

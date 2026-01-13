/**
 * WinWing MCDU Emulation Test
 *
 * This firmware emulates a WinWing MCDU to test compatibility with SimAppPro.
 *
 * Expected behavior:
 * 1. Device appears as "WinWing MCDU" with VID 0x4098, PID 0xBB36
 * 2. SimAppPro should recognize and connect to the device
 * 3. Display data sent by SimAppPro is logged to Serial
 * 4. Button presses can be simulated via Serial commands
 *
 * Serial Commands:
 *   'b' + button_number (0-71) = simulate button press
 *   'r' = release all buttons
 *   's' = print status
 */

#include <Arduino.h>
#include "mcdu_hid.h"

// Statistics
uint32_t outputReportsReceived = 0;
uint32_t lastReportTime = 0;
uint8_t lastReportHeader = 0;

// Button state (72 buttons = 9 bytes, but we use 24 for padding)
uint8_t buttonState[MCDU_INPUT_REPORT_SIZE] = {0};

// Callback for display data from PC
void onMCDUOutput(const uint8_t* data, uint16_t len) {
    outputReportsReceived++;
    lastReportTime = millis();
    lastReportHeader = data[0];

    // Log interesting packets
    if (data[0] == 0xF0) {
        // Initialization/command packet
        Serial.printf("[CMD] 0xF0 packet, len=%d, seq=%d\n", len, data[2]);
    } else if (data[0] == 0xF2) {
        // Display data packet
        // Don't spam - just count
        static uint32_t displayPackets = 0;
        displayPackets++;
        if (displayPackets % 100 == 0) {
            Serial.printf("[DISPLAY] %lu packets received\n", displayPackets);
        }
    } else if (data[0] == 0x02) {
        // Brightness/config command
        Serial.printf("[CFG] 0x02 packet, len=%d\n", len);
    } else {
        // Unknown packet type
        Serial.printf("[???] Header=0x%02X, len=%d\n", data[0], len);
    }
}

void printStatus() {
    Serial.println("\n=== MCDU Emulator Status ===");
    Serial.printf("Connected: %s\n", mcduDevice.isConnected() ? "YES" : "NO");
    Serial.printf("Output reports received: %lu\n", outputReportsReceived);
    Serial.printf("Last report: %lu ms ago\n", millis() - lastReportTime);
    Serial.printf("Last report header: 0x%02X\n", lastReportHeader);
    Serial.println("============================\n");
}

void simulateButtonPress(uint8_t button) {
    if (button >= 72) {
        Serial.println("Invalid button (0-71)");
        return;
    }

    // Set button bit
    uint8_t byteIndex = button / 8;
    uint8_t bitIndex = button % 8;
    buttonState[byteIndex] |= (1 << bitIndex);

    mcduDevice.sendButtonReport(buttonState, MCDU_INPUT_REPORT_SIZE);
    Serial.printf("Button %d pressed\n", button);
}

void releaseAllButtons() {
    memset(buttonState, 0, sizeof(buttonState));
    mcduDevice.sendButtonReport(buttonState, MCDU_INPUT_REPORT_SIZE);
    Serial.println("All buttons released");
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial

    Serial.println("\n========================================");
    Serial.println("  WinWing MCDU Emulator Test");
    Serial.println("  VID: 0x4098  PID: 0xBB36");
    Serial.println("========================================\n");

    // Initialize MCDU device
    mcduDevice.setOutputCallback(onMCDUOutput);

    if (!mcduDevice.begin()) {
        Serial.println("ERROR: Failed to initialize MCDU device!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("\nDevice ready. Waiting for SimAppPro connection...");
    Serial.println("Commands: 'b'+num=button, 'r'=release, 's'=status\n");
}

void loop() {
    // Process USB events
    mcduDevice.process();

    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 's':
            case 'S':
                printStatus();
                break;

            case 'r':
            case 'R':
                releaseAllButtons();
                break;

            case 'b':
            case 'B':
                // Read button number
                delay(10);
                if (Serial.available()) {
                    int button = Serial.parseInt();
                    simulateButtonPress(button);

                    // Auto-release after 100ms
                    delay(100);
                    releaseAllButtons();
                }
                break;

            case '\n':
            case '\r':
                // Ignore newlines
                break;

            default:
                Serial.printf("Unknown command: '%c'\n", cmd);
                break;
        }
    }

    // Periodic status (every 10 seconds if receiving data)
    static uint32_t lastStatusPrint = 0;
    if (outputReportsReceived > 0 && millis() - lastStatusPrint > 10000) {
        lastStatusPrint = millis();
        Serial.printf("[STATUS] %lu reports, last %lu ms ago\n",
                      outputReportsReceived, millis() - lastReportTime);
    }

    delay(1);
}

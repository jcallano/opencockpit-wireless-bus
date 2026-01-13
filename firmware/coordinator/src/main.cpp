/**
 * OpenCockpit Wireless Avionics Bus
 * Node A - Coordinator Main
 *
 * USB Composite Device (HID Joystick + HID Gamepad + CDC Serial)
 * ESP-NOW Coordinator for peripheral nodes
 *
 * Target: ESP32-S3 with native USB
 */

#include <Arduino.h>
#include "usb_device.h"
#include "espnow_coordinator.h"

// Task handles
TaskHandle_t espnowTaskHandle = nullptr;
TaskHandle_t usbTaskHandle = nullptr;

// Current HID reports
JoystickReport joystickReport;
GamepadReport gamepadReport;
SemaphoreHandle_t reportMutex;

// Serial passthrough buffer
#define SERIAL_BUFFER_SIZE 256
uint8_t serialTxBuffer[SERIAL_BUFFER_SIZE];
volatile uint16_t serialTxHead = 0;
volatile uint16_t serialTxTail = 0;

// Forward declarations
void espnowTask(void* parameter);
void usbTask(void* parameter);
void onHIDInput(uint8_t device_id, const uint8_t* report, uint8_t len);
void onSerialData(const uint8_t* data, uint8_t len);
void onNodeStatus(uint8_t node_id, bool connected);
void onMCDUOutput(const uint8_t* data, uint16_t len);

void setup() {
    // Initialize debug serial (USB CDC debug)
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== OpenCockpit Wireless Avionics Bus ===");
    Serial.println("Node A - Coordinator");
    Serial.println("=====================================\n");

    // Create mutex for report protection
    reportMutex = xSemaphoreCreateMutex();

    // Initialize reports
    memset(&joystickReport, 0, sizeof(joystickReport));
    joystickReport.report_id = REPORT_ID_JOYSTICK;
    joystickReport.x = 32768;  // Center
    joystickReport.y = 32768;  // Center
    joystickReport.z = 128;    // Center
    joystickReport.hat = 0x0F; // Released

    memset(&gamepadReport, 0, sizeof(gamepadReport));
    gamepadReport.report_id = REPORT_ID_GAMEPAD;
    gamepadReport.throttle1 = 48000; // IDLE position
    gamepadReport.throttle2 = 48000; // IDLE position

    // Initialize USB
    Serial.println("Initializing USB...");
    cockpitHID.begin();
    cockpitHID.setMCDUOutputCallback(onMCDUOutput);
    Serial.println("USB initialized");

    // Initialize ESP-NOW Coordinator
    Serial.println("Initializing ESP-NOW Coordinator...");
    if (!coordinator.begin()) {
        Serial.println("ERROR: Failed to initialize coordinator!");
        while (1) {
            delay(1000);
        }
    }

    // Set callbacks
    coordinator.setHIDInputCallback(onHIDInput);
    coordinator.setSerialDataCallback(onSerialData);
    coordinator.setNodeStatusCallback(onNodeStatus);

    // Create ESP-NOW task on Core 0 (highest priority)
    xTaskCreatePinnedToCore(
        espnowTask,
        "ESP-NOW",
        4096,
        nullptr,
        24,
        &espnowTaskHandle,
        0  // Core 0
    );

    // Create USB task on Core 1
    xTaskCreatePinnedToCore(
        usbTask,
        "USB",
        4096,
        nullptr,
        20,
        &usbTaskHandle,
        1  // Core 1
    );

    Serial.println("Setup complete. Running...\n");
}

void loop() {
    // Main loop handles serial passthrough to CDC
    // Forward CDC data to nodes
    while (serialCDC.available()) {
        uint8_t buffer[64];
        int len = serialCDC.read(buffer, sizeof(buffer));
        if (len > 0) {
            // Send to Node C (MiniFCU/EFIS)
            coordinator.sendSerialData(NODE_C_QUADRANT, buffer, len);
        }
    }

    // Forward buffered serial data from nodes to CDC
    while (serialTxHead != serialTxTail) {
        serialCDC.write(serialTxBuffer[serialTxTail]);
        serialTxTail = (serialTxTail + 1) % SERIAL_BUFFER_SIZE;
    }

    delay(1); // Small yield
}

// ESP-NOW task - runs on Core 0
void espnowTask(void* parameter) {
    Serial.println("ESP-NOW task started on Core 0");

    while (true) {
        coordinator.process();
        vTaskDelay(pdMS_TO_TICKS(1)); // ~1ms cycle
    }
}

// USB task - runs on Core 1
void usbTask(void* parameter) {
    Serial.println("USB task started on Core 1");

    uint32_t lastReport = 0;
    const uint32_t reportInterval = 8; // 125Hz

    while (true) {
        uint32_t now = millis();

        if (now - lastReport >= reportInterval) {
            lastReport = now;

            xSemaphoreTake(reportMutex, portMAX_DELAY);

            // Send joystick report
            cockpitHID.sendJoystickReport(joystickReport);

            // Send gamepad report
            cockpitHID.sendGamepadReport(gamepadReport);

            xSemaphoreGive(reportMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Callback: HID input from nodes
void onHIDInput(uint8_t device_id, const uint8_t* report, uint8_t len) {
    xSemaphoreTake(reportMutex, portMAX_DELAY);

    switch (device_id) {
        case DEV_TCA_SIDESTICK:
            // Parse Thrustmaster TCA Sidestick report (10 bytes)
            if (len >= 9) {
                // Skip report_id (byte 0), read axes
                joystickReport.x = report[1] | (report[2] << 8);
                joystickReport.y = report[3] | (report[4] << 8);
                joystickReport.z = report[5];
                joystickReport.slider = report[6];
                joystickReport.hat = report[7] & 0x0F;
                joystickReport.buttons = report[8] | (len > 9 ? report[9] << 8 : 0);
            }
            break;

        case DEV_TCA_QUADRANT:
            // Parse Thrustmaster TCA Quadrant report (15 bytes)
            if (len >= 14) {
                // B01: Engine controls (map to buttons)
                uint32_t buttons = 0;
                buttons |= (report[1] & 0xFF); // Engine master, fire, mode

                // B02: Throttle detents (map to buttons 8-15)
                buttons |= ((uint32_t)(report[2] & 0xFF) << 8);

                // Axes
                gamepadReport.throttle1 = report[3] | (report[4] << 8);
                gamepadReport.throttle2 = report[5] | (report[6] << 8);
                gamepadReport.flaps = report[7] | (report[8] << 8);

                // B09: Parking brake, rudder trim (map to buttons 16-23)
                buttons |= ((uint32_t)(report[9] & 0xFF) << 16);

                // Speedbrake axis
                gamepadReport.speedbrake = report[10] | (report[11] << 8);

                // B12: Gear, autobrake (map to buttons 24-31)
                buttons |= ((uint32_t)(report[12] & 0xFF) << 24);

                gamepadReport.buttons = buttons;
            }
            break;

        case DEV_WINWING_MCDU:
            // MCDU button press - could map to joystick buttons or handle separately
            if (len >= 2) {
                Serial.printf("MCDU Button: %d = %d\n", report[0], report[1]);
            }
            break;

        default:
            Serial.printf("Unknown device: %d\n", device_id);
            break;
    }

    xSemaphoreGive(reportMutex);
}

// Callback: Serial data from nodes
void onSerialData(const uint8_t* data, uint8_t len) {
    // Buffer for CDC output
    for (uint8_t i = 0; i < len; i++) {
        uint16_t nextHead = (serialTxHead + 1) % SERIAL_BUFFER_SIZE;
        if (nextHead != serialTxTail) {
            serialTxBuffer[serialTxHead] = data[i];
            serialTxHead = nextHead;
        }
    }
}

// Callback: Node connection status changed
void onNodeStatus(uint8_t node_id, bool connected) {
    Serial.printf("Node %d: %s\n", node_id, connected ? "CONNECTED" : "DISCONNECTED");
}

// Callback: MCDU output data from PC (display updates)
void onMCDUOutput(const uint8_t* data, uint16_t len) {
    // Forward MCDU display data to Node B
    if (len > 0) {
        coordinator.sendMCDUDisplay(NODE_B_JOYSTICK, data, len);
    }
}

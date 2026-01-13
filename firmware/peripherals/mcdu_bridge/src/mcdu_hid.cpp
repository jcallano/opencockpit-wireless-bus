/**
 * WinWing MCDU HID Emulation Implementation
 * Using EXACT HID descriptor from real MCDU hardware
 */

#include "mcdu_hid.h"
#include "USB.h"
#include "USBHID.h"

// Global instance
MCDUDevice mcduDevice;

// EXACT HID Report Descriptor from real WinWing MCDU (160 bytes)
static const uint8_t mcdu_hid_report_desc[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x04,       // Usage (Joystick)
    0xa1, 0x01,       // Collection (Application)

    // Report ID 1: Buttons + Axes
    0x85, 0x01,       //   Report ID (1)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (1)
    0x29, 0x80,       //   Usage Maximum (128)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x35, 0x00,       //   Physical Minimum (0)
    0x45, 0x01,       //   Physical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x80,       //   Report Count (128) = 16 bytes
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x33,       //   Usage (Rx)
    0x15, 0x00,       //   Logical Minimum (0)
    0x27, 0xff, 0x0f, 0x00, 0x00,  // Logical Maximum (4095)
    0x35, 0x00,       //   Physical Minimum (0)
    0x47, 0xff, 0x0f, 0x00, 0x00,  // Physical Maximum (4095)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x34,       //   Usage (Ry)
    0x15, 0x00,       //   Logical Minimum (0)
    0x27, 0xff, 0x0f, 0x00, 0x00,  // Logical Maximum (4095)
    0x35, 0x00,       //   Physical Minimum (0)
    0x47, 0xff, 0x0f, 0x00, 0x00,  // Physical Maximum (4095)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x15, 0x00,       //   Logical Minimum (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  // Logical Maximum (65535)
    0x35, 0x00,       //   Physical Minimum (0)
    0x47, 0xff, 0xff, 0x00, 0x00,  // Physical Maximum (65535)
    0x09, 0xd0,       //   Usage (0xD0)
    0x09, 0xd1,       //   Usage (0xD1)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x01,       //   Input (Const,Array,Abs)

    // Report ID 2: Vendor data (13 bytes)
    0x85, 0x02,       //   Report ID (2)
    0x06, 0xff, 0x00, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,       //   Usage (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xff, 0x00, //   Logical Maximum (255)
    0x35, 0x00,       //   Physical Minimum (0)
    0x46, 0xff, 0x00, //   Physical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x0d,       //   Report Count (13)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x09, 0x02,       //   Usage (2)
    0x91, 0x02,       //   Output (Data,Var,Abs)

    // Report ID 0xF0: Init commands (63 bytes)
    0x85, 0xf0,       //   Report ID (240)
    0x06, 0xff, 0x00, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x03,       //   Usage (3)
    0x95, 0x3f,       //   Report Count (63)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x09, 0x04,       //   Usage (4)
    0x91, 0x02,       //   Output (Data,Var,Abs)

    // Report ID 0xF2: Display data (63 bytes)
    0x85, 0xf2,       //   Report ID (242)
    0x06, 0xff, 0x00, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x05,       //   Usage (5)
    0x95, 0x3f,       //   Report Count (63)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x09, 0x06,       //   Usage (6)
    0x91, 0x02,       //   Output (Data,Var,Abs)

    0xc0              // End Collection
};

// Report structures
struct __attribute__((packed)) ButtonReport {
    uint8_t buttons[16];  // 128 buttons
    uint16_t rx;          // Rx axis
    uint16_t ry;          // Ry axis
    uint16_t const1;      // Constant
    uint16_t const2;      // Constant
};

// Custom HID class for MCDU
class MCDUHIDDevice : public USBHIDDevice {
public:
    MCDUHIDDevice() : _output_cb(nullptr) {
        memset(&_buttonReport, 0, sizeof(_buttonReport));
    }

    uint16_t _onGetDescriptor(uint8_t* buffer) override {
        memcpy(buffer, mcdu_hid_report_desc, sizeof(mcdu_hid_report_desc));
        return sizeof(mcdu_hid_report_desc);
    }

    void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) override {
        Serial.printf("[RX] Report ID: 0x%02X, len: %d\n", report_id, len);

        if (_output_cb) {
            // Prepend report_id to the data for the callback
            uint8_t fullData[64];
            fullData[0] = report_id;
            memcpy(fullData + 1, buffer, min((int)len, 63));
            _output_cb(fullData, len + 1);
        }

        // Send ACK response based on report type
        sendAckResponse(report_id, buffer, len);
    }

    void _onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len) override {
        Serial.printf("[FEATURE] Report ID: 0x%02X, len: %d\n", report_id, len);
    }

    void setOutputCallback(MCDUOutputCallback cb) {
        _output_cb = cb;
    }

    void sendAckResponse(uint8_t report_id, const uint8_t* data, uint16_t len) {
        uint8_t response[64] = {0};

        switch (report_id) {
            case 0x02:
                handleReport02(data, len);
                break;

            case 0xF0:
                handleReportF0(data, len);
                break;

            case 0xF2:
                // Display data - no response needed based on capture
                break;
        }
    }

    void handleReport02(const uint8_t* data, uint16_t len) {
        // Report 0x02 protocol from capture:
        // Query IN:  01 00 00 00 CMD_TYPE CMD_PARAM ... (without report ID)
        // Response:  32 CB 00 00 RESP_TYPE QUERY_ECHO DATA...

        uint8_t response[13] = {0};

        // Fixed header: Device ID 0xCB32
        response[0] = 0x32;  // Device ID low
        response[1] = 0xCB;  // Device ID high
        response[2] = 0x00;
        response[3] = 0x00;

        // Parse command (data doesn't include report ID)
        uint8_t cmdType = (len > 4) ? data[4] : 0;
        uint8_t cmdParam1 = (len > 5) ? data[5] : 0;
        uint8_t cmdParam2 = (len > 6) ? data[6] : 0;

        Serial.printf("[0x02] CMD type=%02X param=%02X %02X\n", cmdType, cmdParam1, cmdParam2);

        if (cmdType == 0x01) {
            // Query commands - response[5] echoes the query type
            switch (cmdParam1) {
                case 0x00:
                    // Query 0x00 - Echo: 32cb0000 01 00 ...
                    response[4] = 0x01;
                    response[5] = 0x00;
                    break;
                case 0x01:
                    // Query 0x01 - Device type: 32cb0000 05 01 26010001
                    response[4] = 0x05;
                    response[5] = 0x01;
                    response[6] = 0x26;
                    response[7] = 0x01;
                    response[8] = 0x00;
                    response[9] = 0x01;
                    break;
                case 0x02:
                    // Query 0x02 - VID/PID: 32cb0000 05 02 32bb0301
                    response[4] = 0x05;
                    response[5] = 0x02;
                    response[6] = 0x32;
                    response[7] = 0xBB;
                    response[8] = 0x03;
                    response[9] = 0x01;
                    break;
                case 0x03:
                    // Query 0x03 - Firmware: 32cb0000 07 03 121a565b2307
                    response[4] = 0x07;
                    response[5] = 0x03;
                    response[6] = 0x12;  // Firmware version
                    response[7] = 0x1A;
                    response[8] = 0x56;
                    response[9] = 0x5B;
                    response[10] = 0x23;
                    response[11] = 0x07;
                    break;
                case 0x18:
                    // Query 0x18 - Status: 32cb0000 02 18 00...
                    response[4] = 0x02;
                    response[5] = 0x18;
                    break;
                default:
                    response[4] = 0x05;
                    response[5] = cmdParam1;
                    break;
            }
        } else if (cmdType == 0x04) {
            // Register read: 32cb0000 08 REG_HI REG_LO 00 DATA...
            response[4] = 0x08;
            response[5] = cmdParam1;  // Register byte 1
            response[6] = cmdParam2;  // Register byte 2
            response[7] = 0x00;

            // Register values from capture (cmdParam1=05, cmdParam2=9c for 0x059C)
            if (cmdParam1 == 0x05 && cmdParam2 == 0x9C) {
                response[8] = 0x5B; response[9] = 0x56;
                response[10] = 0x1A; response[11] = 0x12;
            } else if (cmdParam1 == 0x05 && cmdParam2 == 0xA0) {
                response[8] = 0x7D; response[9] = 0x07;
                response[10] = 0x07; response[11] = 0x23;
            } else if (cmdParam1 == 0x05 && cmdParam2 == 0xA4) {
                response[8] = 0x4F; response[9] = 0x62;
                response[10] = 0x01; response[11] = 0x20;
            } else if (cmdParam1 == 0x05 && cmdParam2 == 0x04) {
                response[8] = 0x01;
            }
        }

        _hid->SendReport(0x02, response, 13);
        Serial.printf("[TX] 0x02: %02x%02x%02x%02x %02x%02x%02x%02x\n",
            response[0], response[1], response[2], response[3],
            response[4], response[5], response[6], response[7]);
    }

    void handleReportF0(const uint8_t* data, uint16_t len) {
        // Report 0xF0 protocol from capture:
        // Response: f0 00 06 12 32 cb 00 00 05 01 00 00 eb 07 ...

        uint8_t response[63] = {0};
        uint8_t cmd0 = (len > 0) ? data[0] : 0;
        uint8_t cmd1 = (len > 1) ? data[1] : 0;

        Serial.printf("[0xF0] CMD: %02X %02X\n", cmd0, cmd1);

        if (cmd0 == 0x02 && cmd1 == 0x00) {
            // Init start command - no response needed, wait for next
        } else if (cmd0 == 0x00) {
            // Sequence commands - send ACK
            uint8_t seq = cmd1;  // Sequence number (01, 02, 03...)

            if (seq <= 0x03) {
                // ACK for init sequences
                response[0] = 0x01;
                response[1] = seq + 3;  // Response sequence 04, 05, 06...
            } else {
                // Device identification response
                response[0] = 0x00;
                response[1] = 0x06;
                response[2] = 0x12;  // Firmware version 18 (0x12)
                response[3] = 0x32;  // Device ID low (0xCB32)
                response[4] = 0xCB;  // Device ID high
                response[5] = 0x00;
                response[6] = 0x00;
                response[7] = 0x05;
                response[8] = 0x01;
                response[9] = 0x00;
                response[10] = 0x00;
                response[11] = 0xEB;
                response[12] = 0x07;
            }
        } else if (cmd0 == 0x01) {
            // Simple command - echo ACK
            response[0] = 0x01;
            response[1] = cmd1;
        }

        _hid->SendReport(0xF0, response, 63);
        Serial.printf("[TX] 0xF0: %02x%02x%02x%02x\n",
            response[0], response[1], response[2], response[3]);
    }

    void setHID(USBHID* hid) {
        _hid = hid;
    }

    bool sendButtonReport() {
        return _hid->SendReport(0x01, (uint8_t*)&_buttonReport, sizeof(_buttonReport));
    }

    void setButton(uint8_t button, bool pressed) {
        if (button >= 128) return;
        uint8_t byteIdx = button / 8;
        uint8_t bitIdx = button % 8;
        if (pressed) {
            _buttonReport.buttons[byteIdx] |= (1 << bitIdx);
        } else {
            _buttonReport.buttons[byteIdx] &= ~(1 << bitIdx);
        }
    }

    void clearButtons() {
        memset(&_buttonReport, 0, sizeof(_buttonReport));
    }

private:
    MCDUOutputCallback _output_cb;
    USBHID* _hid;
    ButtonReport _buttonReport;
};

static USBHID hid;
static MCDUHIDDevice mcduHID;

MCDUDevice::MCDUDevice()
    : _output_callback(nullptr)
    , _connected(false)
{
    memset(_last_buttons, 0, sizeof(_last_buttons));
}

bool MCDUDevice::begin() {
    // Configure USB with WinWing VID/PID
    USB.VID(MCDU_VID);
    USB.PID(MCDU_PID);
    USB.firmwareVersion(0x0103);  // bcdDevice 1.03 to match real MCDU
    USB.productName("WINWING MCDU-32-CAPTAIN");
    USB.manufacturerName("Winwing");
    USB.serialNumber("EMULATED001");

    // Add MCDU HID device
    hid.addDevice(&mcduHID, sizeof(mcdu_hid_report_desc));
    mcduHID.setHID(&hid);

    // Start HID
    hid.begin();

    // Start USB
    USB.begin();

    _connected = true;
    Serial.println("MCDU HID device started (EXACT descriptor)");
    Serial.printf("VID: 0x%04X, PID: 0x%04X\n", MCDU_VID, MCDU_PID);
    Serial.printf("Descriptor size: %d bytes\n", sizeof(mcdu_hid_report_desc));

    return true;
}

bool MCDUDevice::sendButtonReport(const uint8_t* buttons, uint8_t len) {
    if (!_connected) return false;

    for (int i = 0; i < min((int)len, 16); i++) {
        for (int bit = 0; bit < 8; bit++) {
            mcduHID.setButton(i * 8 + bit, (buttons[i] >> bit) & 1);
        }
    }
    return mcduHID.sendButtonReport();
}

void MCDUDevice::setOutputCallback(MCDUOutputCallback callback) {
    _output_callback = callback;
    mcduHID.setOutputCallback(callback);
}

bool MCDUDevice::isConnected() {
    return _connected && USB;
}

void MCDUDevice::process() {
    // USB events handled internally
}

/**
 * OpenCockpit Wireless Avionics Bus
 * USB Composite Device Implementation - Node A Coordinator
 */

#include "usb_device.h"

// Global instances
CockpitHID cockpitHID;
USBCDC serialCDC;

CockpitHID::CockpitHID() : _hid(), _started(false), _mcdu_callback(nullptr) {
    memset(&_last_joystick, 0, sizeof(_last_joystick));
    memset(&_last_gamepad, 0, sizeof(_last_gamepad));
}

bool CockpitHID::begin() {
    if (_started) return true;

    // Add this device to the HID interface
    _hid.addDevice(this, sizeof(hid_report_descriptor));

    // Initialize USB
    USB.productName("OpenCockpit Wireless Bridge");
    USB.manufacturerName("OpenCockpit");
    USB.serialNumber("OC001");

    // Start HID
    _hid.begin();

    // Start CDC for serial passthrough
    serialCDC.begin(9600);

    // Start USB
    USB.begin();

    _started = true;
    return true;
}

bool CockpitHID::sendJoystickReport(const JoystickReport& report) {
    if (!_started) return false;

    // Only send if changed
    if (memcmp(&report, &_last_joystick, sizeof(report)) == 0) {
        return true;
    }

    _last_joystick = report;
    return _hid.SendReport(REPORT_ID_JOYSTICK, (uint8_t*)&report + 1, sizeof(report) - 1);
}

bool CockpitHID::sendGamepadReport(const GamepadReport& report) {
    if (!_started) return false;

    // Only send if changed
    if (memcmp(&report, &_last_gamepad, sizeof(report)) == 0) {
        return true;
    }

    _last_gamepad = report;
    return _hid.SendReport(REPORT_ID_GAMEPAD, (uint8_t*)&report + 1, sizeof(report) - 1);
}

uint16_t CockpitHID::_onGetDescriptor(uint8_t* buffer) {
    memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
    return sizeof(hid_report_descriptor);
}

void CockpitHID::_onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    // Handle output reports (e.g., from MCDU software)
    if (_mcdu_callback && len > 0) {
        _mcdu_callback(buffer, len);
    }
}

void CockpitHID::_onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    // Handle feature reports if needed
}

void CockpitHID::setMCDUOutputCallback(MCDUOutputCallback callback) {
    _mcdu_callback = callback;
}

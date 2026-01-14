/**
 * OpenCockpit Wireless Avionics Bus
 * USB Composite Device Header - Node A Coordinator
 *
 * Implements:
 * - HID Joystick (Thrustmaster TCA emulation)
 * - HID Gamepad (for Quadrant)
 * - CDC Serial (MiniFCU passthrough)
 */

#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "USBCDC.h"

// Report IDs
#define REPORT_ID_JOYSTICK 1
#define REPORT_ID_GAMEPAD  2

// Joystick report structure (matches Thrustmaster TCA Sidestick)
#pragma pack(push, 1)
struct JoystickReport {
    uint8_t report_id;      // REPORT_ID_JOYSTICK
    uint16_t x;             // X axis (roll)
    uint16_t y;             // Y axis (pitch)
    uint8_t z;              // Z twist (rudder/yaw)
    uint8_t slider;         // Slider axis
    uint8_t hat;            // Hat switch (0-8, 0xF = center)
    uint16_t buttons;       // 16 buttons
};

// Gamepad report structure (for Quadrant)
struct GamepadReport {
    uint8_t report_id;      // REPORT_ID_GAMEPAD
    uint16_t throttle1;     // Throttle 1 axis
    uint16_t throttle2;     // Throttle 2 axis
    uint16_t flaps;         // Flaps axis
    uint16_t speedbrake;    // Speedbrake axis
    uint32_t buttons;       // 32 buttons (includes detent flags)
};
#pragma pack(pop)

// HID Report Descriptor for combined Joystick + Gamepad
static const uint8_t hid_report_descriptor[] = {
    // Joystick (Report ID 1)
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xA1, 0x01,        // COLLECTION (Application)
    0x85, REPORT_ID_JOYSTICK, // REPORT_ID (1)

    // X, Y axes (16-bit each)
    0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //   USAGE (X)
    0x09, 0x31,        //   USAGE (Y)
    0x16, 0x00, 0x00,  //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0xFF,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // Z axis (8-bit)
    0x09, 0x32,        //   USAGE (Z)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // Slider (8-bit)
    0x09, 0x36,        //   USAGE (Slider)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // Hat Switch
    0x09, 0x39,        //   USAGE (Hat switch)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x07,        //   LOGICAL_MAXIMUM (7)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x46, 0x3B, 0x01,  //   PHYSICAL_MAXIMUM (315)
    0x65, 0x14,        //   UNIT (Eng Rot:Angular Pos)
    0x75, 0x04,        //   REPORT_SIZE (4)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x81, 0x42,        //   INPUT (Data,Var,Abs,Null)

    // Padding (4 bits)
    0x75, 0x04,        //   REPORT_SIZE (4)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x81, 0x03,        //   INPUT (Const,Var,Abs)

    // Buttons (16)
    0x05, 0x09,        //   USAGE_PAGE (Button)
    0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
    0x29, 0x10,        //   USAGE_MAXIMUM (Button 16)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x10,        //   REPORT_COUNT (16)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    0xC0,              // END_COLLECTION

    // Gamepad (Report ID 2) - For Quadrant
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,        // USAGE (Game Pad)
    0xA1, 0x01,        // COLLECTION (Application)
    0x85, REPORT_ID_GAMEPAD, // REPORT_ID (2)

    // 4 axes (16-bit each): throttle1, throttle2, flaps, speedbrake
    0x05, 0x02,        //   USAGE_PAGE (Simulation Controls)
    0x09, 0xBB,        //   USAGE (Throttle)
    0x09, 0xBB,        //   USAGE (Throttle)
    0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
    0x09, 0x36,        //   USAGE (Slider)
    0x09, 0x36,        //   USAGE (Slider)
    0x16, 0x00, 0x00,  //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0xFF,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x04,        //   REPORT_COUNT (4)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // Buttons (32)
    0x05, 0x09,        //   USAGE_PAGE (Button)
    0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
    0x29, 0x20,        //   USAGE_MAXIMUM (Button 32)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x20,        //   REPORT_COUNT (32)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    0xC0               // END_COLLECTION
};

// Custom HID device class
class CockpitHID : public USBHIDDevice {
public:
    CockpitHID();

    // Initialize the HID device
    bool begin();

    // Send joystick report
    bool sendJoystickReport(const JoystickReport& report);

    // Send gamepad report
    bool sendGamepadReport(const GamepadReport& report);

    // USBHIDDevice interface
    uint16_t _onGetDescriptor(uint8_t* buffer) override;
    void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) override;
    void _onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len) override;

    // Callback for MCDU output data
    typedef void (*MCDUOutputCallback)(const uint8_t* data, uint16_t len);
    void setMCDUOutputCallback(MCDUOutputCallback callback);

private:
    USBHID _hid;
    bool _started;
    MCDUOutputCallback _mcdu_callback;

    // Last sent reports (for change detection)
    JoystickReport _last_joystick;
    GamepadReport _last_gamepad;
};

// Global instances
extern CockpitHID cockpitHID;
extern USBCDC serialCDC;

#endif // USB_DEVICE_H

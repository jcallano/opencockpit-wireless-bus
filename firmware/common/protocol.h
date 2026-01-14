/**
 * OpenCockpit Wireless Avionics Bus
 * Protocol Definition Header
 *
 * Common message structures and types for ESP-NOW communication
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>

// Frame header marker
#define FRAME_HEADER 0xAA

// Message Types
enum MessageType : uint8_t {
    MSG_DISCOVERY_REQ   = 0x01,
    MSG_DISCOVERY_RSP   = 0x02,
    MSG_REGISTER_REQ    = 0x03,
    MSG_REGISTER_ACK    = 0x04,
    MSG_HEARTBEAT       = 0x10,
    MSG_HEARTBEAT_ACK   = 0x11,
    MSG_HID_INPUT       = 0x20,
    MSG_HID_OUTPUT      = 0x21,
    MSG_SERIAL_DATA     = 0x30,
    MSG_MCDU_DISPLAY    = 0x40,
    MSG_MCDU_INPUT      = 0x41,
    MSG_TEST_REQ        = 0xE0,
    MSG_TEST_RSP        = 0xE1,
    MSG_ERROR           = 0xF0,
    MSG_RESET           = 0xFF
};

// Node Identifiers
enum NodeId : uint8_t {
    NODE_COORDINATOR = 0x00,
    NODE_B_JOYSTICK  = 0x01,
    NODE_C_QUADRANT  = 0x02,
    NODE_BROADCAST   = 0xFF
};

// Node Types
enum NodeType : uint8_t {
    NODE_TYPE_HID       = 0x01,
    NODE_TYPE_SERIAL    = 0x02,
    NODE_TYPE_MIXED     = 0x03
};

// Capability Flags
enum NodeCapabilities : uint8_t {
    CAP_HID_INPUT       = 0x01,
    CAP_HID_OUTPUT      = 0x02,
    CAP_SERIAL          = 0x04,
    CAP_DISPLAY         = 0x08,
    CAP_BIDIRECTIONAL   = 0x10,
    CAP_LOW_LATENCY     = 0x20
};

// Device IDs for HID routing
enum DeviceId : uint8_t {
    DEV_TCA_SIDESTICK   = 0x01,
    DEV_TCA_QUADRANT    = 0x02,
    DEV_WINWING_MCDU    = 0x03,
    DEV_MINIFCU         = 0x04,
    DEV_MINIEFIS        = 0x05
};

// Maximum payload size (ESP-NOW limit is 250 bytes)
#define MAX_ESPNOW_PAYLOAD 250
#define MAX_HID_REPORT_SIZE 64
#define MAX_SERIAL_DATA_SIZE 200
#define MAX_NODE_NAME_SIZE 16
#define FIRMWARE_VERSION_SIZE 8

// CRC8 calculation (Dallas/Maxim polynomial 0x31)
inline uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        uint8_t inbyte = *data++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

// Base message frame header (4 bytes)
#pragma pack(push, 1)
struct MessageHeader {
    uint8_t header;     // Always FRAME_HEADER (0xAA)
    uint8_t msg_type;   // MessageType enum
    uint8_t src_node;   // Source NodeId
    uint8_t dst_node;   // Destination NodeId
};

// Discovery Response payload
struct DiscoveryResponse {
    uint8_t node_type;              // NodeType enum
    uint8_t capabilities;           // NodeCapabilities bitmap
    uint8_t device_count;           // Number of connected USB devices
    uint8_t mac_address[6];         // Node MAC address
    char firmware_version[FIRMWARE_VERSION_SIZE];
    char node_name[MAX_NODE_NAME_SIZE];
};

// Register Request payload
struct RegisterRequest {
    uint8_t assigned_node_id;       // Assigned node ID
    uint8_t poll_interval_ms;       // Suggested polling interval
    uint8_t reserved[2];            // Reserved for future use
};

// HID Input Message payload
struct HIDInputPayload {
    uint8_t device_id;              // DeviceId enum
    uint8_t report_id;              // HID report ID
    uint8_t report_length;          // Length of HID report
    uint8_t report_data[MAX_HID_REPORT_SIZE];
};

// HID Output Message payload
struct HIDOutputPayload {
    uint8_t device_id;              // Target device
    uint8_t report_id;              // HID report ID
    uint8_t report_length;          // Length of HID report
    uint8_t report_data[MAX_HID_REPORT_SIZE];
};

// Serial Data Message payload
struct SerialDataPayload {
    uint8_t port_id;                // Serial port identifier
    uint8_t data_length;            // Length of serial data
    uint8_t data[MAX_SERIAL_DATA_SIZE];
};

// MCDU Display Message payload
struct MCDUDisplayPayload {
    uint8_t command_type;           // 0x01=Text, 0x02=LED, 0x03=Brightness
    uint8_t row;                    // Display row (0-13)
    uint8_t column;                 // Display column (0-23)
    uint8_t color;                  // Text color
    uint8_t length;                 // Text length
    uint8_t text[24];               // Text data
};

// MCDU Input Message payload (button press)
struct MCDUInputPayload {
    uint8_t button_index;           // Button ID (0-71)
    uint8_t button_state;           // 0=released, 1=pressed
};

// Error Message payload
struct ErrorPayload {
    uint8_t error_code;
    uint8_t error_data[4];
};

// Heartbeat payload
struct HeartbeatPayload {
    uint32_t timestamp_ms;          // Sender's millis()
    uint8_t status;                 // Node status flags
};

// Test message payload (echoed back by node)
struct TestPayload {
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint8_t data[0];                // Optional variable-length data
};

// Complete message structure (header + max payload + CRC)
struct ESPNowMessage {
    MessageHeader header;
    uint8_t payload[MAX_ESPNOW_PAYLOAD - sizeof(MessageHeader) - 1];
    // CRC is appended at the end of actual payload
};
#pragma pack(pop)

// Peer information structure
struct PeerInfo {
    uint8_t mac_address[6];
    uint8_t node_id;
    uint8_t node_type;
    uint8_t capabilities;
    uint8_t device_count;
    bool registered;
    bool connected;
    uint32_t last_heartbeat_ms;
    char node_name[MAX_NODE_NAME_SIZE];
};

// Helper function to build a message
inline size_t buildMessage(uint8_t* buffer, MessageType type, uint8_t src, uint8_t dst,
                           const void* payload, size_t payload_len) {
    MessageHeader* hdr = (MessageHeader*)buffer;
    hdr->header = FRAME_HEADER;
    hdr->msg_type = type;
    hdr->src_node = src;
    hdr->dst_node = dst;

    if (payload && payload_len > 0) {
        memcpy(buffer + sizeof(MessageHeader), payload, payload_len);
    }

    size_t total_len = sizeof(MessageHeader) + payload_len;
    buffer[total_len] = crc8(buffer, total_len);

    return total_len + 1; // Include CRC byte
}

// Helper function to validate message CRC
inline bool validateMessage(const uint8_t* buffer, size_t len) {
    if (len < sizeof(MessageHeader) + 1) return false;
    if (buffer[0] != FRAME_HEADER) return false;

    uint8_t received_crc = buffer[len - 1];
    uint8_t calc_crc = crc8(buffer, len - 1);

    return received_crc == calc_crc;
}

#endif // PROTOCOL_H

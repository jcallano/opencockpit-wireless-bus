/**
 * OpenCockpit Wireless Avionics Bus
 * Node B - Sidestick + MCDU Peripheral
 *
 * Responsibilities:
 * - USB Host: Thrustmaster TCA Sidestick HID input
 * - USB Host: WinWing MCDU HID input/output
 * - ESP-NOW: HID input + MCDU display + button events
 */

#include <Arduino.h>
#include "usb/usb_host.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define ESPNOW_LOG_SERIAL Serial0

#define BOARD_WEACT_STUDIO_S3
#include "../../common/hardware_config.h"
#include "../../common/espnow_config.h"
#include "../../common/protocol.h"

// -----------------------------
// Board/Console Configuration
// -----------------------------
// UART0 pins should be defined by board variant or defaults
#ifndef UART0_TX_PIN
  #define UART0_TX_PIN 43
#endif
#ifndef UART0_RX_PIN
  #define UART0_RX_PIN 44
#endif

// USB definitions are now in hardware_config.h

#define LOG_SERIAL          Serial0
#define LOG_SERIAL_BEGIN()  Serial0.begin(115200, SERIAL_8N1, UART0_RX_PIN, UART0_TX_PIN)

// -----------------------------
// Configuration
// -----------------------------
#define NODE_NAME           "NODE_B_MCDU"
#define FW_VERSION          "1.0.0"

#define THRUSTMASTER_VID    0x044F
#define THRUSTMASTER_PID_STICK 0x0405

#define MCDU_VID            0x4098
#define MCDU_PID_PRIMARY    0xBB36
#define MCDU_PID_ALT        0xBB3A

#define HID_MAX_PKT         64
#define MCDU_PACKET_SIZE    64
#define MCDU_PAYLOAD_SIZE   63
#define MCDU_BUTTON_BYTES   16
#define MCDU_ROWS           14
#define MCDU_COLS           24
#define MCDU_BYTES_PER_CHAR 3
#define MCDU_DISPLAY_SIZE   1024
#define PACKET_PACING_MS    2


// Coordinator MAC (set after first discovery/register message)
static uint8_t g_coordinator_mac[6] = {0};
static bool g_have_coordinator = false;

// Node ID assigned by coordinator (default to Node B)
static uint8_t g_node_id = NODE_B_JOYSTICK;

// -----------------------------
// USB Host Globals
// -----------------------------
static usb_host_client_handle_t g_client = nullptr;
static usb_device_handle_t g_stick_dev = nullptr;
static usb_device_handle_t g_mcdu_dev = nullptr;

static usb_transfer_t *g_stick_in_xfer = nullptr;
static usb_transfer_t *g_mcdu_in_xfer = nullptr;
static usb_transfer_t *g_mcdu_out_xfer = nullptr;

static bool g_stick_connected = false;
static bool g_mcdu_connected = false;
static bool g_mcdu_ready = false;
static uint8_t g_mcdu_ep_in = 0;
static uint8_t g_mcdu_ep_out = 0;
static uint16_t g_mcdu_mps_in = 64;
static uint16_t g_mcdu_mps_out = 64;
static int g_mcdu_ifc_in = 0;
static int g_mcdu_ifc_out = 0;
static int g_mcdu_alt_in = 0;
static int g_mcdu_alt_out = 0;
static volatile bool g_mcdu_out_busy = false;

// -----------------------------
// ESP-NOW RX Queue
// -----------------------------
struct RxPacket {
    uint8_t mac[6];
    uint8_t data[MAX_ESPNOW_PAYLOAD];
    int len;
};

static QueueHandle_t g_rx_queue = nullptr;

// -----------------------------
// HID State
// -----------------------------
static uint8_t g_last_stick_report[HID_MAX_PKT];
static size_t g_last_stick_len = 0;

// Rate Limiting & Optimization Globals
static PackedSidestickPayload g_last_sent_stick = {};
static uint32_t g_last_stick_send_ms = 0;
static const uint32_t STICK_RATE_LIMIT_MS = 20; // Max 50Hz
static const uint16_t STICK_AXIS_DEADBAND = 8;  // For 12-bit quantization

// Forward declaration
static void send_sidestick_packed(const uint8_t* raw_data, size_t len);

static portMUX_TYPE g_mcdu_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_mcdu_last_buttons[MCDU_BUTTON_BYTES];
static QueueHandle_t g_mcdu_rx_queue = nullptr;

struct McduRxPacket {
    uint8_t len;
    uint8_t data[MCDU_PACKET_SIZE];
};

// -----------------------------
// MCDU Display Buffer
// -----------------------------
static uint8_t g_mcdu_display[MCDU_DISPLAY_SIZE];
static bool g_mcdu_flush_active = false;
static size_t g_mcdu_flush_offset = 0;
static uint32_t g_mcdu_last_send_ms = 0;
static bool g_mcdu_init_done = false;
static bool g_mcdu_brightness_pending = false;

static uint8_t g_mcdu_pending_report[MCDU_PACKET_SIZE];
static size_t g_mcdu_pending_len = 0;

// -----------------------------
// WinWing MCDU init sequence (captured)
// -----------------------------
static const uint8_t kMcduInitSeq[][64] = {
    {0xf0, 0x0, 0x1, 0x38, 0x32, 0xbb, 0x0, 0x0, 0x1e, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x18, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0x8, 0x0, 0x0, 0x0, 0x34, 0x0, 0x18, 0x0, 0xe, 0x0, 0x18, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x2, 0x38, 0x0, 0x0, 0x0, 0x1, 0x0, 0x5, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x1, 0x0, 0x6, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x3, 0x38, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0xff, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0xa5, 0xff, 0xff, 0x5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x4, 0x38, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0xff, 0xff, 0xff, 0xff, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0xff, 0xff, 0x0, 0xff, 0x7, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x5, 0x38, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x3d, 0xff, 0x0, 0xff, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0xff, 0x63, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x6, 0x38, 0xff, 0xff, 0x9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0xff, 0xff, 0xa, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x7, 0x38, 0x0, 0x0, 0x2, 0x0, 0x0, 0xff, 0xff, 0xff, 0xb, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x42, 0x5c, 0x61, 0xff, 0xc, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x8, 0x38, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x77, 0x77, 0x77, 0xff, 0xd, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x5e, 0x73, 0x79, 0xff, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x9, 0x38, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x20, 0x20, 0x20, 0xff, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0xa5, 0xff, 0xff, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0xa, 0x38, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0xff, 0xff, 0xff, 0xff, 0x11, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0xb, 0x38, 0xff, 0x12, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x3d, 0xff, 0x0, 0xff, 0x13, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0xc, 0x38, 0x0, 0x3, 0x0, 0xff, 0x63, 0xff, 0xff, 0x14, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0xff, 0xff, 0x15, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0xd, 0x38, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0xff, 0xff, 0xff, 0x16, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x42, 0x5c, 0x61, 0xff, 0x17, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0xe, 0x38, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x77, 0x77, 0x77, 0xff, 0x18, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x5e, 0x73, 0x79, 0xff, 0x19, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0xf, 0x38, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1a, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x4, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x10, 0x38, 0x1b, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x4, 0x0, 0x2, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0xbb, 0x0, 0x0, 0x1a, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
    {0xf0, 0x0, 0x11, 0x12, 0x2, 0x32, 0xbb, 0x0, 0x0, 0x1c, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
};

static const uint8_t kMcduBrightnessCmd[64] = {
    0x02, 0x32, 0xbb, 0x00, 0x00, 0x03, 0x49, 0x00,
    0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// -----------------------------
// Helpers
// -----------------------------
static void set_coordinator_mac(const uint8_t* mac) {
    if (g_have_coordinator) {
        // Sticky MAC: Don't change coordinator once locked
        return;
    }
    memcpy(g_coordinator_mac, mac, 6);
    g_have_coordinator = true;
    addEspNowPeer(g_coordinator_mac);
    
    char mac_str[18];
    macToString(g_coordinator_mac, mac_str);
    LOG_SERIAL.printf("Locked to Coordinator: %s\n", mac_str);
}

static void send_message_to_coord(const uint8_t* data, size_t len) {
    if (!g_have_coordinator) {
        return;
    }
    esp_now_send(g_coordinator_mac, data, len);
}

static void send_discovery_response(const uint8_t* mac) {
    DiscoveryResponse rsp = {};
    rsp.node_type = NODE_TYPE_MIXED;
    rsp.capabilities = CAP_HID_INPUT | CAP_HID_OUTPUT | CAP_DISPLAY | CAP_BIDIRECTIONAL;
    rsp.device_count = (g_stick_connected ? 1 : 0) + (g_mcdu_connected ? 1 : 0);
    memcpy(rsp.mac_address, mac, sizeof(rsp.mac_address));
    strncpy(rsp.firmware_version, FW_VERSION, FIRMWARE_VERSION_SIZE - 1);
    strncpy(rsp.node_name, NODE_NAME, MAX_NODE_NAME_SIZE - 1);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t len = buildMessage(buffer, MSG_DISCOVERY_RSP, g_node_id,
                              NODE_COORDINATOR, &rsp, sizeof(rsp));
    esp_now_send(mac, buffer, len);
}

static void send_register_ack(const uint8_t* mac) {
    uint8_t buffer[32];
    size_t len = buildMessage(buffer, MSG_REGISTER_ACK, g_node_id,
                              NODE_COORDINATOR, nullptr, 0);
    esp_now_send(mac, buffer, len);
}

static void send_heartbeat_ack() {
    HeartbeatPayload payload = {millis(), 0};
    uint8_t buffer[32];
    size_t len = buildMessage(buffer, MSG_HEARTBEAT_ACK, g_node_id,
                              NODE_COORDINATOR, &payload, sizeof(payload));
    send_message_to_coord(buffer, len);
}

static void send_heartbeat() {
    HeartbeatPayload payload = {millis(), 0};
    uint8_t buffer[32];
    size_t len = buildMessage(buffer, MSG_HEARTBEAT, g_node_id,
                              NODE_COORDINATOR, &payload, sizeof(payload));
    send_message_to_coord(buffer, len);
}

static void send_hid_input(uint8_t device_id, const uint8_t* report, size_t len) {
    if (!g_have_coordinator || len == 0) {
        return;
    }
    if (len > MAX_HID_REPORT_SIZE) {
        len = MAX_HID_REPORT_SIZE;
    }

    HIDInputPayload payload = {};
    payload.device_id = device_id;
    payload.report_id = report[0];
    payload.report_length = len;
    memcpy(payload.report_data, report, len);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t msg_len = buildMessage(buffer, MSG_HID_INPUT, g_node_id,
                                  NODE_COORDINATOR, &payload, 3 + len);
    send_message_to_coord(buffer, msg_len);
}

static void send_mcdu_button(uint8_t button_index, uint8_t state) {
    if (!g_have_coordinator) {
        return;
    }
    MCDUInputPayload payload = {};
    payload.button_index = button_index;
    payload.button_state = state;

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t msg_len = buildMessage(buffer, MSG_MCDU_INPUT, g_node_id,
                                  NODE_COORDINATOR, &payload, sizeof(payload));
    send_message_to_coord(buffer, msg_len);
}

// -----------------------------
// MCDU display helpers
// -----------------------------
#define COLOR_BLACK   0x0000
#define COLOR_AMBER   0x0021
#define COLOR_WHITE   0x0042
#define COLOR_CYAN    0x0063
#define COLOR_GREEN   0x0084
#define COLOR_MAGENTA 0x00A5
#define COLOR_RED     0x00C6
#define COLOR_YELLOW  0x00E7

static uint16_t mcdu_color_from_index(uint8_t idx) {
    switch (idx & 0x07) {
        case 0: return COLOR_BLACK;
        case 1: return COLOR_AMBER;
        case 2: return COLOR_WHITE;
        case 3: return COLOR_CYAN;
        case 4: return COLOR_GREEN;
        case 5: return COLOR_MAGENTA;
        case 6: return COLOR_RED;
        case 7: return COLOR_YELLOW;
        default: return COLOR_WHITE;
    }
}

static void mcdu_clear_buffer() {
    for (size_t i = 0; i + 2 < MCDU_DISPLAY_SIZE; i += 3) {
        g_mcdu_display[i] = COLOR_WHITE & 0xFF;
        g_mcdu_display[i + 1] = (COLOR_WHITE >> 8) & 0xFF;
        g_mcdu_display[i + 2] = 0x20;
    }
}

static void mcdu_draw_char(uint8_t row, uint8_t col, char c, uint16_t color) {
    if (row >= MCDU_ROWS || col >= MCDU_COLS) {
        return;
    }
    size_t offset = (row * MCDU_COLS * MCDU_BYTES_PER_CHAR) + (col * MCDU_BYTES_PER_CHAR);
    if (offset + 2 >= MCDU_DISPLAY_SIZE) {
        return;
    }
    g_mcdu_display[offset] = color & 0xFF;
    g_mcdu_display[offset + 1] = (color >> 8) & 0xFF;
    g_mcdu_display[offset + 2] = static_cast<uint8_t>(c);
}

static void mcdu_schedule_flush() {
    g_mcdu_flush_active = true;
    g_mcdu_flush_offset = 0;
}

static void mcdu_queue_report(const uint8_t* data, size_t len) {
    if (len == 0 || len > MCDU_PACKET_SIZE) {
        return;
    }
    memcpy(g_mcdu_pending_report, data, len);
    g_mcdu_pending_len = len;
}

static void mcdu_handle_display(const MCDUDisplayPayload* payload) {
    if (!payload) {
        return;
    }

    switch (payload->command_type) {
        case 0x01: {
            uint8_t row = payload->row;
            uint8_t col = payload->column;
            uint8_t len = payload->length;
            if (len > sizeof(payload->text)) {
                len = sizeof(payload->text);
            }
            uint16_t color = mcdu_color_from_index(payload->color);
            for (uint8_t i = 0; i < len && (col + i) < MCDU_COLS; i++) {
                mcdu_draw_char(row, col + i, static_cast<char>(payload->text[i]), color);
            }
            mcdu_schedule_flush();
            break;
        }
        case 0x03:
            g_mcdu_brightness_pending = true;
            break;
        default:
            break;
    }
}

static bool find_hid_int_in_ep(usb_device_handle_t dev, uint8_t* out_ep, uint16_t* out_mps) {
    struct UsbDescHeader {
        uint8_t bLength;
        uint8_t bDescriptorType;
    };

    const usb_config_desc_t *cfg = nullptr;
    esp_err_t err = usb_host_get_active_config_descriptor(dev, &cfg);
    if (err != ESP_OK || !cfg) return false;

    const uint8_t *p = reinterpret_cast<const uint8_t *>(cfg);
    const int total = cfg->wTotalLength;

    int cur_if = -1;
    int cur_alt = 0;
    uint8_t cur_class = 0;

    for (int off = 0; off + 2 <= total; ) {
        const UsbDescHeader *h = reinterpret_cast<const UsbDescHeader *>(p + off);
        if (h->bLength == 0) break;

        if (h->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *ifd = reinterpret_cast<const usb_intf_desc_t *>(p + off);
            cur_if = ifd->bInterfaceNumber;
            cur_alt = ifd->bAlternateSetting;
            cur_class = ifd->bInterfaceClass;
        } else if (h->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = reinterpret_cast<const usb_ep_desc_t *>(p + off);
            bool is_in = (ep->bEndpointAddress & 0x80) != 0;
            bool is_int = (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT;

            if (cur_if >= 0 && cur_alt == 0 && cur_class == USB_CLASS_HID && is_in && is_int) {
                *out_ep = ep->bEndpointAddress;
                *out_mps = ep->wMaxPacketSize;
                return true;
            }
        }
        off += h->bLength;
    }

    return false;
}

static bool find_hid_in_out_ep(usb_device_handle_t dev,
                               int* out_ifc_in, int* out_alt_in,
                               int* out_ifc_out, int* out_alt_out,
                               uint8_t* out_in_ep, uint8_t* out_out_ep,
                               uint16_t* out_in_mps, uint16_t* out_out_mps) {
    struct UsbDescHeader {
        uint8_t bLength;
        uint8_t bDescriptorType;
    };

    const usb_config_desc_t *cfg = nullptr;
    esp_err_t err = usb_host_get_active_config_descriptor(dev, &cfg);
    if (err != ESP_OK || !cfg) return false;

    const uint8_t *p = reinterpret_cast<const uint8_t *>(cfg);
    const int total = cfg->wTotalLength;

    int cur_if = -1;
    int cur_alt = 0;
    uint8_t cur_class = 0;
    uint8_t if_in_ep = 0;
    uint8_t if_out_ep = 0;
    uint16_t if_in_mps = HID_MAX_PKT;
    uint16_t if_out_mps = HID_MAX_PKT;

    for (int off = 0; off + 2 <= total; ) {
        const UsbDescHeader *h = reinterpret_cast<const UsbDescHeader *>(p + off);
        if (h->bLength == 0) break;

        if (h->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *ifd = reinterpret_cast<const usb_intf_desc_t *>(p + off);
            cur_if = ifd->bInterfaceNumber;
            cur_alt = ifd->bAlternateSetting;
            cur_class = ifd->bInterfaceClass;
            if_in_ep = 0;
            if_out_ep = 0;
            if_in_mps = HID_MAX_PKT;
            if_out_mps = HID_MAX_PKT;
        } else if (h->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = reinterpret_cast<const usb_ep_desc_t *>(p + off);
            bool is_in = (ep->bEndpointAddress & 0x80) != 0;
            bool is_out = (ep->bEndpointAddress & 0x80) == 0;
            bool is_int = (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT;

            if (cur_if >= 0 && cur_class == USB_CLASS_HID && is_int) {
                if (is_in) {
                    if_in_ep = ep->bEndpointAddress;
                    if_in_mps = ep->wMaxPacketSize;
                    *out_in_ep = if_in_ep;
                    *out_in_mps = if_in_mps;
                    *out_ifc_in = cur_if;
                    *out_alt_in = cur_alt;
                } else if (is_out) {
                    if_out_ep = ep->bEndpointAddress;
                    if_out_mps = ep->wMaxPacketSize;
                    *out_out_ep = if_out_ep;
                    *out_out_mps = if_out_mps;
                    *out_ifc_out = cur_if;
                    *out_alt_out = cur_alt;
                }
                if (*out_in_ep != 0 && *out_out_ep != 0) {
                    return true;
                }
            }
        }
        off += h->bLength;
    }

    return false;
}

// -----------------------------
// USB Callbacks
// -----------------------------
static void stick_in_cb(usb_transfer_t *t) {
    if (!g_stick_connected || !t) {
        return;
    }

    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
        // Optimize: Convert to packed struct and apply rate limiting
        send_sidestick_packed(t->data_buffer, t->actual_num_bytes);
    }

    usb_host_transfer_submit(t);
}

static void send_sidestick_packed(const uint8_t* data, size_t len) {
    if (len < 8) return; // Minimum size check (TCA stick is usually larger)

    // TCA Sidestick Mapping (Approximate based on analysis)
    // Offset 1: X (16-bit, effective 14-bit)
    // Offset 3: Y (16-bit, effective 14-bit)
    // Offset 5: Twist (8-bit)
    // Offset 6: Slider (8-bit) - Verify this
    // Offset 7+ bits: Buttons/Hat
    
    // Extract Raw (Little Endian)
    uint16_t raw_x = data[1] | (data[2] << 8);
    uint16_t raw_y = data[3] | (data[4] << 8);
    uint8_t raw_z = data[5];
    uint8_t raw_slider = data[6]; // Assumption
    
    // Buttons (complex mapping, taking bytes 7-9 roughly)
    uint32_t raw_buttons = data[7] | (data[8] << 8) | (data[9] << 16); // Mask later
    
    // Quantize to target resolution (12-bit for X/Y, 10-bit for Z/Slider)
    // X/Y are 14-bit (0-16383), target 12-bit (0-4095) -> Shift Right 2
    uint16_t q_x = raw_x >> 2;
    uint16_t q_y = raw_y >> 2;
    
    // Z/Slider are 8-bit (0-255), target 10-bit (0-1023) -> Shift Left 2
    uint16_t q_z = raw_z << 2; 
    uint16_t q_slider = raw_slider << 2;
    
    // Extract Hat (Bits 0-3 of Byte 7 usually)
    uint8_t hat = raw_buttons & 0x0F;
    uint32_t buttons = (raw_buttons >> 4) & 0xFFFF; // Shift out hat

    // Change Detection
    bool changed = false;
    if (abs((int)q_x - (int)g_last_sent_stick.axis_x) > STICK_AXIS_DEADBAND) changed = true;
    if (abs((int)q_y - (int)g_last_sent_stick.axis_y) > STICK_AXIS_DEADBAND) changed = true;
    if (abs((int)q_z - (int)g_last_sent_stick.axis_z) > STICK_AXIS_DEADBAND) changed = true;
    if (abs((int)q_slider - (int)g_last_sent_stick.axis_slider) > STICK_AXIS_DEADBAND) changed = true;
    if (buttons != g_last_sent_stick.buttons) changed = true;
    if (hat != g_last_sent_stick.hat_switch) changed = true;

    uint32_t now = millis();
    if (changed && (now - g_last_stick_send_ms >= STICK_RATE_LIMIT_MS)) {
        PackedSidestickPayload payload;
        payload.axis_x = q_x;
        payload.axis_y = q_y;
        payload.axis_z = q_z;
        payload.axis_slider = q_slider;
        payload.buttons = buttons;
        payload.hat_switch = hat;

        uint8_t msg_buffer[sizeof(MessageHeader) + sizeof(PackedSidestickPayload) + 1];
        size_t msg_len = buildMessage(msg_buffer, MSG_HID_PACKED_SIDESTICK, g_node_id, 
                                    NODE_COORDINATOR, &payload, sizeof(payload));
        
        esp_now_send(g_coordinator_mac, msg_buffer, msg_len);
        
        g_last_sent_stick = payload;
        g_last_stick_send_ms = now;
    }
}

static void mcdu_in_cb(usb_transfer_t *t) {
    if (!g_mcdu_connected || !t) {
        return;
    }

    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
        McduRxPacket pkt = {};
        pkt.len = static_cast<uint8_t>((t->actual_num_bytes > MCDU_PACKET_SIZE)
                                       ? MCDU_PACKET_SIZE
                                       : t->actual_num_bytes);
        memcpy(pkt.data, t->data_buffer, pkt.len);
        xQueueSendFromISR(g_mcdu_rx_queue, &pkt, nullptr);
    }

    usb_host_transfer_submit(t);
}

static void mcdu_out_cb(usb_transfer_t *t) {
    (void)t;
    g_mcdu_out_busy = false;
}

static void usb_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    (void)arg;
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        usb_device_handle_t dev;
        if (usb_host_device_open(g_client, event_msg->new_dev.address, &dev) != ESP_OK) {
            return;
        }

        const usb_device_desc_t *desc;
        if (usb_host_get_device_descriptor(dev, &desc) != ESP_OK || !desc) {
            usb_host_device_close(g_client, dev);
            return;
        }

        if (desc->idVendor == THRUSTMASTER_VID && desc->idProduct == THRUSTMASTER_PID_STICK) {
            g_stick_dev = dev;
            g_stick_connected = true;

            uint8_t ep_in = 0;
            uint16_t mps = HID_MAX_PKT;
            if (!find_hid_int_in_ep(dev, &ep_in, &mps)) {
                ep_in = 0x81;
                mps = HID_MAX_PKT;
            }

            usb_host_interface_claim(g_client, dev, 0, 0);
            g_stick_in_xfer->device_handle = dev;
            g_stick_in_xfer->bEndpointAddress = ep_in;
            g_stick_in_xfer->num_bytes = mps;
            g_stick_in_xfer->callback = stick_in_cb;
            usb_host_transfer_submit(g_stick_in_xfer);

            LOG_SERIAL.println("Sidestick connected.");
            return;
        }

        if (desc->idVendor == MCDU_VID &&
            (desc->idProduct == MCDU_PID_PRIMARY || desc->idProduct == MCDU_PID_ALT)) {
            g_mcdu_dev = dev;
            g_mcdu_connected = false;
            g_mcdu_ready = false;
            g_mcdu_init_done = false;
            g_mcdu_brightness_pending = true;
            g_mcdu_flush_active = false;
            g_mcdu_pending_len = 0;

            g_mcdu_ep_in = 0;
            g_mcdu_ep_out = 0;
            g_mcdu_mps_in = HID_MAX_PKT;
            g_mcdu_mps_out = HID_MAX_PKT;
            g_mcdu_ifc_in = 0;
            g_mcdu_ifc_out = 0;
            g_mcdu_alt_in = 0;
            g_mcdu_alt_out = 0;

            find_hid_in_out_ep(dev, &g_mcdu_ifc_in, &g_mcdu_alt_in,
                               &g_mcdu_ifc_out, &g_mcdu_alt_out,
                               &g_mcdu_ep_in, &g_mcdu_ep_out,
                               &g_mcdu_mps_in, &g_mcdu_mps_out);
            if (g_mcdu_ep_in == 0) g_mcdu_ep_in = 0x81;
            if (g_mcdu_ep_out == 0) g_mcdu_ep_out = 0x02;

            esp_err_t claim_err = ESP_OK;
            if (g_mcdu_ifc_in == g_mcdu_ifc_out) {
                claim_err = usb_host_interface_claim(g_client, dev, g_mcdu_ifc_in, g_mcdu_alt_in);
            } else {
                claim_err = usb_host_interface_claim(g_client, dev, g_mcdu_ifc_in, g_mcdu_alt_in);
                esp_err_t claim_out_err = usb_host_interface_claim(g_client, dev, g_mcdu_ifc_out, g_mcdu_alt_out);
                if (claim_err == ESP_OK && claim_out_err != ESP_OK) {
                    claim_err = claim_out_err;
                }
            }

            if (claim_err == ESP_OK) {
                g_mcdu_in_xfer->device_handle = dev;
                g_mcdu_in_xfer->bEndpointAddress = g_mcdu_ep_in;
                g_mcdu_in_xfer->num_bytes = g_mcdu_mps_in;
                g_mcdu_in_xfer->callback = mcdu_in_cb;
                esp_err_t submit_err = usb_host_transfer_submit(g_mcdu_in_xfer);
                if (submit_err == ESP_OK) {
                    g_mcdu_connected = true;
                    g_mcdu_ready = true;
                }
            }

            mcdu_clear_buffer();
            mcdu_schedule_flush();
            LOG_SERIAL.println("MCDU connected.");
            return;
        }

        usb_host_device_close(g_client, dev);
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        if (g_stick_dev == event_msg->dev_gone.dev_hdl) {
            usb_host_device_close(g_client, g_stick_dev);
            g_stick_connected = false;
            g_stick_dev = nullptr;
            LOG_SERIAL.println("Sidestick disconnected.");
        }
        if (g_mcdu_dev == event_msg->dev_gone.dev_hdl) {
            usb_host_device_close(g_client, g_mcdu_dev);
            g_mcdu_connected = false;
            g_mcdu_ready = false;
            g_mcdu_dev = nullptr;
            g_mcdu_out_busy = false;
            g_mcdu_pending_len = 0;
            g_mcdu_flush_active = false;
            g_mcdu_init_done = false;
            LOG_SERIAL.println("MCDU disconnected.");
        }
    }
}

// -----------------------------
// MCDU transmit scheduler
// -----------------------------
static void mcdu_send_packet(const uint8_t* data, size_t len) {
    if (!g_mcdu_connected || !g_mcdu_ready || !g_mcdu_out_xfer || g_mcdu_out_busy) {
        return;
    }
    size_t send_len = (len > MCDU_PACKET_SIZE) ? MCDU_PACKET_SIZE : len;
    memcpy(g_mcdu_out_xfer->data_buffer, data, send_len);
    if (send_len < MCDU_PACKET_SIZE) {
        memset(g_mcdu_out_xfer->data_buffer + send_len, 0, MCDU_PACKET_SIZE - send_len);
    }
    g_mcdu_out_xfer->device_handle = g_mcdu_dev;
    g_mcdu_out_xfer->bEndpointAddress = g_mcdu_ep_out;
    g_mcdu_out_xfer->num_bytes = MCDU_PACKET_SIZE;
    g_mcdu_out_xfer->callback = mcdu_out_cb;

    g_mcdu_out_busy = true;
    if (usb_host_transfer_submit(g_mcdu_out_xfer) != ESP_OK) {
        g_mcdu_out_busy = false;
    }
}

static void mcdu_task(void *arg) {
    (void)arg;
    size_t init_index = 0;
    bool brightness_sent = false;

    while (true) {
        McduRxPacket rx = {};
        while (xQueueReceive(g_mcdu_rx_queue, &rx, 0) == pdTRUE) {
            if (rx.len >= 2) {
                uint8_t report_id = rx.data[0];
                if (report_id != 0x01) {
                    continue;
                }

                size_t available = rx.len - 1;
                size_t count = (available < MCDU_BUTTON_BYTES) ? available : MCDU_BUTTON_BYTES;
                for (size_t i = 0; i < count; i++) {
                    uint8_t current_byte = rx.data[i + 1];
                    uint8_t prev_byte = g_mcdu_last_buttons[i];
                    if (current_byte != prev_byte) {
                        for (int bit = 0; bit < 8; bit++) {
                            bool was_pressed = (prev_byte >> bit) & 1;
                            bool is_pressed = (current_byte >> bit) & 1;
                            if (was_pressed != is_pressed) {
                                uint8_t button_id = static_cast<uint8_t>((i * 8) + bit);
                                send_mcdu_button(button_id, is_pressed ? 1 : 0);
                            }
                        }
                        g_mcdu_last_buttons[i] = current_byte;
                    }
                }
            }
        }

        if (g_mcdu_connected && g_mcdu_ready) {
            uint32_t now = millis();
            if (!g_mcdu_out_busy && (now - g_mcdu_last_send_ms) >= PACKET_PACING_MS) {
                if (!g_mcdu_init_done) {
                    if (init_index < (sizeof(kMcduInitSeq) / sizeof(kMcduInitSeq[0]))) {
                        mcdu_send_packet(kMcduInitSeq[init_index], MCDU_PACKET_SIZE);
                        init_index++;
                        g_mcdu_last_send_ms = now;
                    } else if (!brightness_sent) {
                        mcdu_send_packet(kMcduBrightnessCmd, sizeof(kMcduBrightnessCmd));
                        brightness_sent = true;
                        g_mcdu_last_send_ms = now;
                    } else {
                        g_mcdu_init_done = true;
                        g_mcdu_brightness_pending = false;
                        mcdu_schedule_flush();
                    }
                } else if (g_mcdu_pending_len > 0) {
                    mcdu_send_packet(g_mcdu_pending_report, g_mcdu_pending_len);
                    g_mcdu_pending_len = 0;
                    g_mcdu_last_send_ms = now;
                } else if (g_mcdu_brightness_pending) {
                    mcdu_send_packet(kMcduBrightnessCmd, sizeof(kMcduBrightnessCmd));
                    g_mcdu_brightness_pending = false;
                    g_mcdu_last_send_ms = now;
                } else if (g_mcdu_flush_active) {
                    uint8_t packet[MCDU_PACKET_SIZE];
                    size_t remaining = MCDU_DISPLAY_SIZE - g_mcdu_flush_offset;
                    size_t chunk = (remaining > MCDU_PAYLOAD_SIZE) ? MCDU_PAYLOAD_SIZE : remaining;
                    packet[0] = 0xF2;
                    memcpy(&packet[1], &g_mcdu_display[g_mcdu_flush_offset], chunk);
                    mcdu_send_packet(packet, chunk + 1);

                    g_mcdu_flush_offset += chunk;
                    if (g_mcdu_flush_offset >= MCDU_DISPLAY_SIZE) {
                        g_mcdu_flush_active = false;
                    }
                    g_mcdu_last_send_ms = now;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -----------------------------
// ESP-NOW Callbacks/Task
// -----------------------------
static void espnow_recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len) {
    if (!info || !data || len <= 0 || len > MAX_ESPNOW_PAYLOAD) {
        return;
    }
    const uint8_t* mac_addr = info->src_addr;

    RxPacket pkt;
    memcpy(pkt.mac, mac_addr, 6);
    memcpy(pkt.data, data, len);
    pkt.len = len;

    xQueueSendFromISR(g_rx_queue, &pkt, nullptr);
}

static void espnow_task(void *arg) {
    (void)arg;
    RxPacket pkt;
    uint32_t last_hb = 0;

    while (true) {
        while (xQueueReceive(g_rx_queue, &pkt, 0) == pdTRUE) {
            set_coordinator_mac(pkt.mac);
            if (!validateMessage(pkt.data, pkt.len)) {
                continue;
            }

            const MessageHeader* hdr = reinterpret_cast<const MessageHeader*>(pkt.data);
            if (hdr->dst_node != g_node_id && hdr->dst_node != NODE_BROADCAST &&
                hdr->msg_type != MSG_REGISTER_REQ) {
                continue;
            }

            const uint8_t* payload = pkt.data + sizeof(MessageHeader);
            size_t payload_len = pkt.len - sizeof(MessageHeader) - 1;

            switch (hdr->msg_type) {
                case MSG_DISCOVERY_REQ:
                    send_discovery_response(pkt.mac);
                    break;

                case MSG_REGISTER_REQ:
                    if (payload_len >= sizeof(RegisterRequest)) {
                        const RegisterRequest* req = reinterpret_cast<const RegisterRequest*>(payload);
                        g_node_id = req->assigned_node_id;
                        send_register_ack(pkt.mac);
                    }
                    break;

                case MSG_HEARTBEAT:
                    send_heartbeat_ack();
                    break;

                case MSG_MCDU_DISPLAY:
                    if (payload_len >= sizeof(MCDUDisplayPayload)) {
                        const MCDUDisplayPayload* mcdu = reinterpret_cast<const MCDUDisplayPayload*>(payload);
                        mcdu_handle_display(mcdu);
                    }
                    break;

                case MSG_HID_OUTPUT:
                    if (payload_len >= 3) {
                        const HIDOutputPayload* out = reinterpret_cast<const HIDOutputPayload*>(payload);
                        if (out->device_id == DEV_WINWING_MCDU && out->report_length > 0) {
                            uint8_t packet[MCDU_PACKET_SIZE] = {0};
                            size_t len = out->report_length;
                            if (len > MCDU_PAYLOAD_SIZE) {
                                len = MCDU_PAYLOAD_SIZE;
                            }
                            packet[0] = out->report_id;
                            memcpy(&packet[1], out->report_data, len);
                            mcdu_queue_report(packet, len + 1);
                        }
                    }
                    break;

                case MSG_TEST_REQ: {
                    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
                    size_t msg_len = buildMessage(buffer, MSG_TEST_RSP, g_node_id,
                                                  NODE_COORDINATOR, payload, payload_len);
                    send_message_to_coord(buffer, msg_len);
                    break;
                }

                case MSG_RESET:
                    ESP.restart();
                    break;

                default:
                    break;
            }
        }

        if (g_have_coordinator && millis() - last_hb >= HEARTBEAT_INTERVAL_MS) {
            last_hb = millis();
            send_heartbeat();
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// -----------------------------
// USB Task
// -----------------------------
static void usb_task(void *arg) {
    (void)arg;
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(1, &flags);
        if (g_client) {
            usb_host_client_handle_events(g_client, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -----------------------------
// Setup/Loop
// -----------------------------
void setup() {
    LOG_SERIAL_BEGIN();
    LOG_SERIAL.println("\n=== Node B: Sidestick + MCDU ===");

    setupHardware();
    enableUsbHostPower(true);
    delay(500);

    // Log reset reason to help diagnose crash loops
    esp_reset_reason_t reason = esp_reset_reason();
    LOG_SERIAL.printf("Boot Reason: %d\n", reason);

    g_rx_queue = xQueueCreate(16, sizeof(RxPacket));

    if (!initEspNowOptimized()) {
        LOG_SERIAL.println("ESP-NOW init failed.");
        while (true) {
            delay(1000);
        }
    }
    esp_now_register_recv_cb(espnow_recv_cb);

    const usb_host_config_t h_cfg = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    usb_host_install(&h_cfg);
    static usb_host_client_config_t c_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 10,
        .async = { .client_event_callback = usb_event_cb, .callback_arg = nullptr }
    };
    usb_host_client_register(&c_cfg, &g_client);

    usb_host_transfer_alloc(HID_MAX_PKT, 0, &g_stick_in_xfer);
    usb_host_transfer_alloc(MCDU_PACKET_SIZE, 0, &g_mcdu_in_xfer);
    usb_host_transfer_alloc(MCDU_PACKET_SIZE, 0, &g_mcdu_out_xfer);

    g_mcdu_rx_queue = xQueueCreate(8, sizeof(McduRxPacket));
    mcdu_clear_buffer();

    xTaskCreatePinnedToCore(usb_task, "usb_task", 4096, nullptr, 20, nullptr, 0);
    xTaskCreatePinnedToCore(espnow_task, "espnow_task", 4096, nullptr, 24, nullptr, 1);
    xTaskCreatePinnedToCore(mcdu_task, "mcdu_task", 4096, nullptr, 22, nullptr, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

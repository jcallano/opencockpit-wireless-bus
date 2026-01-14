/**
 * OpenCockpit Wireless Avionics Bus
 * Node C - Quadrant + MiniFCU Peripheral
 *
 * Responsibilities:
 * - USB Host: Thrustmaster TCA Quadrant HID input
 * - USB Host: CH340 serial (MiniFCU/MiniEFIS)
 * - ESP-NOW: HID input + serial passthrough
 */

#include <Arduino.h>
#include "usb/usb_host.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define ESPNOW_LOG_SERIAL Serial0

#include "../../common/espnow_config.h"
#include "../../common/protocol.h"

// -----------------------------
// Board/Console Configuration (WeAct ESP32-S3 DevKit Rev A)
// -----------------------------
#define UART0_TX_PIN        43
#define UART0_RX_PIN        44

#define LOG_SERIAL          Serial0
#define LOG_SERIAL_BEGIN()  Serial0.begin(115200, SERIAL_8N1, UART0_RX_PIN, UART0_TX_PIN)

// -----------------------------
// Configuration
// -----------------------------
#define NODE_NAME           "NODE_C_FCU"
#define FW_VERSION          "1.0.0"

#define THRUSTMASTER_VID    0x044F
#define THRUSTMASTER_PID_Q  0x0407

#define CH340_VID           0x1A86
#define CH340_PID           0x7523

#define CH340_EP_IN         0x82
#define CH340_EP_OUT        0x02
#define HID_EP_IN           0x81

#define CH340_MAX_PKT       64
#define HID_MAX_PKT         64

#define FLUSH_TIMEOUT_MS    2
#define OUT_RING_SIZE       256

#define MINIFCU_AUTO_INIT   1
#define DEBUG_LOGS          1
#define USB_ENUM_ONLY       0

// Coordinator MAC (set after first discovery/register message)
static uint8_t g_coordinator_mac[6] = {0};
static bool g_have_coordinator = false;

// Node ID assigned by coordinator (default to Node C)
static uint8_t g_node_id = NODE_C_QUADRANT;

// -----------------------------
// USB Host Globals
// -----------------------------
static usb_host_client_handle_t g_client = nullptr;
static usb_device_handle_t g_quadrant_dev = nullptr;
static usb_device_handle_t g_ch340_dev = nullptr;
static usb_transfer_t *g_hid_in_xfer = nullptr;
static usb_transfer_t *g_ch340_ctrl = nullptr;
static usb_transfer_t *g_ch340_in = nullptr;
static usb_transfer_t *g_ch340_out = nullptr;

static bool g_quadrant_connected = false;
static bool g_ch340_connected = false;
static bool g_ch340_out_busy = false;

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
// Serial Buffers
// -----------------------------
static portMUX_TYPE g_serial_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_serial_tx_buffer[MAX_SERIAL_DATA_SIZE];
static size_t g_serial_tx_len = 0;
static uint32_t g_serial_last_byte_ms = 0;

static uint8_t g_serial_out_ring[OUT_RING_SIZE];
static size_t g_serial_out_head = 0;
static size_t g_serial_out_tail = 0;

// -----------------------------
// HID State
// -----------------------------
static uint8_t g_last_quadrant_report[HID_MAX_PKT];
static size_t g_last_quadrant_len = 0;
static uint32_t g_last_quadrant_log_ms = 0;
static uint32_t g_last_ch340_log_ms = 0;
static uint32_t g_last_ch340_tx_log_ms = 0;
static char g_minifcu_line[128];
static size_t g_minifcu_line_len = 0;
static uint8_t g_minifcu_init_state = 0;
static uint32_t g_minifcu_init_ms = 0;

// -----------------------------
// USB Descriptor Logging (ported from enumok.ino)
// -----------------------------
static const char* usb_class_str(uint8_t c) {
    switch (c) {
        case 0x00: return "Interface-defined";
        case 0x01: return "Audio";
        case 0x02: return "CDC";
        case 0x03: return "HID";
        case 0x08: return "Mass Storage";
        case 0x09: return "Hub";
        case 0xE0: return "Wireless";
        case 0xFF: return "Vendor Specific";
        default: return "Other";
    }
}

static void log_usb_device_desc(usb_device_handle_t dev) {
#if DEBUG_LOGS
    const usb_device_desc_t *desc = nullptr;
    if (usb_host_get_device_descriptor(dev, &desc) != ESP_OK || !desc) {
        LOG_SERIAL.println("  Error getting descriptor");
        return;
    }

    LOG_SERIAL.println("\n+----------------------------------+");
    LOG_SERIAL.println("|     DISPOSITIVO DETECTADO        |");
    LOG_SERIAL.println("+----------------------------------+");
    LOG_SERIAL.printf("  VID:        0x%04X\n", desc->idVendor);
    LOG_SERIAL.printf("  PID:        0x%04X\n", desc->idProduct);
    LOG_SERIAL.printf("  Class:      0x%02X (%s)\n", desc->bDeviceClass, usb_class_str(desc->bDeviceClass));
    LOG_SERIAL.printf("  SubClass:   0x%02X\n", desc->bDeviceSubClass);
    LOG_SERIAL.printf("  Protocol:   0x%02X\n", desc->bDeviceProtocol);
    LOG_SERIAL.printf("  MaxPacket:  %u bytes\n", desc->bMaxPacketSize0);

    usb_device_info_t info;
    if (usb_host_device_info(dev, &info) == ESP_OK) {
        const char* speed =
            info.speed == USB_SPEED_LOW ? "Low (1.5 Mbps)" :
            info.speed == USB_SPEED_FULL ? "Full (12 Mbps)" : "High (480 Mbps)";
#ifndef USB_SPEED_HIGH
        if (info.speed != USB_SPEED_LOW && info.speed != USB_SPEED_FULL) {
            speed = "Unknown";
        }
#endif
        LOG_SERIAL.printf("  Speed:      %s\n", speed);
    }

    const usb_config_desc_t *cfg = nullptr;
    if (usb_host_get_active_config_descriptor(dev, &cfg) == ESP_OK && cfg) {
        LOG_SERIAL.printf("  Interfaces: %u\n", cfg->bNumInterfaces);
        LOG_SERIAL.printf("  Max Power:  %u mA\n", cfg->bMaxPower * 2);

        const uint8_t *p = reinterpret_cast<const uint8_t*>(cfg);
        int offset = USB_CONFIG_DESC_SIZE;
        while (offset < cfg->wTotalLength) {
            uint8_t len = p[offset];
            uint8_t type = p[offset + 1];
            if (len == 0) break;

            if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
                const usb_intf_desc_t *intf = reinterpret_cast<const usb_intf_desc_t*>(p + offset);
                LOG_SERIAL.printf("\n  [Interface %u] %s\n",
                                  intf->bInterfaceNumber, usb_class_str(intf->bInterfaceClass));
            } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
                const usb_ep_desc_t *ep = reinterpret_cast<const usb_ep_desc_t*>(p + offset);
                LOG_SERIAL.printf("    EP 0x%02X %s %u bytes\n",
                                  ep->bEndpointAddress,
                                  (ep->bEndpointAddress & 0x80) ? "IN " : "OUT",
                                  ep->wMaxPacketSize & 0x7FF);
            }
            offset += len;
        }
    }
    LOG_SERIAL.println("+----------------------------------+\n");
#endif
}

// -----------------------------
// Helpers
// -----------------------------
static void set_coordinator_mac(const uint8_t* mac) {
    if (!g_have_coordinator || memcmp(g_coordinator_mac, mac, 6) != 0) {
        memcpy(g_coordinator_mac, mac, 6);
        g_have_coordinator = true;
        addEspNowPeer(g_coordinator_mac);
    }
}

static uint8_t get_device_count() {
    uint8_t count = 0;
    if (g_quadrant_connected) count++;
    if (g_ch340_connected) count++;
    return count;
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
    rsp.capabilities = CAP_HID_INPUT | CAP_SERIAL | CAP_BIDIRECTIONAL;
    rsp.device_count = get_device_count();
    getDeviceMac(rsp.mac_address);
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
    if (mac) {
        esp_err_t err = esp_now_send(mac, buffer, len);
        if (DEBUG_LOGS) {
            char mac_str[18];
            macToString(mac, mac_str);
            LOG_SERIAL.printf("Register ACK -> %s (id=0x%02X) err=%d\n", mac_str, g_node_id, err);
        }
    } else {
        send_message_to_coord(buffer, len);
    }
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

static void send_hid_input(const uint8_t* report, size_t len) {
    if (!g_have_coordinator || len == 0) {
        return;
    }

    if (len > MAX_HID_REPORT_SIZE) {
        len = MAX_HID_REPORT_SIZE;
    }

    HIDInputPayload payload = {};
    payload.device_id = DEV_TCA_QUADRANT;
    payload.report_id = report[0];
    payload.report_length = len;
    memcpy(payload.report_data, report, len);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t msg_len = buildMessage(buffer, MSG_HID_INPUT, g_node_id,
                                  NODE_COORDINATOR, &payload, 3 + len);
    send_message_to_coord(buffer, msg_len);
}

static void flush_serial_to_coord() {
    if (!g_have_coordinator) {
        return;
    }

    size_t len = 0;

    portENTER_CRITICAL(&g_serial_lock);
    len = g_serial_tx_len;
    if (len > 0) {
        if (len > MAX_SERIAL_DATA_SIZE) {
            len = MAX_SERIAL_DATA_SIZE;
        }
        SerialDataPayload payload = {};
        payload.port_id = 0;
        payload.data_length = len;
        memcpy(payload.data, g_serial_tx_buffer, len);

        g_serial_tx_len -= len;
        if (g_serial_tx_len > 0) {
            memmove(g_serial_tx_buffer, g_serial_tx_buffer + len, g_serial_tx_len);
        }

        uint8_t buffer[MAX_ESPNOW_PAYLOAD];
        size_t msg_len = buildMessage(buffer, MSG_SERIAL_DATA, g_node_id,
                                      NODE_COORDINATOR, &payload, 2 + len);
        send_message_to_coord(buffer, msg_len);
    }
    portEXIT_CRITICAL(&g_serial_lock);
}

static void enqueue_serial_out(const uint8_t* data, size_t len) {
    portENTER_CRITICAL(&g_serial_lock);
    for (size_t i = 0; i < len; i++) {
        size_t next = (g_serial_out_head + 1) % OUT_RING_SIZE;
        if (next == g_serial_out_tail) {
            break;
        }
        g_serial_out_ring[g_serial_out_head] = data[i];
        g_serial_out_head = next;
    }
    portEXIT_CRITICAL(&g_serial_lock);
}

static size_t dequeue_serial_out(uint8_t* out, size_t max_len) {
    size_t count = 0;
    portENTER_CRITICAL(&g_serial_lock);
    while (g_serial_out_tail != g_serial_out_head && count < max_len) {
        out[count++] = g_serial_out_ring[g_serial_out_tail];
        g_serial_out_tail = (g_serial_out_tail + 1) % OUT_RING_SIZE;
    }
    portEXIT_CRITICAL(&g_serial_lock);
    return count;
}

static void send_minifcu_init() {
#if MINIFCU_AUTO_INIT
    const char* init_burst =
        "Q400,"
        "K100,"
        "-99,"
        "+10,"
        "n49000,"
        "b100,"
        "[6000,"
        "]-6000,"
        "Z9900,"
        "X-9900,"
        "I,"
        "Y,"
        "W,"
        "O,"
        "{1,"
        "(3248,"
        "}2200,"
        "=1100,"
        "$745,"
        "%0,";
    const uint8_t* data = reinterpret_cast<const uint8_t*>(init_burst);
    size_t len = strlen(init_burst);

    LOG_SERIAL.println("MiniFCU init burst -> CH340");
    for (size_t i = 0; i < len; i += 120) {
        size_t chunk = (len - i > 120) ? 120 : len - i;
        enqueue_serial_out(data + i, chunk);
    }
#endif
}

static void log_quadrant_report(const uint8_t* data, size_t len) {
    if (len < 15) return;
    uint8_t b01 = data[1];
    uint8_t b02 = data[2];
    uint16_t t1 = data[3] | (static_cast<uint16_t>(data[4]) << 8);
    uint16_t t2 = data[5] | (static_cast<uint16_t>(data[6]) << 8);
    uint16_t flaps = data[7] | (static_cast<uint16_t>(data[8]) << 8);
    uint8_t b09 = data[9];
    uint16_t spd = data[10] | (static_cast<uint16_t>(data[11]) << 8);
    uint8_t b12 = data[12];
    uint8_t b13 = data[13];

    if (DEBUG_LOGS) {
        LOG_SERIAL.printf("Quadrant B01:%02X B02:%02X B09:%02X B12:%02X B13:%02X ",
                       b01, b02, b09, b12, b13);
        LOG_SERIAL.printf("T1:%u T2:%u FLP:%u SPD:%u\n", t1, t2, flaps, spd);
    }
}

static void log_minifcu_bytes(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        char c = static_cast<char>(data[i]);
        if (c == ';') {
            g_minifcu_line[g_minifcu_line_len] = '\0';
            if (g_minifcu_line_len > 0) {
                if (DEBUG_LOGS) {
                    LOG_SERIAL.print("MiniFCU MSG: ");
                    LOG_SERIAL.println(g_minifcu_line);
                }
            }
            g_minifcu_line_len = 0;
        } else if (c >= 32 && c <= 126) {
            if (g_minifcu_line_len < sizeof(g_minifcu_line) - 1) {
                g_minifcu_line[g_minifcu_line_len++] = c;
            }
        }
    }
}

static void log_hex_bytes(const char* tag, const uint8_t* data, int len, int max_len) {
    if (!tag || !data || len <= 0) {
        return;
    }
    if (!DEBUG_LOGS) {
        return;
    }
    int dump_len = (len > max_len) ? max_len : len;
    LOG_SERIAL.printf("%s len=%d data=", tag, len);
    for (int i = 0; i < dump_len; i++) {
        LOG_SERIAL.printf("%02X", data[i]);
        if (i + 1 < dump_len) {
            LOG_SERIAL.print(" ");
        }
    }
    if (len > dump_len) {
        LOG_SERIAL.print(" ...");
    }
    LOG_SERIAL.println();
}

static bool find_hid_int_in_ep_active(usb_device_handle_t dev,
                                      int *out_if_num,
                                      uint8_t *out_ep_addr,
                                      uint16_t *out_mps) {
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
                *out_if_num = cur_if;
                *out_ep_addr = ep->bEndpointAddress;
                *out_mps = ep->wMaxPacketSize;
                return true;
            }
        }
        off += h->bLength;
    }

    return false;
}

// -----------------------------
// CH340 Control
// -----------------------------
static void ch340_ctrl(uint8_t req, uint16_t val, uint16_t idx) {
    if (!g_ch340_dev || !g_ch340_ctrl) return;
    g_ch340_ctrl->device_handle = g_ch340_dev;
    g_ch340_ctrl->bEndpointAddress = 0;
    g_ch340_ctrl->callback = [](usb_transfer_t *t){ (void)t; };
    g_ch340_ctrl->num_bytes = 8;

    usb_setup_packet_t *s = reinterpret_cast<usb_setup_packet_t*>(g_ch340_ctrl->data_buffer);
    s->bmRequestType = 0x40;
    s->bRequest = req;
    s->wValue = val;
    s->wIndex = idx;
    s->wLength = 0;

    usb_host_transfer_submit_control(g_client, g_ch340_ctrl);
}

static void init_ch340() {
    ch340_ctrl(0xA4, 0x0101, 0);
    delay(500);
    ch340_ctrl(0x9A, 0x1312, 0xB202);
    ch340_ctrl(0x9A, 0x2727, 0);
    delay(500);
    LOG_SERIAL.println("CH340 initialized.");
}

// -----------------------------
// USB Callbacks
// -----------------------------
static void hid_in_cb(usb_transfer_t *t) {
    if (!g_quadrant_connected || !t) {
        return;
    }

    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
        const uint8_t* data = t->data_buffer;
        size_t len = t->actual_num_bytes;

        bool changed = (len != g_last_quadrant_len) ||
                       (memcmp(g_last_quadrant_report, data, len) != 0);
        if (changed) {
            memcpy(g_last_quadrant_report, data, len);
            g_last_quadrant_len = len;
            uint32_t now = millis();
            if (now - g_last_quadrant_log_ms > 50) {
                g_last_quadrant_log_ms = now;
                log_quadrant_report(data, len);
                log_hex_bytes("Quadrant HID", data, static_cast<int>(len), 16);
            }
            send_hid_input(data, len);
        }
    }

    usb_host_transfer_submit(t);
}

static void ch340_in_cb(usb_transfer_t *t) {
    if (!g_ch340_connected || !t) {
        return;
    }

    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
        portENTER_CRITICAL(&g_serial_lock);
        for (int i = 0; i < t->actual_num_bytes; i++) {
            if (g_serial_tx_len < sizeof(g_serial_tx_buffer)) {
                g_serial_tx_buffer[g_serial_tx_len++] = t->data_buffer[i];
            }
        }
        g_serial_last_byte_ms = millis();
        portEXIT_CRITICAL(&g_serial_lock);

        uint32_t now = millis();
        if (now - g_last_ch340_log_ms > 50) {
            g_last_ch340_log_ms = now;
            log_minifcu_bytes(t->data_buffer, t->actual_num_bytes);
            log_hex_bytes("CH340 RX", t->data_buffer, t->actual_num_bytes, 32);
        }
    }

    usb_host_transfer_submit(t);
}

static void ch340_out_cb(usb_transfer_t *t) {
    (void)t;
    g_ch340_out_busy = false;
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    (void)arg;
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        usb_device_handle_t dev;
#if DEBUG_LOGS
        LOG_SERIAL.printf("\nUSB NEW DEV addr=%d\n", event_msg->new_dev.address);
#endif
        if (usb_host_device_open(g_client, event_msg->new_dev.address, &dev) != ESP_OK) {
#if DEBUG_LOGS
            LOG_SERIAL.println("USB open failed.");
#endif
            return;
        }

        const usb_device_desc_t *dd;
        if (usb_host_get_device_descriptor(dev, &dd) != ESP_OK || !dd) {
#if DEBUG_LOGS
            LOG_SERIAL.println("USB descriptor read failed.");
#endif
            usb_host_device_close(g_client, dev);
            return;
        }
#if DEBUG_LOGS
        LOG_SERIAL.printf("USB VID:PID %04X:%04X Class=%02X\n",
                          dd->idVendor, dd->idProduct, dd->bDeviceClass);
#endif

        if (dd->idVendor == THRUSTMASTER_VID && dd->idProduct == THRUSTMASTER_PID_Q) {
            g_quadrant_dev = dev;
            g_quadrant_connected = true;
            usb_host_interface_claim(g_client, dev, 0, 0);

            g_hid_in_xfer->device_handle = dev;
            g_hid_in_xfer->bEndpointAddress = HID_EP_IN;
            g_hid_in_xfer->num_bytes = HID_MAX_PKT;
            g_hid_in_xfer->callback = hid_in_cb;
            usb_host_transfer_submit(g_hid_in_xfer);
            LOG_SERIAL.println("Quadrant connected.");
            return;
        }

        if (dd->idVendor == CH340_VID && dd->idProduct == CH340_PID) {
            g_ch340_dev = dev;
            g_ch340_connected = true;
            usb_host_interface_claim(g_client, dev, 0, 0);
            init_ch340();
            g_minifcu_init_state = 1;
            g_minifcu_init_ms = millis();

            g_ch340_in->device_handle = dev;
            g_ch340_in->bEndpointAddress = CH340_EP_IN;
            g_ch340_in->num_bytes = CH340_MAX_PKT;
            g_ch340_in->callback = ch340_in_cb;
            usb_host_transfer_submit(g_ch340_in);
            LOG_SERIAL.println("CH340 connected (MiniFCU).");
            return;
        }

        usb_host_device_close(g_client, dev);
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
#if DEBUG_LOGS
        LOG_SERIAL.println("USB DEV GONE.");
#endif
        if (g_quadrant_dev == event_msg->dev_gone.dev_hdl) {
            usb_host_device_close(g_client, g_quadrant_dev);
            g_quadrant_connected = false;
            g_quadrant_dev = nullptr;
            g_last_quadrant_len = 0;
            LOG_SERIAL.println("Quadrant disconnected.");
        }
        if (g_ch340_dev == event_msg->dev_gone.dev_hdl) {
            usb_host_device_close(g_client, g_ch340_dev);
            g_ch340_connected = false;
            g_ch340_dev = nullptr;
            g_ch340_out_busy = false;
            g_minifcu_init_state = 0;
            LOG_SERIAL.println("CH340 disconnected.");
        }
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
                        if (DEBUG_LOGS) {
                            LOG_SERIAL.printf("Register REQ: dst=0x%02X assign=0x%02X\n",
                                              hdr->dst_node, req->assigned_node_id);
                        }
                        g_node_id = req->assigned_node_id;
                        send_register_ack(pkt.mac);
                    }
                    break;

                case MSG_HEARTBEAT:
                    send_heartbeat_ack();
                    break;

                case MSG_SERIAL_DATA:
                    if (payload_len >= 2) {
                        const SerialDataPayload* serial = reinterpret_cast<const SerialDataPayload*>(payload);
                        if (serial->data_length > 0 && serial->data_length <= MAX_SERIAL_DATA_SIZE) {
                            enqueue_serial_out(serial->data, serial->data_length);
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
// CH340 TX Task
// -----------------------------
static void ch340_tx_task(void *arg) {
    (void)arg;
    uint8_t out_buf[CH340_MAX_PKT];

    while (true) {
        if (g_ch340_connected && !g_ch340_out_busy) {
            size_t len = dequeue_serial_out(out_buf, sizeof(out_buf));
            if (len > 0) {
                uint32_t now = millis();
                if (now - g_last_ch340_tx_log_ms > 50) {
                    g_last_ch340_tx_log_ms = now;
                    log_hex_bytes("CH340 TX", out_buf, static_cast<int>(len), 32);
                }
                g_ch340_out_busy = true;
                g_ch340_out->device_handle = g_ch340_dev;
                g_ch340_out->bEndpointAddress = CH340_EP_OUT;
                g_ch340_out->num_bytes = len;
                g_ch340_out->callback = ch340_out_cb;
                memcpy(g_ch340_out->data_buffer, out_buf, len);
                usb_host_transfer_submit(g_ch340_out);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -----------------------------
// Arduino Setup/Loop
// -----------------------------
void setup() {
    LOG_SERIAL_BEGIN();
    delay(500);
    LOG_SERIAL.println("\n=== Node C: Quadrant + MiniFCU ===");

    g_rx_queue = xQueueCreate(16, sizeof(RxPacket));

    // ESP-NOW init
    if (!initEspNowOptimized()) {
        LOG_SERIAL.println("ESP-NOW init failed.");
        return;
    }
    esp_now_register_recv_cb(espnow_recv_cb);
    addEspNowPeer(BROADCAST_MAC);

    // USB Host init
    static usb_host_config_t h_cfg = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    usb_host_install(&h_cfg);

    static usb_host_client_config_t c_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 10,
        .async = { .client_event_callback = client_event_cb, .callback_arg = nullptr }
    };
    usb_host_client_register(&c_cfg, &g_client);

    usb_host_transfer_alloc(HID_MAX_PKT, 0, &g_hid_in_xfer);
    usb_host_transfer_alloc(8, 0, &g_ch340_ctrl);
    usb_host_transfer_alloc(CH340_MAX_PKT, 0, &g_ch340_in);
    usb_host_transfer_alloc(CH340_MAX_PKT, 0, &g_ch340_out);

    xTaskCreatePinnedToCore(usb_task, "usb_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(espnow_task, "espnow_task", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(ch340_tx_task, "ch340_tx_task", 2048, nullptr, 4, nullptr, 1);
}

void loop() {
    if (g_serial_tx_len > 0 && (millis() - g_serial_last_byte_ms > FLUSH_TIMEOUT_MS)) {
        flush_serial_to_coord();
    }
    if (g_ch340_connected && g_minifcu_init_state != 0) {
        uint32_t now = millis();
        if (g_minifcu_init_state == 1 && now - g_minifcu_init_ms > 500) {
            LOG_SERIAL.println("MiniFCU init: send C,");
            const char* c = "C,";
            enqueue_serial_out(reinterpret_cast<const uint8_t*>(c), strlen(c));
            g_minifcu_init_state = 2;
            g_minifcu_init_ms = now;
        } else if (g_minifcu_init_state == 2 && now - g_minifcu_init_ms > 500) {
            send_minifcu_init();
            g_minifcu_init_state = 0;
        }
    }
    delay(1);
}

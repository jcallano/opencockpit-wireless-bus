/**
 * OpenCockpit Wireless Avionics Bus
 * Node D - WinWing Ursa Minor 32 Throttle
 *
 * Responsibilities:
 * - USB Host: WinWing Ursa Minor HID
 * - ESP-NOW: Bidirectional (Inputs + LCD/Backlight)
 *
 * USB driver ported from standalone reference implementation.
 * All output via Interrupt OUT endpoint (EP 0x02).
 *
 * Serial commands on Serial0 @ 115200:
 *   LED <id> <val> [type]  - Report 0x02 LED control (default type=0x10)
 *   BL <val>               - All 4 backlight zones to same value
 *   VIB <id> <val>         - Vibration motor (id: 0x0E or 0x0F)
 *   VIBTEST                - Sweep both motors
 *   LCD <side> <int>.<frac> - 7-segment display (e.g. LCD L 1.5, range 0.0-99.9)
 *   STATUS                 - Print connection state + endpoint info
 *   HELP                   - Print command reference
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
#ifndef UART0_TX_PIN
  #define UART0_TX_PIN 43
#endif
#ifndef UART0_RX_PIN
  #define UART0_RX_PIN 44
#endif

#define LOG_SERIAL          Serial0
#define LOG_SERIAL_BEGIN()  Serial0.begin(115200, SERIAL_8N1, UART0_RX_PIN, UART0_TX_PIN)

// -----------------------------
// Configuration
// -----------------------------
#define NODE_NAME           "NODE_D_URSA"
#define FW_VERSION          "2.0.0"

#define URSA_VID            0x4098
#define URSA_PID            0xB920

#define REPORT_ID_INPUT     0x01
#define REPORT_ID_AUX       0x02
#define REPORT_ID_LCD       0xF0

// =====================================================================
// USB Globals
// =====================================================================
static usb_host_client_handle_t g_client   = nullptr;
static usb_device_handle_t      g_dev      = nullptr;
static usb_transfer_t          *g_xfer_in   = nullptr;
static usb_transfer_t          *g_xfer_ctrl = nullptr;
static usb_transfer_t          *g_xfer_out  = nullptr;

static SemaphoreHandle_t g_ctrl_mux = nullptr;   // mutex for control xfers
static SemaphoreHandle_t g_ctrl_sem = nullptr;    // binary sem for ctrl sync
static SemaphoreHandle_t g_out_sem  = nullptr;    // binary sem for OUT sync

static bool     g_connected = false;
static bool     g_setup_done = false;
static uint8_t  g_ep_in  = 0;
static uint8_t  g_ep_out = 0;
static uint16_t g_mps_in  = 64;
static uint16_t g_mps_out = 64;
static int      g_ifc    = 0;
static uint16_t g_hid_report_desc_len = 0;

// Output report sizes parsed from HID report descriptor
static uint16_t g_report02_size = 64;  // default, overridden by descriptor
static uint16_t g_reportF0_size = 64;  // default, overridden by descriptor

// =====================================================================
// ESP-NOW Globals
// =====================================================================
static uint8_t g_coordinator_mac[6] = {0};
static bool g_have_coordinator = false;
static uint8_t g_node_id = NODE_D_THROTTLE;

static QueueHandle_t g_rx_queue = nullptr;

struct RxPacket {
    uint8_t mac[6];
    uint8_t data[MAX_ESPNOW_PAYLOAD];
    int len;
};

// =====================================================================
// Button Names (42)
// =====================================================================
static const char* kBtnNames[] = {
    "UNK",
    "ENG 1 MASTER ON",       "ENG 1 MASTER OFF",
    "ENG 2 MASTER ON",       "ENG 2 MASTER OFF",
    "ENG 1 FIRE BUTTON",     "ENG 2 FIRE BUTTON",
    "ENG MODE CRANK",        "ENG MODE NORM",
    "ENG MODE IGN/START",    "L THROT AUTO DISC",
    "R THROT AUTO DISC",     "L THROT TO/GA",
    "L THROT FLEX/MCT",      "L THROT CL",
    "L THROT IDLE",          "L THROT IDLE REV",
    "L THROT FULL REV",      "R THROT TO/GA",
    "R THROT FLEX/MCT",      "R THROT CL",
    "R THROT IDLE",          "R THROT IDLE REV",
    "R THROT FULL REV",      "ENG MODE PUSH BTN",
    "TRIM RESET",            "RUDDER TRIM L",
    "RUDDER TRIM NEUTRAL",   "RUDDER TRIM R",
    "PARKING BRAKE OFF",     "PARKING BRAKE ON",
    "FLAPS 4",               "FLAPS 3",
    "FLAPS 2",               "FLAPS 1",
    "FLAPS 0",               "SPOILER FULL",
    "SPOILER ONE HALF",      "SPOILER RET",
    "SPOILER ARM",           "L THROT REV MODE ON",
    "R THROT REV MODE ON",   "BUTTON 42"
};

// Axis definitions
struct AxisDef { int offset; const char* name; };
static const AxisDef kAxes[] = {
    {13, "LEFT THROTTLE"},
    {15, "RIGHT THROTTLE"},
    {19, "SPOILER"},
    {21, "FLAPS"}
};

// =====================================================================
// 7-Segment LCD Tables (from reference)
// =====================================================================
static const uint8_t SEVEN_SEG[10][7] = {
    {1,1,1,1,1,1,0},  // 0
    {0,1,1,0,0,0,0},  // 1
    {1,1,0,1,1,0,1},  // 2
    {1,1,1,1,0,0,1},  // 3
    {0,1,1,0,0,1,1},  // 4
    {1,0,1,1,0,1,1},  // 5
    {1,0,1,1,1,1,1},  // 6
    {1,1,1,0,0,0,0},  // 7
    {1,1,1,1,1,1,1},  // 8
    {1,1,1,1,0,1,1},  // 9
};

static const uint8_t BLANK_SEG[7] = {0,0,0,0,0,0,0};

// slot0=f, slot1=e, slot2=d, slot3=c, slot4=b, slot5=a
static const int SLOT_SEG_INDEX[6] = {5, 4, 3, 2, 1, 0};

// Side bit patterns (bit 0 of each slot)
static const uint8_t SIDE_BITS_L[6] = {1, 1, 1, 0, 0, 0};
static const uint8_t SIDE_BITS_R[6] = {1, 1, 0, 1, 1, 1};

// =====================================================================
// send_ctrl_sync() — Synchronous Control Transfer (uses submit_control)
// =====================================================================
static void ctrl_cb(usb_transfer_t *t) {
    if (t->status != USB_TRANSFER_STATUS_COMPLETED) {
        LOG_SERIAL.printf("CTRL CB err=%d\n", t->status);
    }
    if (g_ctrl_sem) xSemaphoreGive(g_ctrl_sem);
}

static esp_err_t send_ctrl_sync(const usb_setup_packet_t *setup,
                                const uint8_t *data, size_t len) {
    if (!g_dev || !g_xfer_ctrl || !g_ctrl_sem || !g_ctrl_mux) return ESP_FAIL;

    // Acquire mutex — only one control transfer at a time
    if (xSemaphoreTake(g_ctrl_mux, pdMS_TO_TICKS(2000)) != pdTRUE) {
        LOG_SERIAL.println("CTRL: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    // Clear the binary semaphore (non-blocking take)
    xSemaphoreTake(g_ctrl_sem, 0);

    // Build transfer
    memcpy(g_xfer_ctrl->data_buffer, setup, sizeof(usb_setup_packet_t));
    if (data && len > 0 && (setup->bmRequestType & 0x80) == 0) {
        memcpy(g_xfer_ctrl->data_buffer + sizeof(usb_setup_packet_t), data, len);
    }
    g_xfer_ctrl->num_bytes        = sizeof(usb_setup_packet_t) + len;
    g_xfer_ctrl->device_handle    = g_dev;
    g_xfer_ctrl->bEndpointAddress = 0;
    g_xfer_ctrl->callback         = ctrl_cb;
    g_xfer_ctrl->timeout_ms       = 1000;
    g_xfer_ctrl->flags            = 0;

    esp_err_t err = usb_host_transfer_submit_control(g_client, g_xfer_ctrl);
    if (err != ESP_OK) {
        LOG_SERIAL.printf("CTRL submit err=%d\n", err);
        xSemaphoreGive(g_ctrl_mux);
        return err;
    }

    // Wait for callback
    if (xSemaphoreTake(g_ctrl_sem, pdMS_TO_TICKS(1100)) != pdTRUE) {
        LOG_SERIAL.println("CTRL timeout");
        xSemaphoreGive(g_ctrl_mux);
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(g_ctrl_mux);
    return ESP_OK;
}

// =====================================================================
// send_out_sync() — Synchronous Interrupt OUT Transfer
// =====================================================================
static void out_cb(usb_transfer_t *t) {
    if (t->status != USB_TRANSFER_STATUS_COMPLETED) {
        LOG_SERIAL.printf("OUT CB err=%d\n", t->status);
    }
    if (g_out_sem) xSemaphoreGive(g_out_sem);
}

static esp_err_t send_out_sync(const uint8_t *data, size_t len) {
    if (!g_ep_out || !g_xfer_out || !g_out_sem) return ESP_FAIL;

    xSemaphoreTake(g_out_sem, 0);  // clear

    memcpy(g_xfer_out->data_buffer, data, len);
    g_xfer_out->device_handle    = g_dev;
    g_xfer_out->bEndpointAddress = g_ep_out;
    g_xfer_out->num_bytes        = len;
    g_xfer_out->callback         = out_cb;
    g_xfer_out->timeout_ms       = 1000;

    esp_err_t err = usb_host_transfer_submit(g_xfer_out);
    if (err != ESP_OK) return err;

    if (xSemaphoreTake(g_out_sem, pdMS_TO_TICKS(1100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// =====================================================================
// send_report02() — Backlight / LED / Motor via Interrupt OUT
// =====================================================================
static esp_err_t send_report02(uint8_t type, uint8_t id, uint8_t val) {
    static uint8_t buf[64];
    uint16_t sz = g_report02_size;
    memset(buf, 0, sz);

    buf[0] = 0x02;   // Report ID
    buf[1] = type;
    buf[2] = 0xB9;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x03;
    buf[6] = 0x49;
    buf[7] = id;
    buf[8] = val;

    esp_err_t err = send_out_sync(buf, sz);
    LOG_SERIAL.printf("RPT02 type=0x%02X id=0x%02X val=%d sz=%d %s\n",
               type, id, val, sz, err == ESP_OK ? "OK" : "FAIL");
    return err;
}

// =====================================================================
// LCD Encode + Send (supports 0-99.9, from reference)
// =====================================================================
static uint8_t g_lcd_ctr_data   = 0;
static uint8_t g_lcd_ctr_commit = 0;

static void encode_lcd(char side, uint8_t integer, uint8_t fractional,
                       uint8_t *pkt, uint16_t pkt_size) {
    uint8_t tens = integer / 10;
    uint8_t ones = integer % 10;

    const uint8_t *frac_segs = SEVEN_SEG[fractional];
    const uint8_t *ones_segs = SEVEN_SEG[ones];
    const uint8_t *tens_segs = (tens > 0) ? SEVEN_SEG[tens] : BLANK_SEG;

    uint8_t side_bit = (side == 'R') ? 1 : 0;
    uint8_t b29 = (frac_segs[6] << 3) | (ones_segs[6] << 2)
                | (tens_segs[6] << 1) | side_bit;

    const uint8_t *bit0 = (side == 'R') ? SIDE_BITS_R : SIDE_BITS_L;

    memset(pkt, 0, pkt_size);
    pkt[0]  = 0xF0;          // Report ID
    pkt[2]  = g_lcd_ctr_data++;
    pkt[3]  = 0x38;          // DATA type
    pkt[4]  = 0x01;  pkt[5]  = 0xB9;
    pkt[8]  = 0x02;  pkt[9]  = 0x01;
    pkt[17] = 0x24;
    pkt[25] = 0x04;
    pkt[29] = b29;

    for (int i = 0; i < 6; i++) {
        int seg_idx   = SLOT_SEG_INDEX[i];
        uint8_t frac_bit = frac_segs[seg_idx];
        uint8_t ones_bit = ones_segs[seg_idx];
        uint8_t tens_bit = tens_segs[seg_idx];
        pkt[33 + i * 4] = (frac_bit << 3) | (ones_bit << 2)
                         | (tens_bit << 1) | bit0[i];
    }

    pkt[57] = 0x01;  pkt[58] = 0xB9;
}

static void build_lcd_commit(uint8_t *pkt, uint16_t pkt_size) {
    memset(pkt, 0, pkt_size);
    pkt[0] = 0xF0;
    pkt[2] = g_lcd_ctr_commit++;
    pkt[3] = 0x0E;           // COMMIT type
    pkt[5] = 0x03;  pkt[6] = 0x01;
}

static esp_err_t send_lcd(char side, uint8_t integer, uint8_t fractional) {
    if (integer > 99 || fractional > 9) {
        LOG_SERIAL.println("LCD: value out of range (0.0-99.9)");
        return ESP_ERR_INVALID_ARG;
    }

    static uint8_t data_pkt[64];
    static uint8_t commit_pkt[64];
    uint16_t sz = g_reportF0_size;

    encode_lcd(side, integer, fractional, data_pkt, sz);
    build_lcd_commit(commit_pkt, sz);

    esp_err_t err = send_out_sync(data_pkt, sz);
    if (err != ESP_OK) {
        LOG_SERIAL.printf("LCD DATA FAIL err=%d\n", err);
        return err;
    }
    delay(5);
    err = send_out_sync(commit_pkt, sz);
    if (err != ESP_OK) LOG_SERIAL.printf("LCD COMMIT FAIL err=%d\n", err);

    return err;
}

// =====================================================================
// Input Report Parser with Delta Logging + ESP-NOW Forwarding
// =====================================================================
static bool    g_first_input = true;

static void log_input_changes(const uint8_t *cur, const uint8_t *prev) {
    for (int i = 0; i < 6; i++) {
        uint8_t diff = cur[1 + i] ^ prev[1 + i];
        if (!diff) continue;
        for (int b = 0; b < 8; b++) {
            if (!(diff & (1 << b))) continue;
            int btn = i * 8 + b + 1;
            if (btn > 42) continue;
            bool pressed = cur[1 + i] & (1 << b);
            LOG_SERIAL.printf("BTN %02d [%s]: %s\n", btn, kBtnNames[btn],
                       pressed ? "PRESS" : "REL");
        }
    }

    for (int i = 0; i < 4; i++) {
        int off = kAxes[i].offset;
        if (cur[off] != prev[off] || cur[off + 1] != prev[off + 1]) {
            uint16_t val = cur[off] | (cur[off + 1] << 8);
            LOG_SERIAL.printf("%s: %d\n", kAxes[i].name, val);
        }
    }
}

static void send_hid_input_espnow(const uint8_t* report, size_t len) {
    if (!g_have_coordinator) return;
    if (len > MAX_HID_REPORT_SIZE) len = MAX_HID_REPORT_SIZE;

    HIDInputPayload payload = {};
    payload.device_id = DEV_URSA_MINOR;
    payload.report_id = report[0];
    payload.report_length = len;
    memcpy(payload.report_data, report, len);

    uint8_t buffer[96];
    size_t msg_len = buildMessage(buffer, MSG_HID_INPUT, g_node_id, NODE_COORDINATOR, &payload, 3 + len);
    esp_now_send(g_coordinator_mac, buffer, msg_len);
}

static uint8_t g_last_sent_report[64];

static void process_input_report(const uint8_t *data, size_t len) {
    if (data[0] != REPORT_ID_INPUT || len < 23) return;

    // Filter noise: Mask LSB of axes (16-bit -> 8-bit)
    uint8_t filtered[64];
    size_t cap_len = len > 64 ? 64 : len;
    memcpy(filtered, data, cap_len);

    // Axis LSB offsets: 13, 15, 19, 21
    filtered[13] = 0; // Throttle L
    filtered[15] = 0; // Throttle R
    filtered[19] = 0; // Spoiler
    filtered[21] = 0; // Flaps

    if (g_first_input) {
        LOG_SERIAL.println("HID Input Active (Optimized).");
        memcpy(g_last_sent_report, filtered, cap_len);  // Init filtered history
        g_first_input = false;
        // Send initial state
        send_hid_input_espnow(filtered, len);
        return;
    }

    // Check against last SENT (filtered) report
    if (memcmp(filtered, g_last_sent_report, cap_len) != 0) {
        // Log changes based on filtered data (shows e.g. "LEFT THROTTLE: 32512")
        log_input_changes(filtered, g_last_sent_report);
        
        // Update history
        memcpy(g_last_sent_report, filtered, cap_len);

        // Send Optimized Report via ESP-NOW
        send_hid_input_espnow(filtered, len);
    }
}

// =====================================================================
// Endpoint Discovery (from reference)
// =====================================================================
static bool discover_endpoints(usb_device_handle_t dev) {
    const usb_config_desc_t *cfg = nullptr;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK || !cfg)
        return false;

    const uint8_t *p = (const uint8_t *)cfg;
    int cur_if = -1, cur_alt = 0;

    LOG_SERIAL.println("--- Descriptor Dump ---");
    for (int off = 0; off + 2 <= cfg->wTotalLength; ) {
        const usb_standard_desc_t *d = (const usb_standard_desc_t *)(p + off);
        if (d->bLength == 0) break;

        if (d->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *ifd = (const usb_intf_desc_t *)d;
            cur_if  = ifd->bInterfaceNumber;
            cur_alt = ifd->bAlternateSetting;
            LOG_SERIAL.printf("IFC: Num=%d Alt=%d Class=%d\n",
                       cur_if, cur_alt, ifd->bInterfaceClass);
        } else if (d->bDescriptorType == 0x21) {
            // HID descriptor — extract report descriptor length
            if (d->bLength >= 9) {
                g_hid_report_desc_len = p[off + 7] | (p[off + 8] << 8);
                LOG_SERIAL.printf("HID desc: report_desc_len=%d\n", g_hid_report_desc_len);
            }
        } else if (d->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)d;
            LOG_SERIAL.printf("  EP: 0x%02X MPS=%d\n",
                       ep->bEndpointAddress, ep->wMaxPacketSize);
            if (cur_if >= 0) {
                if ((ep->bEndpointAddress & 0x80) && g_ep_in == 0) {
                    g_ep_in  = ep->bEndpointAddress;
                    g_mps_in = ep->wMaxPacketSize;
                    g_ifc    = cur_if;
                } else if (!(ep->bEndpointAddress & 0x80) && g_ep_out == 0) {
                    g_ep_out  = ep->bEndpointAddress;
                    g_mps_out = ep->wMaxPacketSize;
                }
            }
        }
        off += d->bLength;
    }
    LOG_SERIAL.println("-----------------------");

    if (g_ep_in == 0) {
        LOG_SERIAL.println("WARNING: No IN endpoint found, defaulting to 0x81");
        g_ep_in = 0x81;
    }
    if (g_ep_out == 0) {
        LOG_SERIAL.println("ERROR: No OUT endpoint found! Output commands will fail.");
    }

    return true;
}

// =====================================================================
// Fetch HID Report Descriptor (auto-detect report sizes)
// =====================================================================
static void fetch_report_descriptor(usb_device_handle_t dev) {
    if (g_hid_report_desc_len == 0 || g_hid_report_desc_len > 1024) {
        LOG_SERIAL.println("HID report desc: invalid length");
        return;
    }

    usb_setup_packet_t setup;
    setup.bmRequestType = 0x81;  // Device->Host, Standard, Interface
    setup.bRequest      = 0x06;  // GET_DESCRIPTOR
    setup.wValue        = (0x22 << 8);  // HID Report descriptor
    setup.wIndex        = g_ifc;
    setup.wLength       = g_hid_report_desc_len;

    usb_transfer_t *xfer = nullptr;
    size_t buf_size = sizeof(usb_setup_packet_t) + g_hid_report_desc_len;
    if (buf_size < 64) buf_size = 64;
    if (usb_host_transfer_alloc(buf_size + 64, 0, &xfer) != ESP_OK) {
        LOG_SERIAL.println("HID report desc: alloc failed");
        return;
    }

    memcpy(xfer->data_buffer, &setup, sizeof(usb_setup_packet_t));
    xfer->num_bytes        = sizeof(usb_setup_packet_t) + g_hid_report_desc_len;
    xfer->device_handle    = dev;
    xfer->bEndpointAddress = 0;
    xfer->callback         = ctrl_cb;
    xfer->timeout_ms       = 2000;
    xfer->flags            = 0;

    xSemaphoreTake(g_ctrl_sem, 0);
    esp_err_t err = usb_host_transfer_submit_control(g_client, xfer);
    if (err != ESP_OK) {
        LOG_SERIAL.printf("HID report desc: submit err=%d\n", err);
        usb_host_transfer_free(xfer);
        return;
    }

    if (xSemaphoreTake(g_ctrl_sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
        LOG_SERIAL.println("HID report desc: timeout");
        usb_host_transfer_free(xfer);
        return;
    }

    const uint8_t *rd = xfer->data_buffer + sizeof(usb_setup_packet_t);
    int rd_len = xfer->actual_num_bytes - sizeof(usb_setup_packet_t);
    LOG_SERIAL.printf("HID Report Descriptor (%d bytes):\n", rd_len);

    for (int i = 0; i < rd_len; i++) {
        LOG_SERIAL.printf("%02X ", rd[i]);
        if ((i + 1) % 16 == 0) LOG_SERIAL.println();
    }
    if (rd_len % 16) LOG_SERIAL.println();

    // Parse to find output report sizes per report ID
    uint8_t cur_report_id = 0;
    uint16_t cur_report_size = 0;
    uint16_t cur_report_count = 0;
    uint32_t rpt02_bits = 0, rptF0_bits = 0;

    for (int i = 0; i < rd_len; ) {
        uint8_t prefix = rd[i];
        int item_size = prefix & 0x03;
        if (item_size == 3) item_size = 4;
        uint8_t item_tag  = (prefix >> 4) & 0x0F;
        uint8_t item_type = (prefix >> 2) & 0x03;

        uint32_t val = 0;
        if (i + 1 + item_size <= rd_len) {
            for (int j = 0; j < item_size; j++)
                val |= (uint32_t)rd[i + 1 + j] << (j * 8);
        }

        if (item_type == 1) {
            if (item_tag == 7) cur_report_size = val;
            if (item_tag == 9) cur_report_count = val;
            if (item_tag == 8) cur_report_id = val;
        }
        if (item_type == 0 && item_tag == 9) {  // OUTPUT
            uint32_t bits = (uint32_t)cur_report_size * cur_report_count;
            if (cur_report_id == 0x02) rpt02_bits += bits;
            if (cur_report_id == 0xF0) rptF0_bits += bits;
            LOG_SERIAL.printf("  OUTPUT: id=0x%02X size=%d count=%d (%d bits)\n",
                       cur_report_id, cur_report_size, cur_report_count, bits);
        }

        i += 1 + item_size;
    }

    if (rpt02_bits > 0) {
        g_report02_size = (rpt02_bits + 7) / 8 + 1;
        LOG_SERIAL.printf("Report 0x02 OUT size: %d bytes (%d bits + report ID)\n",
                   g_report02_size, rpt02_bits);
    }
    if (rptF0_bits > 0) {
        g_reportF0_size = (rptF0_bits + 7) / 8 + 1;
        LOG_SERIAL.printf("Report 0xF0 OUT size: %d bytes (%d bits + report ID)\n",
                   g_reportF0_size, rptF0_bits);
    }

    usb_host_transfer_free(xfer);
}

// =====================================================================
// USB Event Callback (connect / disconnect)
// =====================================================================
static void in_cb(usb_transfer_t *t) {
    if (!g_connected || !t) return;
    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
        process_input_report(t->data_buffer, t->actual_num_bytes);
    }
    usb_host_transfer_submit(t);
}

static void usb_event_cb(const usb_host_client_event_msg_t *msg, void *arg) {
    (void)arg;

    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        LOG_SERIAL.println("USB: New device detected");

        usb_device_handle_t dev;
        if (usb_host_device_open(g_client, msg->new_dev.address, &dev) != ESP_OK) {
            LOG_SERIAL.println("USB: Failed to open device");
            return;
        }

        const usb_device_desc_t *desc;
        usb_host_get_device_descriptor(dev, &desc);
        LOG_SERIAL.printf("USB: VID=%04X PID=%04X\n", desc->idVendor, desc->idProduct);

        if (desc->idVendor != URSA_VID || desc->idProduct != URSA_PID) {
            LOG_SERIAL.println("USB: Not Ursa Minor, ignoring");
            usb_host_device_close(g_client, dev);
            return;
        }

        g_dev    = dev;
        g_ep_in  = 0;
        g_ep_out = 0;

        discover_endpoints(dev);

        // Claim interface
        usb_host_interface_claim(g_client, dev, g_ifc, 0);
        LOG_SERIAL.printf("USB: Claimed ifc=%d ep_in=0x%02X ep_out=0x%02X\n",
                   g_ifc, g_ep_in, g_ep_out);

        // Start IN transfers
        g_xfer_in->device_handle    = dev;
        g_xfer_in->bEndpointAddress = g_ep_in;
        g_xfer_in->num_bytes        = (g_mps_in > 64) ? 64 : g_mps_in;
        g_xfer_in->callback         = in_cb;
        usb_host_transfer_submit(g_xfer_in);

        g_connected  = true;
        g_setup_done = false;
        g_first_input = true;
        LOG_SERIAL.println("Ursa Minor connected.");

    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        if (g_dev == msg->dev_gone.dev_hdl) {
            LOG_SERIAL.println("Ursa Minor disconnected.");
            usb_host_interface_release(g_client, g_dev, g_ifc);
            usb_host_device_close(g_client, g_dev);
            g_connected  = false;
            g_setup_done = false;
            g_dev        = nullptr;
            g_ep_in      = 0;
            g_ep_out     = 0;
        }
    }
}

// =====================================================================
// USB Host Task (Core 0, Priority 20)
// =====================================================================
static void usb_task(void *arg) {
    (void)arg;
    LOG_SERIAL.println("USB task started");
    while (1) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(1, &flags);
        if (g_client) {
            usb_host_client_handle_events(g_client, 1);
        }
        vTaskDelay(1);
    }
}

// =====================================================================
// Device Setup Sequence (backlights init via Interrupt OUT)
// =====================================================================
static void device_setup_sequence() {
    LOG_SERIAL.println("Running device setup sequence...");

    // Fetch HID report descriptor to learn actual output report sizes
    fetch_report_descriptor(g_dev);

    // Init backlights — all 4 zones to max
    struct { uint8_t type; uint8_t id; } zones[] = {
        {0x10, 0x00},  // Throttle
        {0x10, 0x02},  // Marker
        {0x01, 0x00},  // Flaps
        {0x01, 0x02},  // LCD
    };
    for (auto &z : zones) {
        esp_err_t err = send_report02(z.type, z.id, 255);
        LOG_SERIAL.printf("BL init type=0x%02X id=0x%02X: %s\n",
                   z.type, z.id, err == ESP_OK ? "OK" : "FAIL");
        delay(50);
    }

    LOG_SERIAL.println("Setup sequence complete.");
}

// =====================================================================
// ESP-NOW Helpers
// =====================================================================
static void set_coordinator_mac(const uint8_t* mac) {
    if (g_have_coordinator) return;
    memcpy(g_coordinator_mac, mac, 6);
    g_have_coordinator = true;
    addEspNowPeer(g_coordinator_mac);
    LOG_SERIAL.println("Locked to Coordinator.");
}

static void send_message_to_coord(const uint8_t* data, size_t len) {
    if (!g_have_coordinator) return;
    esp_now_send(g_coordinator_mac, data, len);
}

static void send_discovery_response(const uint8_t* mac) {
    DiscoveryResponse rsp = {};
    rsp.node_type = NODE_TYPE_HID;
    rsp.capabilities = CAP_HID_INPUT | CAP_HID_OUTPUT | CAP_DISPLAY | CAP_BIDIRECTIONAL;
    rsp.device_count = (g_connected ? 1 : 0);
    memcpy(rsp.mac_address, mac, 6);
    strncpy(rsp.firmware_version, FW_VERSION, FIRMWARE_VERSION_SIZE - 1);
    strncpy(rsp.node_name, NODE_NAME, MAX_NODE_NAME_SIZE - 1);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t len = buildMessage(buffer, MSG_DISCOVERY_RSP, g_node_id, NODE_COORDINATOR, &rsp, sizeof(rsp));
    esp_now_send(mac, buffer, len);
}

// =====================================================================
// ESP-NOW Receive Callback + Protocol Task
// =====================================================================
static void espnow_recv_cb(const esp_now_recv_info* info, const uint8_t* data, int len) {
    if (!info || !data || len > MAX_ESPNOW_PAYLOAD) return;

    RxPacket pkt;
    memcpy(pkt.mac, info->src_addr, 6);
    memcpy(pkt.data, data, len);
    pkt.len = len;
    xQueueSendFromISR(g_rx_queue, &pkt, nullptr);
}

static void process_lcd_command(const char* text) {
    // Expected format: "L 1.5" or "R 12.3"
    if (strlen(text) < 3) return;

    char side = text[0];
    if (side != 'L' && side != 'R') return;

    int numStart = 1;
    while (text[numStart] == ' ') numStart++;

    // Parse integer.fractional
    const char* numStr = &text[numStart];
    int dotPos = -1;
    for (int i = 0; numStr[i]; i++) {
        if (numStr[i] == '.') { dotPos = i; break; }
    }

    uint8_t integer, fractional;
    if (dotPos >= 0) {
        char intPart[4] = {0};
        strncpy(intPart, numStr, dotPos < 3 ? dotPos : 3);
        integer = (uint8_t)atoi(intPart);
        fractional = (uint8_t)atoi(&numStr[dotPos + 1]);
    } else {
        integer = (uint8_t)atoi(numStr);
        fractional = 0;
    }

    send_lcd(side, integer, fractional);
}

static void protocol_task(void *arg) {
    (void)arg;
    RxPacket pkt;
    uint32_t last_hb = 0;
    static uint8_t tx_buf[MAX_ESPNOW_PAYLOAD];

    while(1) {
        if (xQueueReceive(g_rx_queue, &pkt, 1)) {
            set_coordinator_mac(pkt.mac);
            if (!validateMessage(pkt.data, pkt.len)) continue;

            const MessageHeader* hdr = (const MessageHeader*)pkt.data;
            if (hdr->dst_node != g_node_id && hdr->dst_node != NODE_BROADCAST) continue;

            const uint8_t* payload = pkt.data + sizeof(MessageHeader);

            switch(hdr->msg_type) {
                case MSG_DISCOVERY_REQ:
                    LOG_SERIAL.println("RX: MSG_DISCOVERY_REQ");
                    send_discovery_response(pkt.mac);
                    break;
                case MSG_REGISTER_REQ:
                    if (pkt.len >= (int)(sizeof(MessageHeader) + sizeof(RegisterRequest))) {
                        const RegisterRequest* req = (const RegisterRequest*)payload;
                        g_node_id = req->assigned_node_id;
                        size_t l = buildMessage(tx_buf, MSG_REGISTER_ACK, g_node_id, NODE_COORDINATOR, nullptr, 0);
                        esp_now_send(pkt.mac, tx_buf, l);
                    }
                    break;
                case MSG_HEARTBEAT: {
                    HeartbeatPayload hb = {millis(), 0};
                    size_t l = buildMessage(tx_buf, MSG_HEARTBEAT_ACK, g_node_id, NODE_COORDINATOR, &hb, sizeof(hb));
                    esp_now_send(pkt.mac, tx_buf, l);
                    break;
                }
                case MSG_TEST_REQ: {
                    size_t payload_len = pkt.len - sizeof(MessageHeader);
                    size_t l = buildMessage(tx_buf, MSG_TEST_RSP, g_node_id, NODE_COORDINATOR, payload, payload_len);
                    esp_now_send(pkt.mac, tx_buf, l);
                    // LOG_SERIAL.println("RX: MSG_TEST_REQ"); // Commented out to avoid flooding during throughput test
                    break;
                }
                case MSG_MCDU_DISPLAY: {
                    const MCDUDisplayPayload* p = (const MCDUDisplayPayload*)payload;
                    if (p->command_type == 0x01) {
                        process_lcd_command((const char*)p->text);
                    }
                    break;
                }
                case MSG_HID_OUTPUT: {
                    const HIDOutputPayload* out = (const HIDOutputPayload*)payload;
                    if (out->device_id == DEV_URSA_MINOR && g_connected && g_setup_done) {
                        send_out_sync(out->report_data, out->report_length);
                    }
                    break;
                }
            }
        }

        if (g_have_coordinator && millis() - last_hb > 500) {
            last_hb = millis();
            HeartbeatPayload hb = {millis(), 0};
            size_t l = buildMessage(tx_buf, MSG_HEARTBEAT, g_node_id, NODE_COORDINATOR, &hb, sizeof(hb));
            send_message_to_coord(tx_buf, l);
        }
    }
}

// =====================================================================
// Serial Command Processor
// =====================================================================
static int parse_int(const char *str) {
    return (int)strtol(str, nullptr, 0);
}

static void print_help() {
    LOG_SERIAL.println(
        "=== Ursa Minor Commands ===\n"
        "LED <id> <val> [type]  Set LED (default type=0x10)\n"
        "BL <val>               All 4 backlight zones\n"
        "VIB <id> <val>         Vibration motor (0x0E/0x0F)\n"
        "VIBTEST                Sweep both motors\n"
        "LCD <side> <val>       7-seg display (LCD L 1.5, 0.0-99.9)\n"
        "STATUS                 Connection info\n"
        "HELP                   This help\n"
        "===========================");
}

static void print_status() {
    LOG_SERIAL.printf(
        "=== STATUS ===\n"
        "Connected: %s\n"
        "Setup done: %s\n"
        "Interface: %d\n"
        "EP IN:  0x%02X (MPS %d)\n"
        "EP OUT: 0x%02X (MPS %d)%s\n"
        "Report 0x02 size: %d bytes\n"
        "Report 0xF0 size: %d bytes\n"
        "ESP-NOW Coordinator: %s\n"
        "==============\n",
        g_connected ? "YES" : "NO",
        g_setup_done ? "YES" : "NO",
        g_ifc,
        g_ep_in, g_mps_in,
        g_ep_out, g_mps_out,
        g_ep_out == 0 ? " [NONE]" : "",
        g_report02_size, g_reportF0_size,
        g_have_coordinator ? "YES" : "NO");
}

static void process_serial() {
    if (!LOG_SERIAL.available()) return;

    String line = LOG_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    LOG_SERIAL.printf("> %s\n", line.c_str());

    if (line.equalsIgnoreCase("HELP")) {
        print_help();
        return;
    }

    if (line.equalsIgnoreCase("STATUS")) {
        print_status();
        return;
    }

    if (!g_connected || !g_setup_done) {
        LOG_SERIAL.println("Device not ready");
        return;
    }

    if (line.startsWith("LED ")) {
        int sp1 = line.indexOf(' ');
        int sp2 = line.indexOf(' ', sp1 + 1);
        int sp3 = (sp2 > 0) ? line.indexOf(' ', sp2 + 1) : -1;

        if (sp1 > 0 && sp2 > 0) {
            int id  = parse_int(line.substring(sp1 + 1, sp2).c_str());
            int val, type_val = 0x10;
            if (sp3 > 0) {
                val      = parse_int(line.substring(sp2 + 1, sp3).c_str());
                type_val = parse_int(line.substring(sp3 + 1).c_str());
            } else {
                val = parse_int(line.substring(sp2 + 1).c_str());
            }
            send_report02((uint8_t)type_val, (uint8_t)id, (uint8_t)val);
        } else {
            LOG_SERIAL.println("Usage: LED <id> <val> [type]");
        }
    }
    else if (line.startsWith("BL ")) {
        int val = parse_int(line.substring(3).c_str());
        struct { uint8_t type; uint8_t id; } zones[] = {
            {0x10, 0x00}, {0x10, 0x02}, {0x01, 0x00}, {0x01, 0x02}
        };
        for (auto &z : zones) {
            send_report02(z.type, z.id, (uint8_t)val);
            delay(20);
        }
        LOG_SERIAL.printf("BL set to %d on all zones\n", val);
    }
    else if (line.startsWith("VIB ")) {
        int sp1 = line.indexOf(' ');
        int sp2 = line.indexOf(' ', sp1 + 1);
        if (sp1 > 0 && sp2 > 0) {
            int id  = parse_int(line.substring(sp1 + 1, sp2).c_str());
            int val = parse_int(line.substring(sp2 + 1).c_str());
            send_report02(0x10, (uint8_t)id, (uint8_t)val);
        } else {
            LOG_SERIAL.println("Usage: VIB <id> <val>");
        }
    }
    else if (line.equalsIgnoreCase("VIBTEST")) {
        LOG_SERIAL.println("Testing motors (0x0E, 0x0F)...");
        uint8_t motors[] = {0x0E, 0x0F};
        for (uint8_t m : motors) {
            send_report02(0x10, m, 255);
            delay(500);
            send_report02(0x10, m, 0);
            delay(200);
        }
        LOG_SERIAL.println("VIBTEST done.");
    }
    else if (line.startsWith("LCD ")) {
        String args = line.substring(4);
        args.trim();
        if (args.length() < 3) {
            LOG_SERIAL.println("Usage: LCD <L|R> <value>  e.g. LCD L 1.5");
            return;
        }
        char side = args.charAt(0);
        if (side != 'L' && side != 'R') {
            LOG_SERIAL.println("Side must be L or R");
            return;
        }
        int numStart = args.indexOf(' ');
        if (numStart < 0) {
            LOG_SERIAL.println("Usage: LCD <L|R> <value>");
            return;
        }
        String numStr = args.substring(numStart + 1);
        numStr.trim();

        int dotPos = numStr.indexOf('.');
        uint8_t integer, fractional;
        if (dotPos >= 0) {
            integer    = (uint8_t)numStr.substring(0, dotPos).toInt();
            fractional = (uint8_t)numStr.substring(dotPos + 1).toInt();
        } else {
            integer    = (uint8_t)numStr.toInt();
            fractional = 0;
        }

        esp_err_t err = send_lcd(side, integer, fractional);
        LOG_SERIAL.printf("LCD %c %d.%d: %s\n", side, integer, fractional,
                   err == ESP_OK ? "OK" : "FAIL");
    }
    else {
        LOG_SERIAL.println("Unknown command. Type HELP for reference.");
    }
}

// =====================================================================
// setup() + loop()
// =====================================================================
void setup() {
    delay(3000);

    LOG_SERIAL_BEGIN();
    LOG_SERIAL.println("\n=== Node D: Ursa Minor Throttle (V2) ===");
    LOG_SERIAL.flush();

    setupHardware();
    enableUsbHostPower(true);

    // --- ESP-NOW Init ---
    g_rx_queue = xQueueCreate(10, sizeof(RxPacket));

    initEspNowOptimized();
    esp_now_register_recv_cb(espnow_recv_cb);
    LOG_SERIAL.println("ESP-NOW Initialized");

    // --- Create synchronization primitives (ONCE) ---
    g_ctrl_mux = xSemaphoreCreateMutex();
    g_ctrl_sem = xSemaphoreCreateBinary();
    g_out_sem  = xSemaphoreCreateBinary();

    // --- USB Host Init ---
    LOG_SERIAL.println("Installing USB Host...");
    const usb_host_config_t h_cfg = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    esp_err_t err = usb_host_install(&h_cfg);
    if (err != ESP_OK) {
        LOG_SERIAL.printf("USB Host install FAILED: %d\n", err);
        return;
    }
    LOG_SERIAL.println("USB Host installed.");
    #ifdef PIN_LED_STATUS
    digitalWrite(PIN_LED_STATUS, HIGH);
    #endif

    delay(500);

    // Register client
    static usb_host_client_config_t c_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 10,
        .async = {
            .client_event_callback = usb_event_cb,
            .callback_arg = nullptr
        }
    };
    err = usb_host_client_register(&c_cfg, &g_client);
    if (err != ESP_OK) {
        LOG_SERIAL.printf("USB client register FAILED: %d\n", err);
        return;
    }
    LOG_SERIAL.println("USB client registered.");

    // Allocate transfers
    usb_host_transfer_alloc(64,   0, &g_xfer_in);
    usb_host_transfer_alloc(1024, 0, &g_xfer_out);   // large enough for full report
    usb_host_transfer_alloc(1024, 0, &g_xfer_ctrl);  // large enough for descriptors

    // Start tasks
    xTaskCreatePinnedToCore(usb_task, "usb", 4096, nullptr, 20, nullptr, 0);
    xTaskCreatePinnedToCore(protocol_task, "proto", 4096, nullptr, 5, nullptr, 1);

    LOG_SERIAL.println("Ready. Waiting for device...");
    print_help();
}

void loop() {
    process_serial();

    if (!g_connected) {
        static uint32_t last_msg = 0;
        if (millis() - last_msg > 5000) {
            last_msg = millis();
            LOG_SERIAL.println("Waiting for USB device...");
        }
    } else if (!g_setup_done) {
        delay(500);  // Let device settle
        device_setup_sequence();
        g_setup_done = true;
    }

    vTaskDelay(10);
}

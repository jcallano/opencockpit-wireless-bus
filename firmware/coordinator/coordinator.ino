/**
 * OpenCockpit Wireless Avionics Bus
 * Node A - Coordinator (UART0 + SLIP)
 *
 * - UART0 (SLIP framing) to PC
 * - ESP-NOW coordinator to peripheral nodes
 * - LCD metrics (messages/sec)
 */

#include <Arduino.h>
#include <SPI.h>
#include "USB.h"

struct NullSerial {
    template <typename... Args>
    void printf(const char*, Args...) {}
    void println(const char*) {}
};
static NullSerial NullSerialInstance;
#define DEBUG_SLIP 0
#include "include/slip_config.h"

#if DEBUG_SLIP
#define SLIP_LOG_SERIAL Serial0
#else
#define SLIP_LOG_SERIAL NullSerialInstance
#endif

#if SLIP_USE_CDC
#define ESPNOW_LOG_SERIAL Serial0
#else
#define ESPNOW_LOG_SERIAL Serial
#endif

#include "../common/slip.h"
#include "include/espnow_coordinator.h"

// ------------------------------------------------------------
// Forward declarations (PlatformIO build uses C++ compilation)
// ------------------------------------------------------------
static void process_slip_rx();
static void send_to_pc(uint8_t msg_type, uint8_t src_node, const void* payload, size_t payload_len);
static void onHIDInput(uint8_t node_id, uint8_t device_id, uint8_t report_id,
                       const uint8_t* report, uint8_t len);
static void onSerialData(uint8_t node_id, uint8_t port_id, const uint8_t* data, uint8_t len);
static void onNodeStatus(uint8_t node_id, bool connected);
static void onTestMessage(uint8_t msg_type, uint8_t node_id,
                          const uint8_t* data, uint8_t len);
static void espnow_task(void* parameter);

static void log_uart_status();
static bool slip_send_frame(const uint8_t* data, size_t len);

// ------------------------------------------------------------
// ESP32-S3-USB-OTG board pins (from Espressif reference)
// ------------------------------------------------------------
#define USB_SEL_PIN         18  // USB_SEL: HIGH=USB_HOST, LOW=USB_DEV (default)
#define USB_D_MINUS_PIN     19
#define USB_D_PLUS_PIN      20

#define LIMIT_EN_PIN        17  // LIMIT_EN: high enables current limit IC
#define DEV_VBUS_EN_PIN     12  // DEV_VBUS_EN: high enables DEV_VBUS supply
#define BOOST_EN_PIN        13  // BOOST_EN: high enables boost circuit

#define LCD_RST_PIN         8   // LCD_RET (active low)
#define LCD_EN_PIN          5   // LCD_EN (active low, used as CS)
#define LCD_DC_PIN          4   // LCD_DC: data/command select
#define LCD_SCLK_PIN        6   // LCD_SCLK: SPI clock
#define LCD_MOSI_PIN        7   // LCD_SDA: SPI MOSI
#define LCD_BL_PIN          9   // LCD_BL: backlight control

// Buttons (active low)
#define BTN_OK_PIN          0   // BUTTON_OK
#define BTN_DW_PIN          11  // BUTTON_DW
#define BTN_UP_PIN          10  // BUTTON_UP
#define BTN_MENU_PIN        14  // BUTTON_MENU

// LEDs (active high)
#define LED_GREEN_PIN       15  // LED_GREEN
#define LED_YELLOW_PIN      16  // LED_YELLOW

// ADC monitoring
#define HOST_VOL_PIN        1   // HOST_VOL (ADC1 CH0)
#define BAT_VOL_PIN         2   // BAT_VOL (ADC1 CH1)

// Over-current indicator
#define OVER_CURRENT_PIN    21  // OVER_CURRENT: high means overrun

// SD pins (shared with SDIO/SPI)
#define SD_SCK_PIN          36  // SD_SCK / SDIO CLK
#define SD_DO_PIN           37  // SD_DO / SDIO D0
#define SD_D1_PIN           38  // SDIO D1
#define SD_D2_PIN           33  // SDIO D2
#define SD_D3_PIN           34  // SD_D3 / SDIO D3 (SPI CS)

// ------------------------------------------------------------
// LCD parameters (ST7789, 240x240)
// ------------------------------------------------------------
#define LCD_WIDTH           240
#define LCD_HEIGHT          240

#define COLOR_BLACK         0x0000
#define COLOR_WHITE         0xFFFF
#define COLOR_GREEN         0x07E0
#define COLOR_CYAN          0x07FF
#define COLOR_YELLOW        0xFFE0

static SPIClass lcd_spi(SPI);
static SPISettings lcd_spi_settings(10000000, MSBFIRST, SPI_MODE0);

// ------------------------------------------------------------
// SLIP transport
// ------------------------------------------------------------
#if SLIP_USE_CDC
static Stream& slipSerial = USBSerial;
#else
static Stream& slipSerial = Serial0;
#endif
static SlipDecoder slip_decoder;

// ------------------------------------------------------------
// Metrics
// ------------------------------------------------------------
static volatile uint32_t g_slip_rx_frames = 0;
static volatile uint32_t g_slip_tx_frames = 0;
static volatile uint32_t g_esp_rx_frames = 0;
static volatile uint32_t g_esp_tx_frames = 0;
static volatile uint32_t g_esp_rx_bytes = 0;
static volatile uint32_t g_esp_tx_bytes = 0;

static uint32_t g_latency_sum_ms = 0;
static uint32_t g_latency_samples = 0;
static uint32_t g_jitter_max_ms = 0;
static uint32_t g_last_rtt_ms = 0;
static bool g_has_last_rtt = false;

static uint32_t g_test_seq = 0;
static uint32_t g_test_last_send_ms = 0;
static bool g_test_inflight = false;
static uint8_t g_latency_target_node = NODE_BROADCAST;

#define ESPNOW_LINK_BPS 6000000UL
#define TEST_PING_INTERVAL_MS 500
#define TEST_PING_TIMEOUT_MS 250

// ------------------------------------------------------------
// Font (5x7)
// ------------------------------------------------------------
static const uint8_t kFont5x7[] = {
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00,
    0x14,0x7F,0x14,0x7F,0x14, 0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62,
    0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00, 0x00,0x1C,0x22,0x41,0x00,
    0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00,
    0x20,0x10,0x08,0x04,0x02, 0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00,
    0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31, 0x18,0x14,0x12,0x7F,0x10,
    0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00,
    0x00,0x56,0x36,0x00,0x00, 0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14,
    0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06, 0x32,0x49,0x79,0x41,0x3E,
    0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01,
    0x3E,0x41,0x49,0x49,0x7A, 0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00,
    0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41, 0x7F,0x40,0x40,0x40,0x40,
    0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46,
    0x46,0x49,0x49,0x49,0x31, 0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F,
    0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F, 0x63,0x14,0x08,0x14,0x63,
    0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04,
    0x40,0x40,0x40,0x40,0x40, 0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78,
    0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20, 0x38,0x44,0x44,0x48,0x7F,
    0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x08,0x14,0x54,0x54,0x3C,
    0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00,
    0x7F,0x10,0x28,0x44,0x00, 0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78,
    0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38, 0x7C,0x14,0x14,0x14,0x08,
    0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
    0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C,
    0x3C,0x40,0x30,0x40,0x3C, 0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C,
    0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00, 0x00,0x00,0x7F,0x00,0x00,
    0x00,0x41,0x36,0x08,0x00, 0x08,0x08,0x2A,0x1C,0x08, 0x08,0x1C,0x2A,0x08,0x08
};

// ------------------------------------------------------------
// LCD helpers
// ------------------------------------------------------------
static void lcd_write_cmd(uint8_t cmd) {
    digitalWrite(LCD_DC_PIN, LOW);
    lcd_spi.beginTransaction(lcd_spi_settings);
    digitalWrite(LCD_EN_PIN, LOW);
    lcd_spi.write(cmd);
    digitalWrite(LCD_EN_PIN, HIGH);
    lcd_spi.endTransaction();
}

static void lcd_write_data(const uint8_t* data, size_t len) {
    digitalWrite(LCD_DC_PIN, HIGH);
    lcd_spi.beginTransaction(lcd_spi_settings);
    digitalWrite(LCD_EN_PIN, LOW);
    lcd_spi.writeBytes(data, len);
    digitalWrite(LCD_EN_PIN, HIGH);
    lcd_spi.endTransaction();
}

static void lcd_write_data8(uint8_t data) {
    lcd_write_data(&data, 1);
}

static void lcd_write_data16(uint16_t data) {
    uint8_t buf[2] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
    lcd_write_data(buf, sizeof(buf));
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_write_cmd(0x2A);
    lcd_write_data16(x0);
    lcd_write_data16(x1);
    lcd_write_cmd(0x2B);
    lcd_write_data16(y0);
    lcd_write_data16(y1);
    lcd_write_cmd(0x2C);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    lcd_set_window(x, y, x + w - 1, y + h - 1);
    digitalWrite(LCD_DC_PIN, HIGH);
    lcd_spi.beginTransaction(lcd_spi_settings);
    digitalWrite(LCD_EN_PIN, LOW);
    for (uint32_t i = 0; i < static_cast<uint32_t>(w) * h; i++) {
        lcd_spi.write16(color);
    }
    digitalWrite(LCD_EN_PIN, HIGH);
    lcd_spi.endTransaction();
}

static void lcd_fill_screen(uint16_t color) {
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale) {
    if (c < 0x20 || c > 0x7E) c = '?';
    uint16_t idx = (c - 0x20) * 5;

    for (uint8_t col = 0; col < 5; col++) {
        uint8_t line = kFont5x7[idx + col];
        for (uint8_t row = 0; row < 7; row++) {
            uint16_t px = x + col * scale;
            uint16_t py = y + row * scale;
            uint16_t draw_color = (line & 0x01) ? color : bg;
            if (scale == 1) {
                lcd_fill_rect(px, py, 1, 1, draw_color);
            } else {
                lcd_fill_rect(px, py, scale, scale, draw_color);
            }
            line >>= 1;
        }
    }
}

static void lcd_draw_text(uint16_t x, uint16_t y, const char* text, uint16_t color, uint16_t bg, uint8_t scale) {
    while (*text) {
        lcd_draw_char(x, y, *text, color, bg, scale);
        x += 6 * scale;
        text++;
    }
}

static void lcd_update_line(uint8_t index, const char* text, uint16_t color) {
    static char last_lines[8][32] = {};
    static const uint16_t kLineHeight = 20;
    static const uint16_t kStartY = 35;

    if (index >= 8) {
        return;
    }

    if (strncmp(last_lines[index], text, sizeof(last_lines[index]) - 1) == 0) {
        return;
    }

    uint16_t y = kStartY + index * kLineHeight;
    lcd_fill_rect(0, y, LCD_WIDTH, kLineHeight, COLOR_BLACK);
    lcd_draw_text(10, y, text, color, COLOR_BLACK, 2);
    strncpy(last_lines[index], text, sizeof(last_lines[index]) - 1);
    last_lines[index][sizeof(last_lines[index]) - 1] = '\0';
}

static void lcd_init() {
    pinMode(LCD_RST_PIN, OUTPUT);
    pinMode(LCD_EN_PIN, OUTPUT);
    pinMode(LCD_DC_PIN, OUTPUT);
    pinMode(LCD_BL_PIN, OUTPUT);

    digitalWrite(LCD_EN_PIN, HIGH);
    digitalWrite(LCD_BL_PIN, HIGH);

    lcd_spi.begin(LCD_SCLK_PIN, -1, LCD_MOSI_PIN, LCD_EN_PIN);

    digitalWrite(LCD_RST_PIN, LOW);
    delay(20);
    digitalWrite(LCD_RST_PIN, HIGH);
    delay(120);

    lcd_write_cmd(0x01); // SWRESET
    delay(150);
    lcd_write_cmd(0x11); // SLPOUT
    delay(120);
    lcd_write_cmd(0x3A); // COLMOD
    lcd_write_data8(0x55);
    lcd_write_cmd(0x36); // MADCTL
    lcd_write_data8(0x00);

    lcd_write_cmd(0x2A); // CASET
    lcd_write_data16(0);
    lcd_write_data16(LCD_WIDTH - 1);
    lcd_write_cmd(0x2B); // RASET
    lcd_write_data16(0);
    lcd_write_data16(LCD_HEIGHT - 1);

    lcd_write_cmd(0x20); // INVOFF
    lcd_write_cmd(0x13); // NORON
    lcd_write_cmd(0x29); // DISPON
    delay(100);

    lcd_fill_screen(COLOR_BLACK);
}

// ------------------------------------------------------------
// SLIP -> ESP-NOW routing
// ------------------------------------------------------------
static void handle_slip_frame(const uint8_t* data, size_t len) {
    if (!validateMessage(data, len)) {
#if DEBUG_SLIP
        SLIP_LOG_SERIAL.println("SLIP: invalid frame");
#endif
        return;
    }

    const MessageHeader* hdr = reinterpret_cast<const MessageHeader*>(data);
    const uint8_t* payload = data + sizeof(MessageHeader);
    size_t payload_len = len - sizeof(MessageHeader) - 1;
#if DEBUG_SLIP
    SLIP_LOG_SERIAL.printf("SLIP: msg=0x%02X src=0x%02X dst=0x%02X len=%u\n",
                   hdr->msg_type, hdr->src_node, hdr->dst_node,
                   static_cast<unsigned>(len));
#endif

    if (hdr->dst_node == NODE_COORDINATOR && hdr->msg_type == MSG_RESET) {
        ESP.restart();
        return;
    }

    switch (hdr->msg_type) {
        case MSG_HID_OUTPUT: {
            if (payload_len < 3) break;
            const HIDOutputPayload* out = reinterpret_cast<const HIDOutputPayload*>(payload);
            if (payload_len < static_cast<size_t>(3 + out->report_length)) break;
            if (coordinator.sendHIDOutput(hdr->dst_node, out->device_id,
                                          out->report_data, out->report_length)) {
                g_esp_tx_frames++;
                g_esp_tx_bytes += sizeof(MessageHeader) + 3 + out->report_length + 1;
            }
            break;
        }
        case MSG_SERIAL_DATA: {
            if (payload_len < 2) break;
            const SerialDataPayload* serial = reinterpret_cast<const SerialDataPayload*>(payload);
            if (payload_len < static_cast<size_t>(2 + serial->data_length)) break;
            if (coordinator.sendSerialData(hdr->dst_node, serial->port_id,
                                           serial->data, serial->data_length)) {
                g_esp_tx_frames++;
                g_esp_tx_bytes += sizeof(MessageHeader) + 2 + serial->data_length + 1;
            }
            break;
        }
        case MSG_MCDU_DISPLAY: {
            if (payload_len == 0) break;
            if (coordinator.sendMCDUDisplay(hdr->dst_node, payload,
                                            static_cast<uint8_t>(payload_len))) {
                g_esp_tx_frames++;
                g_esp_tx_bytes += sizeof(MessageHeader) + payload_len + 1;
            }
            break;
        }
        case MSG_TEST_REQ: {
            if (hdr->dst_node == NODE_COORDINATOR) {
                send_to_pc(MSG_TEST_RSP, NODE_COORDINATOR, payload, payload_len);
            } else {
                const PeerInfo* peer = coordinator.getPeerInfo(hdr->dst_node);
                if (peer) {
                    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
                    size_t msg_len = buildMessage(buffer, MSG_TEST_REQ, NODE_COORDINATOR,
                                                  hdr->dst_node, payload, payload_len);
                    esp_now_send(peer->mac_address, buffer, msg_len);
                    g_esp_tx_frames++;
                    g_esp_tx_bytes += msg_len;
                }
            }
            break;
        }
        case MSG_DISCOVERY_REQ: {
            for (uint8_t id = NODE_B_JOYSTICK; id < NODE_BROADCAST; id++) {
                const PeerInfo* peer = coordinator.getPeerInfo(id);
                if (!peer) continue;
                DiscoveryResponse rsp = {};
                rsp.node_type = peer->node_type;
                rsp.capabilities = peer->capabilities;
                rsp.device_count = peer->device_count;
                memcpy(rsp.mac_address, peer->mac_address, sizeof(rsp.mac_address));
                strncpy(rsp.firmware_version, "n/a", FIRMWARE_VERSION_SIZE - 1);
                strncpy(rsp.node_name, peer->node_name, MAX_NODE_NAME_SIZE - 1);

                uint8_t buffer[MAX_ESPNOW_PAYLOAD];
                size_t msg_len = buildMessage(buffer, MSG_DISCOVERY_RSP, peer->node_id,
                                              NODE_COORDINATOR, &rsp, sizeof(rsp));
                if (slip_send_frame(buffer, msg_len)) {
                    g_slip_tx_frames++;
                }
            }
            break;
        }
        default:
            break;
    }
}

static void process_slip_rx() {
    while (slipSerial.available()) {
        uint8_t byte = slipSerial.read();
        if (slip_decode_byte(&slip_decoder, byte)) {
            size_t len = slip_decoder.index;
            g_slip_rx_frames++;
            handle_slip_frame(slip_decoder.buffer, len);
            slip_decoder_reset(&slip_decoder);
        }
    }
}

static void send_to_pc(uint8_t msg_type, uint8_t src_node, const void* payload, size_t payload_len) {
    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t len = buildMessage(buffer, static_cast<MessageType>(msg_type),
                              src_node, NODE_COORDINATOR, payload, payload_len);
    if (slip_send_frame(buffer, len)) {
        g_slip_tx_frames++;
    }
}

// ------------------------------------------------------------
// ESP-NOW callbacks
// ------------------------------------------------------------
static void onHIDInput(uint8_t node_id, uint8_t device_id, uint8_t report_id,
                       const uint8_t* report, uint8_t len) {
    if (len > MAX_HID_REPORT_SIZE) {
        len = MAX_HID_REPORT_SIZE;
    }
    HIDInputPayload payload = {};
    payload.device_id = device_id;
    payload.report_id = report_id;
    payload.report_length = len;
    memcpy(payload.report_data, report, len);
    send_to_pc(MSG_HID_INPUT, node_id, &payload, 3 + len);
    g_esp_rx_frames++;
    g_esp_rx_bytes += sizeof(MessageHeader) + 3 + len + 1;
}

static void onSerialData(uint8_t node_id, uint8_t port_id, const uint8_t* data, uint8_t len) {
    if (len > MAX_SERIAL_DATA_SIZE) {
        len = MAX_SERIAL_DATA_SIZE;
    }
    SerialDataPayload payload = {};
    payload.port_id = port_id;
    payload.data_length = len;
    memcpy(payload.data, data, len);
    send_to_pc(MSG_SERIAL_DATA, node_id, &payload, 2 + len);
    g_esp_rx_frames++;
    g_esp_rx_bytes += sizeof(MessageHeader) + 2 + len + 1;
}

static void onNodeStatus(uint8_t node_id, bool connected) {
    if (!connected) {
        return;
    }
    const PeerInfo* peer = coordinator.getPeerInfo(node_id);
    if (!peer) {
        return;
    }
    DiscoveryResponse rsp = {};
    rsp.node_type = peer->node_type;
    rsp.capabilities = peer->capabilities;
    rsp.device_count = peer->device_count;
    memcpy(rsp.mac_address, peer->mac_address, sizeof(rsp.mac_address));
    strncpy(rsp.firmware_version, "n/a", FIRMWARE_VERSION_SIZE - 1);
    strncpy(rsp.node_name, peer->node_name, MAX_NODE_NAME_SIZE - 1);
    send_to_pc(MSG_DISCOVERY_RSP, node_id, &rsp, sizeof(rsp));
}

static void onTestMessage(uint8_t msg_type, uint8_t node_id,
                          const uint8_t* data, uint8_t len) {
    send_to_pc(msg_type, node_id, data, len);
    g_esp_rx_frames++;
    g_esp_rx_bytes += sizeof(MessageHeader) + len + 1;

    if (msg_type != MSG_TEST_RSP || len < 8) {
        return;
    }

    const TestPayload* payload = reinterpret_cast<const TestPayload*>(data);
    if ((payload->sequence & 0x80000000UL) == 0) {
        return;
    }

    uint32_t now = millis();
    uint32_t rtt_ms = now - payload->timestamp_ms;
    g_latency_sum_ms += rtt_ms;
    g_latency_samples++;

    if (g_has_last_rtt) {
        uint32_t delta = (rtt_ms > g_last_rtt_ms) ? (rtt_ms - g_last_rtt_ms)
                                                  : (g_last_rtt_ms - rtt_ms);
        if (delta > g_jitter_max_ms) {
            g_jitter_max_ms = delta;
        }
    }
    g_last_rtt_ms = rtt_ms;
    g_has_last_rtt = true;
    g_test_inflight = false;
}

// ------------------------------------------------------------
// Tasks
// ------------------------------------------------------------
static void espnow_task(void* parameter) {
    (void)parameter;
    while (true) {
        coordinator.process();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  #if !ARDUINO_USB_MODE
    USB.productName("OpenCockpit Wireless Bridge");
    USB.manufacturerName("OpenCockpit");
    USB.serialNumber("OC-A-001");
    USB.begin();
  #endif
    USBSerial.begin(115200);
    USBSerial.setRxBufferSize(4096);
    USBSerial.setTxTimeoutMs(50);
    USBSerial.enableReboot(false);
    Serial0.begin(115200);
    delay(500);

    pinMode(USB_SEL_PIN, OUTPUT);
    digitalWrite(USB_SEL_PIN, LOW); // USB_DEV to PC (per USB-OTG board routing)

    pinMode(DEV_VBUS_EN_PIN, OUTPUT);
    pinMode(BOOST_EN_PIN, OUTPUT);
    pinMode(LIMIT_EN_PIN, OUTPUT);
    digitalWrite(DEV_VBUS_EN_PIN, LOW);
    digitalWrite(BOOST_EN_PIN, LOW);
    digitalWrite(LIMIT_EN_PIN, LOW);

    slip_decoder_init(&slip_decoder);

    lcd_init();
    lcd_draw_text(10, 10, "OC WIRELESS BUS", COLOR_CYAN, COLOR_BLACK, 2);

    if (!coordinator.begin()) {
        while (true) {
            delay(1000);
        }
    }

    coordinator.setHIDInputCallback(onHIDInput);
    coordinator.setSerialDataCallback(onSerialData);
    coordinator.setNodeStatusCallback(onNodeStatus);
    coordinator.setTestMessageCallback(onTestMessage);

    xTaskCreatePinnedToCore(espnow_task, "espnow_task", 4096, nullptr, 24, nullptr, 0);
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop() {
    process_slip_rx();
    log_uart_status();

    static uint32_t last_stats_ms = 0;
    static uint32_t last_slip_rx = 0;
    static uint32_t last_slip_tx = 0;
    static uint32_t last_esp_rx = 0;
    static uint32_t last_esp_tx = 0;
    static uint32_t last_esp_rx_bytes = 0;
    static uint32_t last_esp_tx_bytes = 0;
    static uint32_t last_ping_attempt_ms = 0;

    uint32_t now = millis();
    if (now - last_ping_attempt_ms >= TEST_PING_INTERVAL_MS) {
        if (g_test_inflight && (now - g_test_last_send_ms) > TEST_PING_TIMEOUT_MS) {
            g_test_inflight = false;
        }

        if (!g_test_inflight) {
            if (g_latency_target_node == NODE_BROADCAST) {
                for (uint8_t id = NODE_B_JOYSTICK; id < NODE_BROADCAST; id++) {
                    const PeerInfo* peer = coordinator.getPeerInfo(id);
                    if (peer && peer->connected) {
                        g_latency_target_node = id;
                        break;
                    }
                }
            }

            if (g_latency_target_node != NODE_BROADCAST) {
                const PeerInfo* peer = coordinator.getPeerInfo(g_latency_target_node);
                if (peer && peer->connected) {
                    TestPayload payload = {};
                    payload.sequence = 0x80000000UL | (g_test_seq++ & 0x7FFFFFFFUL);
                    payload.timestamp_ms = now;

                    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
                    size_t msg_len = buildMessage(buffer, MSG_TEST_REQ, NODE_COORDINATOR,
                                                  g_latency_target_node, &payload, sizeof(payload));
                    esp_now_send(peer->mac_address, buffer, msg_len);
                    g_esp_tx_frames++;
                    g_esp_tx_bytes += msg_len;
                    g_test_inflight = true;
                    g_test_last_send_ms = now;
                } else {
                    g_latency_target_node = NODE_BROADCAST;
                }
            }
        }
        last_ping_attempt_ms = now;
    }

    if (now - last_stats_ms >= 1000) {
        uint32_t slip_rx = g_slip_rx_frames;
        uint32_t slip_tx = g_slip_tx_frames;
        uint32_t esp_rx = g_esp_rx_frames;
        uint32_t esp_tx = g_esp_tx_frames;
        uint32_t esp_rx_bytes = g_esp_rx_bytes;
        uint32_t esp_tx_bytes = g_esp_tx_bytes;

        uint32_t slip_rx_rate = slip_rx - last_slip_rx;
        uint32_t slip_tx_rate = slip_tx - last_slip_tx;
        uint32_t esp_rx_rate = esp_rx - last_esp_rx;
        uint32_t esp_tx_rate = esp_tx - last_esp_tx;
        uint32_t esp_rx_bps = esp_rx_bytes - last_esp_rx_bytes;
        uint32_t esp_tx_bps = esp_tx_bytes - last_esp_tx_bytes;

        last_slip_rx = slip_rx;
        last_slip_tx = slip_tx;
        last_esp_rx = esp_rx;
        last_esp_tx = esp_tx;
        last_esp_rx_bytes = esp_rx_bytes;
        last_esp_tx_bytes = esp_tx_bytes;
        last_stats_ms = now;

        char line1[32];
        char line2[32];
        char line3[32];
        char line4[32];
        char line5[32];
        char line6[32];
        char line7[32];
        char line8[32];

        uint32_t total_bps = (esp_rx_bps + esp_tx_bps) * 8;
        uint32_t channel_use = (total_bps * 100UL) / ESPNOW_LINK_BPS;
        if (channel_use > 100) {
            channel_use = 100;
        }

        uint32_t avg_rtt_ms = 0;
        if (g_latency_samples > 0) {
            avg_rtt_ms = (g_latency_sum_ms + (g_latency_samples / 2)) / g_latency_samples;
        }
        uint8_t nodes = coordinator.getConnectedNodeCount();

        snprintf(line1, sizeof(line1), "Nodes: %u", nodes);
        snprintf(line2, sizeof(line2), "SLIP RX: %lu/s", static_cast<unsigned long>(slip_rx_rate));
        snprintf(line3, sizeof(line3), "SLIP TX: %lu/s", static_cast<unsigned long>(slip_tx_rate));
        snprintf(line4, sizeof(line4), "ESP RX: %lu/s", static_cast<unsigned long>(esp_rx_rate));
        snprintf(line5, sizeof(line5), "ESP TX: %lu/s", static_cast<unsigned long>(esp_tx_rate));
        if (g_latency_samples > 0) {
            snprintf(line6, sizeof(line6), "RTT avg: %lums", static_cast<unsigned long>(avg_rtt_ms));
            snprintf(line7, sizeof(line7), "Jitter max: %lums", static_cast<unsigned long>(g_jitter_max_ms));
        } else {
            snprintf(line6, sizeof(line6), "RTT avg: --");
            snprintf(line7, sizeof(line7), "Jitter max: --");
        }
        snprintf(line8, sizeof(line8), "Chan use: %lu%%", static_cast<unsigned long>(channel_use));

        lcd_update_line(0, line1, COLOR_GREEN);
        lcd_update_line(1, line2, COLOR_GREEN);
        lcd_update_line(2, line3, COLOR_GREEN);
        lcd_update_line(3, line4, COLOR_YELLOW);
        lcd_update_line(4, line5, COLOR_YELLOW);
        lcd_update_line(5, line6, COLOR_CYAN);
        lcd_update_line(6, line7, COLOR_CYAN);
        lcd_update_line(7, line8, COLOR_CYAN);

        g_latency_sum_ms = 0;
        g_latency_samples = 0;
        g_jitter_max_ms = 0;
    }

    delay(1);
}

static void log_uart_status() {
    (void)slipSerial;
}

static bool slip_send_frame(const uint8_t* data, size_t len) {
#if SLIP_USE_CDC
    if (!USBSerial) {
        return false;
    }

    size_t encoded_len = slip_encoded_size(data, len);
    static uint8_t encoded[SLIP_MAX_FRAME_SIZE * 2 + 2];
    if (encoded_len > sizeof(encoded)) {
        return false;
    }
    if (USBSerial.availableForWrite() < static_cast<int>(encoded_len)) {
        return false;
    }

    slip_encode(data, len, encoded);
    size_t written = USBSerial.write(encoded, encoded_len);
    return written == encoded_len;
#else
    slip_send(data, len, slipSerial);
    return true;
#endif
}
